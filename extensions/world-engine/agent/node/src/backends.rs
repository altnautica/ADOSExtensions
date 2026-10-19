//! The live-lane reconstructor backends, behind the [`Reconstructor`] trait.
//!
//! A keyframe dataset streamed during a capture (Nerfstudio `transforms.json`
//! + `images/`, written by the keyframe persister) trains a splat directly on
//! its recorded poses: Brush (Rust, Metal/Vulkan/wgpu) everywhere, or msplat
//! (native Metal) on Apple Silicon when the dataset carries a `points3D.ply`
//! seed. Both resolve from the pinned tool set ([`crate::tools`]); a node
//! without them fails the job with `E_TOOL_MISSING` rather than producing a
//! placeholder. Session reconstructions (COLMAP, gsplat, mesh, products) run
//! on the staged pipeline ([`crate::recon`]), not here.
//!
//! The command builder and the gaussian-count parse are pure and unit-tested.

use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::Arc;

use crate::recon::{ProcessRunner, ToolRunner};
use crate::{ComputeError, Dataset, ReconstructOutput, Reconstructor};

/// A reconstructor that picks the trainer per job (reading the `backend`
/// param hint via [`select_reconstructor`]) and writes artifacts under one
/// work root.
pub struct SelectingReconstructor {
    work_root: PathBuf,
}

impl SelectingReconstructor {
    /// A selector that writes artifacts under `work_root`.
    pub fn new(work_root: impl Into<PathBuf>) -> Self {
        Self {
            work_root: work_root.into(),
        }
    }
}

impl Reconstructor for SelectingReconstructor {
    fn name(&self) -> &str {
        "selecting"
    }

    fn reconstruct(
        &self,
        dataset: &Dataset,
        params: &serde_json::Value,
    ) -> Result<ReconstructOutput, ComputeError> {
        select_reconstructor(params, &self.work_root).reconstruct(dataset, params)
    }
}

/// The live-lane trainers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReconstructorKind {
    /// Brush: Rust gaussian-splat trainer (Metal / Vulkan / wgpu). Initialises
    /// its own cloud, so it trains from a pose-only dataset.
    Brush,
    /// msplat: native-Metal trainer for Apple Silicon; needs a `points3D.ply`
    /// beside `transforms.json`.
    Msplat,
}

impl ReconstructorKind {
    /// Parse a backend hint (the `backend` job param), if known.
    pub fn from_hint(hint: &str) -> Option<Self> {
        match hint.to_ascii_lowercase().as_str() {
            "brush" => Some(Self::Brush),
            "msplat" => Some(Self::Msplat),
            _ => None,
        }
    }

    /// The tool name in the pinned set (also the program name).
    pub fn program(self) -> &'static str {
        match self {
            Self::Brush => "brush",
            Self::Msplat => "msplat",
        }
    }

    /// A stable backend name for logs + the job result.
    pub fn name(self) -> &'static str {
        self.program()
    }
}

/// A fully-built reconstruction command, built purely from a dataset + params
/// so it can be asserted in a test without executing.
#[derive(Debug, Clone, PartialEq)]
pub struct ReconstructCommand {
    pub program: String,
    pub args: Vec<String>,
    pub workdir: PathBuf,
    pub output_path: PathBuf,
    pub output_uri: String,
}

/// A reconstructor that drives one trainer via the command line.
pub struct CliReconstructor {
    kind: ReconstructorKind,
    work_root: PathBuf,
}

impl CliReconstructor {
    /// A reconstructor for `kind`, writing artifacts under `work_root`.
    pub fn new(kind: ReconstructorKind, work_root: impl Into<PathBuf>) -> Self {
        Self {
            kind,
            work_root: work_root.into(),
        }
    }

    /// The tool this reconstructor drives.
    pub fn kind(&self) -> ReconstructorKind {
        self.kind
    }

    /// Build the command for `dataset`: the input is the dataset's
    /// `input_path` (else `<work_root>/<id>/input`); the trainer writes
    /// `<work_root>/<id>/output.ply`. `params.steps`, `max_splats`,
    /// `sh_degree` and `growth_stop_iter` tune it.
    pub fn command(&self, dataset: &Dataset, params: &serde_json::Value) -> ReconstructCommand {
        let workdir = self.work_root.join(&dataset.id);
        let output_path = workdir.join("output.ply");
        let input = dataset
            .meta
            .get("input_path")
            .and_then(|v| v.as_str())
            .map(PathBuf::from)
            .unwrap_or_else(|| workdir.join("input"));
        let input_s = input.to_string_lossy().into_owned();
        let output_s = output_path.to_string_lossy().into_owned();
        let output_uri = path_to_file_uri(&output_s);
        let u = |k: &str| params.get(k).and_then(|v| v.as_u64());
        let steps = u("steps").unwrap_or(30_000);
        let sh_degree = u("sh_degree").unwrap_or(3).min(4);
        let args: Vec<String> = match self.kind {
            ReconstructorKind::Brush => {
                // Bound densification: Brush's defaults grow for 15k steps with
                // a 10M cap, which makes each step progressively slower.
                let growth_stop = u("growth_stop_iter").unwrap_or_else(|| (steps / 2).min(10_000));
                let max_splats = u("max_splats").unwrap_or(1_500_000);
                vec![
                    input_s,
                    "--total-steps".into(),
                    steps.to_string(),
                    "--growth-stop-iter".into(),
                    growth_stop.to_string(),
                    "--max-splats".into(),
                    max_splats.to_string(),
                    "--sh-degree".into(),
                    sh_degree.to_string(),
                    "--export-every".into(),
                    steps.to_string(),
                    "--export-path".into(),
                    workdir.to_string_lossy().into_owned(),
                    "--export-name".into(),
                    "output.ply".into(),
                ]
            }
            ReconstructorKind::Msplat => vec![
                input_s,
                "-n".into(),
                steps.to_string(),
                "--sh-degree".into(),
                sh_degree.to_string(),
                "-o".into(),
                output_s,
            ],
        };
        ReconstructCommand {
            program: self.kind.program().to_string(),
            args,
            workdir,
            output_path,
            output_uri,
        }
    }
}

fn backend_err(kind: ReconstructorKind, message: String) -> ComputeError {
    ComputeError::Backend {
        backend: kind.name().into(),
        message,
    }
}

impl Reconstructor for CliReconstructor {
    fn name(&self) -> &str {
        self.kind.name()
    }

    fn reconstruct(
        &self,
        dataset: &Dataset,
        params: &serde_json::Value,
    ) -> Result<ReconstructOutput, ComputeError> {
        let tool = ProcessRunner::default()
            .resolve(self.kind.program())
            .map_err(|e| backend_err(self.kind, e.to_string()))?;
        let cmd = self.command(dataset, params);
        std::fs::create_dir_all(&cmd.workdir)
            .map_err(|e| backend_err(self.kind, format!("create workdir: {e}")))?;
        let output = Command::new(&tool.program)
            .args(&tool.prefix_args)
            .args(&cmd.args)
            .current_dir(&cmd.workdir)
            .output()
            .map_err(|e| {
                backend_err(self.kind, format!("spawn {}: {e}", tool.program.display()))
            })?;
        if !output.status.success() {
            let stderr = String::from_utf8_lossy(&output.stderr);
            return Err(backend_err(
                self.kind,
                format!(
                    "{} exited {}: {}",
                    cmd.program,
                    output.status,
                    stderr.trim()
                ),
            ));
        }
        let stdout = String::from_utf8_lossy(&output.stdout);
        Ok(ReconstructOutput {
            kind: "splat".into(),
            uri: cmd.output_uri,
            gaussian_count: parse_gaussian_count(&stdout),
            backend: self.kind.name().into(),
        })
    }
}

/// Build a `file://` URI from an absolute filesystem path, percent-encoding the
/// few characters that break a URI path (space, `#`, `?`, `%`).
pub fn path_to_file_uri(path: &str) -> String {
    let mut out = String::from("file://");
    for ch in path.chars() {
        match ch {
            ' ' => out.push_str("%20"),
            '#' => out.push_str("%23"),
            '?' => out.push_str("%3F"),
            '%' => out.push_str("%25"),
            c => out.push(c),
        }
    }
    out
}

/// Resolve a `file://` URI (or a bare path) back to a filesystem path, reversing
/// the percent-encoding [`path_to_file_uri`] applies.
pub fn file_uri_to_path(uri: &str) -> PathBuf {
    let path = uri.strip_prefix("file://").unwrap_or(uri);
    if !path.contains('%') {
        return PathBuf::from(path);
    }
    let mut out = String::with_capacity(path.len());
    let mut chars = path.chars();
    while let Some(c) = chars.next() {
        if c == '%' {
            let hex: String = chars.by_ref().take(2).collect();
            if hex.len() == 2 {
                if let Ok(b) = u8::from_str_radix(&hex, 16) {
                    out.push(b as char);
                    continue;
                }
            }
            out.push('%');
            out.push_str(&hex);
        } else {
            out.push(c);
        }
    }
    PathBuf::from(out)
}

/// Pull a gaussian count out of a splat trainer's output (`gaussians: N`,
/// `Num splats: N`); the last match wins, 0 when absent.
pub fn parse_gaussian_count(stdout: &str) -> u64 {
    let mut count = 0u64;
    for line in stdout.lines() {
        let lower = line.to_ascii_lowercase();
        if let Some(idx) = lower
            .find("gaussians")
            .or_else(|| lower.find("num splats"))
            .or_else(|| lower.find("splats:"))
        {
            // The first contiguous digit run after the marker (a line often
            // carries a second number).
            let run: String = line[idx..]
                .chars()
                .skip_while(|c| !c.is_ascii_digit())
                .take_while(|c| c.is_ascii_digit())
                .collect();
            if let Ok(n) = run.parse::<u64>() {
                count = n;
            }
        }
    }
    count
}

/// True on an Apple-Silicon Mac, where msplat is the fast path.
pub fn is_apple_silicon() -> bool {
    cfg!(target_os = "macos") && cfg!(target_arch = "aarch64")
}

/// msplat when the dataset carries a `points3D.ply` seed (from a recorded
/// LiDAR or SfM cloud) and msplat is installed; Brush otherwise.
pub struct SeededSplatReconstructor {
    work_root: PathBuf,
}

impl SeededSplatReconstructor {
    pub fn new(work_root: impl Into<PathBuf>) -> Self {
        Self {
            work_root: work_root.into(),
        }
    }

    /// Whether the dataset carries a seed cloud msplat can start from.
    fn has_seed(dataset: &Dataset) -> bool {
        dataset
            .meta
            .get("input_path")
            .and_then(|v| v.as_str())
            .is_some_and(|p| Path::new(p).join("points3D.ply").is_file())
    }
}

impl Reconstructor for SeededSplatReconstructor {
    fn name(&self) -> &str {
        "seeded-splat"
    }

    fn reconstruct(
        &self,
        dataset: &Dataset,
        params: &serde_json::Value,
    ) -> Result<ReconstructOutput, ComputeError> {
        let msplat = Self::has_seed(dataset)
            && ProcessRunner::default()
                .resolve(ReconstructorKind::Msplat.program())
                .is_ok();
        let kind = if msplat {
            ReconstructorKind::Msplat
        } else {
            ReconstructorKind::Brush
        };
        CliReconstructor::new(kind, &self.work_root).reconstruct(dataset, params)
    }
}

/// Pick the trainer for a live-lane job: `msplat` (seeded path) or `brush` by
/// hint; `auto` (or no hint) is the seeded path on Apple Silicon, Brush
/// elsewhere. An unknown hint runs Brush.
pub fn select_reconstructor(
    params: &serde_json::Value,
    work_root: impl Into<PathBuf>,
) -> Arc<dyn Reconstructor> {
    let work_root = work_root.into();
    let hint = params
        .get("backend")
        .and_then(|v| v.as_str())
        .unwrap_or("auto");
    match ReconstructorKind::from_hint(hint) {
        Some(ReconstructorKind::Msplat) => Arc::new(SeededSplatReconstructor::new(work_root)),
        Some(ReconstructorKind::Brush) => {
            Arc::new(CliReconstructor::new(ReconstructorKind::Brush, work_root))
        }
        None if is_apple_silicon() => Arc::new(SeededSplatReconstructor::new(work_root)),
        None => Arc::new(CliReconstructor::new(ReconstructorKind::Brush, work_root)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dataset(id: &str, input: Option<&str>) -> Dataset {
        let meta = match input {
            Some(p) => serde_json::json!({ "input_path": p }),
            None => serde_json::json!({}),
        };
        Dataset {
            id: id.into(),
            kind: "bag".into(),
            created_ms: 0,
            meta,
        }
    }

    fn arg_val(args: &[String], flag: &str) -> Option<String> {
        args.iter()
            .position(|a| a == flag)
            .and_then(|i| args.get(i + 1).cloned())
    }

    #[test]
    fn kind_from_hint_maps_known_tools() {
        assert_eq!(
            ReconstructorKind::from_hint("BRUSH"),
            Some(ReconstructorKind::Brush)
        );
        assert_eq!(
            ReconstructorKind::from_hint("msplat"),
            Some(ReconstructorKind::Msplat)
        );
        assert_eq!(ReconstructorKind::from_hint("splatfacto"), None);
    }

    #[test]
    fn brush_command_bounds_densification_and_honors_overrides() {
        let r = CliReconstructor::new(ReconstructorKind::Brush, "/work");
        let cmd = r.command(
            &dataset("ds", Some("/d")),
            &serde_json::json!({ "steps": 7000 }),
        );
        assert_eq!(cmd.program, "brush");
        assert_eq!(cmd.output_path, Path::new("/work/ds/output.ply"));
        assert_eq!(cmd.args.first().map(String::as_str), Some("/d"));
        assert_eq!(
            arg_val(&cmd.args, "--growth-stop-iter").as_deref(),
            Some("3500")
        );
        assert_eq!(
            arg_val(&cmd.args, "--max-splats").as_deref(),
            Some("1500000")
        );
        assert_eq!(
            arg_val(&cmd.args, "--export-name").as_deref(),
            Some("output.ply")
        );
        let cmd = r.command(
            &dataset("ds", Some("/d")),
            &serde_json::json!({ "steps": 15000, "max_splats": 600000, "sh_degree": 2, "growth_stop_iter": 5000 }),
        );
        assert_eq!(
            arg_val(&cmd.args, "--max-splats").as_deref(),
            Some("600000")
        );
        assert_eq!(arg_val(&cmd.args, "--sh-degree").as_deref(), Some("2"));
        assert_eq!(
            arg_val(&cmd.args, "--growth-stop-iter").as_deref(),
            Some("5000")
        );
    }

    #[test]
    fn msplat_command_maps_dataset_steps_and_sh() {
        let r = CliReconstructor::new(ReconstructorKind::Msplat, "/work");
        let cmd = r.command(
            &dataset("ds", Some("/colmap")),
            &serde_json::json!({ "steps": 7000, "sh_degree": 2 }),
        );
        assert_eq!(cmd.program, "msplat");
        assert_eq!(arg_val(&cmd.args, "-n").as_deref(), Some("7000"));
        assert_eq!(
            arg_val(&cmd.args, "-o").map(PathBuf::from),
            Some(PathBuf::from("/work/ds").join("output.ply"))
        );
    }

    #[test]
    fn gaussian_count_parses_the_first_run_only() {
        assert_eq!(
            parse_gaussian_count("training...\ngaussians: 248123\n"),
            248123
        );
        assert_eq!(
            parse_gaussian_count("Num splats: 1000\nNum splats: 2500"),
            2500
        );
        assert_eq!(parse_gaussian_count("no count here"), 0);
        assert_eq!(parse_gaussian_count("gaussians: 248123 of 1000000"), 248123);
    }

    #[test]
    fn selection_never_falls_back_to_a_placeholder() {
        let r = select_reconstructor(&serde_json::json!({ "backend": "msplat" }), "/work");
        assert_eq!(r.name(), "seeded-splat");
        let r = select_reconstructor(&serde_json::json!({ "backend": "no-such-tool" }), "/work");
        assert_ne!(r.name(), "mock");
    }

    #[test]
    fn an_uninstalled_trainer_fails_with_tool_missing() {
        let dir = tempfile::tempdir().unwrap();
        std::env::set_var("ADOS_WE_TOOLS_DIR", dir.path());
        let err = CliReconstructor::new(ReconstructorKind::Brush, dir.path())
            .reconstruct(&dataset("ds-x", None), &serde_json::json!({}))
            .unwrap_err();
        match err {
            ComputeError::Backend { backend, message } => {
                assert_eq!(backend, "brush");
                assert!(message.contains("E_TOOL_MISSING"), "{message}");
            }
            other => panic!("expected a backend error, got {other:?}"),
        }
    }

    #[test]
    fn file_uri_round_trips_a_spaced_path() {
        let uri = path_to_file_uri("/My Work/ds 1/output.ply");
        assert_eq!(uri, "file:///My%20Work/ds%201/output.ply");
        assert_eq!(
            file_uri_to_path(&uri),
            Path::new("/My Work/ds 1/output.ply")
        );
        assert_eq!(file_uri_to_path("/plain/path"), Path::new("/plain/path"));
    }
}
