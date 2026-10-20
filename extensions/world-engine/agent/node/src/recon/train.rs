//! Stage `train` (gaussian splat), plus the python helper steps (masks,
//! depth priors, hero stills) that run in the pinned `gsplat_train` project.

use std::fs;
use std::path::{Path, PathBuf};

use serde::{Deserialize, Serialize};

use super::{StageError, ToolInvocation, ToolRunner};

/// What `train` produced (its stage detail).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct TrainSummary {
    /// `brush` or `gsplat`.
    pub trainer: String,
    pub steps: u64,
    pub max_splats: u64,
    pub gaussians: Option<u64>,
    /// Mean training-view PSNR of the exported splat as rendered.
    pub psnr: Option<f64>,
    /// The same after a per-view least-squares colour affine: the geometry
    /// fit apart from per-view colour (which the appearance model absorbs in
    /// training). A wide gap to `psnr` means the colours went into it.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub psnr_cc: Option<f64>,
    /// The per-image appearance model gsplat trained with.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub appearance: Option<Appearance>,
    /// Gaussians left out of the exported splat (lost or invisible).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub culled: Option<u64>,
    /// Steps skipped because their loss was not finite.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub nonfinite_steps: Option<u64>,
    /// 3DGUT (unscented transform) was on.
    pub ut: bool,
    /// Why the high-fidelity trainer did not run (Brush stood in).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fallback_reason: Option<String>,
}

/// The per-image appearance model of gsplat training (job param
/// `appearance`, `affine` when absent). It absorbs exposure and white balance
/// in training only; the exported splat never carries it.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Appearance {
    /// One 3x4 colour affine per image.
    #[default]
    Affine,
    /// A bilateral grid of affines per image (position and luminance).
    Grid,
    None,
}

impl Appearance {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "affine" => Some(Self::Affine),
            "grid" => Some(Self::Grid),
            "none" => Some(Self::None),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Affine => "affine",
            Self::Grid => "grid",
            Self::None => "none",
        }
    }
}

/// One JSON event line a project script prints.
#[derive(Debug, Clone, Deserialize)]
struct Event {
    event: String,
    #[serde(default)]
    progress: Option<f32>,
    #[serde(default)]
    message: Option<String>,
    #[serde(flatten)]
    rest: serde_json::Map<String, serde_json::Value>,
}

/// Run a `gsplat_train` project script, forwarding its progress events;
/// returns the fields of its final `done` event.
pub fn run_script(
    runner: &dyn ToolRunner,
    script: &str,
    args: Vec<std::ffi::OsString>,
    cwd: &Path,
    progress: &mut dyn FnMut(f32),
) -> Result<serde_json::Map<String, serde_json::Value>, StageError> {
    let tool = runner.resolve("gsplat_train")?;
    let script_path = tool.home.join(script);
    let inv = ToolInvocation::new("gsplat_train", cwd)
        .arg(script_path)
        .args(args);
    let mut done = None;
    let mut failure = None;
    let ran = runner.run(&inv, &mut |line| {
        let Ok(ev) = serde_json::from_str::<Event>(line.trim()) else {
            return;
        };
        match ev.event.as_str() {
            "progress" => {
                if let Some(p) = ev.progress {
                    progress(p.clamp(0.0, 1.0));
                }
            }
            "done" => done = Some(ev.rest),
            "error" => failure = ev.message,
            _ => {}
        }
    });
    // The script's own error event names the cause; the exit status alone
    // carries only the output tail.
    if let Some(msg) = failure {
        return Err(StageError::tool_failed(
            "gsplat_train",
            format!("{script}: {msg}"),
        ));
    }
    ran?;
    done.ok_or_else(|| {
        StageError::tool_failed("gsplat_train", format!("{script} reported no result"))
    })
}

/// The splat cap for this GPU: the preset's 8 GB budget scaled by VRAM.
pub fn max_splats_for(budget_8gb: u64, vram_mb: Option<u64>) -> u64 {
    match vram_mb {
        Some(mb) if mb > 0 => ((budget_8gb as f64) * (mb as f64 / 8192.0)).round() as u64,
        _ => budget_8gb,
    }
    .max(100_000)
}

/// Train with Brush on the COLMAP dataset dir (`dataset/` holds `images/`
/// and `sparse/0`). Brush logs progress only with `RUST_LOG=info`.
pub fn brush(
    runner: &dyn ToolRunner,
    dataset: &Path,
    train_dir: &Path,
    steps: u64,
    max_splats: u64,
    reason: Option<&str>,
    progress: &mut dyn FnMut(f32),
) -> Result<TrainSummary, StageError> {
    fs::create_dir_all(train_dir)?;
    let mut inv = ToolInvocation::new("brush", train_dir)
        .arg(dataset)
        .arg("--total-steps")
        .arg(steps.to_string())
        .arg("--growth-stop-iter")
        .arg((steps / 2).min(15_000).to_string())
        .arg("--max-splats")
        .arg(max_splats.to_string())
        .arg("--export-every")
        .arg(steps.to_string())
        .arg("--export-path")
        .arg(train_dir)
        .args(["--export-name", "splat.ply"]);
    inv.env.push(("RUST_LOG".into(), "info".into()));
    runner.run(&inv, &mut |line| {
        if let Some(step) = parse_step(line) {
            progress((step as f32 / steps as f32).min(1.0));
        }
    })?;
    let splat = train_dir.join("splat.ply");
    if !splat.is_file() {
        return Err(StageError::tool_failed("brush", "no splat.ply exported"));
    }
    Ok(TrainSummary {
        trainer: "brush".into(),
        steps,
        max_splats,
        // The exported file is the authoritative count: Brush's log reports
        // counts only while it densifies.
        gaussians: ply_vertex_count(&splat),
        psnr: None,
        fallback_reason: reason.map(str::to_string),
        ..Default::default()
    })
}

/// The `element vertex N` count of a PLY header.
pub fn ply_vertex_count(path: &Path) -> Option<u64> {
    use std::io::BufRead;
    let f = fs::File::open(path).ok()?;
    for line in std::io::BufReader::new(f).split(b'\n').take(256) {
        let line = line.ok()?;
        let line = String::from_utf8_lossy(&line);
        let line = line.trim();
        if line == "end_header" {
            return None;
        }
        if let Some(n) = line.strip_prefix("element vertex ") {
            return n.trim().parse().ok();
        }
    }
    None
}

/// Whether the gsplat kernels run on this node (`common.py --probe`). The
/// error says why not.
pub fn gsplat_available(runner: &dyn ToolRunner, cwd: &Path) -> Result<(), StageError> {
    let tool = runner.resolve("gsplat_train")?;
    let inv = ToolInvocation::new("gsplat_train", cwd)
        .arg(tool.home.join("common.py"))
        .arg("--probe");
    runner.run(&inv, &mut |_| {})
}

/// A `step N` / `iter N` counter in a trainer log line.
fn parse_step(line: &str) -> Option<u64> {
    let lower = line.to_ascii_lowercase();
    let at = lower.find("step").or_else(|| lower.find("iter"))?;
    lower[at..]
        .chars()
        .skip_while(|c| !c.is_ascii_digit())
        .take_while(|c| c.is_ascii_digit())
        .collect::<String>()
        .parse()
        .ok()
}

/// Train with gsplat (MCMC; 3DGUT for fisheye or rolling-shutter cameras;
/// a per-image appearance model for exposure variation).
#[allow(clippy::too_many_arguments)]
pub fn gsplat(
    runner: &dyn ToolRunner,
    dataset: &Path,
    train_dir: &Path,
    steps: u64,
    max_splats: u64,
    ut: bool,
    appearance: Appearance,
    depths: Option<&Path>,
    progress: &mut dyn FnMut(f32),
) -> Result<TrainSummary, StageError> {
    fs::create_dir_all(train_dir)?;
    let mut args: Vec<std::ffi::OsString> = vec![
        "--colmap".into(),
        dataset.into(),
        "--images".into(),
        dataset.join("images").into(),
        "--out".into(),
        train_dir.into(),
        "--steps".into(),
        steps.to_string().into(),
        "--max-splats".into(),
        max_splats.to_string().into(),
        "--appearance".into(),
        appearance.as_str().into(),
    ];
    let masks = dataset.join("masks");
    if masks.is_dir() {
        args.extend(["--masks".into(), masks.into()]);
    }
    if let Some(d) = depths {
        args.extend(["--depths".into(), d.into()]);
    }
    if ut {
        args.push("--ut".into());
    }
    let done = run_script(runner, "train.py", args, train_dir, progress)?;
    if !train_dir.join("splat.ply").is_file() {
        return Err(StageError::tool_failed(
            "gsplat_train",
            "no splat.ply written",
        ));
    }
    let count = |k: &str| done.get(k).and_then(|v| v.as_u64());
    Ok(TrainSummary {
        trainer: "gsplat".into(),
        steps,
        max_splats,
        gaussians: count("gaussians"),
        psnr: done.get("psnr").and_then(|v| v.as_f64()),
        psnr_cc: done.get("psnr_cc").and_then(|v| v.as_f64()),
        appearance: Some(appearance),
        culled: count("culled"),
        nonfinite_steps: count("nonfinite_steps"),
        ut,
        fallback_reason: None,
    })
}

/// Monocular metric depth priors for every image (`dataset/depths/`).
pub fn depth_priors(
    runner: &dyn ToolRunner,
    dataset: &Path,
    progress: &mut dyn FnMut(f32),
) -> Result<PathBuf, StageError> {
    let out = dataset.join("depths");
    fs::create_dir_all(&out)?;
    run_script(
        runner,
        "depth.py",
        vec![
            "--images".into(),
            dataset.join("images").into(),
            "--out".into(),
            out.clone().into(),
        ],
        dataset,
        progress,
    )?;
    Ok(out)
}

/// Which mask passes ran, reported on the `prepare` stage so a missing pass
/// is visible on the job status rather than silent.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MaskSummary {
    /// Ego-motion flow-residual masks (always on).
    pub motion: bool,
    /// Person masks from the detector model.
    pub person: bool,
    /// Mirror/glass masks from depth vs LiDAR disagreement.
    pub mirror_glass: bool,
    /// Why a pass did not run.
    pub off: Vec<String>,
}

/// Dynamic-object and mirror/glass masks (`dataset/masks/`).
pub fn masks(
    runner: &dyn ToolRunner,
    dataset: &Path,
    person_model: Option<&Path>,
    progress: &mut dyn FnMut(f32),
) -> Result<MaskSummary, StageError> {
    let out = dataset.join("masks");
    fs::create_dir_all(&out)?;
    let mut args: Vec<std::ffi::OsString> = vec![
        "--images".into(),
        dataset.join("images").into(),
        "--frames".into(),
        dataset.join("frames.json").into(),
        "--out".into(),
        out.into(),
    ];
    let mut summary = MaskSummary {
        motion: true,
        person: false,
        mirror_glass: false,
        off: Vec::new(),
    };
    match person_model {
        Some(m) if m.is_file() => {
            args.extend(["--person-model".into(), m.into()]);
            summary.person = true;
        }
        Some(m) => summary
            .off
            .push(format!("person: model {} not found", m.display())),
        None => summary
            .off
            .push("person: no detector model configured (ADOS_WE_PERSON_MODEL)".into()),
    }
    let lidar = dataset.join("lidar_points.laz");
    let depths = dataset.join("depths");
    if lidar.is_file() && depths.is_dir() {
        args.extend([
            "--depths".into(),
            depths.into(),
            "--lidar".into(),
            lidar.into(),
        ]);
        summary.mirror_glass = true;
    } else {
        summary
            .off
            .push("mirror_glass: needs LiDAR and depth".into());
    }
    run_script(runner, "masks.py", args, dataset, progress)?;
    Ok(summary)
}

/// Hero stills rendered from the splat at the given camera poses.
pub fn stills(
    runner: &dyn ToolRunner,
    splat: &Path,
    poses_json: &Path,
    out: &Path,
    progress: &mut dyn FnMut(f32),
) -> Result<(), StageError> {
    fs::create_dir_all(out)?;
    run_script(
        runner,
        "stills.py",
        vec![
            "--splat".into(),
            splat.into(),
            "--poses".into(),
            poses_json.into(),
            "--out".into(),
            out.into(),
        ],
        out,
        progress,
    )
    .map(|_| ())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splat_cap_scales_with_vram() {
        assert_eq!(max_splats_for(1_000_000, Some(8192)), 1_000_000);
        assert_eq!(max_splats_for(1_000_000, Some(16384)), 2_000_000);
        assert_eq!(max_splats_for(1_000_000, None), 1_000_000);
        assert_eq!(max_splats_for(1_000_000, Some(512)), 100_000);
    }

    #[test]
    fn ply_header_vertex_counts_parse() {
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("s.ply");
        let mut bytes = b"ply\nformat binary_little_endian 1.0\nelement vertex 7244\nproperty float x\nend_header\n".to_vec();
        bytes.extend_from_slice(&[0u8; 16]);
        fs::write(&p, bytes).unwrap();
        assert_eq!(ply_vertex_count(&p), Some(7244));
        fs::write(&p, "ply\nend_header\n").unwrap();
        assert_eq!(ply_vertex_count(&p), None);
        assert_eq!(
            parse_step("[INFO brush_cli] Refine iter 201, 1621 splats."),
            Some(201)
        );
    }

    #[test]
    fn trainer_step_counters_parse() {
        assert_eq!(parse_step("Step 1200/7000 loss 0.1"), Some(1200));
        assert_eq!(parse_step("iter=55"), Some(55));
        assert_eq!(parse_step("loading"), None);
    }
}
