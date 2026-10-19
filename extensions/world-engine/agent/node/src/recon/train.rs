//! Stage `train` (gaussian splat) and stage `mesh`, plus the python helper
//! steps (masks, depth priors, hero stills) that run in the pinned
//! `gsplat_train` project.

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
    pub psnr: Option<f64>,
    /// 3DGUT (unscented transform) was on.
    pub ut: bool,
    /// Why the high-fidelity trainer did not run (Brush stood in).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub fallback_reason: Option<String>,
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
    runner.run(&inv, &mut |line| {
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
    })?;
    if let Some(msg) = failure {
        return Err(StageError::tool_failed("gsplat_train", msg));
    }
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
    let mut gaussians = 0u64;
    runner.run(&inv, &mut |line| {
        let n = crate::parse_gaussian_count(line);
        if n > 0 {
            gaussians = n;
        }
        if let Some(step) = parse_step(line) {
            progress((step as f32 / steps as f32).min(1.0));
        }
    })?;
    if !train_dir.join("splat.ply").is_file() {
        return Err(StageError::tool_failed("brush", "no splat.ply exported"));
    }
    Ok(TrainSummary {
        trainer: "brush".into(),
        steps,
        max_splats,
        gaussians: (gaussians > 0).then_some(gaussians),
        psnr: None,
        ut: false,
        fallback_reason: reason.map(str::to_string),
    })
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
/// bilateral grid for exposure variation).
#[allow(clippy::too_many_arguments)]
pub fn gsplat(
    runner: &dyn ToolRunner,
    dataset: &Path,
    train_dir: &Path,
    steps: u64,
    max_splats: u64,
    ut: bool,
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
        "--bilateral-grid".into(),
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
    Ok(TrainSummary {
        trainer: "gsplat".into(),
        steps,
        max_splats,
        gaussians: done.get("gaussians").and_then(|v| v.as_u64()),
        psnr: done.get("psnr").and_then(|v| v.as_f64()),
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

/// Dynamic-object and mirror/glass masks (`dataset/masks/`).
pub fn masks(
    runner: &dyn ToolRunner,
    dataset: &Path,
    person_model: Option<&Path>,
    progress: &mut dyn FnMut(f32),
) -> Result<(), StageError> {
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
    if let Some(m) = person_model {
        args.extend(["--person-model".into(), m.into()]);
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
    }
    run_script(runner, "masks.py", args, dataset, progress).map(|_| ())
}

/// Stage `mesh`: depth renders of the splat fused by TSDF, decimated.
pub fn mesh(
    runner: &dyn ToolRunner,
    dataset: &Path,
    splat: &Path,
    mesh_dir: &Path,
    progress: &mut dyn FnMut(f32),
) -> Result<serde_json::Value, StageError> {
    fs::create_dir_all(mesh_dir)?;
    let done = run_script(
        runner,
        "mesh.py",
        vec![
            "--colmap".into(),
            dataset.into(),
            "--images".into(),
            dataset.join("images").into(),
            "--splat".into(),
            splat.into(),
            "--out".into(),
            mesh_dir.into(),
            "--voxel".into(),
            "0.02".into(),
            "--max-tris".into(),
            "300000".into(),
        ],
        mesh_dir,
        progress,
    )?;
    for f in ["mesh.ply", "mesh.glb"] {
        if !mesh_dir.join(f).is_file() {
            return Err(StageError::tool_failed(
                "gsplat_train",
                format!("no {f} written"),
            ));
        }
    }
    Ok(serde_json::Value::Object(done))
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
    fn trainer_step_counters_parse() {
        assert_eq!(parse_step("Step 1200/7000 loss 0.1"), Some(1200));
        assert_eq!(parse_step("iter=55"), Some(55));
        assert_eq!(parse_step("loading"), None);
    }
}
