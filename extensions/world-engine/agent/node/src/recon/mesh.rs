//! Stage `mesh`: a triangle mesh of the trained scene (`mesh.py` in the
//! pinned `gsplat_train` project), refused when training diverged.
//!
//! The surface comes from, in order: a second 2D-gaussian training pass when
//! the job asks for it (`mesh_method = "2dgs"`); Poisson reconstruction of the
//! registered cloud of a 3D (solid-state or spinning) LiDAR; otherwise TSDF
//! fusion of depth rendered from the splat.

use std::fs;
use std::path::Path;

use serde::Serialize;
use serde_json::Value;

use super::train::run_script;
use super::{StageError, ToolRunner};

/// Below this training PSNR the splat has diverged and no mesh is built.
pub const PSNR_FLOOR_DB: f64 = 18.0;

/// How the mesh is built (job param `mesh_method`, `tsdf` when absent).
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
pub enum MeshMethod {
    /// TSDF over the splat's depth (or Poisson over a 3D LiDAR's cloud).
    #[default]
    #[serde(rename = "tsdf")]
    Tsdf,
    /// A second training pass with 2D gaussians, its median depth fused by
    /// the same TSDF.
    #[serde(rename = "2dgs")]
    TwoDgs,
}

impl MeshMethod {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "tsdf" => Some(Self::Tsdf),
            "2dgs" => Some(Self::TwoDgs),
            _ => None,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Self::Tsdf => "tsdf",
            Self::TwoDgs => "2dgs",
        }
    }
}

/// `E_TRAIN_DIVERGED` when the train stage's `psnr` is below
/// [`PSNR_FLOOR_DB`]: a diverged splat yields no products. A trainer that
/// reports no PSNR (Brush) passes.
pub fn check_psnr_floor(train_detail: &Value) -> Result<(), StageError> {
    match train_detail["psnr"].as_f64() {
        Some(psnr) if psnr < PSNR_FLOOR_DB => Err(StageError::new(
            "E_TRAIN_DIVERGED",
            format!("training PSNR {psnr} dB is below the 18 dB mesh floor"),
        )),
        _ => Ok(()),
    }
}

/// Whether a session rig (`rig.json` as JSON) carries a 3D LiDAR: a `lidar`
/// sensor whose `lidar` model block has pattern `solid` or `spinning`.
pub fn rig_has_3d_lidar(rig: &Value) -> bool {
    rig["sensors"].as_array().is_some_and(|sensors| {
        sensors.iter().any(|s| {
            s["kind"] == "lidar"
                && matches!(s["lidar"]["pattern"].as_str(), Some("solid" | "spinning"))
        })
    })
}

/// Stage `mesh` of a reconstruction in `job_dir` (its `dataset/` and
/// `train/splat.ply`, writing `mesh/`): refused below the PSNR floor of
/// `train_detail`; a 3D LiDAR's registered cloud is the surface source unless
/// the job asked for the 2DGS pass.
pub fn stage(
    runner: &dyn ToolRunner,
    job_dir: &Path,
    session_dir: &Path,
    train_detail: &Value,
    method: MeshMethod,
    steps: u64,
    progress: &mut dyn FnMut(f32),
) -> Result<Value, StageError> {
    check_psnr_floor(train_detail)?;
    let dataset = job_dir.join("dataset");
    let cloud = dataset.join("lidar_points.laz");
    let lidar = method == MeshMethod::Tsdf && cloud.is_file() && {
        let rig: Value = serde_json::from_slice(&fs::read(session_dir.join("rig.json"))?)
            .map_err(|e| StageError::input(format!("rig.json: {e}")))?;
        rig_has_3d_lidar(&rig)
    };
    run(
        runner,
        &MeshJob {
            dataset: &dataset,
            splat: &job_dir.join("train").join("splat.ply"),
            mesh_dir: &job_dir.join("mesh"),
            method,
            steps,
            lidar: lidar.then_some(cloud.as_path()),
        },
        progress,
    )
}

/// One mesh run.
pub struct MeshJob<'a> {
    /// COLMAP dataset dir (`images/`, `sparse/0`, optional `masks/`).
    pub dataset: &'a Path,
    pub splat: &'a Path,
    pub mesh_dir: &'a Path,
    pub method: MeshMethod,
    /// Training steps of the 2DGS pass (the splat's own step count).
    pub steps: u64,
    /// The registered 3D LiDAR cloud to mesh instead of the splat's depth.
    pub lidar: Option<&'a Path>,
}

/// Run `mesh.py`; the stage detail is its `done` event without paths.
pub fn run(
    runner: &dyn ToolRunner,
    job: &MeshJob<'_>,
    progress: &mut dyn FnMut(f32),
) -> Result<Value, StageError> {
    fs::create_dir_all(job.mesh_dir)?;
    let mut args: Vec<std::ffi::OsString> = vec![
        "--colmap".into(),
        job.dataset.into(),
        "--images".into(),
        job.dataset.join("images").into(),
        "--splat".into(),
        job.splat.into(),
        "--out".into(),
        job.mesh_dir.into(),
        "--voxel".into(),
        "0.02".into(),
        "--max-tris".into(),
        "300000".into(),
        "--method".into(),
        job.method.as_str().into(),
    ];
    if job.method == MeshMethod::TwoDgs {
        args.extend(["--steps".into(), job.steps.to_string().into()]);
        let masks = job.dataset.join("masks");
        if masks.is_dir() {
            args.extend(["--masks".into(), masks.into()]);
        }
    }
    if let Some(l) = job.lidar {
        args.extend(["--lidar".into(), l.into()]);
    }
    let mut done = run_script(runner, "mesh.py", args, job.mesh_dir, progress)?;
    for f in ["mesh.ply", "mesh.glb"] {
        if !job.mesh_dir.join(f).is_file() {
            return Err(StageError::tool_failed(
                "gsplat_train",
                format!("no {f} written"),
            ));
        }
    }
    // The script may run in WSL and report paths in its own form; the files
    // are at known places in the job, so only the facts are kept.
    done.retain(|k, _| !matches!(k.as_str(), "ply" | "glb" | "mesh" | "path"));
    Ok(Value::Object(done))
}

#[cfg(test)]
mod tests {
    use super::super::ResolvedTool;
    use super::*;
    use serde_json::json;
    use std::sync::Mutex;

    /// Runs `mesh.py` by writing its two outputs and a `done` event,
    /// recording the arguments.
    #[derive(Default)]
    struct FakeMesh {
        calls: Mutex<Vec<Vec<String>>>,
    }

    impl ToolRunner for FakeMesh {
        fn resolve(&self, tool: &str) -> Result<ResolvedTool, StageError> {
            Ok(ResolvedTool {
                name: tool.into(),
                version: "test".into(),
                program: "python".into(),
                prefix_args: Vec::new(),
                home: "gsplat_train".into(),
                wsl: false,
            })
        }

        fn run(
            &self,
            inv: &super::super::ToolInvocation,
            on_line: &mut dyn FnMut(&str),
        ) -> Result<(), StageError> {
            let args = inv.args_lossy();
            let out = Path::new(&args[args.iter().position(|a| a == "--out").unwrap() + 1]);
            fs::write(out.join("mesh.ply"), b"ply")?;
            fs::write(out.join("mesh.glb"), b"glTF")?;
            on_line(r#"{"event":"done","triangles":8,"ply":"/x/mesh.ply"}"#);
            self.calls.lock().unwrap().push(args);
            Ok(())
        }
    }

    /// A job dir with a LiDAR cloud and a session whose LiDAR has `pattern`.
    fn job(root: &Path, pattern: &str) -> (std::path::PathBuf, std::path::PathBuf) {
        let (job, session) = (root.join("job"), root.join("session"));
        fs::create_dir_all(job.join("dataset")).unwrap();
        fs::create_dir_all(&session).unwrap();
        fs::write(job.join("dataset/lidar_points.laz"), b"laz").unwrap();
        let rig =
            json!({"sensors": [{"id": "lidar0", "kind": "lidar", "lidar": {"pattern": pattern}}]});
        fs::write(session.join("rig.json"), rig.to_string()).unwrap();
        (job, session)
    }

    fn flag_value(args: &[String], flag: &str) -> Option<String> {
        args.iter()
            .position(|a| a == flag)
            .map(|i| args[i + 1].clone())
    }

    #[test]
    fn psnr_floor_refuses_a_diverged_splat() {
        let e = check_psnr_floor(&json!({"trainer": "gsplat", "psnr": 12.3})).unwrap_err();
        assert_eq!(e.code, "E_TRAIN_DIVERGED");
        assert_eq!(
            e.message,
            "training PSNR 12.3 dB is below the 18 dB mesh floor"
        );
        check_psnr_floor(&json!({"psnr": 18.0})).unwrap();
        check_psnr_floor(&json!({"psnr": 27.4})).unwrap();
        check_psnr_floor(&json!({"trainer": "brush", "psnr": null})).unwrap();
    }

    #[test]
    fn mesh_stage_with_a_diverged_splat_runs_nothing() {
        let dir = tempfile::tempdir().unwrap();
        let (job, session) = job(dir.path(), "planar");
        let runner = FakeMesh::default();
        let detail = json!({"psnr": 12.3});
        let e = stage(
            &runner,
            &job,
            &session,
            &detail,
            MeshMethod::Tsdf,
            100,
            &mut |_| {},
        )
        .unwrap_err();
        assert_eq!(
            e.message,
            "training PSNR 12.3 dB is below the 18 dB mesh floor"
        );
        assert!(runner.calls.lock().unwrap().is_empty());
        assert!(!job.join("mesh").exists());
    }

    #[test]
    fn mesh_stage_picks_the_surface_source() {
        let detail = json!({"psnr": 25.0});
        let run_with = |pattern: &str, method: MeshMethod| {
            let dir = tempfile::tempdir().unwrap();
            let (job, session) = job(dir.path(), pattern);
            let runner = FakeMesh::default();
            let out = stage(&runner, &job, &session, &detail, method, 7000, &mut |_| {}).unwrap();
            assert_eq!(out, json!({"triangles": 8}));
            let args = runner.calls.lock().unwrap().pop().unwrap();
            let lidar = flag_value(&args, "--lidar");
            (args, lidar, job)
        };
        let (args, lidar, job) = run_with("solid", MeshMethod::Tsdf);
        let cloud = job.join("dataset").join("lidar_points.laz");
        assert_eq!(lidar, Some(cloud.display().to_string()));
        assert_eq!(flag_value(&args, "--method").as_deref(), Some("tsdf"));
        assert!(run_with("spinning", MeshMethod::Tsdf).1.is_some());
        let (args, lidar, _) = run_with("planar", MeshMethod::Tsdf);
        assert_eq!(lidar, None);
        assert_eq!(flag_value(&args, "--steps"), None);
        let (args, lidar, _) = run_with("solid", MeshMethod::TwoDgs);
        assert_eq!(lidar, None);
        assert_eq!(flag_value(&args, "--method").as_deref(), Some("2dgs"));
        assert_eq!(flag_value(&args, "--steps").as_deref(), Some("7000"));
    }

    #[test]
    fn mesh_method_parses_its_two_values_only() {
        assert_eq!(MeshMethod::parse("tsdf"), Some(MeshMethod::Tsdf));
        assert_eq!(MeshMethod::parse("2dgs"), Some(MeshMethod::TwoDgs));
        assert_eq!(MeshMethod::parse("2DGS"), None);
        assert_eq!(MeshMethod::parse("poisson"), None);
        assert_eq!(serde_json::to_value(MeshMethod::TwoDgs).unwrap(), "2dgs");
    }

    #[test]
    fn only_solid_or_spinning_lidar_rigs_are_3d() {
        let rig = |lidar: Value| {
            json!({"sensors": [
                {"id": "cam0", "kind": "camera"},
                {"id": "lidar0", "kind": "lidar", "lidar": lidar}
            ]})
        };
        assert!(rig_has_3d_lidar(&rig(json!({"pattern": "solid"}))));
        assert!(rig_has_3d_lidar(&rig(json!({"pattern": "spinning"}))));
        assert!(!rig_has_3d_lidar(&rig(json!({"pattern": "planar"}))));
        assert!(!rig_has_3d_lidar(&rig(Value::Null)));
        assert!(!rig_has_3d_lidar(&json!({"sensors": [{"kind": "camera"}]})));
    }
}
