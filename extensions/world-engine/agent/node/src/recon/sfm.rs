//! Stage `sfm`: COLMAP over the prepared dataset.
//!
//! A session with a recorded pose track triangulates against those poses
//! (fixed for a LiDAR-odometry track, refined by bundle adjustment for a
//! visual-inertial or flight-controller track), so the model lands in the
//! session's ENU world frame. A session without poses runs incremental (or,
//! for large sites, hierarchical) mapping and stays in COLMAP's own frame;
//! a calibrated multi-camera rig is configured as a COLMAP rig first, so the
//! frames of one instant are matched to each other and mapped as one rigid
//! frame with the session's camera mounts.

use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

use serde::{Deserialize, Serialize};

use super::colmap::{self, camera_params, params_arg};
use super::geom;
use super::prepare::{read_frames, FrameRecord};
use super::{StageError, ToolInvocation, ToolRunner};

/// What `sfm` produced (its stage detail).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SfmSummary {
    /// `known_poses` or `mapper` / `hierarchical_mapper`.
    pub method: String,
    /// The model is in the session's ENU frame (pose priors were used).
    pub world_frame: bool,
    pub registered_images: usize,
    pub points: usize,
    /// Cameras tied together as one COLMAP rig (0 when each camera mapped on its own).
    #[serde(default)]
    pub rig_cameras: usize,
}

/// A prior-posed session needs at least this share of posed keyframes.
const MIN_POSED_SHARE: f64 = 0.5;

/// Run the stage. `dataset` holds `images/` + `frames.json`; the model lands
/// in `dataset/sparse/0`, the database and the text export in `sfm_dir`.
pub fn run(
    runner: &dyn ToolRunner,
    dataset: &Path,
    sfm_dir: &Path,
    fixed_poses: bool,
    refine: bool,
    hierarchical: bool,
    progress: &mut dyn FnMut(f32),
) -> Result<SfmSummary, StageError> {
    if sfm_dir.exists() {
        fs::remove_dir_all(sfm_dir)?;
    }
    let sparse = dataset.join("sparse");
    if sparse.exists() {
        fs::remove_dir_all(&sparse)?;
    }
    fs::create_dir_all(sfm_dir)?;
    let frames = read_frames(dataset)?;
    let db = sfm_dir.join("database.db");
    let images = dataset.join("images");
    let masks = dataset.join("masks");

    // One extraction per camera so each keeps its calibrated intrinsics.
    let mut by_cam: BTreeMap<&str, Vec<&FrameRecord>> = BTreeMap::new();
    for f in &frames {
        by_cam.entry(&f.camera_id).or_default().push(f);
    }
    let n_cams = by_cam.len() as f32;
    for (i, (cam, list)) in by_cam.iter().enumerate() {
        let list_path = sfm_dir.join(format!("images_{cam}.txt"));
        let names: Vec<&str> = list.iter().map(|f| f.name.as_str()).collect();
        fs::write(&list_path, names.join("\n"))?;
        let (model, params) = camera_params(list[0]);
        let mut inv = ToolInvocation::new("colmap", sfm_dir)
            .arg("feature_extractor")
            .arg("--database_path")
            .arg(&db)
            .arg("--image_path")
            .arg(&images)
            .arg("--image_list_path")
            .arg(&list_path)
            .args(["--ImageReader.camera_model", model])
            .args(["--ImageReader.single_camera", "1"])
            .arg("--ImageReader.camera_params")
            .arg(params_arg(&params));
        if masks.is_dir() {
            inv = inv.arg("--ImageReader.mask_path").arg(&masks);
        }
        runner.run(&inv, &mut |_| {})?;
        progress(0.25 * (i as f32 + 1.0) / n_cams);
    }

    let posed = frames.iter().filter(|f| f.t_world_cam.is_some()).count();
    let use_priors = posed >= 3 && posed as f64 >= MIN_POSED_SHARE * frames.len() as f64;
    // The rig goes into the database before matching: the sequential matcher
    // then also pairs the images of neighbouring frames across cameras.
    let rig = match use_priors {
        true => None,
        false => rig_config(&by_cam),
    };
    if let Some((config, _)) = &rig {
        let path = sfm_dir.join("rig_config.json");
        fs::write(&path, serde_json::to_vec_pretty(config)?)?;
        runner.run(
            &ToolInvocation::new("colmap", sfm_dir)
                .arg("rig_configurator")
                .arg("--database_path")
                .arg(&db)
                .arg("--rig_config_path")
                .arg(&path),
            &mut |_| {},
        )?;
    }

    let matcher = if frames.len() <= 150 {
        ToolInvocation::new("colmap", sfm_dir).arg("exhaustive_matcher")
    } else {
        ToolInvocation::new("colmap", sfm_dir)
            .arg("sequential_matcher")
            .args(["--SequentialMatching.overlap", "15"])
            .args(["--SequentialMatching.quadratic_overlap", "1"])
    };
    runner.run(&matcher.arg("--database_path").arg(&db), &mut |_| {})?;
    progress(0.5);

    let model0 = sparse.join("0");
    fs::create_dir_all(&model0)?;
    // Calibrated intrinsics are kept: refining focal length against nearly
    // fixed poses can trade it for scene scale along one axis.
    let calibrated = !frames.is_empty() && frames.iter().all(|f| f.calibrated);
    let method = if use_priors {
        let ids = colmap::read_db_ids(&db)?;
        let known = sfm_dir.join("known");
        if colmap::write_known_model(&known, &frames, &ids)? < 3 {
            return Err(StageError::input(
                "fewer than three posed keyframes reached the COLMAP database",
            ));
        }
        runner.run(
            &ToolInvocation::new("colmap", sfm_dir)
                .arg("point_triangulator")
                .arg("--database_path")
                .arg(&db)
                .arg("--image_path")
                .arg(&images)
                .arg("--input_path")
                .arg(&known)
                .arg("--output_path")
                .arg(&model0),
            &mut |_| {},
        )?;
        progress(0.75);
        if refine && !fixed_poses {
            let mut ba = ToolInvocation::new("colmap", sfm_dir)
                .arg("bundle_adjuster")
                .arg("--input_path")
                .arg(&model0)
                .arg("--output_path")
                .arg(&model0)
                .args(["--BundleAdjustment.refine_principal_point", "0"]);
            if calibrated {
                ba = ba.args([
                    "--BundleAdjustment.refine_focal_length",
                    "0",
                    "--BundleAdjustment.refine_extra_params",
                    "0",
                ]);
            }
            runner.run(&ba, &mut |_| {})?;
        }
        "known_poses"
    } else {
        let mapped = sfm_dir.join("mapped");
        fs::create_dir_all(&mapped)?;
        let mapper = if hierarchical {
            "hierarchical_mapper"
        } else {
            "mapper"
        };
        let mut map = ToolInvocation::new("colmap", sfm_dir)
            .arg(mapper)
            .arg("--database_path")
            .arg(&db)
            .arg("--image_path")
            .arg(&images)
            .arg("--output_path")
            .arg(&mapped);
        // Both mappers take the same `--Mapper.*` options (COLMAP 4.2.1 help).
        if calibrated {
            map = map.args([
                "--Mapper.ba_refine_focal_length",
                "0",
                "--Mapper.ba_refine_extra_params",
                "0",
            ]);
        }
        if rig.is_some() {
            // The mounts are calibrated: keep them, so the model takes their
            // metric baseline instead of trading it for drift.
            map = map.args(["--Mapper.ba_refine_sensor_from_rig", "0"]);
        }
        runner.run(&map, &mut |_| {})?;
        let best = largest_model(&mapped)?;
        for entry in fs::read_dir(&best)? {
            let e = entry?;
            fs::copy(e.path(), model0.join(e.file_name()))?;
        }
        mapper
    };
    progress(0.85);

    let txt = sfm_dir.join("txt");
    fs::create_dir_all(&txt)?;
    runner.run(
        &ToolInvocation::new("colmap", sfm_dir)
            .arg("model_converter")
            .arg("--input_path")
            .arg(&model0)
            .arg("--output_path")
            .arg(&txt)
            .args(["--output_type", "TXT"]),
        &mut |_| {},
    )?;
    let registered = colmap::read_image_poses(&txt)?.len();
    let points = colmap::read_points(&txt)?.len();
    if registered == 0 {
        return Err(StageError::tool_failed("colmap", "no image registered"));
    }
    progress(1.0);
    Ok(SfmSummary {
        method: method.into(),
        world_frame: use_priors,
        registered_images: registered,
        points,
        rig_cameras: rig.map_or(0, |(_, n)| n),
    })
}

/// The COLMAP `rig_configurator` config (and its camera count) for one rig made
/// of every camera: the first camera is the reference sensor, each other camera
/// carries `cam_from_rig = T_body_cam⁻¹ · T_body_ref` from the session mounts.
/// Frames of one instant share a file name across camera folders, which is how
/// COLMAP groups them. `None` for a single camera, or when any frame is
/// uncalibrated or lacks its mount (then each camera maps on its own).
fn rig_config(by_cam: &BTreeMap<&str, Vec<&FrameRecord>>) -> Option<(serde_json::Value, usize)> {
    if by_cam.len() < 2 {
        return None;
    }
    let mut mounts = Vec::with_capacity(by_cam.len());
    for (cam, list) in by_cam {
        if !list.iter().all(|f| f.calibrated) {
            return None;
        }
        mounts.push((*cam, list.first()?.t_body_cam?));
    }
    let (ref_cam, body_ref) = mounts[0];
    let mut cameras = vec![serde_json::json!({
        "image_prefix": format!("{ref_cam}/"),
        "ref_sensor": true,
    })];
    for (cam, body_cam) in &mounts[1..] {
        let cam_from_rig = geom::mul(&geom::inverse_rigid(body_cam), &body_ref);
        cameras.push(serde_json::json!({
            "image_prefix": format!("{cam}/"),
            "cam_from_rig_rotation": geom::quat_of(&cam_from_rig),
            "cam_from_rig_translation": geom::translation(&cam_from_rig),
        }));
    }
    Some((serde_json::json!([{ "cameras": cameras }]), mounts.len()))
}

/// The sub-model with the most images (by `images.bin` size) under a mapper
/// output directory.
fn largest_model(mapped: &Path) -> Result<std::path::PathBuf, StageError> {
    let mut best: Option<(u64, std::path::PathBuf)> = None;
    for entry in fs::read_dir(mapped)? {
        let p = entry?.path();
        let size = fs::metadata(p.join("images.bin"))
            .map(|m| m.len())
            .unwrap_or(0);
        if size > 0 && best.as_ref().is_none_or(|(s, _)| size > *s) {
            best = Some((size, p));
        }
    }
    best.map(|(_, p)| p)
        .ok_or_else(|| StageError::tool_failed("colmap", "the mapper produced no model"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recon::geom;
    use crate::recon::stand_in::StandInRunner;
    use ados_capture_contract::CameraModelModel;

    fn frames(calibrated: bool) -> Vec<FrameRecord> {
        (0..4)
            .map(|i| FrameRecord {
                name: format!("cam0/{i}.png"),
                camera_id: "cam0".into(),
                t_ns: i,
                t_world_cam: Some(geom::from_quat_trans(
                    [1.0, 0.0, 0.0, 0.0],
                    [i as f64 * 0.3, 0.0, 0.0],
                )),
                width: 64,
                height: 48,
                fx: 40.0,
                fy: 40.0,
                cx: 32.0,
                cy: 24.0,
                model: CameraModelModel::Pinhole,
                dist: vec![],
                calibrated,
                t_body_cam: None,
            })
            .collect()
    }

    fn ba_args(calibrated: bool) -> Vec<String> {
        let dir = tempfile::tempdir().unwrap();
        let dataset = dir.path().join("dataset");
        fs::create_dir_all(dataset.join("images/cam0")).unwrap();
        fs::write(
            dataset.join("frames.json"),
            serde_json::to_vec(&frames(calibrated)).unwrap(),
        )
        .unwrap();
        let runner = StandInRunner::default();
        run(
            &runner,
            &dataset,
            &dir.path().join("sfm"),
            false,
            true,
            false,
            &mut |_| {},
        )
        .unwrap();
        runner
            .runs_of("colmap")
            .into_iter()
            .find(|i| i.args_lossy().first().map(String::as_str) == Some("bundle_adjuster"))
            .expect("bundle adjustment ran")
            .args_lossy()
    }

    #[test]
    fn calibrated_intrinsics_stay_fixed_in_bundle_adjustment() {
        let fixed = ba_args(true);
        let pos = fixed
            .iter()
            .position(|a| a == "--BundleAdjustment.refine_focal_length")
            .expect("focal length pinned");
        assert_eq!(fixed[pos + 1], "0");
        assert!(fixed
            .iter()
            .any(|a| a == "--BundleAdjustment.refine_extra_params"));
        let free = ba_args(false);
        assert!(!free
            .iter()
            .any(|a| a == "--BundleAdjustment.refine_focal_length"));
    }

    #[test]
    fn calibrated_intrinsics_stay_fixed_in_the_hierarchical_mapper() {
        let dir = tempfile::tempdir().unwrap();
        let dataset = dir.path().join("dataset");
        fs::create_dir_all(dataset.join("images/cam0")).unwrap();
        let unposed: Vec<FrameRecord> = frames(true)
            .into_iter()
            .map(|f| FrameRecord {
                t_world_cam: None,
                ..f
            })
            .collect();
        fs::write(
            dataset.join("frames.json"),
            serde_json::to_vec(&unposed).unwrap(),
        )
        .unwrap();
        let runner = StandInRunner::default();
        run(
            &runner,
            &dataset,
            &dir.path().join("sfm"),
            false,
            true,
            true,
            &mut |_| {},
        )
        .unwrap();
        let args = runner
            .runs_of("colmap")
            .into_iter()
            .find(|i| i.args_lossy().first().map(String::as_str) == Some("hierarchical_mapper"))
            .expect("hierarchical mapper ran")
            .args_lossy();
        let pos = args
            .iter()
            .position(|a| a == "--Mapper.ba_refine_focal_length")
            .expect("focal length pinned");
        assert_eq!(args[pos + 1], "0");
    }

    /// The stereo mounts of the sim Visor rig (body FLU, OpenCV cameras 10 cm apart).
    fn stereo_mount(y: f64) -> geom::Mat4 {
        [
            0.0, 0.0, 1.0, 0.1, -1.0, 0.0, 0.0, y, 0.0, -1.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0,
        ]
    }

    /// Six unposed stereo frames; `cam_fr` at `fr_calibrated`.
    fn stereo_frames(fr_calibrated: bool) -> Vec<FrameRecord> {
        let mut out = Vec::new();
        for (cam, y, calibrated) in [("cam_fl", 0.05, true), ("cam_fr", -0.05, fr_calibrated)] {
            for t in [300_000_000u64, 9_900_000_000, 10_200_000_000] {
                out.push(FrameRecord {
                    name: crate::recon::prepare::frame_name(cam, t),
                    camera_id: cam.into(),
                    t_ns: t,
                    t_world_cam: None,
                    t_body_cam: Some(stereo_mount(y)),
                    calibrated,
                    ..frames(true).remove(0)
                });
            }
        }
        out
    }

    fn run_stereo(
        frames: &[FrameRecord],
    ) -> (SfmSummary, Vec<Vec<String>>, Option<serde_json::Value>) {
        let dir = tempfile::tempdir().unwrap();
        let dataset = dir.path().join("dataset");
        fs::create_dir_all(dataset.join("images")).unwrap();
        fs::write(
            dataset.join("frames.json"),
            serde_json::to_vec(frames).unwrap(),
        )
        .unwrap();
        let runner = StandInRunner::default();
        let sfm = dir.path().join("sfm");
        let s = run(&runner, &dataset, &sfm, false, true, false, &mut |_| {}).unwrap();
        let config = fs::read(sfm.join("rig_config.json"))
            .ok()
            .map(|b| serde_json::from_slice(&b).unwrap());
        let runs = runner
            .runs_of("colmap")
            .into_iter()
            .map(|i| i.args_lossy())
            .collect();
        (s, runs, config)
    }

    #[test]
    fn frame_names_sort_in_time_order() {
        let a = crate::recon::prepare::frame_name("cam_fl", 10_152_000_000);
        let b = crate::recon::prepare::frame_name("cam_fl", 100_152_000_000);
        assert!(a < b, "{a} vs {b}");
        assert_eq!(a, "cam_fl/00000000010152000000.png");
    }

    #[test]
    fn a_calibrated_stereo_rig_is_configured_before_matching_and_kept_in_mapping() {
        let (s, runs, config) = run_stereo(&stereo_frames(true));
        assert_eq!(s.rig_cameras, 2);
        let cmd = |name: &str| runs.iter().position(|a| a[0] == name).unwrap_or(usize::MAX);
        assert!(cmd("rig_configurator") < cmd("exhaustive_matcher"));
        let config = config.expect("rig config written");
        let cams = config[0]["cameras"].as_array().unwrap();
        assert_eq!(cams[0]["image_prefix"], "cam_fl/");
        assert_eq!(cams[0]["ref_sensor"], true);
        assert_eq!(cams[1]["image_prefix"], "cam_fr/");
        // cam_fr sits 10 cm to the right of cam_fl (+x of the OpenCV camera).
        let t: Vec<f64> =
            serde_json::from_value(cams[1]["cam_from_rig_translation"].clone()).unwrap();
        let q: Vec<f64> = serde_json::from_value(cams[1]["cam_from_rig_rotation"].clone()).unwrap();
        assert!(
            (t[0] + 0.1).abs() < 1e-12 && t[1].abs() < 1e-12 && t[2].abs() < 1e-12,
            "{t:?}"
        );
        assert!((q[0] - 1.0).abs() < 1e-12, "{q:?}");
        let mapper = &runs[cmd("mapper")];
        let pos = mapper
            .iter()
            .position(|a| a == "--Mapper.ba_refine_sensor_from_rig")
            .expect("mounts kept");
        assert_eq!(mapper[pos + 1], "0");
    }

    #[test]
    fn no_rig_for_an_uncalibrated_camera_or_a_posed_session() {
        let (s, runs, config) = run_stereo(&stereo_frames(false));
        assert_eq!(s.rig_cameras, 0);
        assert!(config.is_none());
        assert!(!runs.iter().any(|a| a[0] == "rig_configurator"));
        let posed: Vec<FrameRecord> = stereo_frames(true)
            .into_iter()
            .map(|f| FrameRecord {
                t_world_cam: Some(geom::IDENTITY),
                ..f
            })
            .collect();
        let (s, runs, _) = run_stereo(&posed);
        assert_eq!((s.rig_cameras, s.method.as_str()), (0, "known_poses"));
        assert!(!runs.iter().any(|a| a[0] == "rig_configurator"));
    }
}
