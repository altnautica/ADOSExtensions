//! Stage `sfm`: COLMAP over the prepared dataset.
//!
//! A session with a recorded pose track matches only the image pairs its
//! poses make plausible (neighbours in time, nearby views looking the same
//! way, the cameras of one rig instant), drops every verified pair whose
//! relative pose contradicts the priors, and maps with the priors as
//! weighted position constraints (`pose_prior_mapper`), so the model lands in
//! the session's ENU world frame. A LiDAR-odometry track is trusted as is:
//! its poses stay fixed and only the points are triangulated. A session
//! without poses runs incremental (or, for large sites, hierarchical)
//! mapping and stays in COLMAP's own frame. A calibrated multi-camera rig is
//! configured as a COLMAP rig first, so the frames of one instant are
//! matched to each other and mapped as one rigid frame with the session's
//! camera mounts. Short and depth-inconsistent tracks are dropped before
//! training (see [`super::track_filter`]).

use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

use serde::{Deserialize, Serialize};

use super::colmap::{self, camera_params, params_arg};
use super::geom;
use super::pairs::{self, PairCfg};
use super::prepare::{read_frames, FrameRecord};
use super::track_filter;
use super::{StageError, ToolInvocation, ToolRunner};

/// What `sfm` produced (its stage detail).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct SfmSummary {
    /// `known_poses` (fixed prior poses), `pose_prior_mapper`, or `mapper` /
    /// `hierarchical_mapper` (no prior).
    pub method: String,
    /// The model is in the session's ENU frame (pose priors were used).
    pub world_frame: bool,
    pub registered_images: usize,
    pub points: usize,
    /// Rig instants mapped as one rigid frame each (0 when each camera mapped on its own).
    #[serde(default)]
    pub rig_frames_paired: usize,
    /// Verified image pairs dropped because their relative pose contradicted the pose prior.
    #[serde(default)]
    pub pairs_rejected_by_prior: usize,
    /// Sparse points dropped for a track shorter than three images.
    #[serde(default)]
    pub points_dropped_short_track: usize,
    /// Sparse points dropped for disagreeing with the LiDAR or depth-prior depth.
    #[serde(default)]
    pub points_dropped_depth: usize,
}

/// How one `sfm` run treats the session's poses.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct SfmOptions<'a> {
    /// Keep the prior poses fixed and only triangulate (a trusted LiDAR trajectory).
    pub fixed_poses: bool,
    /// Refine poses against the images; off, prior poses are only triangulated against.
    pub refine: bool,
    /// Hierarchical mapping for a large unposed site.
    pub hierarchical: bool,
    /// The source of the frames' `T_world_cam` (`vio`, `fc_ekf`, `lidar`, `lidar2d`,
    /// `truth`), which sets the prior's default uncertainty.
    pub prior_source: Option<&'a str>,
}

/// A prior-posed session needs at least this share of posed keyframes.
const MIN_POSED_SHARE: f64 = 0.5;
/// Fewest verified inliers a pair needs, and fewest matches the mapper uses a pair with.
const MIN_INLIERS: &str = "30";
/// Fewest 2D-3D inliers to register an image.
const ABS_POSE_MIN_INLIERS: &str = "50";
/// A verified pair is dropped when its relative rotation is this far from the
/// priors' (degrees), or, over a prior baseline of at least
/// [`PRIOR_MIN_BASELINE_M`], its translation direction this far (degrees).
const PRIOR_MAX_ROT_DEG: f64 = 10.0;
const PRIOR_MAX_DIR_DEG: f64 = 20.0;
const PRIOR_MIN_BASELINE_M: f64 = 0.3;

/// One-sigma prior position uncertainty (x, y, z metres) of a pose source
/// when its track recorded no covariance.
pub fn default_prior_sigma_m(source: Option<&str>) -> [f64; 3] {
    match source {
        // A planar LiDAR fixes x and y; height comes from the camera rig alone.
        Some("lidar2d") => [0.03, 0.03, 0.25],
        Some("lidar") => [0.03; 3],
        Some("vio") => [0.05; 3],
        Some("truth") => [0.01; 3],
        // `fc_ekf`, and any source without a known accuracy.
        _ => [0.5; 3],
    }
}

/// Run the stage. `dataset` holds `images/` + `frames.json`; the model lands
/// in `dataset/sparse/0`, the database and the text export in `sfm_dir`.
pub fn run(
    runner: &dyn ToolRunner,
    dataset: &Path,
    sfm_dir: &Path,
    opts: &SfmOptions<'_>,
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
    let triangulate_only = use_priors && (opts.fixed_poses || !opts.refine);
    // The rig goes into the database before matching: the matchers then also
    // pair the images of neighbouring frames across cameras. Triangulating
    // against fixed poses gains nothing from it, and COLMAP 4.2.1's
    // `point_triangulator` aborts in its final bundle adjustment when the
    // database holds a rig its rig-less input model does not.
    let rig = match triangulate_only {
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

    let window = pairs::window(&frames);
    let matcher = if use_priors {
        // Pairs from the poses; the matchers' own candidate search (quadratic
        // overlap, vocabulary-tree loops) is what pairs look-alike bays.
        let list = sfm_dir.join("pairs.txt");
        pairs::write(&list, &pairs::from_poses(&frames, &PairCfg::new(window)))?;
        ToolInvocation::new("colmap", sfm_dir)
            .arg("matches_importer")
            .arg("--match_list_path")
            .arg(&list)
            .args(["--match_type", "pairs"])
    } else if frames.len() <= 150 {
        ToolInvocation::new("colmap", sfm_dir).arg("exhaustive_matcher")
    } else {
        ToolInvocation::new("colmap", sfm_dir)
            .arg("sequential_matcher")
            .arg("--SequentialMatching.overlap")
            .arg(window.to_string())
            .args(["--SequentialMatching.quadratic_overlap", "0"])
            .args(["--SequentialMatching.loop_detection", "0"])
    };
    let mut matcher = matcher
        .arg("--database_path")
        .arg(&db)
        .args(["--TwoViewGeometry.compute_relative_pose", "1"])
        .args(["--TwoViewGeometry.min_num_inliers", MIN_INLIERS]);
    if rig.is_some() {
        matcher = matcher.args(["--FeatureMatching.rig_verification", "1"]);
    }
    runner.run(&matcher, &mut |_| {})?;
    let pairs_rejected_by_prior = match use_priors {
        true => colmap::invalidate_inconsistent_pairs(
            &db,
            &frames,
            PRIOR_MAX_ROT_DEG,
            PRIOR_MAX_DIR_DEG,
            PRIOR_MIN_BASELINE_M,
        )?,
        false => 0,
    };
    if use_priors {
        tracing::info!(
            pairs = pairs_rejected_by_prior,
            "sfm_pairs_rejected_by_prior"
        );
    }
    progress(0.5);

    let model0 = sparse.join("0");
    fs::create_dir_all(&model0)?;
    // Calibrated intrinsics are kept: refining focal length against nearly
    // fixed poses can trade it for scene scale along one axis.
    let calibrated = !frames.is_empty() && frames.iter().all(|f| f.calibrated);
    let pins = |mut inv: ToolInvocation| {
        inv = inv.args([
            "--Mapper.min_num_matches",
            MIN_INLIERS,
            "--Mapper.abs_pose_min_num_inliers",
            ABS_POSE_MIN_INLIERS,
        ]);
        // Every mapper takes the same `--Mapper.*` options (COLMAP 4.2.1 help).
        if calibrated {
            inv = inv.args([
                "--Mapper.ba_refine_focal_length",
                "0",
                "--Mapper.ba_refine_extra_params",
                "0",
            ]);
        }
        if rig.is_some() {
            // The mounts are calibrated: keep them, so the model takes their
            // metric baseline instead of trading it for drift.
            inv = inv.args(["--Mapper.ba_refine_sensor_from_rig", "0"]);
        }
        inv
    };
    let method = if triangulate_only {
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
                .arg(&model0)
                .args(["--Mapper.min_num_matches", MIN_INLIERS])
                .args(["--Mapper.abs_pose_min_num_inliers", ABS_POSE_MIN_INLIERS]),
            &mut |_| {},
        )?;
        "known_poses"
    } else {
        let mapper = match (use_priors, opts.hierarchical) {
            (true, _) => {
                let default = default_prior_sigma_m(opts.prior_source);
                let sigma = |f: &FrameRecord| f.prior_sigma_m.unwrap_or(default);
                if colmap::write_pose_priors(&db, &frames, &sigma)? < 3 {
                    return Err(StageError::input(
                        "fewer than three posed keyframes reached the COLMAP database",
                    ));
                }
                "pose_prior_mapper"
            }
            (false, true) => "hierarchical_mapper",
            (false, false) => "mapper",
        };
        let mapped = sfm_dir.join("mapped");
        fs::create_dir_all(&mapped)?;
        let mut map = pins(
            ToolInvocation::new("colmap", sfm_dir)
                .arg(mapper)
                .arg("--database_path")
                .arg(&db)
                .arg("--image_path")
                .arg(&images)
                .arg("--output_path")
                .arg(&mapped),
        );
        if use_priors {
            // The per-image covariances written above, with a robust loss so a
            // stretch of bad prior cannot drag the model.
            map = map.args([
                "--overwrite_priors_covariance",
                "0",
                "--use_robust_loss_on_prior_position",
                "1",
            ]);
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
    let convert = |from: &Path, to: &Path, kind: &str| {
        runner.run(
            &ToolInvocation::new("colmap", sfm_dir)
                .arg("model_converter")
                .arg("--input_path")
                .arg(from)
                .arg("--output_path")
                .arg(to)
                .args(["--output_type", kind]),
            &mut |_| {},
        )
    };
    convert(&model0, &txt, "TXT")?;
    let dropped = track_filter::run(&txt, dataset, use_priors)?;
    if dropped != track_filter::Dropped::default() {
        tracing::info!(
            short_track = dropped.short_track,
            depth = dropped.depth,
            "sfm_points_filtered"
        );
        convert(&txt, &model0, "BIN")?;
    }
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
        rig_frames_paired: rig.map_or(0, |(_, n)| n),
        pairs_rejected_by_prior,
        points_dropped_short_track: dropped.short_track,
        points_dropped_depth: dropped.depth,
    })
}

/// The COLMAP `rig_configurator` config (and its rig frame count) for one rig
/// made of every camera: the first camera is the reference sensor, each other
/// camera carries `cam_from_rig = T_body_cam⁻¹ · T_body_ref` from the session
/// mounts. `prepare` exported every rig instant with each camera's frame under
/// one file name, which is how COLMAP groups them. `None` for a single camera,
/// when `prepare` exported the frames outside a rig or an instant lacks a
/// camera, or when any frame is uncalibrated or lacks its mount (then each
/// camera maps on its own).
fn rig_config(by_cam: &BTreeMap<&str, Vec<&FrameRecord>>) -> Option<(serde_json::Value, usize)> {
    if by_cam.len() < 2 {
        return None;
    }
    let mut instants = BTreeMap::<u64, usize>::new();
    let mut mounts = Vec::with_capacity(by_cam.len());
    for (cam, list) in by_cam {
        if !list.iter().all(|f| f.calibrated) {
            return None;
        }
        for f in list {
            *instants.entry(f.rig_instant?).or_default() += 1;
        }
        mounts.push((*cam, list.first()?.t_body_cam?));
    }
    if instants.values().any(|n| *n != by_cam.len()) {
        return None;
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
    Some((serde_json::json!([{ "cameras": cameras }]), instants.len()))
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
#[path = "sfm_tests.rs"]
mod tests;
