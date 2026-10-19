//! Stage `sfm`: COLMAP over the prepared dataset.
//!
//! A session with a recorded pose track triangulates against those poses
//! (fixed for a LiDAR-odometry track, refined by bundle adjustment for a
//! visual-inertial or flight-controller track), so the model lands in the
//! session's ENU world frame. A session without poses runs incremental (or,
//! for large sites, hierarchical) mapping and stays in COLMAP's own frame.

use std::collections::BTreeMap;
use std::fs;
use std::path::Path;

use serde::{Deserialize, Serialize};

use super::colmap::{self, camera_params, params_arg};
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
    let posed = frames.iter().filter(|f| f.t_world_cam.is_some()).count();
    let use_priors = posed >= 3 && posed as f64 >= MIN_POSED_SHARE * frames.len() as f64;
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
            runner.run(
                &ToolInvocation::new("colmap", sfm_dir)
                    .arg("bundle_adjuster")
                    .arg("--input_path")
                    .arg(&model0)
                    .arg("--output_path")
                    .arg(&model0)
                    .args(["--BundleAdjustment.refine_principal_point", "0"]),
                &mut |_| {},
            )?;
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
        runner.run(
            &ToolInvocation::new("colmap", sfm_dir)
                .arg(mapper)
                .arg("--database_path")
                .arg(&db)
                .arg("--image_path")
                .arg(&images)
                .arg("--output_path")
                .arg(&mapped),
            &mut |_| {},
        )?;
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
    })
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
