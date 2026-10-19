//! Turn received Atlas keyframes into a reconstructor-ingestible dataset.
//!
//! Each keyframe carries the compressed image bytes plus a world-from-camera
//! pose and the camera model (the drone computed the pose on-board from its
//! own odometry). This module streams each keyframe's image
//! to disk as it arrives and accumulates the lightweight per-frame camera
//! record, then on bag-finalize writes the capture contract's `transforms.json`
//! (Nerfstudio layout plus `contract`, `ados_anchor` and a per-frame `t_ns`)
//! next to the images.
//!
//! A gaussian-splat trainer (Brush) ingests a directory that holds a
//! `transforms.json` plus an `images/` subdir directly through its Nerfstudio
//! reader, so a captured session becomes a real splat input with no
//! structure-from-motion pre-pass when the poses are already known. The same
//! directory feeds a COLMAP pre-pass when poses must be refined, and its
//! `ados_anchor` georeferences whatever is reconstructed from it.
//!
//! Memory stays bounded: the image bytes are written to disk immediately and only
//! the small per-frame record (a file path, a 4x4 matrix, and the intrinsics) is
//! held in memory until finalize.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use ados_capture_contract::{
    CameraModel, CameraModelModel, ContractId, GeoAnchor, TransformsFrame, TransformsJson,
    TransformsJsonCameraModel,
};
use world_engine_protocol::atlas::{ImageEncoding, KeyframeEnvelope};

/// Subdirectory under a dataset directory that holds the keyframe images.
const IMAGES_DIR: &str = "images";
/// The dataset manifest filename the trainer reads.
const TRANSFORMS_FILE: &str = "transforms.json";

/// The dataset id a session's keyframes persist under. Matches the id the ingest
/// enqueues its reconstruct job against, so the trainer's `input_path` resolves
/// to this directory.
pub fn dataset_id_for(session_id: &str) -> String {
    format!("ds-{session_id}")
}

/// The georeference a dataset directory was written with: the `ados_anchor` of
/// its `transforms.json`. `None` when the dataset is in a local frame, or the
/// manifest is absent or unreadable (then nothing can claim a georeference).
pub fn dataset_anchor(dataset_dir: &Path) -> Option<GeoAnchor> {
    let bytes = std::fs::read(dataset_dir.join(TRANSFORMS_FILE)).ok()?;
    serde_json::from_slice::<TransformsJson>(&bytes)
        .ok()?
        .ados_anchor
}

/// The on-disk file extension for an image encoding. The capture path encodes
/// keyframes as JPEG; the HEVC keyframe branch is carried for completeness.
fn image_ext(encoding: ImageEncoding) -> &'static str {
    match encoding {
        ImageEncoding::Jpeg => "jpg",
        ImageEncoding::HevcKeyframe => "h265",
    }
}

/// The Nerfstudio camera model a contract camera model is expressed in:
/// pinhole and Brown–Conrady both read as `OPENCV` (a pinhole simply carries
/// no coefficients), the two four-coefficient fisheye models as
/// `OPENCV_FISHEYE`.
fn nerf_camera_model(model: CameraModelModel) -> TransformsJsonCameraModel {
    match model {
        CameraModelModel::Pinhole | CameraModelModel::Opencv => TransformsJsonCameraModel::Opencv,
        CameraModelModel::OpencvFisheye | CameraModelModel::Kb4 => {
            TransformsJsonCameraModel::OpencvFisheye
        }
    }
}

/// Distortion coefficients in Nerfstudio's named fields.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
struct NerfDist {
    k1: Option<f64>,
    k2: Option<f64>,
    k3: Option<f64>,
    k4: Option<f64>,
    p1: Option<f64>,
    p2: Option<f64>,
}

/// The coefficients of `camera` by name, or none at all when `dist` does not
/// have the length its model defines (never guessed: a misread coefficient
/// bends the reconstruction, an absent one only leaves it undistorted).
fn nerf_dist(camera: &CameraModel) -> NerfDist {
    if !camera.dist_len_ok() {
        return NerfDist::default();
    }
    let d = &camera.dist;
    match camera.model {
        CameraModelModel::Pinhole => NerfDist::default(),
        CameraModelModel::Opencv => NerfDist {
            k1: Some(d[0]),
            k2: Some(d[1]),
            p1: Some(d[2]),
            p2: Some(d[3]),
            k3: Some(d[4]),
            k4: None,
        },
        CameraModelModel::OpencvFisheye | CameraModelModel::Kb4 => NerfDist {
            k1: Some(d[0]),
            k2: Some(d[1]),
            k3: Some(d[2]),
            k4: Some(d[3]),
            p1: None,
            p2: None,
        },
    }
}

/// Build the Nerfstudio `transform_matrix` (a 4x4 camera-to-world in the
/// OpenGL/Blender convention the trainer reads) from a world-from-camera pose
/// expressed in the OpenCV camera convention (+X right, +Y down, +Z forward —
/// the contract's camera frame). The conversion negates the camera-frame Y and
/// Z basis vectors (post-multiply by `diag(1, -1, -1, 1)`), the same flip a
/// Nerfstudio dataset writer applies to COLMAP / OpenCV extrinsics; the camera
/// position is unchanged.
fn opencv_c2w_to_opengl_matrix(r: &[f64; 9], t: &[f64; 3]) -> [[f64; 4]; 4] {
    [
        [r[0], -r[1], -r[2], t[0]],
        [r[3], -r[4], -r[5], t[1]],
        [r[6], -r[7], -r[8], t[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

/// The per-frame record for one keyframe (its own intrinsics, pose flipped into
/// the trainer's convention, image path relative to the manifest, exposure time
/// on the session timebase).
fn nerf_frame(kf: &KeyframeEnvelope, file_path: String) -> TransformsFrame {
    let c = &kf.camera;
    let d = nerf_dist(c);
    TransformsFrame {
        file_path,
        transform_matrix: opencv_c2w_to_opengl_matrix(&kf.pose.r, &kf.pose.t),
        t_ns: kf.time.t_exposure_mid_ns,
        camera_id: Some(kf.camera_id.clone()),
        mask_path: None,
        fl_x: Some(c.fx),
        fl_y: Some(c.fy),
        cx: Some(c.cx),
        cy: Some(c.cy),
        w: Some(kf.image.width),
        h: Some(kf.image.height),
        k1: d.k1,
        k2: d.k2,
        k3: d.k3,
        k4: d.k4,
        p1: d.p1,
        p2: d.p2,
    }
}

/// Accumulates a single dataset's frame records until finalize.
#[derive(Debug)]
struct DatasetAccum {
    frames: Vec<TransformsFrame>,
    /// The session's world origin, from the first keyframe.
    anchor: Option<GeoAnchor>,
    /// The Nerfstudio camera model, from the first keyframe.
    camera_model: TransformsJsonCameraModel,
}

impl DatasetAccum {
    /// The manifest built from the accumulated frames. Scene-level intrinsics
    /// are the first frame's (a single-camera dataset is then fully described at
    /// the scene level, and a multi-camera one is corrected per frame).
    fn manifest(&self) -> TransformsJson {
        let f0 = &self.frames[0];
        TransformsJson {
            contract: ContractId::AdosCapture1,
            ados_anchor: self.anchor.clone(),
            camera_model: Some(self.camera_model),
            fl_x: f0.fl_x,
            fl_y: f0.fl_y,
            cx: f0.cx,
            cy: f0.cy,
            w: f0.w,
            h: f0.h,
            k1: f0.k1,
            k2: f0.k2,
            k3: f0.k3,
            k4: f0.k4,
            p1: f0.p1,
            p2: f0.p2,
            frames: self.frames.clone(),
        }
    }
}

/// Write a manifest to `<dir>/transforms.json` atomically: a temp file in the
/// same directory then a rename. A periodic live reconstruct may still be
/// reading a prior snapshot's manifest when the next snapshot (or the final bag)
/// is written; the rename makes the reader see a whole manifest, never a torn one.
fn write_manifest_atomic(dir: &Path, manifest: &TransformsJson) -> std::io::Result<()> {
    std::fs::create_dir_all(dir)?;
    let bytes = serde_json::to_vec_pretty(manifest).map_err(std::io::Error::other)?;
    let tmp = dir.join(format!("{TRANSFORMS_FILE}.tmp"));
    std::fs::write(&tmp, bytes)?;
    std::fs::rename(&tmp, dir.join(TRANSFORMS_FILE))
}

/// Streams keyframe images to disk and writes the dataset manifest per dataset.
/// One persister serves the whole receive loop; it keys by dataset id so
/// interleaved sessions never cross-contaminate.
#[derive(Debug, Default)]
pub struct KeyframePersister {
    work_root: PathBuf,
    datasets: HashMap<String, DatasetAccum>,
}

impl KeyframePersister {
    /// A persister that writes datasets under `work_root` (the same root the
    /// reconstructor reads `input_path` from).
    pub fn new(work_root: impl Into<PathBuf>) -> Self {
        Self {
            work_root: work_root.into(),
            datasets: HashMap::new(),
        }
    }

    /// Persist one keyframe: write its image to
    /// `<work_root>/<dataset_id>/images/<kf_id>.<ext>` and record its frame entry.
    /// The image bytes are written immediately (memory stays bounded); only the
    /// small frame record is retained. Returns an I/O error on a filesystem fault;
    /// the caller logs and drops (a single lost keyframe is not fatal).
    pub fn persist(&mut self, kf: &KeyframeEnvelope) -> std::io::Result<()> {
        let dataset_id = dataset_id_for(&kf.session_id);
        let dataset_dir = self.work_root.join(&dataset_id);
        let images_dir = dataset_dir.join(IMAGES_DIR);
        std::fs::create_dir_all(&images_dir)?;
        let file_path = format!("{IMAGES_DIR}/{}.{}", kf.kf_id, image_ext(kf.image.encoding));
        std::fs::write(dataset_dir.join(&file_path), &kf.image.bytes)?;
        let accum = self
            .datasets
            .entry(dataset_id)
            .or_insert_with(|| DatasetAccum {
                frames: Vec::new(),
                anchor: kf.anchor.clone(),
                camera_model: nerf_camera_model(kf.camera.model),
            });
        accum.frames.push(nerf_frame(kf, file_path));
        Ok(())
    }

    /// Frames recorded for a dataset so far (the persisted-image count, distinct
    /// from the received-event count the ingest tracks).
    pub fn frame_count(&self, dataset_id: &str) -> usize {
        self.datasets
            .get(dataset_id)
            .map(|d| d.frames.len())
            .unwrap_or(0)
    }

    /// Write `<work_root>/<dataset_id>/transforms.json` from the recorded frames
    /// and return the dataset directory (the `input_path` the reconstructor reads).
    /// The accumulated frames are consumed, so a re-finalize (a re-sent terminal
    /// state on the lossy fire-and-forget lane) is idempotent: it returns the same
    /// directory when the manifest already exists. Returns `None` when no frame was
    /// persisted (an empty dataset has nothing to reconstruct).
    pub fn finalize(&mut self, dataset_id: &str) -> std::io::Result<Option<PathBuf>> {
        let dataset_dir = self.work_root.join(dataset_id);
        let Some(accum) = self.datasets.remove(dataset_id) else {
            // Already finalized this session, or never saw a frame: the manifest's
            // presence is the truth.
            return Ok(dataset_dir
                .join(TRANSFORMS_FILE)
                .exists()
                .then_some(dataset_dir));
        };
        if accum.frames.is_empty() {
            return Ok(None);
        }
        write_manifest_atomic(&dataset_dir, &accum.manifest())?;
        Ok(Some(dataset_dir))
    }

    /// Snapshot the frames recorded so far to `<work_root>/<dataset_id>/transforms.json`
    /// WITHOUT consuming them (the session keeps accumulating) and return the
    /// dataset directory — the `input_path` a periodic live reconstruct reads, the
    /// same directory the growing `images/` already sits in. The reconstruct over
    /// this snapshot is a real, full reconstruct of the keyframes captured up to
    /// now (NOT incremental training). Returns `None` when no frame has been
    /// persisted yet (nothing to reconstruct). The manifest is written atomically,
    /// so a reconstruct still reading a prior snapshot is never torn.
    pub fn snapshot(&self, dataset_id: &str) -> std::io::Result<Option<PathBuf>> {
        let Some(accum) = self.datasets.get(dataset_id) else {
            return Ok(None);
        };
        if accum.frames.is_empty() {
            return Ok(None);
        }
        let dataset_dir = self.work_root.join(dataset_id);
        write_manifest_atomic(&dataset_dir, &accum.manifest())?;
        Ok(Some(dataset_dir))
    }
}

#[cfg(test)]
#[path = "keyframe_persister_tests.rs"]
mod tests;
