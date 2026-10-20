//! Stage `prepare`: turn an uploaded session into a COLMAP dataset directory —
//! keyframes chosen by baseline/rotation/time and blur, written as PNG per
//! camera; their pose priors (`frames.json`); the operator's rooms with the
//! trajectory inside each; and the LiDAR sweeps registered into one world
//! cloud.

use std::collections::{HashMap, HashSet};
use std::fs;
use std::path::Path;

use ados_capture_contract::foxglove::{CompressedImage, PointCloud, RawImage};
use ados_capture_contract::{CameraModelModel, RigSensorKind, SessionReader};
use serde::{Deserialize, Serialize};

use super::geom::{self, Mat4};
use super::preset::PresetSpec;
use super::session_data::{self, DecodedImage, FrameRef, SessionIndex};
use super::StageError;
use crate::products::cloud::{write_laz, PointCloud as Cloud};

/// One exported keyframe (`dataset/frames.json`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrameRecord {
    /// Path under `dataset/images/`, `/`-separated: `<camera>/<t_ns, 20 digits>.png`. The
    /// zero padding keeps name order equal to time order (COLMAP's sequential matcher pairs
    /// images in name order) and gives the frames of one instant one file name across cameras.
    pub name: String,
    pub camera_id: String,
    pub t_ns: u64,
    /// World-from-camera (OpenCV camera), row-major; `None` without a prior.
    #[serde(rename = "T_world_cam")]
    pub t_world_cam: Option<[f64; 16]>,
    pub width: u32,
    pub height: u32,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    pub model: CameraModelModel,
    pub dist: Vec<f64>,
    /// The intrinsics come from a calibration (bundle adjustment keeps them).
    #[serde(default)]
    pub calibrated: bool,
    /// Body-from-camera mount (OpenCV camera), row-major, from the session rig. SfM ties the
    /// cameras of a calibrated multi-camera rig together with it.
    #[serde(
        rename = "T_body_cam",
        default,
        skip_serializing_if = "Option::is_none"
    )]
    pub t_body_cam: Option<[f64; 16]>,
}

/// The dataset name of camera `cam`'s frame at `t_ns` (see [`FrameRecord::name`]).
pub fn frame_name(cam: &str, t_ns: u64) -> String {
    format!("{cam}/{t_ns:020}.png")
}

/// One operator room with the body trajectory recorded inside it.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct RoomRecord {
    pub name: String,
    pub floor: i32,
    pub t_start_ns: u64,
    pub t_end_ns: u64,
    pub positions: Vec<[f64; 3]>,
}

/// What `prepare` produced, persisted as its stage detail.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PrepareSummary {
    pub frames: usize,
    pub frames_dropped_blur: usize,
    pub cameras: Vec<String>,
    pub pose_prior: Option<String>,
    pub lidar_points: usize,
    pub rooms: usize,
}

/// The LiDAR world cloud is kept at this voxel size.
const LIDAR_VOXEL_M: f64 = 0.02;
/// A frame is dropped when its blur score is under this fraction of its
/// camera's median.
const BLUR_KEEP_FRACTION: f64 = 0.35;

/// Choose keyframes from the index (per camera: far enough from the last
/// selected one in position, rotation or time), then thin uniformly to the
/// preset's image cap.
pub fn select_keyframes(index: &SessionIndex, spec: &PresetSpec) -> Vec<FrameRef> {
    let mut last: HashMap<&str, (u64, Option<Mat4>)> = HashMap::new();
    let mut picked = Vec::new();
    let min_rot = spec.min_rotation_deg.to_radians();
    for f in &index.frames {
        let pose = index.poses.as_ref().and_then(|p| p.at(f.t_ns));
        let take = match last.get(f.cam.as_str()) {
            None => true,
            Some((t_prev, prev)) => {
                let dt = f.t_ns.saturating_sub(*t_prev);
                match (prev, pose) {
                    (Some(a), Some(b)) => {
                        geom::dist(geom::translation(a), geom::translation(&b))
                            >= spec.min_baseline_m
                            || geom::rotation_angle(a, &b) >= min_rot
                            || dt >= 2_000_000_000
                    }
                    _ => dt >= 250_000_000,
                }
            }
        };
        if take {
            last.insert(&f.cam, (f.t_ns, pose));
            picked.push(f.clone());
        }
    }
    if picked.len() > spec.max_images {
        let stride = picked.len() as f64 / spec.max_images as f64;
        picked = (0..spec.max_images)
            .map(|i| picked[(i as f64 * stride) as usize].clone())
            .collect();
    }
    picked
}

/// Run the stage into `dataset_dir`.
pub fn run(
    reader: &SessionReader,
    dataset_dir: &Path,
    spec: &PresetSpec,
    progress: &mut dyn FnMut(f32),
) -> Result<PrepareSummary, StageError> {
    if dataset_dir.exists() {
        fs::remove_dir_all(dataset_dir)?;
    }
    fs::create_dir_all(dataset_dir.join("images"))?;
    // Rig sensor ids name folders and list entries below; refuse any that is
    // not a plain contract id, whatever path the session arrived by.
    check_ids(reader.rig())?;
    let index = session_data::index(reader)?;
    if index.frames.is_empty() {
        return Err(StageError::input("the session has no camera frames"));
    }
    progress(0.1);
    let wanted: HashSet<(String, u64)> = select_keyframes(&index, spec)
        .into_iter()
        .map(|f| (f.cam, f.t_ns))
        .collect();
    let rig = &index.rig;
    let cams: HashMap<&str, &ados_capture_contract::RigSensor> = rig
        .sensors
        .iter()
        .filter(|s| matches!(s.kind, RigSensorKind::Camera) && s.camera.is_some())
        .map(|s| (s.id.as_str(), s))
        .collect();
    let lidar_mount: HashMap<&str, Mat4> = rig
        .sensors
        .iter()
        .filter(|s| matches!(s.kind, RigSensorKind::Lidar))
        .map(|s| (s.id.as_str(), s.t_body_sensor.0))
        .collect();

    let mut records: Vec<(FrameRecord, f64)> = Vec::new();
    let mut voxels: HashSet<[i64; 3]> = HashSet::new();
    let mut lidar: Vec<[f64; 3]> = Vec::new();
    let total = wanted.len().max(1) as f32;
    reader.for_each_message(|m| {
        if let Some(cam) = m
            .topic
            .strip_prefix("/cam/")
            .and_then(|r| r.strip_suffix("/image"))
        {
            if !wanted.contains(&(cam.to_string(), m.t_ns)) {
                return Ok(());
            }
            let Some(sensor) = cams.get(cam) else {
                return Ok(());
            };
            let decoded: DecodedImage = match m.schema_name {
                Some("foxglove.RawImage") => session_data::decode_raw(&m.decode::<RawImage>()?),
                Some("foxglove.CompressedImage") => {
                    session_data::decode_compressed(&m.decode::<CompressedImage>()?)
                }
                _ => return Ok(()),
            }
            .map_err(|e| ados_capture_contract::Error::Invalid(e.message))?;
            let name = frame_name(cam, m.t_ns);
            let path = dataset_dir.join("images").join(&name);
            fs::create_dir_all(path.parent().expect("camera folder"))?;
            decoded
                .write_png(&path)
                .map_err(|e| ados_capture_contract::Error::Invalid(e.message))?;
            let model = sensor.camera.as_ref().expect("filtered on camera");
            let t_sensor = (m.t_ns as i64 + sensor.time_offset_ns).max(0) as u64;
            let t_world_cam = index
                .poses
                .as_ref()
                .and_then(|p| p.at(t_sensor))
                .map(|body| geom::mul(&body, &sensor.t_body_sensor.0));
            records.push((
                FrameRecord {
                    name,
                    camera_id: cam.to_string(),
                    t_ns: m.t_ns,
                    t_world_cam,
                    width: decoded.width,
                    height: decoded.height,
                    fx: model.fx,
                    fy: model.fy,
                    cx: model.cx,
                    cy: model.cy,
                    model: model.model,
                    dist: model.dist.clone(),
                    calibrated: model.calibrated,
                    t_body_cam: Some(sensor.t_body_sensor.0),
                },
                decoded.blur_score(),
            ));
            progress(0.1 + 0.8 * records.len() as f32 / total);
        } else if let Some(id) = m
            .topic
            .strip_prefix("/lidar/")
            .and_then(|r| r.strip_suffix("/points"))
        {
            let (Some(track), Some(mount)) = (index.poses.as_ref(), lidar_mount.get(id)) else {
                return Ok(());
            };
            let pc = m.decode::<PointCloud>()?;
            let pts = session_data::decode_points(&pc)
                .map_err(|e| ados_capture_contract::Error::Invalid(e.message))?;
            for (p, dt) in pts {
                let Some(body) = track.at(m.t_ns + dt) else {
                    continue;
                };
                let w = geom::apply(&geom::mul(&body, mount), p);
                let key = [
                    (w[0] / LIDAR_VOXEL_M).floor() as i64,
                    (w[1] / LIDAR_VOXEL_M).floor() as i64,
                    (w[2] / LIDAR_VOXEL_M).floor() as i64,
                ];
                if voxels.insert(key) {
                    lidar.push(w);
                }
            }
        }
        Ok(())
    })?;

    let dropped = drop_blurred(dataset_dir, &mut records)?;
    let frames: Vec<FrameRecord> = records.into_iter().map(|(r, _)| r).collect();
    if frames.is_empty() {
        return Err(StageError::input(
            "no usable keyframes after blur rejection",
        ));
    }
    fs::write(
        dataset_dir.join("frames.json"),
        serde_json::to_vec_pretty(&frames)?,
    )?;
    let rooms = room_records(&index);
    fs::write(
        dataset_dir.join("rooms.json"),
        serde_json::to_vec_pretty(&rooms)?,
    )?;
    if !lidar.is_empty() {
        write_laz(
            &dataset_dir.join("lidar_points.laz"),
            &Cloud {
                points: lidar.clone(),
                colors: None,
            },
        )
        .map_err(StageError::io)?;
    }
    let mut cameras: Vec<String> = frames.iter().map(|f| f.camera_id.clone()).collect();
    cameras.sort();
    cameras.dedup();
    progress(1.0);
    Ok(PrepareSummary {
        frames: frames.len(),
        frames_dropped_blur: dropped,
        cameras,
        pose_prior: index.poses.as_ref().map(|p| p.source.clone()),
        lidar_points: lidar.len(),
        rooms: rooms.len(),
    })
}

/// Every rig id the pipeline uses as a path component or a list entry must be
/// a plain contract id.
pub fn check_ids(rig: &ados_capture_contract::Rig) -> Result<(), StageError> {
    for id in std::iter::once(rig.rig_id.as_str()).chain(rig.sensors.iter().map(|s| s.id.as_str()))
    {
        if !crate::bulk::is_safe_id(id) {
            return Err(StageError::new(
                "E_INVALID_SESSION",
                format!("rig id {id:?} is not a plain contract id"),
            ));
        }
    }
    Ok(())
}
/// Drop frames whose blur score is far under their camera's median (and
/// delete their PNGs). Returns how many were dropped.
fn drop_blurred(
    dataset_dir: &Path,
    records: &mut Vec<(FrameRecord, f64)>,
) -> Result<usize, StageError> {
    let mut medians: HashMap<String, f64> = HashMap::new();
    let mut by_cam: HashMap<&str, Vec<f64>> = HashMap::new();
    for (r, s) in records.iter() {
        by_cam.entry(&r.camera_id).or_default().push(*s);
    }
    for (cam, mut scores) in by_cam {
        scores.sort_by(|a, b| a.total_cmp(b));
        medians.insert(cam.to_string(), scores[scores.len() / 2]);
    }
    let before = records.len();
    let mut keep = Vec::with_capacity(before);
    for (r, s) in records.drain(..) {
        if s >= BLUR_KEEP_FRACTION * medians[&r.camera_id] {
            keep.push((r, s));
        } else {
            let _ = fs::remove_file(dataset_dir.join("images").join(&r.name));
        }
    }
    *records = keep;
    Ok(before - records.len())
}

/// The operator's rooms with the body positions recorded inside each
/// (sampled every half second).
fn room_records(index: &SessionIndex) -> Vec<RoomRecord> {
    index
        .manifest
        .operator
        .rooms
        .iter()
        .map(|room| {
            let positions = index
                .poses
                .as_ref()
                .map(|track| {
                    track
                        .samples
                        .iter()
                        .filter(|(t, _)| *t >= room.t_start_ns && *t <= room.t_end_ns)
                        .scan(None::<u64>, |last, (t, m)| {
                            let due = last.is_none_or(|l| t - l >= 500_000_000);
                            if due {
                                *last = Some(*t);
                            }
                            Some(due.then(|| geom::translation(m)))
                        })
                        .flatten()
                        .collect()
                })
                .unwrap_or_default();
            RoomRecord {
                name: room.name.clone(),
                floor: room.floor,
                t_start_ns: room.t_start_ns,
                t_end_ns: room.t_end_ns,
                positions,
            }
        })
        .collect()
}

/// Read `frames.json` back.
pub fn read_frames(dataset_dir: &Path) -> Result<Vec<FrameRecord>, StageError> {
    Ok(serde_json::from_slice(&fs::read(
        dataset_dir.join("frames.json"),
    )?)?)
}

/// Read `rooms.json` back.
pub fn read_rooms(dataset_dir: &Path) -> Result<Vec<RoomRecord>, StageError> {
    match fs::read(dataset_dir.join("rooms.json")) {
        Ok(b) => Ok(serde_json::from_slice(&b)?),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Vec::new()),
        Err(e) => Err(e.into()),
    }
}
