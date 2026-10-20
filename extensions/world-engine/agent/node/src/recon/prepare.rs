//! Stage `prepare`: turn an uploaded session into a COLMAP dataset directory —
//! keyframes chosen by a depth-scaled baseline, rotation and time, the
//! sharpest of each short window, blurred picks dropped (see [`keyframes`]),
//! written as PNG per camera; their pose priors (`frames.json`); the
//! operator's rooms with the trajectory inside each; and the LiDAR sweeps
//! registered into one world cloud.
//!
//! A multi-camera session is selected per time instant, not per camera: the
//! frames of one instant (one trigger sequence and within half a frame period
//! of each other, else within half a frame period alone) are kept or dropped
//! together, and an instant missing any camera's frame is dropped as a whole,
//! so the rig frames COLMAP groups by file name stay paired however many
//! frames a camera missed. Trigger sequences that pair too few instants
//! (cameras counting their own triggers) give way to time grouping. When so
//! few instants hold every camera that dropping them would collapse coverage
//! (unsynchronised cameras, different rates), every frame is exported on its
//! own, outside a rig, and SfM maps each camera separately.

use std::collections::{HashMap, HashSet};
use std::fs;
use std::path::Path;

use ados_capture_contract::foxglove::{CompressedImage, PointCloud, RawImage};
use ados_capture_contract::{CameraModelModel, RigSensorKind, SessionMessage, SessionReader};
use serde::{Deserialize, Serialize};

use super::geom::{self, Mat4};
use super::preset::PresetSpec;
use super::session_data::{self, DecodedImage, SessionIndex};
use super::StageError;
use crate::products::cloud::{write_laz, PointCloud as Cloud};

/// One exported keyframe (`dataset/frames.json`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct FrameRecord {
    /// Path under `dataset/images/`, `/`-separated: `<camera>/<t_ns, 20 digits>.png`, the
    /// time being the rig instant's when the frame is in one. The zero padding keeps name
    /// order equal to time order (COLMAP's sequential matcher pairs images in name order)
    /// and gives the frames of one rig instant one file name across cameras.
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
    /// The rig instant (its time) this frame was exported in, together with a frame of
    /// every other camera; `None` outside a rig.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rig_instant: Option<u64>,
    /// One-sigma position uncertainty (x, y, z metres) of `T_world_cam` from the recorded
    /// pose covariance; `None` when the track carried none (SfM then takes the prior
    /// source's default).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub prior_sigma_m: Option<[f64; 3]>,
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
    /// Rig instants exported with every camera's frame (0 when the cameras are
    /// exported outside a rig).
    #[serde(default)]
    pub rig_frames_paired: usize,
    /// Recorded rig instants dropped as a whole: a camera had no frame in
    /// them, or its frame was too blurred.
    #[serde(default)]
    pub rig_frames_dropped: usize,
}

/// The LiDAR world cloud is kept at this voxel size.
const LIDAR_VOXEL_M: f64 = 0.02;
/// The LiDAR scan origins in the world frame (`[[x, y, z], ...]`), one per
/// sweep: the floor plan carves free space along rays from them.
pub const LIDAR_ORIGINS_FILE: &str = "lidar_origins.json";

#[path = "prepare_keyframes.rs"]
pub mod keyframes;

use keyframes::{DepthTrack, Selection};

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
    let rig = &index.rig;
    let lidar_mount: HashMap<&str, Mat4> = rig
        .sensors
        .iter()
        .filter(|s| matches!(s.kind, RigSensorKind::Lidar))
        .map(|s| (s.id.as_str(), s.t_body_sensor.0))
        .collect();
    let depth = match lidar_mount.is_empty() {
        true => None,
        false => Some(scene_depth(reader, &lidar_mount)?).filter(|d| !d.is_empty()),
    };
    progress(0.15);
    let selection =
        keyframes::select_keyframes(&index.frames, index.poses.as_ref(), depth.as_ref(), spec);
    let mut scores = score_candidates(reader, &selection, &mut |f| progress(0.15 + 0.3 * f))?;
    let picks = keyframes::pick_sharpest(
        &selection,
        &scores,
        &keyframes::camera_medians(scores.iter().map(|((c, _), s)| (c.as_str(), *s))),
    );
    // (camera, frame time) → the time of the instant it is exported in.
    let wanted: HashMap<(String, u64), u64> = picks
        .iter()
        .flat_map(|s| s.frames.iter().map(|f| ((f.cam.clone(), f.t_ns), s.t_ns)))
        .collect();
    let rig_cams = match selection.rig {
        true => picks.first().map_or(0, |s| s.frames.len()),
        false => 0,
    };
    let cams: HashMap<&str, &ados_capture_contract::RigSensor> = rig
        .sensors
        .iter()
        .filter(|s| matches!(s.kind, RigSensorKind::Camera) && s.camera.is_some())
        .map(|s| (s.id.as_str(), s))
        .collect();

    let mut records: Vec<(FrameRecord, f64)> = Vec::new();
    let mut voxels: HashSet<[i64; 3]> = HashSet::new();
    let mut lidar: Vec<[f64; 3]> = Vec::new();
    let mut lidar_origins: Vec<[f64; 3]> = Vec::new();
    let total = wanted.len().max(1) as f32;
    reader.for_each_message(|m| {
        if let Some(cam) = image_camera(m.topic) {
            let Some(&instant) = wanted.get(&(cam.to_string(), m.t_ns)) else {
                return Ok(());
            };
            let Some(sensor) = cams.get(cam) else {
                return Ok(());
            };
            let Some(decoded) = decode_image(&m)? else {
                return Ok(());
            };
            let name = frame_name(cam, instant);
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
                    rig_instant: selection.rig.then_some(instant),
                    prior_sigma_m: t_world_cam.and_then(|_| index.pose_sigma_at(t_sensor)),
                },
                decoded.blur_score(),
            ));
            progress(0.45 + 0.5 * records.len() as f32 / total);
        } else if let Some(id) = lidar_sensor(m.topic) {
            let (Some(track), Some(mount)) = (index.poses.as_ref(), lidar_mount.get(id)) else {
                return Ok(());
            };
            let pc = m.decode::<PointCloud>()?;
            let pts = session_data::decode_points(&pc)
                .map_err(|e| ados_capture_contract::Error::Invalid(e.message))?;
            if let Some(body) = track.at(m.t_ns) {
                lidar_origins.push(geom::apply(&geom::mul(&body, mount), [0.0; 3]));
            }
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

    // Medians over every scored frame: the window candidates and the picks.
    for (r, s) in &records {
        scores.insert((r.camera_id.clone(), r.t_ns), *s);
    }
    let medians = keyframes::camera_medians(scores.iter().map(|((c, _), s)| (c.as_str(), *s)));
    let (dropped, rig_dropped) =
        keyframes::drop_unusable(dataset_dir, &mut records, rig_cams, &medians);
    let frames: Vec<FrameRecord> = records.into_iter().map(|(r, _)| r).collect();
    if frames.is_empty() {
        return Err(StageError::input(
            "no usable keyframes after blur rejection",
        ));
    }
    let rig_frames_paired = match rig_cams {
        0 => 0,
        n => frames.len() / n,
    };
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
    if !lidar_origins.is_empty() {
        fs::write(
            dataset_dir.join(LIDAR_ORIGINS_FILE),
            serde_json::to_vec(&lidar_origins)?,
        )?;
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
        rig_frames_paired,
        rig_frames_dropped: match rig_cams {
            0 => 0,
            _ => selection.incomplete + rig_dropped,
        },
    })
}

/// The camera of a `/cam/<id>/image` topic.
fn image_camera(topic: &str) -> Option<&str> {
    topic.strip_prefix("/cam/")?.strip_suffix("/image")
}

/// The sensor of a `/lidar/<id>/points` topic.
fn lidar_sensor(topic: &str) -> Option<&str> {
    topic.strip_prefix("/lidar/")?.strip_suffix("/points")
}

/// Decode an image message; `None` for a schema that is not an image.
fn decode_image(
    m: &SessionMessage<'_>,
) -> Result<Option<DecodedImage>, ados_capture_contract::Error> {
    let decoded = match m.schema_name {
        Some("foxglove.RawImage") => session_data::decode_raw(&m.decode::<RawImage>()?),
        Some("foxglove.CompressedImage") => {
            session_data::decode_compressed(&m.decode::<CompressedImage>()?)
        }
        _ => return Ok(None),
    };
    decoded
        .map(Some)
        .map_err(|e| ados_capture_contract::Error::Invalid(e.message))
}

/// The median range of every scan of the rig LiDARs in `lidars`.
fn scene_depth(
    reader: &SessionReader,
    lidars: &HashMap<&str, Mat4>,
) -> Result<DepthTrack, StageError> {
    let mut depth = DepthTrack::default();
    reader.for_each_message(|m| {
        if lidar_sensor(m.topic).is_some_and(|id| lidars.contains_key(id)) {
            let pc = m.decode::<PointCloud>()?;
            let pts = session_data::decode_points(&pc)
                .map_err(|e| ados_capture_contract::Error::Invalid(e.message))?;
            depth.push_scan(m.t_ns, pts.into_iter().map(|(p, _)| p));
        }
        Ok(())
    })?;
    Ok(depth)
}

/// Blur scores of every frame in a keyframe window of more than one instant
/// (a single-instant window has nothing to choose between).
fn score_candidates(
    reader: &SessionReader,
    selection: &Selection,
    progress: &mut dyn FnMut(f32),
) -> Result<HashMap<(String, u64), f64>, StageError> {
    let wanted: HashSet<(String, u64)> = selection
        .windows
        .iter()
        .filter(|w| w.len() > 1)
        .flatten()
        .flat_map(|s| s.frames.iter().map(|f| (f.cam.clone(), f.t_ns)))
        .collect();
    let mut scores = HashMap::with_capacity(wanted.len());
    if wanted.is_empty() {
        return Ok(scores);
    }
    let total = wanted.len() as f32;
    reader.for_each_message(|m| {
        let Some(cam) = image_camera(m.topic) else {
            return Ok(());
        };
        let key = (cam.to_string(), m.t_ns);
        if !wanted.contains(&key) {
            return Ok(());
        }
        if let Some(decoded) = decode_image(&m)? {
            scores.insert(key, decoded.blur_score());
            progress(scores.len() as f32 / total);
        }
        Ok(())
    })?;
    Ok(scores)
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
