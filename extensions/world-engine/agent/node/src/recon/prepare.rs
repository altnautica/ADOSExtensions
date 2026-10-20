//! Stage `prepare`: turn an uploaded session into a COLMAP dataset directory —
//! keyframes chosen by baseline/rotation/time and blur, written as PNG per
//! camera; their pose priors (`frames.json`); the operator's rooms with the
//! trajectory inside each; and the LiDAR sweeps registered into one world
//! cloud.
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

use std::collections::{BTreeMap, HashMap, HashSet};
use std::fs;
use std::path::Path;

use ados_capture_contract::foxglove::{CompressedImage, PointCloud, RawImage};
use ados_capture_contract::{CameraModelModel, RigSensorKind, SessionReader};
use serde::{Deserialize, Serialize};

use super::geom::{self, Mat4};
use super::preset::PresetSpec;
use super::session_data::{self, DecodedImage, FrameRef, PoseTrack, SessionIndex};
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
/// A frame is dropped when its blur score is under this fraction of its
/// camera's median.
const BLUR_KEEP_FRACTION: f64 = 0.35;
/// A multi-camera session is selected as a rig only while at least this share
/// of its sparsest camera's frames fall in instants that hold every camera.
const RIG_MIN_PAIRED_SHARE: f64 = 0.5;

/// The frames of one time instant: one per rig camera, or, outside a rig,
/// one camera's frame.
#[derive(Debug, Clone, PartialEq)]
pub struct FrameSet {
    /// The reference (first by id) camera's frame time; a rig instant's
    /// frames are all named by it, each keeping its own time in its record.
    pub t_ns: u64,
    /// Ordered by camera.
    pub frames: Vec<FrameRef>,
}

/// The keyframes chosen from a session.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Selection {
    pub sets: Vec<FrameSet>,
    /// The sets are rig instants holding every camera's frame.
    pub rig: bool,
    /// Recorded instants dropped because a camera had no frame in them.
    pub incomplete: usize,
}

/// Choose keyframes from the time-sorted `frames`: group a multi-camera
/// session into instants and keep only those holding every camera (see the
/// module docs for the fallback), pick the instants far enough from the last
/// picked one in position, rotation or time (per camera outside a rig), then
/// thin uniformly to the preset's image cap, whole instants at a time.
pub fn select_keyframes(
    frames: &[FrameRef],
    poses: Option<&PoseTrack>,
    spec: &PresetSpec,
) -> Selection {
    let mut cams: Vec<&str> = frames.iter().map(|f| f.cam.as_str()).collect();
    cams.sort_unstable();
    cams.dedup();
    let mut selection = Selection::default();
    if cams.len() >= 2 {
        let sparsest = cams
            .iter()
            .map(|c| frames.iter().filter(|f| f.cam == *c).count())
            .min()
            .unwrap_or(0);
        let period = frame_period(frames);
        let by_seq = frames
            .iter()
            .all(|f| f.trigger_seq.is_some())
            .then(|| group_by_trigger(frames, period));
        for grouped in by_seq
            .into_iter()
            .chain(std::iter::once_with(|| group_by_time(frames, period)))
        {
            let (complete, incomplete): (Vec<FrameSet>, Vec<FrameSet>) = grouped
                .into_iter()
                .partition(|s| s.frames.len() == cams.len());
            if complete.len() as f64 >= RIG_MIN_PAIRED_SHARE * sparsest as f64 {
                selection = Selection {
                    sets: complete,
                    rig: true,
                    incomplete: incomplete.len(),
                };
                break;
            }
        }
    }
    if !selection.rig {
        selection.sets = frames
            .iter()
            .map(|f| FrameSet {
                t_ns: f.t_ns,
                frames: vec![f.clone()],
            })
            .collect();
    }

    let min_rot = spec.min_rotation_deg.to_radians();
    let mut last: HashMap<&str, (u64, Option<Mat4>)> = HashMap::new();
    let mut picked: Vec<usize> = Vec::new();
    for (i, s) in selection.sets.iter().enumerate() {
        let key = if selection.rig {
            ""
        } else {
            s.frames[0].cam.as_str()
        };
        let pose = poses.and_then(|p| p.at(s.t_ns));
        let take = match last.get(key) {
            None => true,
            Some((t_prev, prev)) => {
                let dt = s.t_ns.saturating_sub(*t_prev);
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
            last.insert(key, (s.t_ns, pose));
            picked.push(i);
        }
    }
    let per_set = if selection.rig { cams.len() } else { 1 };
    let cap = (spec.max_images / per_set).max(1);
    if picked.len() > cap {
        let stride = picked.len() as f64 / cap as f64;
        picked = (0..cap)
            .map(|i| picked[(i as f64 * stride) as usize])
            .collect();
    }
    let mut sets = std::mem::take(&mut selection.sets);
    let mut keep = picked.into_iter().peekable();
    let mut i = 0;
    sets.retain(|_| {
        let k = keep.next_if_eq(&i).is_some();
        i += 1;
        k
    });
    selection.sets = sets;
    selection
}

/// `2·dt` under a frame period: two frames of one instant.
fn same_instant(dt: u64, period: u64) -> bool {
    dt.saturating_mul(2) < period
}

/// Group frames into instants by trigger sequence; a frame more than half a
/// frame period from its sequence's earliest frame, or a camera's second
/// frame under one sequence, stands alone.
fn group_by_trigger(frames: &[FrameRef], period: u64) -> Vec<FrameSet> {
    let mut by_seq: BTreeMap<u64, Vec<FrameRef>> = BTreeMap::new();
    for f in frames {
        by_seq
            .entry(f.trigger_seq.unwrap_or_default())
            .or_default()
            .push(f.clone());
    }
    let mut sets = Vec::new();
    for mut group in by_seq.into_values() {
        group.sort_by_key(|f| f.t_ns);
        let t0 = group[0].t_ns;
        let mut set: Vec<FrameRef> = Vec::new();
        for f in group {
            if set.iter().all(|g| g.cam != f.cam) && same_instant(f.t_ns - t0, period) {
                set.push(f);
            } else {
                sets.push(vec![f]);
            }
        }
        sets.push(set);
    }
    finish_sets(sets)
}

/// Group time-sorted frames into instants by time: a frame joins the open
/// instant while within half a frame period of its first frame and its
/// camera is not in it yet.
fn group_by_time(frames: &[FrameRef], period: u64) -> Vec<FrameSet> {
    let mut sets: Vec<Vec<FrameRef>> = Vec::new();
    for f in frames {
        match sets.last_mut() {
            Some(s)
                if s.iter().all(|g| g.cam != f.cam)
                    && same_instant(f.t_ns.saturating_sub(s[0].t_ns), period) =>
            {
                s.push(f.clone())
            }
            _ => sets.push(vec![f.clone()]),
        }
    }
    finish_sets(sets)
}

/// Order each instant's frames by camera, time it by its reference camera,
/// and order the instants by time.
fn finish_sets(groups: Vec<Vec<FrameRef>>) -> Vec<FrameSet> {
    let mut sets: Vec<FrameSet> = groups
        .into_iter()
        .map(|mut frames| {
            frames.sort_by(|a, b| a.cam.cmp(&b.cam));
            FrameSet {
                t_ns: frames[0].t_ns,
                frames,
            }
        })
        .collect();
    sets.sort_by_key(|s| s.t_ns);
    sets
}

/// The shortest per-camera median frame interval (0 with no camera recording
/// two frames).
fn frame_period(frames: &[FrameRef]) -> u64 {
    let mut by_cam: HashMap<&str, Vec<u64>> = HashMap::new();
    for f in frames {
        by_cam.entry(&f.cam).or_default().push(f.t_ns);
    }
    by_cam
        .into_values()
        .filter_map(|mut ts| {
            ts.sort_unstable();
            let mut dts: Vec<u64> = ts.windows(2).map(|w| w[1] - w[0]).collect();
            dts.sort_unstable();
            dts.get(dts.len() / 2).copied()
        })
        .min()
        .unwrap_or(0)
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
    let selection = select_keyframes(&index.frames, index.poses.as_ref(), spec);
    // (camera, frame time) → the time of the instant it is exported in.
    let wanted: HashMap<(String, u64), u64> = selection
        .sets
        .iter()
        .flat_map(|s| s.frames.iter().map(|f| ((f.cam.clone(), f.t_ns), s.t_ns)))
        .collect();
    let rig_cams = match selection.rig {
        true => selection.sets.first().map_or(0, |s| s.frames.len()),
        false => 0,
    };
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
            let Some(&instant) = wanted.get(&(cam.to_string(), m.t_ns)) else {
                return Ok(());
            };
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

    let (dropped, rig_dropped) = drop_unusable(dataset_dir, &mut records, rig_cams);
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

/// Drop frames whose blur score is far under their camera's median, and with
/// `rig_cams` > 0 every rig instant left without all `rig_cams` usable frames
/// (deleting their PNGs). Returns how many frames were dropped for blur and
/// how many rig instants were dropped.
fn drop_unusable(
    dataset_dir: &Path,
    records: &mut Vec<(FrameRecord, f64)>,
    rig_cams: usize,
) -> (usize, usize) {
    let mut medians: HashMap<String, f64> = HashMap::new();
    let mut by_cam: HashMap<&str, Vec<f64>> = HashMap::new();
    for (r, s) in records.iter() {
        by_cam.entry(&r.camera_id).or_default().push(*s);
    }
    for (cam, mut scores) in by_cam {
        scores.sort_by(|a, b| a.total_cmp(b));
        medians.insert(cam.to_string(), scores[scores.len() / 2]);
    }
    let sharp = |r: &FrameRecord, s: f64| s >= BLUR_KEEP_FRACTION * medians[&r.camera_id];
    let blurred = records.iter().filter(|(r, s)| !sharp(r, *s)).count();
    // Usable frames per rig instant; an instant short of a camera goes whole.
    let mut usable: HashMap<u64, usize> = HashMap::new();
    let mut instants: HashSet<u64> = HashSet::new();
    for (r, s) in records.iter() {
        if let Some(t) = r.rig_instant {
            instants.insert(t);
            if sharp(r, *s) {
                *usable.entry(t).or_default() += 1;
            }
        }
    }
    let complete = |t: u64| usable.get(&t).copied().unwrap_or(0) == rig_cams;
    let rig_dropped = instants.iter().filter(|t| !complete(**t)).count();
    let (keep, gone): (Vec<_>, Vec<_>) = records
        .drain(..)
        .partition(|(r, s)| sharp(r, *s) && r.rig_instant.is_none_or(complete));
    for (r, _) in gone {
        let _ = fs::remove_file(dataset_dir.join("images").join(&r.name));
    }
    *records = keep;
    (blurred, rig_dropped)
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

#[cfg(test)]
#[path = "prepare_tests.rs"]
mod tests;
