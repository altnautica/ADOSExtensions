//! `lidar_camera` for a planar LiDAR: the extrinsic from a calibration board
//! the scan line crosses.
//!
//! The operator holds the camera-calibration board ([`super::AprilGrid`]) so
//! the LiDAR's scan plane cuts across it, at varied tilts. Every scan is paired
//! with the camera frame nearest in time (within [`MAX_PAIR_DT_NS`]); the
//! board pose in that frame gives the board plane in the camera frame, and
//! the scan points near that plane (placed by the rig's current extrinsic,
//! within [`WINDOW_M`]) are fitted by a RANSAC line whose inliers are the
//! points on the board. [`lrf_solve`] turns the views into `T_cam_lidar`,
//! and the LiDAR is placed through the camera's known pose:
//! `T_body_lidar = T_body_cam · T_cam_lidar`. A fit worse than
//! [`MAX_RMS_M`] fails the job.

use std::collections::BTreeMap;

use ados_capture_contract::foxglove::PointCloud;
use ados_capture_contract::{topics, LidarModelPattern, Rig, RigSensorKind, SessionReader};
use nalgebra::{Matrix3, Matrix4, Vector3};
use serde_json::{json, Value};

use super::board::{Board, BoardPose};
use super::geom::{inv_rigid, mul, to_xyz_quat};
use super::kalibr_yaml::rig_cameras;
use super::lrf_solve::{self, PlaneObs, MAX_RMS_M};
use super::{visit, AprilGrid};
use crate::recon::prepare::decode_image;
use crate::recon::session_data::decode_points;
use crate::recon::StageError;

/// A scan pairs with the nearest camera frame at most this far away in time.
pub const MAX_PAIR_DT_NS: u64 = 50_000_000;
/// Scan points further than this from the board plane (placed by the rig's
/// current extrinsic) are not candidates, metres.
pub const WINDOW_M: f64 = 0.25;
/// Candidates may fall this far outside the board's tag area (the paper
/// margin, and the current extrinsic's error), metres.
pub const MARGIN_M: f64 = 0.15;
/// RANSAC line inlier distance and the inliers a view needs.
pub const LINE_INLIER_M: f64 = 0.02;
pub const MIN_LINE_INLIERS: usize = 15;
const RANSAC_ITERATIONS: usize = 200;

/// Whether the rig's LiDAR is planar (this calibration applies).
pub fn rig_lidar_is_planar(rig: &Rig) -> bool {
    rig.sensors.iter().any(|s| {
        s.kind == RigSensorKind::Lidar
            && s.lidar.as_ref().map(|l| l.pattern) == Some(LidarModelPattern::Planar)
    })
}

/// Rig indexes of the modelled camera and the planar LiDAR.
fn pair(rig: &Rig) -> Result<(usize, usize), StageError> {
    let cam = rig_cameras(rig)
        .into_iter()
        .find(|&i| rig.sensors[i].camera.is_some())
        .ok_or_else(|| StageError::input("the rig has no camera with a projection model"))?;
    let lidar = rig
        .sensors
        .iter()
        .position(|s| {
            s.kind == RigSensorKind::Lidar
                && s.lidar.as_ref().map(|l| l.pattern) == Some(LidarModelPattern::Planar)
        })
        .ok_or_else(|| StageError::input("the rig has no planar LiDAR"))?;
    Ok((cam, lidar))
}

fn to_na(m: &[f64; 16]) -> Matrix4<f64> {
    Matrix4::from_row_slice(m)
}

fn from_na(r: &Matrix3<f64>, t: &Vector3<f64>) -> [f64; 16] {
    [
        r[(0, 0)],
        r[(0, 1)],
        r[(0, 2)],
        t.x,
        r[(1, 0)],
        r[(1, 1)],
        r[(1, 2)],
        t.y,
        r[(2, 0)],
        r[(2, 1)],
        r[(2, 2)],
        t.z,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

/// One recorded scan: its mid-revolution time (sensor clock corrected) and
/// its points' LiDAR-frame `(x, y)`.
struct Scan {
    t_ns: u64,
    points: Vec<[f64; 2]>,
}

/// The candidate scan points on the board seen at `pose`: near its plane and
/// over its tag area (plus [`MARGIN_M`]) under the current extrinsic.
fn candidates(
    points: &[[f64; 2]],
    pose: &BoardPose,
    extent: [f64; 2],
    t0: &Matrix4<f64>,
) -> Vec<[f64; 2]> {
    let (n, d) = pose.plane();
    points
        .iter()
        .copied()
        .filter(|p| {
            let q = (t0 * nalgebra::Vector4::new(p[0], p[1], 0.0, 1.0)).xyz();
            let off = n.dot(&q) - d;
            let b = pose.r.transpose() * (q - n * off - pose.t);
            off.abs() <= WINDOW_M
                && (-MARGIN_M..=extent[0] + MARGIN_M).contains(&b.x)
                && (-MARGIN_M..=extent[1] + MARGIN_M).contains(&b.y)
        })
        .collect()
}

/// The inliers of the best RANSAC line through `pts` (deterministic sampling),
/// `None` with fewer than [`MIN_LINE_INLIERS`].
pub fn line_inliers(pts: &[[f64; 2]]) -> Option<Vec<[f64; 2]>> {
    if pts.len() < MIN_LINE_INLIERS {
        return None;
    }
    let mut state = 0x9e37_79b9_7f4a_7c15u64;
    let mut next = |n: usize| {
        state = state
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        (state >> 33) as usize % n
    };
    let mut best: Vec<[f64; 2]> = Vec::new();
    for _ in 0..RANSAC_ITERATIONS {
        let (a, b) = (pts[next(pts.len())], pts[next(pts.len())]);
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let len = dx.hypot(dy);
        if len < 0.05 {
            continue;
        }
        let inliers: Vec<[f64; 2]> = pts
            .iter()
            .copied()
            .filter(|p| ((p[0] - a[0]) * dy - (p[1] - a[1]) * dx).abs() / len <= LINE_INLIER_M)
            .collect();
        if inliers.len() > best.len() {
            best = inliers;
        }
    }
    (best.len() >= MIN_LINE_INLIERS).then_some(best)
}

pub fn lidar_camera(
    reader: &SessionReader,
    grid: &AprilGrid,
    progress: &mut dyn FnMut(f32),
) -> Result<(Rig, Value), StageError> {
    let base = reader.rig();
    let (cam, lidar) = pair(base)?;
    let (cam_s, lidar_s) = (&base.sensors[cam], &base.sensors[lidar]);
    let model = cam_s
        .camera
        .as_ref()
        .expect("pair() picks a modelled camera");
    let (cam_topic, lidar_topic) = (
        topics::cam_image(&cam_s.id),
        topics::lidar_points(&lidar_s.id),
    );
    let shift = |t: u64, off: i64| (t as i64 + off).max(0) as u64;

    // Pass 1: the scans and the frame times.
    let mut scans = Vec::new();
    let mut frame_times = Vec::new();
    visit(reader, |m| {
        if m.topic == cam_topic {
            frame_times.push(m.t_ns);
        } else if m.topic == lidar_topic {
            let pts = decode_points(&m.decode::<PointCloud>()?)?;
            let span = pts.iter().map(|p| p.1).max().unwrap_or(0);
            scans.push(Scan {
                t_ns: shift(m.t_ns + span / 2, lidar_s.time_offset_ns),
                points: pts.iter().map(|(p, _)| [p[0], p[1]]).collect(),
            });
        }
        Ok(())
    })?;
    if scans.is_empty() || frame_times.is_empty() {
        return Err(StageError::input(format!(
            "the session needs frames of {} and scans of {} (got {} and {})",
            cam_s.id,
            lidar_s.id,
            frame_times.len(),
            scans.len()
        )));
    }
    progress(0.2);
    // Each scan's nearest frame (by its recorded time).
    frame_times.sort_unstable();
    let mut by_frame: BTreeMap<u64, Vec<usize>> = BTreeMap::new();
    for (i, s) in scans.iter().enumerate() {
        let at = |t: u64| shift(t, cam_s.time_offset_ns).abs_diff(s.t_ns);
        let k = frame_times.partition_point(|t| shift(*t, cam_s.time_offset_ns) < s.t_ns);
        let best = [k.checked_sub(1), Some(k)]
            .into_iter()
            .flatten()
            .filter_map(|k| frame_times.get(k).copied())
            .min_by_key(|t| at(*t));
        if let Some(t) = best.filter(|t| at(*t) <= MAX_PAIR_DT_NS) {
            by_frame.entry(t).or_default().push(i);
        }
    }

    // Pass 2: the board in each paired frame, and the scan points on it.
    let board = Board::new(grid);
    let extent = board.extent();
    let t0_rows = mul(&inv_rigid(&cam_s.t_body_sensor.0), &lidar_s.t_body_sensor.0);
    let t0 = to_na(&t0_rows);
    let mut obs = Vec::new();
    let (mut boards_found, mut done) = (0usize, 0usize);
    let total = by_frame.len().max(1) as f32;
    visit(reader, |m| {
        let Some(paired) = (m.topic == cam_topic)
            .then(|| by_frame.get(&m.t_ns))
            .flatten()
        else {
            return Ok(());
        };
        done += 1;
        progress(0.2 + 0.6 * done as f32 / total);
        let Some(img) = decode_image(&m)? else {
            return Ok(());
        };
        let Some(pose) = board.detect(&img, model) else {
            return Ok(());
        };
        boards_found += 1;
        let (n, d) = pose.plane();
        for &i in paired {
            if let Some(points) = line_inliers(&candidates(&scans[i].points, &pose, extent, &t0)) {
                obs.push(PlaneObs { n, d, points });
            }
        }
        Ok(())
    })?;
    let x = lrf_solve::solve(&obs)?;
    progress(0.95);
    if x.rms_m > MAX_RMS_M {
        return Err(StageError::new(
            "E_CALIB_FAILED",
            format!(
                "the LiDAR-camera fit leaves {:.4} m RMS point-to-board error (limit {MAX_RMS_M} m)",
                x.rms_m
            ),
        ));
    }
    let t_cam_lidar = from_na(&x.r, &x.t);
    let mut rig = base.clone();
    rig.sensors[lidar].t_body_sensor = mul(&cam_s.t_body_sensor.0, &t_cam_lidar).into();
    let report = json!({
        "reprojection_rms_px": null,
        "tool": "board_plane",
        "camera_id": cam_s.id,
        "lidar_id": lidar_s.id,
        "scans": scans.len(),
        "frames_with_board": boards_found,
        "views": obs.len(),
        "points": obs.iter().map(|o| o.points.len()).sum::<usize>(),
        "tilt_spread_deg": lrf_solve::tilt_spread_deg(&obs),
        "rms_m": x.rms_m,
        "init_T_cam_lidar": to_xyz_quat(&t0_rows),
        "T_cam_lidar": to_xyz_quat(&t_cam_lidar),
    });
    Ok((rig, report))
}

#[cfg(test)]
#[path = "lrf_camera_tests.rs"]
mod tests;
