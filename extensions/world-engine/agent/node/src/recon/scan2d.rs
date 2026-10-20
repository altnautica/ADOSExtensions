//! Planar LiDAR trajectory of a session, ahead of SfM.
//!
//! A rig whose LiDAR scans one level plane (pattern `planar`, an RPLIDAR-class
//! 2D scanner) is matched in-process by [`ados_scan2d`]: the session's scans
//! and IMU samples go through local SLAM, loop search and the pose graph
//! ([`ados_scan2d::offline::run`]). Each LiDAR pose `(x, y, yaw, roll, pitch)`
//! becomes the body pose `T_world_body = T_body_lidar · T_local_lidar ·
//! T_body_lidar⁻¹`, so the world is the body frame at the first scan made
//! level; the body height of every pose is that of the first one (a planar
//! LiDAR does not observe height). Work directory `<job>/lidar2d/`:
//! `trajectory.tum` (that body trajectory) and `summary.json`.
//!
//! With a recorded pose track the trajectory is rigidly aligned onto it (the
//! session's ENU world) and rejected, keeping the recorded poses, when the
//! alignment is worse than [`MAX_ALIGN_RMSE_M`]; without one it is the
//! model's own local frame. A used trajectory re-poses `dataset/frames.json`
//! (each camera at its own capture time through its `T_body_cam`; recorded
//! covariances are cleared, so the prior takes the `lidar2d` default
//! uncertainty) and registers the sweeps again into
//! `dataset/lidar_points.laz` and `dataset/lidar_origins.json`.

use std::fs;
use std::path::Path;

use ados_capture_contract::foxglove::PointCloud;
use ados_capture_contract::v1::ImuBatch;
use ados_capture_contract::{
    topics, LidarModelPattern, Rig, RigSensor, RigSensorKind, SessionReader,
};
use ados_scan2d::{offline, ImuSample, PoseOut, Scan2dConfig, ScanInput};
use serde::Serialize;

use super::geom::{self, Mat4};
use super::glim::{align, max_speed, rewrite_frames, MAX_ALIGN_RMSE_M, MAX_SPEED_MPS};
use super::glim_bag::write_tum;
use super::prepare::sweeps::Sweeps;
use super::session_data::{self, PoseTrack};
use super::StageError;

/// Work directory under the job directory.
pub const SCAN2D_DIR: &str = "lidar2d";
/// The pose source a used trajectory gives the SfM priors.
pub const PRIOR_SOURCE: &str = "lidar2d";

/// The `lidar2d` entry of the sfm stage detail (also `lidar2d/summary.json`).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct Scan2dSummary {
    pub lidar_id: String,
    pub scans: usize,
    pub imu_samples: usize,
    pub submaps: usize,
    pub loops_accepted: usize,
    pub loops_rejected: usize,
    /// Mean match score of the scan poses.
    pub mean_score: f64,
    /// RMS position error of the alignment onto the session pose track;
    /// `None` when the session has no track.
    pub align_rmse_m: Option<f64>,
    /// The trajectory posed the keyframes.
    pub used: bool,
    /// Why the trajectory was not used (`align_rmse`, `speed`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rejected: Option<String>,
    pub frames_posed: usize,
    pub frames_recorded: usize,
    pub frames_dropped: usize,
    /// Points of the re-registered LiDAR cloud.
    pub lidar_points: usize,
}

/// The rig's planar LiDAR, if it has one.
pub fn rig_planar_lidar(rig: &Rig) -> Option<&RigSensor> {
    rig.sensors.iter().find(|s| {
        s.kind == RigSensorKind::Lidar
            && s.lidar.as_ref().map(|l| l.pattern) == Some(LidarModelPattern::Planar)
    })
}

fn shift(t: u64, offset_ns: i64) -> u64 {
    (t as i64 + offset_ns).max(0) as u64
}

/// The scans of `lidar` and the samples of the rig's first IMU, both on the
/// rig clock, in time order.
fn read_inputs(
    reader: &SessionReader,
    lidar: &RigSensor,
) -> Result<(Vec<ScanInput>, Vec<ImuSample>), StageError> {
    let imu = reader
        .rig()
        .sensors
        .iter()
        .find(|s| s.kind == RigSensorKind::Imu);
    let lidar_topic = topics::lidar_points(&lidar.id);
    let imu_topic = imu.map(|s| topics::imu(&s.id));
    let mut scans = Vec::new();
    let mut samples = Vec::new();
    let mut failed = None;
    reader.for_each_message(|m| {
        if m.topic == lidar_topic {
            let pc = m.decode::<PointCloud>()?;
            match session_data::decode_points(&pc) {
                Ok(pts) => scans.push(ScanInput {
                    t_ns: shift(m.t_ns, lidar.time_offset_ns),
                    points: pts.iter().map(|(p, _)| p.map(|v| v as f32)).collect(),
                    t_offset_ns: pts.iter().map(|(_, dt)| *dt as u32).collect(),
                }),
                Err(e) => failed = Some(e),
            }
        } else if imu_topic.as_deref() == Some(m.topic) {
            let offset = imu.map_or(0, |s| s.time_offset_ns);
            for s in m.decode::<ImuBatch>()?.samples {
                if let (Ok(gyro), Ok(accel)) = (
                    <[f64; 3]>::try_from(s.gyro.as_slice()),
                    <[f64; 3]>::try_from(s.accel.as_slice()),
                ) {
                    samples.push(ImuSample {
                        t_ns: shift(s.t_ns, offset),
                        gyro,
                        accel,
                    });
                }
            }
        }
        Ok(())
    })?;
    if let Some(e) = failed {
        return Err(e);
    }
    scans.sort_by_key(|s| s.t_ns);
    samples.sort_by_key(|s| s.t_ns);
    Ok((scans, samples))
}

/// `T_local_lidar` of one pose: level frame, `Rz(yaw) Ry(pitch) Rx(roll)`.
fn lidar_pose(p: &PoseOut) -> Mat4 {
    let (sr, cr) = p.roll.sin_cos();
    let (sp, cp) = p.pitch.sin_cos();
    let (sy, cy) = p.yaw.sin_cos();
    [
        cy * cp,
        cy * sp * sr - sy * cr,
        cy * sp * cr + sy * sr,
        p.x,
        sy * cp,
        sy * sp * sr + cy * cr,
        sy * sp * cr - cy * sr,
        p.y,
        -sp,
        cp * sr,
        cp * cr,
        0.0,
        0.0,
        0.0,
        0.0,
        1.0,
    ]
}

/// Body poses of the LiDAR poses (see the module docs), every one at the
/// first pose's height.
pub fn body_trajectory(poses: &[PoseOut], t_body_lidar: &Mat4) -> Vec<(u64, Mat4)> {
    let inv = geom::inverse_rigid(t_body_lidar);
    let mut out: Vec<(u64, Mat4)> = poses
        .iter()
        .map(|p| {
            (
                p.t_ns,
                geom::mul(&geom::mul(t_body_lidar, &lidar_pose(p)), &inv),
            )
        })
        .collect();
    if let Some(z0) = out.first().map(|(_, m)| m[11]) {
        for (_, m) in &mut out {
            m[11] = z0;
        }
    }
    out
}

/// Run the planar matcher for the session and re-pose the dataset from its
/// trajectory. `None` when the LiDAR recorded no scans (the dataset is left
/// as prepared).
pub fn run(
    reader: &SessionReader,
    job_dir: &Path,
    dataset: &Path,
    progress: &mut dyn FnMut(f32),
) -> Result<Option<Scan2dSummary>, StageError> {
    let rig = reader.rig();
    let lidar =
        rig_planar_lidar(rig).ok_or_else(|| StageError::input("the rig has no planar LiDAR"))?;
    let model = lidar.lidar.as_ref().expect("planar LiDAR has a model");
    let dir = job_dir.join(SCAN2D_DIR);
    if dir.exists() {
        fs::remove_dir_all(&dir)?;
    }
    fs::create_dir_all(&dir)?;
    let (scans, imu) = read_inputs(reader, lidar)?;
    if scans.is_empty() {
        tracing::warn!(lidar = %lidar.id, "the planar LiDAR recorded no scans; scan matching skipped");
        return Ok(None);
    }
    progress(0.1);
    let t_body_lidar = lidar.t_body_sensor.0;
    let mut cfg = Scan2dConfig::for_sensor(
        model.min_range_m,
        model.max_range_m,
        &model.blocked_azimuth_deg,
    );
    // The IMU samples are in the body frame.
    cfg.r_lidar_imu = [0, 1, 2].map(|i| [0, 1, 2].map(|j| t_body_lidar[j * 4 + i]));
    let traj = offline::run(&scans, &imu, &cfg);
    progress(0.8);
    let mut poses = body_trajectory(&traj.poses, &t_body_lidar);
    write_tum(&dir.join("trajectory.tum"), &poses)?;
    let mut summary = Scan2dSummary {
        lidar_id: lidar.id.clone(),
        scans: scans.len(),
        imu_samples: imu.len(),
        submaps: traj.submaps,
        loops_accepted: traj.loops_accepted,
        loops_rejected: traj.loops_rejected,
        mean_score: traj.poses.iter().map(|p| p.score).sum::<f64>()
            / traj.poses.len().max(1) as f64,
        align_rmse_m: None,
        used: false,
        rejected: None,
        frames_posed: 0,
        frames_recorded: 0,
        frames_dropped: 0,
        lidar_points: 0,
    };
    let track = session_data::index(reader)?.poses;
    summary.rejected = match &track {
        Some(track) if poses.len() >= 3 => {
            let (t, rmse) = align(&poses, track)?;
            summary.align_rmse_m = Some(rmse);
            for p in &mut poses {
                p.1 = geom::mul(&t, &p.1);
            }
            (rmse > MAX_ALIGN_RMSE_M).then(|| "align_rmse".to_string())
        }
        Some(_) => Some("too_few_poses".into()),
        None => max_speed(&poses)
            .filter(|m| m.0 > MAX_SPEED_MPS)
            .map(|_| "speed".to_string()),
    };
    if let Some(why) = &summary.rejected {
        tracing::warn!(reason = %why, rmse_m = ?summary.align_rmse_m, "planar LiDAR trajectory rejected; keeping the recorded poses");
    } else {
        let traj = PoseTrack {
            source: PRIOR_SOURCE.into(),
            samples: poses,
        };
        let n = rewrite_frames(dataset, rig, &traj, track.is_some())?;
        let mut sweeps = Sweeps::default();
        for s in &scans {
            let pts: Vec<([f64; 3], u64)> = s
                .points
                .iter()
                .enumerate()
                .map(|(i, p)| {
                    let dt = s.t_offset_ns.get(i).copied().unwrap_or(0) as u64;
                    (p.map(f64::from), dt)
                })
                .collect();
            sweeps.add(&traj, &t_body_lidar, s.t_ns, &pts);
        }
        sweeps.write(dataset)?;
        summary.used = true;
        summary.frames_posed = n.posed;
        summary.frames_recorded = n.recorded;
        summary.frames_dropped = n.dropped;
        summary.lidar_points = sweeps.len();
    }
    fs::write(
        dir.join("summary.json"),
        serde_json::to_vec_pretty(&summary)?,
    )?;
    progress(1.0);
    Ok(Some(summary))
}

#[cfg(test)]
#[path = "scan2d_tests.rs"]
mod tests;
