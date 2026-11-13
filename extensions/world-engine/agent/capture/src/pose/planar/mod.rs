//! [`PlanarLidarPose`]: live 2D LiDAR odometry for a planar LiDAR (one level
//! scan ring, such as an RPLIDAR), with `ados_scan2d::LocalSlam` on a worker
//! thread.
//!
//! Input: [`PoseProvider::feed_imu`] takes the configured IMU's samples (body
//! FLU), [`PoseProvider::feed_velocity`] ground velocities measured in that
//! IMU's frame (the flight controller's optical flow over its downward range,
//! which the matcher follows along a corridor the scans leave unconstrained),
//! [`PoseProvider::feed_height`] the vehicle's height and climb rate (the
//! flight controller's EKF: the matcher neither matches nor maps while the
//! scan plane climbs, and starts a new submap at a new height) and
//! [`PoseProvider::feed_lidar`] the configured LiDAR's scans. All go onto
//! one ordered channel with `try_send`, so no feeder ever blocks. At
//! most [`SCAN_QUEUE_DEPTH`] scans wait at a time; a scan past that is
//! dropped and counted in [`PlanarStats`], as is an IMU, velocity or height
//! batch on a full channel.
//!
//! Output: each scan's planar pose `(x, y, yaw)` plus the IMU's roll and
//! pitch is `T_w_l`, the LiDAR pose in the levelled LiDAR frame at the first
//! scan. The world frame is the body frame at the first scan, as for the 3D
//! provider: `T_world_body = T_body_lidar * T_w_l * inv(T_body_lidar)`. The
//! pose is single-floor: z stays at the mount height.
//!
//! Health: `Lost` until the first submap holds [`MIN_SUBMAP_SCANS`] scans;
//! `Degraded` while the match's `degeneracy` (translation `eig_min / eig_max`,
//! reported as `quality().degeneracy_ratio`) is below the matcher's
//! degenerate ratio (the pose rides a motion prior along the weak direction,
//! measured velocity or not), or the scan could not be matched (a climb
//! among them); `Good` otherwise.

#[cfg(test)]
mod tests;

use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, SyncSender, TrySendError};
use std::sync::Arc;
use std::thread::JoinHandle;

use ados_capture_contract::v1::ImuBatch;
use ados_capture_contract::{Rig, RigSensorKind};
use ados_scan2d::{
    HeightSample, ImuSample, LocalSlam, PoseOut, Scan2dConfig, ScanInput, VelocitySample,
};
use world_engine_protocol::atlas::{PoseSource, VioHealth};

use super::lidar::supervise::{nominal_cov, world_from_body};
use super::{se3, PoseEstimate, PoseProvider, PoseQuality, PoseTrack};
use crate::lidar_scan::LidarScan;

/// Scans that may wait for the matcher (~0.2 s at 10 Hz).
pub const SCAN_QUEUE_DEPTH: usize = 2;
/// IMU, velocity and height messages that may wait beside them: the FC's
/// arrive one sample a message (100 Hz `RAW_IMU`, 50 Hz flow, 20 Hz height),
/// so about 6 s of them.
pub const IMU_QUEUE_DEPTH: usize = 1024;
/// Scans the first submap holds before the pose is trusted.
pub const MIN_SUBMAP_SCANS: usize = 10;

#[derive(Debug, Clone)]
pub struct PlanarLidarConfig {
    pub rig: Rig,
    /// Rig sensor id of the planar LiDAR (with its `lidar` scan model).
    pub lidar_id: String,
    /// Sensor-bus IMU stream whose samples level the scans.
    pub imu_id: String,
}

#[derive(Debug, thiserror::Error)]
pub enum PlanarPoseError {
    #[error("rig has no lidar sensor `{0}` with a scan model")]
    NoLidar(String),
    #[error("starting the scan matcher thread: {0}")]
    Thread(std::io::Error),
}

/// Input counts since spawn.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct PlanarStats {
    /// Scans dropped with [`SCAN_QUEUE_DEPTH`] already waiting.
    pub scans_dropped: u64,
    /// Scans whose per-point vectors disagreed in length.
    pub scans_rejected: u64,
    /// IMU, velocity and height batches dropped on a full channel.
    pub imu_dropped: u64,
    /// Scans the matcher processed.
    pub scans_processed: u64,
}

#[derive(Debug, Default)]
struct Counters {
    queued_scans: AtomicUsize,
    scans_dropped: AtomicU64,
    scans_rejected: AtomicU64,
    imu_dropped: AtomicU64,
    scans_processed: AtomicU64,
}

enum Input {
    Imu(Vec<ImuSample>),
    Velocity(Vec<VelocitySample>),
    Height(Vec<HeightSample>),
    Scan(ScanInput),
}

pub struct PlanarLidarPose {
    track: Arc<PoseTrack>,
    lidar_id: String,
    imu_id: String,
    tx: Option<SyncSender<Input>>,
    counters: Arc<Counters>,
    worker: Option<JoinHandle<()>>,
}

/// Row-major `R_z(yaw) * R_y(pitch) * R_x(roll)`.
fn rotation_zyx(roll: f64, pitch: f64, yaw: f64) -> [f64; 9] {
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    [
        cy * cp,
        cy * sp * sr - sy * cr,
        cy * sp * cr + sy * sr,
        sy * cp,
        sy * sp * sr + cy * cr,
        sy * sp * cr - cy * sr,
        -sp,
        cp * sr,
        cp * cr,
    ]
}

/// `T_w_l` of one matcher pose.
pub fn t_w_l(p: &PoseOut) -> [f64; 16] {
    se3::to_mat4(&rotation_zyx(p.roll, p.pitch, p.yaw), [p.x, p.y, 0.0])
}

/// Health of one matcher pose, `inserted` scans into the first submap.
pub fn health_of(p: &PoseOut, inserted: usize, degenerate_ratio: f64) -> VioHealth {
    if inserted < MIN_SUBMAP_SCANS {
        VioHealth::Lost
    } else if p.score <= 0.0 || p.degeneracy < degenerate_ratio {
        VioHealth::Degraded
    } else {
        VioHealth::Good
    }
}

impl PlanarLidarPose {
    /// Start the matcher thread for the rig's LiDAR `cfg.lidar_id`.
    pub fn spawn(cfg: PlanarLidarConfig) -> Result<Self, PlanarPoseError> {
        let sensor = cfg
            .rig
            .sensors
            .iter()
            .find(|s| s.id == cfg.lidar_id && s.kind == RigSensorKind::Lidar)
            .filter(|s| s.lidar.is_some())
            .ok_or_else(|| PlanarPoseError::NoLidar(cfg.lidar_id.clone()))?;
        let model = sensor.lidar.as_ref().expect("filtered on a model");
        let t_body_lidar = sensor.t_body_sensor.0;
        let mut scfg = Scan2dConfig::for_sensor(
            model.min_range_m,
            model.max_range_m,
            &model.blocked_azimuth_deg,
        );
        // The bus IMU samples are in body axes: R_lidar_imu = R_lidar_body.
        let (r_body_lidar, _) = se3::from_mat4(&t_body_lidar);
        let r = se3::transpose(&r_body_lidar);
        scfg.r_lidar_imu = [[r[0], r[1], r[2]], [r[3], r[4], r[5]], [r[6], r[7], r[8]]];
        let (tx, rx) = sync_channel(SCAN_QUEUE_DEPTH + IMU_QUEUE_DEPTH);
        let track = Arc::new(PoseTrack::new());
        let counters = Arc::new(Counters::default());
        let worker = {
            let (track, counters) = (track.clone(), counters.clone());
            std::thread::Builder::new()
                .name(format!("scan2d-{}", cfg.lidar_id))
                .spawn(move || run(LocalSlam::new(scfg), rx, &track, &counters, t_body_lidar))
                .map_err(PlanarPoseError::Thread)?
        };
        Ok(Self {
            track,
            lidar_id: cfg.lidar_id,
            imu_id: cfg.imu_id,
            tx: Some(tx),
            counters,
            worker: Some(worker),
        })
    }

    pub fn stats(&self) -> PlanarStats {
        let c = &self.counters;
        PlanarStats {
            scans_dropped: c.scans_dropped.load(Ordering::Relaxed),
            scans_rejected: c.scans_rejected.load(Ordering::Relaxed),
            imu_dropped: c.imu_dropped.load(Ordering::Relaxed),
            scans_processed: c.scans_processed.load(Ordering::Relaxed),
        }
    }
}

/// How far the newest IMU sample may trail a scan's last point before the
/// scan waits for more: one 100 Hz `RAW_IMU` period.
const IMU_COVER_SLACK_NS: u64 = 10_000_000;

/// The time of a scan's last point.
fn scan_end_ns(scan: &ScanInput) -> u64 {
    scan.t_ns + u64::from(scan.t_offset_ns.iter().copied().max().unwrap_or(0))
}

/// The matcher loop: IMU and scans in arrival order until the provider drops.
/// A scan whose span the IMU has not covered yet waits (at most one scan,
/// so at most one revolution) until the IMU reaches its end, so the matcher
/// never extrapolates the rotation across it. With no IMU ever fed, scans
/// are matched at once.
fn run(
    mut slam: LocalSlam,
    rx: Receiver<Input>,
    track: &PoseTrack,
    counters: &Counters,
    t_body_lidar: [f64; 16],
) {
    let ratio = slam.config().degenerate_ratio;
    let process = |slam: &mut LocalSlam, scan: ScanInput| {
        let out = slam.feed_scan(&scan);
        let health = health_of(&out, slam.inserted_scans(), ratio);
        let (r, t) = se3::from_mat4(&world_from_body(&t_w_l(&out), &t_body_lidar));
        track.set_quality(PoseQuality {
            degeneracy_ratio: (out.score > 0.0).then_some(out.degeneracy),
            ..PoseQuality::default()
        });
        track.push(PoseEstimate {
            t_ns: out.t_ns,
            r,
            t,
            cov: nominal_cov(health),
            source: PoseSource::Lidar,
            health,
            sigma_xy_m: Some(out.sigma_xy_m),
        });
        counters.queued_scans.fetch_sub(1, Ordering::Release);
        counters.scans_processed.fetch_add(1, Ordering::Release);
    };
    let mut newest_imu: Option<u64> = None;
    let mut pending: Option<ScanInput> = None;
    while let Ok(input) = rx.recv() {
        match input {
            Input::Imu(samples) => {
                for s in &samples {
                    slam.feed_imu(s);
                    newest_imu = Some(newest_imu.map_or(s.t_ns, |n| n.max(s.t_ns)));
                }
                if let (Some(n), Some(p)) = (newest_imu, pending.as_ref()) {
                    if n + IMU_COVER_SLACK_NS >= scan_end_ns(p) {
                        let scan = pending.take().expect("checked");
                        process(&mut slam, scan);
                    }
                }
            }
            Input::Velocity(samples) => samples.iter().for_each(|s| slam.feed_velocity(s)),
            Input::Height(samples) => samples.iter().for_each(|s| slam.feed_height(s)),
            Input::Scan(scan) => {
                if let Some(p) = pending.take() {
                    process(&mut slam, p);
                }
                match newest_imu {
                    Some(n) if n + IMU_COVER_SLACK_NS < scan_end_ns(&scan) => pending = Some(scan),
                    _ => process(&mut slam, scan),
                }
            }
        }
    }
    if let Some(p) = pending.take() {
        process(&mut slam, p);
    }
}

impl Drop for PlanarLidarPose {
    /// Closing the channel ends the worker after the queued input.
    fn drop(&mut self) {
        drop(self.tx.take());
        if let Some(w) = self.worker.take() {
            let _ = w.join();
        }
    }
}

impl PoseProvider for PlanarLidarPose {
    fn source(&self) -> PoseSource {
        PoseSource::Lidar
    }

    fn track(&self) -> &PoseTrack {
        &self.track
    }

    fn single_floor(&self) -> bool {
        true
    }

    fn feed_imu(&self, src: &str, batch: &ImuBatch) {
        if src != self.imu_id {
            return;
        }
        let samples: Vec<ImuSample> = batch
            .samples
            .iter()
            .filter_map(|s| {
                Some(ImuSample {
                    t_ns: s.t_ns,
                    gyro: <[f64; 3]>::try_from(s.gyro.as_slice()).ok()?,
                    accel: <[f64; 3]>::try_from(s.accel.as_slice()).ok()?,
                })
            })
            .collect();
        if samples.is_empty() {
            return;
        }
        let Some(tx) = &self.tx else { return };
        if tx.try_send(Input::Imu(samples)).is_err() {
            self.counters.imu_dropped.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn feed_velocity(&self, src: &str, samples: &[VelocitySample]) {
        if src != self.imu_id || samples.is_empty() {
            return;
        }
        let Some(tx) = &self.tx else { return };
        if tx.try_send(Input::Velocity(samples.to_vec())).is_err() {
            self.counters.imu_dropped.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Whatever the IMU: the height is the vehicle's, not a sensor frame's.
    fn feed_height(&self, samples: &[HeightSample]) {
        if samples.is_empty() {
            return;
        }
        let Some(tx) = &self.tx else { return };
        if tx.try_send(Input::Height(samples.to_vec())).is_err() {
            self.counters.imu_dropped.fetch_add(1, Ordering::Relaxed);
        }
    }

    fn feed_lidar(&self, scan: &LidarScan) {
        if scan.sensor_id != self.lidar_id {
            return;
        }
        if scan.t_offset_ns.len() != scan.points.len() {
            self.counters.scans_rejected.fetch_add(1, Ordering::Relaxed);
            return;
        }
        let c = &self.counters;
        // Claim a queue slot first so a full queue costs no copy.
        if c.queued_scans.fetch_add(1, Ordering::Relaxed) >= SCAN_QUEUE_DEPTH {
            c.queued_scans.fetch_sub(1, Ordering::Relaxed);
            c.scans_dropped.fetch_add(1, Ordering::Relaxed);
            return;
        }
        let input = Input::Scan(ScanInput {
            t_ns: scan.t_ns,
            points: scan.points.clone(),
            t_offset_ns: scan.t_offset_ns.clone(),
        });
        let sent = match &self.tx {
            Some(tx) => match tx.try_send(input) {
                Ok(()) => true,
                Err(TrySendError::Full(_) | TrySendError::Disconnected(_)) => false,
            },
            None => false,
        };
        if !sent {
            c.queued_scans.fetch_sub(1, Ordering::Relaxed);
            c.scans_dropped.fetch_add(1, Ordering::Relaxed);
        }
    }
}
