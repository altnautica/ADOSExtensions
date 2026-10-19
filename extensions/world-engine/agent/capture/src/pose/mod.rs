//! Pose providers: where the capture service gets the rig's pose.
//!
//! Every provider implements [`PoseProvider`]. A provider owns a [`PoseTrack`]
//! (a time-ordered buffer of [`PoseEstimate`]s on the host monotonic clock)
//! and the trait's default methods answer from it:
//!
//! - [`PoseProvider::pose_at`] interpolates between the two estimates that
//!   bracket `t_ns` (quaternion slerp + linear translation). A time before the
//!   first or after the last estimate is answered only when it is within
//!   [`MAX_EXTRAPOLATION_NS`] (50 ms) of that estimate, and then with that
//!   estimate unchanged; anything further is refused (`None`), never
//!   extrapolated.
//! - [`PoseProvider::drain_new`] hands the recorder each estimate once.
//!
//! Implementations:
//! - [`fc_ekf::FcEkfPose`]: the flight controller's EKF (ATTITUDE_QUATERNION +
//!   LOCAL_POSITION_NED + GPS_GLOBAL_ORIGIN), placed in the session ENU frame
//!   with exact WGS-84 geodesy. Outdoor drone default.
//! - [`vio::VioPose`]: Basalt stereo-inertial VIO as a supervised subprocess.
//!   Indoor Visor default.
//! - [`lidar::LidarPose`]: KISS-ICP LiDAR odometry as a supervised
//!   subprocess, with the ICP Hessian eigenvalues for corridor degeneracy.
//! - [`replay::ReplayPose`]: a fixed estimate list, for tests and SITL.
//!
//! All times here are host `CLOCK_MONOTONIC` nanoseconds, the sensor bus and
//! framebus timebase; the recorder maps them onto the session timebase.

use std::collections::VecDeque;

use ados_capture_contract::v1::ImuBatch;
use parking_lot::Mutex;
use world_engine_protocol::atlas::{PoseSource, VioHealth, POSE_COV_LEN};

use crate::frame_source::CapturedFrame;
use crate::lidar_scan::LidarScan;

pub mod fc_ekf;
pub mod lidar;
pub mod replay;
pub mod se3;
pub mod vio;

/// The furthest a pose is answered beyond the first or last estimate.
pub const MAX_EXTRAPOLATION_NS: u64 = 50_000_000;

/// How much history a [`PoseTrack`] keeps.
pub const TRACK_HISTORY_NS: u64 = 30_000_000_000;

/// One pose: `T_world_body` (world = session ENU, body = FLU) at `t_ns`.
#[derive(Debug, Clone, PartialEq)]
pub struct PoseEstimate {
    /// Host monotonic nanoseconds.
    pub t_ns: u64,
    /// Row-major rotation world-from-body.
    pub r: [f64; 9],
    /// Body origin in the world frame, metres.
    pub t: [f64; 3],
    /// Row-major 6x6 covariance over (x, y, z, rot_x, rot_y, rot_z).
    pub cov: [f64; POSE_COV_LEN],
    pub source: PoseSource,
    pub health: VioHealth,
}

/// Producer-side quality signals the HUD shows beside the pose.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct PoseQuality {
    /// Features tracked in the last frame (VIO).
    pub tracked_features: Option<u32>,
    /// Smallest / largest eigenvalue of the translational ICP Hessian (LiDAR)
    /// or of the tracked-feature spread (VIO); small = degenerate.
    pub degeneracy_ratio: Option<f64>,
    /// Producer's own drift estimate, metres.
    pub drift_hint_m: Option<f64>,
}

/// A diagonal covariance from a position and an orientation sigma.
pub fn diagonal_cov(position_sigma_m: f64, orientation_sigma_rad: f64) -> [f64; POSE_COV_LEN] {
    let mut cov = [0.0; POSE_COV_LEN];
    for i in 0..3 {
        cov[i * 6 + i] = position_sigma_m * position_sigma_m;
        cov[(i + 3) * 6 + i + 3] = orientation_sigma_rad * orientation_sigma_rad;
    }
    cov
}

/// The wire string of a pose source (`/pose/<source>`, `pose.source`).
pub fn source_str(source: PoseSource) -> &'static str {
    match source {
        PoseSource::FcEkf => "fc_ekf",
        PoseSource::Vio => "vio",
        PoseSource::Lidar => "lidar",
        PoseSource::Replay => "replay",
    }
}

/// The `PoseCov.health` string.
pub fn health_str(health: VioHealth) -> &'static str {
    match health {
        VioHealth::Good => "good",
        VioHealth::Degraded => "degraded",
        VioHealth::Lost => "lost",
    }
}

/// A time-ordered estimate buffer with interpolation and a drain cursor.
#[derive(Debug, Default)]
pub struct PoseTrack {
    inner: Mutex<TrackInner>,
}

#[derive(Debug, Default)]
struct TrackInner {
    poses: VecDeque<PoseEstimate>,
    /// Estimates not yet drained, by count from the back.
    undrained: usize,
    quality: PoseQuality,
    health: Option<VioHealth>,
}

impl PoseTrack {
    pub fn new() -> Self {
        Self::default()
    }

    /// Append an estimate. One older than the newest is dropped (producers
    /// emit in time order; an out-of-order sample is a producer restart and
    /// must not reorder history).
    pub fn push(&self, est: PoseEstimate) {
        let mut g = self.inner.lock();
        if g.poses.back().is_some_and(|b| est.t_ns <= b.t_ns) {
            return;
        }
        let horizon = est.t_ns.saturating_sub(TRACK_HISTORY_NS);
        g.health = Some(est.health);
        g.poses.push_back(est);
        g.undrained = (g.undrained + 1).min(g.poses.len());
        while g.poses.front().is_some_and(|p| p.t_ns < horizon) {
            g.poses.pop_front();
        }
        g.undrained = g.undrained.min(g.poses.len());
    }

    /// Forget every estimate (a producer restart: its frame changed).
    pub fn reset(&self) {
        let mut g = self.inner.lock();
        g.poses.clear();
        g.undrained = 0;
    }

    pub fn set_quality(&self, q: PoseQuality) {
        self.inner.lock().quality = q;
    }

    pub fn quality(&self) -> PoseQuality {
        self.inner.lock().quality
    }

    /// Force the reported health (a dead producer is `Lost` whatever its last
    /// estimate said).
    pub fn set_health(&self, h: VioHealth) {
        self.inner.lock().health = Some(h);
    }

    pub fn health(&self) -> VioHealth {
        self.inner.lock().health.unwrap_or(VioHealth::Lost)
    }

    pub fn latest(&self) -> Option<PoseEstimate> {
        self.inner.lock().poses.back().cloned()
    }

    /// Move every estimate not yet drained into `out`, oldest first.
    pub fn drain_new(&self, out: &mut Vec<PoseEstimate>) {
        let mut g = self.inner.lock();
        let n = g.poses.len();
        out.extend(g.poses.range(n - g.undrained..).cloned());
        g.undrained = 0;
    }

    /// The pose at `t_ns`; see the module docs for the extrapolation rule.
    pub fn pose_at(&self, t_ns: u64) -> Option<PoseEstimate> {
        let g = self.inner.lock();
        interpolate(&g.poses, t_ns)
    }
}

/// Interpolate a time-ordered buffer at `t_ns`.
pub fn interpolate(poses: &VecDeque<PoseEstimate>, t_ns: u64) -> Option<PoseEstimate> {
    let first = poses.front()?;
    let last = poses.back()?;
    if t_ns <= first.t_ns {
        return (first.t_ns - t_ns <= MAX_EXTRAPOLATION_NS).then(|| at(first, t_ns));
    }
    if t_ns >= last.t_ns {
        return (t_ns - last.t_ns <= MAX_EXTRAPOLATION_NS).then(|| at(last, t_ns));
    }
    // First index whose time is >= t_ns; it has a predecessor (t > first).
    let hi = poses.partition_point(|p| p.t_ns < t_ns);
    let (a, b) = (&poses[hi - 1], &poses[hi]);
    let span = (b.t_ns - a.t_ns) as f64;
    let s = (t_ns - a.t_ns) as f64 / span;
    let q = se3::slerp(se3::quat_from_matrix(&a.r), se3::quat_from_matrix(&b.r), s);
    let mut cov = [0.0; POSE_COV_LEN];
    for (i, c) in cov.iter_mut().enumerate() {
        // The larger of the two: interpolation adds no information.
        *c = if a.cov[i].abs() >= b.cov[i].abs() {
            a.cov[i]
        } else {
            b.cov[i]
        };
    }
    Some(PoseEstimate {
        t_ns,
        r: se3::matrix_from_quat(q),
        t: [
            a.t[0] + s * (b.t[0] - a.t[0]),
            a.t[1] + s * (b.t[1] - a.t[1]),
            a.t[2] + s * (b.t[2] - a.t[2]),
        ],
        cov,
        source: b.source,
        health: worse(a.health, b.health),
    })
}

fn at(p: &PoseEstimate, t_ns: u64) -> PoseEstimate {
    PoseEstimate { t_ns, ..p.clone() }
}

fn worse(a: VioHealth, b: VioHealth) -> VioHealth {
    let rank = |h| match h {
        VioHealth::Good => 0,
        VioHealth::Degraded => 1,
        VioHealth::Lost => 2,
    };
    if rank(a) >= rank(b) {
        a
    } else {
        b
    }
}

/// Where the capture service gets the rig pose. Inputs are pushed in with the
/// `feed_*` methods (a provider ignores what it does not use); outputs are
/// read through the provider's [`PoseTrack`].
pub trait PoseProvider: Send + Sync {
    fn source(&self) -> PoseSource;
    fn track(&self) -> &PoseTrack;

    /// One camera frame (VIO consumes the stereo pair).
    fn feed_frame(&self, _frame: &CapturedFrame) {}
    /// One IMU batch from the sensor bus, host monotonic times, body FLU.
    fn feed_imu(&self, _src: &str, _batch: &ImuBatch) {}
    /// One LiDAR scan.
    fn feed_lidar(&self, _scan: &LidarScan) {}

    fn pose_at(&self, t_ns: u64) -> Option<PoseEstimate> {
        self.track().pose_at(t_ns)
    }
    fn latest(&self) -> Option<PoseEstimate> {
        self.track().latest()
    }
    fn health(&self) -> VioHealth {
        self.track().health()
    }
    fn quality(&self) -> PoseQuality {
        self.track().quality()
    }
    fn drain_new(&self, out: &mut Vec<PoseEstimate>) {
        self.track().drain_new(out)
    }
}

#[cfg(test)]
mod tests;
