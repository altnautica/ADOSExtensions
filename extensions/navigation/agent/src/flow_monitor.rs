//! The flight controller's optical flow against its own EKF velocity, in
//! flight.
//!
//! Under external navigation the EKF follows the pose it is sent. A pose
//! that stops moving while the vehicle flies on (a scan matcher along a
//! featureless corridor with nothing measuring the motion along it) pulls
//! the EKF velocity down with it, the position controller asks for more
//! speed, and the vehicle runs away on an estimate that looks calm. The FC's
//! own downward flow still measures the ground going by. This monitor
//! integrates both over [`WINDOW_NS`]: the flow's rate times its ground
//! distance, turned to north-east by the FC's yaw, against
//! `LOCAL_POSITION_NED`'s velocity. When the two displacements part by more
//! than [`DISAGREE_M`] the pose is degraded (`ekf_flow_disagree`); held for
//! [`LOST_AFTER_NS`] it is lost, and the source-set machine falls back to
//! the flow set. It clears once they agree within [`CLEAR_M`] for
//! [`CLEAR_AFTER_NS`].
//!
//! Flow of any quality above 0 is used, as EKF3 uses it: a sensor over
//! dusty or bare ground reads a low quality at a sound rate.

use std::collections::VecDeque;

use crate::external_nav::{ExtnavHealth, ImuFeed, PoseReason};

/// The window the displacements are compared over.
pub const WINDOW_NS: i64 = 3_000_000_000;
/// Displacements further apart than this, metres, disagree.
pub const DISAGREE_M: f64 = 1.5;
/// A disagreement held this long makes the pose lost.
pub const LOST_AFTER_NS: i64 = 3_000_000_000;
/// The disagreement clears once the displacements have stayed within this,
/// metres, ...
pub const CLEAR_M: f64 = 1.0;
/// ... for this long.
pub const CLEAR_AFTER_NS: i64 = 3_000_000_000;
/// A gap in the flow longer than this starts the window over.
const FLOW_GAP_NS: i64 = 500_000_000;
/// An EKF velocity or yaw older than this at a flow sample is not used.
const STALE_NS: i64 = 500_000_000;

/// The monitor's verdict.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum FlowEkf {
    /// Agreeing, or not enough flow and EKF velocity to tell.
    #[default]
    Agree,
    /// Parted by more than [`DISAGREE_M`] over the window.
    Disagree,
    /// Parted for [`LOST_AFTER_NS`].
    Lost,
}

impl FlowEkf {
    /// The `degradedReasons` / `nav.state.health_reason` key while not
    /// agreeing.
    pub const REASON: &'static str = "ekf_flow_disagree";

    /// The pose's health with this verdict over it: a disagreement degrades
    /// a good pose, a held one loses it (no `ODOMETRY`, and the source-set
    /// machine falls back).
    pub fn over(self, health: ExtnavHealth) -> ExtnavHealth {
        match (self, health) {
            (FlowEkf::Lost, _) => ExtnavHealth::Lost,
            (FlowEkf::Disagree, ExtnavHealth::Good) => ExtnavHealth::Degraded,
            _ => health,
        }
    }
}

/// Every reason key the external-nav tick reports, so a stale one is
/// dropped from `degradedReasons` before the current ones go in.
pub const NAV_REASONS: [&str; 12] = [
    FlowEkf::REASON,
    "corridor_unmeasured",
    "flow_disagrees",
    "heading_unconfirmed",
    "pose_sigma_wide",
    "map_warmup",
    "scan_unmatched",
    "corridor_flow",
    "corridor_flow_unjudged",
    "imu_feed_attitude_stand_in",
    "imu_feed_stalled",
    "imu_feed_starved",
];

/// Why the pose is not good, for `degradedReasons` and
/// `nav.state.health_reason`, one key. Precedence, most direct evidence
/// that the pose is not following the vehicle first:
/// 1. the monitor's verdict (`ekf_flow_disagree`);
/// 2. the publisher's own estimator reason (`corridor_unmeasured`,
///    `flow_disagrees`);
/// 3. the publisher's FC IMU feed (`heading_unconfirmed`, or
///    `imu_feed_<reason>`: `attitude_stand_in`, `stalled`, `starved`).
///
/// `None` while there is none.
pub fn health_reason(
    monitor: FlowEkf,
    pose: Option<PoseReason>,
    imu: Option<ImuFeed>,
) -> Option<&'static str> {
    match monitor {
        FlowEkf::Disagree | FlowEkf::Lost => Some(FlowEkf::REASON),
        // A routine corridor or warm-up reason yields to the IMU feed's.
        FlowEkf::Agree => {
            let routine = |r: &PoseReason| {
                matches!(
                    r,
                    PoseReason::CorridorFlow
                        | PoseReason::CorridorFlowUnjudged
                        | PoseReason::ScanUnmatched
                        | PoseReason::MapWarmup
                )
            };
            pose.filter(|r| !routine(r))
                .map(PoseReason::as_str)
                .or(imu.map(ImuFeed::reason_key))
                .or(pose.map(PoseReason::as_str))
        }
    }
}

#[derive(Debug, Default)]
pub struct FlowEkfMonitor {
    /// `(t, vn, ve)` of the newest `LOCAL_POSITION_NED`.
    ekf: Option<(i64, f64, f64)>,
    /// `(t, yaw)` of the newest `ATTITUDE`, radians clockwise from north.
    yaw: Option<(i64, f64)>,
    /// `(t, flow displacement, EKF displacement)`, north-east metres,
    /// cumulative from the window's start, oldest first.
    track: VecDeque<(i64, [f64; 2], [f64; 2])>,
    /// When the current disagreement began.
    disagree_since: Option<i64>,
    /// When the displacements last came within [`CLEAR_M`] during a
    /// disagreement.
    agree_since: Option<i64>,
    state: FlowEkf,
}

impl FlowEkfMonitor {
    /// `LOCAL_POSITION_NED` velocity, north and east, m/s.
    pub fn on_velocity(&mut self, vn: f32, ve: f32, now_ns: i64) {
        self.ekf = Some((now_ns, f64::from(vn), f64::from(ve)));
    }

    /// `ATTITUDE.yaw`, radians.
    pub fn on_yaw(&mut self, yaw: f32, now_ns: i64) {
        self.yaw = Some((now_ns, f64::from(yaw)));
    }

    /// One `OPTICAL_FLOW`: the body-rate compensated flow about body x and
    /// y (FRD), rad/s, its quality and ground distance, metres (0 or less:
    /// none). Moving forward at `v` over ground `r` reads `comp_y = v / r`;
    /// moving right reads `comp_x = −v / r`.
    pub fn on_flow(&mut self, comp: [f32; 2], quality: u8, ground_m: f32, now_ns: i64) {
        let fresh = |t: i64| now_ns - t <= STALE_NS;
        let (Some((te, vn, ve)), Some((ty, yaw))) = (self.ekf, self.yaw) else {
            return;
        };
        let range = f64::from(ground_m);
        let usable = quality > 0
            && range > 0.0
            && comp.iter().all(|c| c.is_finite())
            && fresh(te)
            && fresh(ty);
        if !usable {
            return;
        }
        let forward = f64::from(comp[1]) * range;
        let right = -f64::from(comp[0]) * range;
        let (s, c) = yaw.sin_cos();
        let flow = [c * forward - s * right, s * forward + c * right];
        let (cum_flow, cum_ekf) = match self.track.back() {
            Some(&(t, f, e)) if now_ns - t <= FLOW_GAP_NS && now_ns > t => {
                let dt = (now_ns - t) as f64 * 1e-9;
                (
                    [f[0] + flow[0] * dt, f[1] + flow[1] * dt],
                    [e[0] + vn * dt, e[1] + ve * dt],
                )
            }
            Some(&(t, _, _)) if now_ns <= t => return,
            _ => {
                self.track.clear();
                ([0.0; 2], [0.0; 2])
            }
        };
        self.track.push_back((now_ns, cum_flow, cum_ekf));
        while self
            .track
            .get(1)
            .is_some_and(|&(t, _, _)| now_ns - t >= WINDOW_NS)
        {
            self.track.pop_front();
        }
        let &(t0, f0, e0) = self.track.front().expect("pushed");
        if now_ns - t0 < WINDOW_NS {
            return;
        }
        let gap = ((cum_flow[0] - f0[0]) - (cum_ekf[0] - e0[0]))
            .hypot((cum_flow[1] - f0[1]) - (cum_ekf[1] - e0[1]));
        self.judge(gap, now_ns);
    }

    fn judge(&mut self, gap: f64, now_ns: i64) {
        if gap > DISAGREE_M {
            self.disagree_since.get_or_insert(now_ns);
            self.agree_since = None;
        } else if self.disagree_since.is_some() {
            if gap <= CLEAR_M {
                let since = *self.agree_since.get_or_insert(now_ns);
                if now_ns - since >= CLEAR_AFTER_NS {
                    self.disagree_since = None;
                    self.agree_since = None;
                }
            } else {
                self.agree_since = None;
            }
        }
        self.state = match self.disagree_since {
            None => FlowEkf::Agree,
            Some(s) if self.state == FlowEkf::Lost || now_ns - s >= LOST_AFTER_NS => FlowEkf::Lost,
            Some(_) => FlowEkf::Disagree,
        };
    }

    /// The verdict as of the newest flow sample.
    pub fn state(&self) -> FlowEkf {
        self.state
    }

    /// A new flight-controller boot: nothing carries over.
    pub fn reset(&mut self) {
        *self = Self::default();
    }
}

#[cfg(test)]
#[path = "flow_monitor_tests.rs"]
mod tests;
