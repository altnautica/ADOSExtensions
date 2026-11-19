//! The flight controller as the planar LiDAR pose's IMU
//! (`navigation.lidar.imu = "fc"`): the attitude stand-in before `RAW_IMU`,
//! the poses held to the feed's health, and what the map status and the log
//! report of it.

use std::time::Duration;

use ados_capture_contract::v1::ImuBatch;
use ados_mapping_pose::PoseEstimate;
use ados_mapping_protocol::capture::{ImuFeedReason, ImuFeedState, ImuFeedStatus, PoseSource};

use super::Lane;
use crate::nav::fc_imu;

/// How often `nav_imu_feed` reports the FC IMU feed.
const NAV_FEED_LOG_EVERY: Duration = Duration::from_secs(10);

/// The scans since the last `nav_imu_feed` log and, for each, the gap
/// between its last point and the newest FC IMU sample handed to the pose
/// provider when the scan reached it.
pub(super) struct NavFeedStats {
    pub(super) scans: u64,
    pub(super) gaps_ns: Vec<i64>,
    since: std::time::Instant,
}

impl Default for NavFeedStats {
    fn default() -> Self {
        Self {
            scans: 0,
            gaps_ns: Vec::with_capacity(128),
            since: std::time::Instant::now(),
        }
    }
}

impl NavFeedStats {
    /// The median and the widest gap, ms (`None` before any).
    fn gaps_ms(&mut self) -> Option<(f64, f64)> {
        self.gaps_ns.sort_unstable();
        let p50 = *self.gaps_ns.get(self.gaps_ns.len() / 2)?;
        let max = *self.gaps_ns.last()?;
        Some((p50 as f64 / 1e6, max as f64 / 1e6))
    }
}

impl Lane {
    /// The FC IMU feeds this run's pose: `navigation.lidar.imu = "fc"` and
    /// the provider running is the planar LiDAR one, the only LiDAR pose that
    /// levels with an IMU. KISS-ICP on a 3D LiDAR, VIO, or no provider at all
    /// has no FC IMU feed, whatever the setting names.
    pub(super) fn fc_feeds_lidar_pose(&self) -> bool {
        self.cfg.lidar.imu == fc_imu::FC_IMU_ID
            && self
                .pose
                .as_ref()
                .is_some_and(|p| p.source() == PoseSource::Lidar)
            && self.lidar_planar
    }

    /// Hand the FC feed this run's sink ([`fc_imu::sink`]): every EKF height,
    /// and with the FC feeding the LiDAR pose every `RAW_IMU` sample and flow
    /// velocity, reach the pose provider as their frames arrive.
    pub(super) fn attach_fc_nav(&self) {
        if let Some(pose) = &self.pose {
            let sink = fc_imu::sink(
                pose.clone(),
                self.fc_feeds_lidar_pose(),
                self.fc_nav.clone(),
            );
            self.fc.set_nav_sink(Some(sink));
        }
    }

    pub(super) fn detach_fc_nav(&self) {
        self.fc.set_nav_sink(None);
    }

    /// With the FC feeding the LiDAR pose and until the FC streams `RAW_IMU`,
    /// the newest FC attitude turned into an IMU sample ([`fc_imu::sample`]).
    pub(super) fn feed_fc_attitude(&mut self) {
        if !self.fc_feeds_lidar_pose() || self.fc_nav.raw_imu() {
            return;
        }
        let Some(cur) = self
            .fc
            .attitude()
            .map(|(t_ns, r)| (t_ns, fc_imu::enu_flu(&r)))
        else {
            return;
        };
        if self.fc_imu_prev.is_some_and(|(t, _)| t == cur.0) {
            return;
        }
        if let (Some(s), Some(pose)) = (fc_imu::sample(self.fc_imu_prev, cur), &self.pose) {
            pose.feed_imu(fc_imu::FC_IMU_ID, &ImuBatch { samples: vec![s] });
            self.fc_nav.note_imu(cur.0);
        }
        self.fc_imu_prev = Some(cur);
    }

    /// With the FC feeding the LiDAR pose, the pose follows the FC IMU feed
    /// ([`fc_imu::FeedState::degraded`], [`fc_imu::hold_poses_to_feed`]).
    pub(super) fn degrade_on_imu_feed(&mut self, poses: &mut [PoseEstimate], now_ns: u64) {
        if !self.fc_feeds_lidar_pose() {
            return;
        }
        let degraded = self.fc_nav.degraded(now_ns);
        if degraded != self.imu_feed_degraded {
            match degraded {
                Some(why) => tracing::warn!(
                    reason = why.as_str(),
                    pose = match why.state() {
                        ImuFeedState::Lost => "lost",
                        _ => "degraded",
                    },
                    "nav_imu_feed_degraded"
                ),
                None => tracing::info!("nav_imu_feed_ok"),
            }
            self.imu_feed_degraded = degraded;
        }
        let raw_since = self.fc_nav.raw_since_ns();
        if let Some(t) = raw_since {
            // The matcher started a new map at the first RAW_IMU sample (the
            // stand-in's could not hold a heading): anchor it afresh.
            self.nav.restart_heading_at(t);
        }
        fc_imu::hold_poses_to_feed(poses, degraded, raw_since);
    }

    /// At most every [`NAV_FEED_LOG_EVERY`], with the FC feeding the LiDAR
    /// pose: the IMU rate, the scan-to-IMU gaps and the `RAW_IMU` waits.
    pub(super) fn log_nav_feed(&mut self) {
        if !self.fc_feeds_lidar_pose() {
            return;
        }
        let elapsed_s = self.nav_feed.since.elapsed().as_secs_f64();
        if elapsed_s < NAV_FEED_LOG_EVERY.as_secs_f64() {
            return;
        }
        let mut f = std::mem::take(&mut self.nav_feed);
        let (p50, max) = f.gaps_ms().unzip();
        let (raw_gaps, raw_gap_max_ms) = self.fc_nav.take_raw_gaps();
        tracing::info!(
            imu_hz = self.fc_nav.take_imu_samples() as f64 / elapsed_s,
            scans = f.scans,
            p50_scan_imu_gap_ms = p50,
            max_scan_imu_gap_ms = max,
            raw_imu_gaps_over_100ms = raw_gaps,
            raw_imu_gap_max_ms = raw_gap_max_ms,
            degraded = self.imu_feed_degraded.map(ImuFeedReason::as_str),
            "nav_imu_feed"
        );
    }

    /// [`ados_mapping_protocol::capture::MapStatus::imu_feed`]: with the FC
    /// feeding the LiDAR pose, whether its feed measures the heading, why
    /// not, and the `RAW_IMU` rate; `None` otherwise.
    pub(super) fn imu_feed_status(&self) -> Option<ImuFeedStatus> {
        if !self.fc_feeds_lidar_pose() {
            return None;
        }
        let now = ados_mapping_pose::clock::mono_ns();
        let unconfirmed = self
            .pose
            .as_ref()
            .is_some_and(|p| p.quality().heading_unconfirmed);
        let reason = self
            .fc_nav
            .degraded(now)
            .or(self.nav_imu_reason(unconfirmed));
        Some(ImuFeedStatus {
            state: reason.map_or(ImuFeedState::Ok, ImuFeedReason::state),
            reason,
            rate_hz: self.fc_nav.raw_rate_hz(now),
        })
    }

    /// Why the pose is lost with the FC IMU feed measuring: the map was
    /// restarted without the FC's turn and has no north, or the matcher has
    /// yet to re-confirm a heading the feed's gap left unmeasured.
    pub(super) fn nav_imu_reason(&self, unconfirmed: bool) -> Option<ImuFeedReason> {
        if self.nav.north_unknown() {
            Some(ImuFeedReason::NorthUnknown)
        } else {
            unconfirmed.then_some(ImuFeedReason::HeadingUnconfirmed)
        }
    }
}
