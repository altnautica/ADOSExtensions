//! LiDAR scans waiting for their pose.
//!
//! A scan's pose describes its first return, and a provider that matches on
//! a worker thread (the planar matcher) estimates it only after the scan was
//! fed. Asked at once, the scan's time lies a revolution past the newest
//! estimate, beyond [`crate::pose::MAX_EXTRAPOLATION_NS`], and the pose is
//! refused; a scan placed then went into neither the live map, the session's
//! polar record nor the node stream (with a 10 Hz planar LiDAR, about 97 % of
//! the scans). Scans wait here in arrival order until the provider's track
//! reaches their time, or at most [`SCAN_POSE_WAIT_NS`].

use std::collections::VecDeque;

use crate::lidar_scan::LidarScan;
use crate::pose::PoseEstimate;

/// How long a scan waits for its pose before it is placed without one
/// (host monotonic nanoseconds past its first return).
pub const SCAN_POSE_WAIT_NS: u64 = 500_000_000;
/// Scans held at most; past it the oldest is placed without a pose.
pub const SCAN_WAIT_DEPTH: usize = 32;

/// The scans fed to the pose provider and not placed yet.
#[derive(Default)]
pub struct ScanWait {
    queue: VecDeque<LidarScan>,
}

impl ScanWait {
    /// Queue a scan; the oldest one when the queue was full, to be placed
    /// without waiting further.
    pub fn push(&mut self, scan: LidarScan) -> Option<LidarScan> {
        let full = (self.queue.len() >= SCAN_WAIT_DEPTH)
            .then(|| self.queue.pop_front())
            .flatten();
        self.queue.push_back(scan);
        full
    }

    pub fn len(&self) -> usize {
        self.queue.len()
    }

    pub fn is_empty(&self) -> bool {
        self.queue.is_empty()
    }

    /// The oldest scan with its pose, once it is settled: `pose_at` answers
    /// for its time, or the track's newest estimate (`newest_ns`) is at or
    /// past it (no later answer is coming), or it has waited
    /// [`SCAN_POSE_WAIT_NS`] by `now_ns` (then without a pose). `None` while
    /// the oldest still waits, so scans leave in arrival order.
    pub fn next(
        &mut self,
        now_ns: u64,
        newest_ns: Option<u64>,
        pose_at: impl Fn(u64) -> Option<PoseEstimate>,
    ) -> Option<(LidarScan, Option<PoseEstimate>)> {
        let t = self.queue.front()?.t_ns;
        let pose = pose_at(t);
        let settled = pose.is_some()
            || newest_ns.is_some_and(|n| n >= t)
            || now_ns.saturating_sub(t) >= SCAN_POSE_WAIT_NS;
        if !settled {
            return None;
        }
        self.queue.pop_front().map(|s| (s, pose))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pose::{interpolate, se3};
    use world_engine_protocol::atlas::{PoseSource, VioHealth};

    fn scan(t_ns: u64) -> LidarScan {
        LidarScan {
            sensor_id: "lidar0".into(),
            t_ns,
            ..LidarScan::default()
        }
    }

    fn estimate(t_ns: u64, x: f64) -> PoseEstimate {
        PoseEstimate {
            t_ns,
            r: se3::IDENTITY,
            t: [x, 0.0, 0.0],
            cov: [0.0; 36],
            source: PoseSource::Lidar,
            health: VioHealth::Good,
        }
    }

    const MS: u64 = 1_000_000;

    /// A 10 Hz scan arrives a revolution after its first return and its pose
    /// comes from the matcher later still: asked at arrival the track ends
    /// 100 ms before the scan (refused), so the scan waits, and leaves with
    /// its own pose once the matcher has estimated it.
    #[test]
    fn a_scan_waits_for_the_pose_the_matcher_estimates_after_it() {
        let mut track: std::collections::VecDeque<PoseEstimate> =
            [estimate(1_000 * MS, 0.0)].into();
        let mut wait = ScanWait::default();
        assert!(wait.push(scan(1_100 * MS)).is_none());
        let at = |tr: &std::collections::VecDeque<PoseEstimate>| {
            let tr = tr.clone();
            move |t| interpolate(&tr, t)
        };
        let newest = |tr: &std::collections::VecDeque<PoseEstimate>| tr.back().map(|e| e.t_ns);
        assert!(interpolate(&track, 1_100 * MS).is_none(), "the old behaviour: refused");
        assert!(wait.next(1_200 * MS, newest(&track), at(&track)).is_none());
        assert_eq!(wait.len(), 1);
        track.push_back(estimate(1_100 * MS, 0.8));
        let (s, pose) = wait
            .next(1_210 * MS, newest(&track), at(&track))
            .expect("placed once its pose exists");
        assert_eq!(s.t_ns, 1_100 * MS);
        assert_eq!(pose.expect("with its pose").t[0], 0.8);
        assert!(wait.is_empty());
    }

    #[test]
    fn a_scan_whose_pose_never_comes_is_placed_without_one_after_the_wait() {
        let mut wait = ScanWait::default();
        wait.push(scan(1_000 * MS));
        assert!(wait.next(1_000 * MS + SCAN_POSE_WAIT_NS - 1, None, |_| None).is_none());
        let (s, pose) = wait
            .next(1_000 * MS + SCAN_POSE_WAIT_NS, None, |_| None)
            .unwrap();
        assert_eq!(s.t_ns, 1_000 * MS);
        assert!(pose.is_none());
    }

    /// A scan older than the whole track (before the first estimate) never
    /// gets an answer; it leaves at once instead of waiting.
    #[test]
    fn a_scan_the_track_has_passed_leaves_at_once() {
        let mut wait = ScanWait::default();
        wait.push(scan(1_000 * MS));
        let out = wait.next(1_001 * MS, Some(2_000 * MS), |_| None).unwrap();
        assert!(out.1.is_none());
    }

    #[test]
    fn scans_leave_in_arrival_order_and_a_full_queue_hands_back_the_oldest() {
        let mut wait = ScanWait::default();
        for k in 0..SCAN_WAIT_DEPTH as u64 {
            assert!(wait.push(scan(k)).is_none());
        }
        let over = wait.push(scan(999)).expect("the oldest");
        assert_eq!(over.t_ns, 0);
        assert_eq!(wait.len(), SCAN_WAIT_DEPTH);
        let first = wait.next(0, None, |t| Some(estimate(t, 0.0))).unwrap();
        assert_eq!(first.0.t_ns, 1);
    }
}
