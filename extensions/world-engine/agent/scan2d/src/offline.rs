//! Whole-session processing: local SLAM over every scan, then the back end.

use crate::attitude::ImuSample;
use crate::backend::optimize;
pub use crate::backend::Trajectory;
use crate::config::Scan2dConfig;
use crate::frontend::{LocalMap, LocalSlam};
use crate::scan::ScanInput;

/// Local SLAM over `scans` (time order) with the `imu` samples (time order)
/// up to each scan's last point fed first.
pub fn local(scans: &[ScanInput], imu: &[ImuSample], cfg: &Scan2dConfig) -> LocalMap {
    let mut slam = LocalSlam::recording(cfg.clone());
    let mut j = 0;
    for s in scans {
        let end = s.t_ns + s.t_offset_ns.iter().copied().max().unwrap_or(0) as u64;
        while j < imu.len() && imu[j].t_ns <= end {
            slam.feed_imu(&imu[j]);
            j += 1;
        }
        slam.feed_scan(s);
    }
    slam.into_map()
}

/// [`local`] followed by loop search and pose-graph optimisation.
pub fn run(scans: &[ScanInput], imu: &[ImuSample], cfg: &Scan2dConfig) -> Trajectory {
    optimize(&local(scans, imu, cfg), cfg)
}
