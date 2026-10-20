//! Submaps and the scan records the back end re-optimises.

use crate::config::Scan2dConfig;
use crate::grid::Grid;
use crate::pose::Pose2;
use crate::scan::Scan2d;

/// A probability grid built from consecutive scans, in its own frame. `pose`
/// places that frame in the local SLAM frame; its origin is the pose of the
/// scan that started it.
#[derive(Debug, Clone)]
pub struct Submap {
    pub pose: Pose2,
    pub(crate) grid: Grid,
    /// Sensor positions of the inserted scans, submap frame.
    stations: Vec<[f64; 2]>,
}

impl Submap {
    pub(crate) fn new(pose: Pose2, cfg: &Scan2dConfig) -> Submap {
        Submap {
            pose,
            grid: Grid::new(cfg),
            stations: Vec::new(),
        }
    }

    /// Insert `scan` taken at local-frame pose `at`.
    pub(crate) fn insert(&mut self, scan: &Scan2d, at: Pose2) {
        let rel = self.pose.inverse() * at;
        let pts: Vec<[f64; 2]> = scan
            .points
            .iter()
            .map(|p| rel.apply([p[0] as f64, p[1] as f64]))
            .collect();
        self.grid.insert([rel.x, rel.y], &pts);
        self.stations.push([rel.x, rel.y]);
    }

    /// Scans inserted so far.
    pub fn scans(&self) -> usize {
        self.stations.len()
    }

    /// Distance from submap-frame point `p` to the nearest position a scan
    /// of this submap was taken from (infinite for an empty submap).
    pub fn distance_to_scans(&self, p: [f64; 2]) -> f64 {
        self.stations
            .iter()
            .map(|s| (s[0] - p[0]).hypot(s[1] - p[1]))
            .fold(f64::INFINITY, f64::min)
    }

    pub fn grid(&self) -> &Grid {
        &self.grid
    }
}

/// One matched scan.
#[derive(Debug, Clone)]
pub struct Node {
    pub t_ns: u64,
    /// Index of the submap the scan was matched against.
    pub submap: usize,
    /// Pose in that submap's frame.
    pub in_submap: Pose2,
    pub roll: f64,
    pub pitch: f64,
    pub score: f64,
    pub degeneracy: f64,
    /// The scan, kept when it was inserted into the submaps.
    pub scan: Option<Scan2d>,
}
