//! The shared topics `navigation-lidar-slam` publishes through the host, and
//! the capability a subscriber needs for each (the manifest's
//! `contributes.shared_topics`).

/// The pose in the north-aligned `map_ned` (`PoseDescriptor`).
pub const POSE: &str = "plugin.navigation.pose";
/// Each LiDAR scan binned in the body frame (`ScanDescriptor`).
pub const SCAN: &str = "plugin.navigation.scan";
/// The live map's state, 1 Hz (`MapStatus`).
pub const MAP_STATUS: &str = "plugin.navigation.map_status";
/// The configured LiDAR's link status (`LidarStatus`).
pub const LIDAR: &str = "plugin.navigation.lidar";
/// Changed live occupancy tiles, 1 Hz (`LiveOccupancy`).
pub const OCCUPANCY: &str = "plugin.navigation.occupancy";
/// Each placed planar scan with its sensor pose (`PolarScan`), while the
/// map is built on a node (`navigation.map_placement = stream`).
pub const SCAN_POLAR: &str = "plugin.navigation.scan_polar";
/// Each LiDAR scan as measured, in parts (`ScanChunk`), while a capture
/// session records.
pub const SCAN_RAW: &str = "plugin.navigation.scan_raw";
/// Every pose estimate, batched (`PoseTrackBatch`).
pub const POSE_TRACK: &str = "plugin.navigation.pose_track";

/// What the capture service reports its session state on.
pub const CAPTURE_STATE: &str = "plugin.capture.state";

/// The capability gating the live map and the full-rate streams; declared by
/// this extension.
pub const MAP_READ_CAP: &str = "plugin.navigation.map.read";
/// The capability gating the flight topics: the same class of data as the
/// vehicle telemetry a plugin reads.
pub const TELEMETRY_READ_CAP: &str = "telemetry.read";

/// Every topic this service publishes.
pub const ALL: [&str; 8] = [
    POSE, SCAN, MAP_STATUS, LIDAR, OCCUPANCY, SCAN_POLAR, SCAN_RAW, POSE_TRACK,
];

/// The capability a subscriber needs for `topic`, by exact match; `None` for
/// a topic this service does not publish.
pub fn subscribe_capability(topic: &str) -> Option<&'static str> {
    match topic {
        POSE | SCAN | MAP_STATUS | LIDAR => Some(TELEMETRY_READ_CAP),
        OCCUPANCY | SCAN_POLAR | SCAN_RAW | POSE_TRACK => Some(MAP_READ_CAP),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_topic_is_gated_and_only_by_exact_match() {
        for t in ALL {
            assert!(t.starts_with("plugin.navigation."), "{t}");
            assert!(subscribe_capability(t).is_some(), "{t}");
            assert_eq!(subscribe_capability(&format!("{t}.x")), None);
            assert_eq!(subscribe_capability(&t[..t.len() - 1]), None);
        }
        assert_eq!(subscribe_capability(POSE), Some("telemetry.read"));
        assert_eq!(subscribe_capability(SCAN_RAW), Some(MAP_READ_CAP));
        assert_eq!(subscribe_capability(CAPTURE_STATE), None);
    }
}
