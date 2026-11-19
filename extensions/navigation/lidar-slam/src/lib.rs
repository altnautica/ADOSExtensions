//! Navigation's LiDAR and VIO half: the pose providers that run a sensor's own
//! odometry (planar scan matching, KISS-ICP, Basalt VIO), the LiDAR drivers
//! that feed them, and the live navigation map built from their poses.
//!
//! - [`pose`]: the LiDAR, planar and VIO providers;
//! - [`lidar_source`]: the RPLIDAR and Livox drivers behind one runner;
//! - [`nav`]: the north-aligned navigation map, live occupancy and sweep
//!   cloud, the flight controller's IMU feed and the scans waiting for their
//!   pose;
//! - [`settings`]: the provider, VIO and LiDAR settings.

pub mod lidar_source;
pub mod nav;
pub mod pose;
pub mod settings;

use std::path::PathBuf;

/// A relative path against the plugin's install directory (the process cwd).
pub fn install_path(p: &str) -> PathBuf {
    let path = PathBuf::from(p);
    if path.is_absolute() {
        path
    } else {
        std::env::current_dir().unwrap_or_default().join(path)
    }
}
