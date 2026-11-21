//! Navigation's LiDAR and VIO half, the extension's `lidar-slam` service:
//! the pose providers that run a sensor's own odometry (planar scan
//! matching, KISS-ICP, Basalt VIO), the LiDAR drivers that feed them, the
//! live navigation map built from their poses, and the
//! `plugin.navigation.*` topics and `/navigation/*` routes that carry them.
//!
//! - [`supervisor`]: the service's lifecycle, config poll and feeds;
//! - [`lane`]: one run, from sensors to topics;
//! - [`pose`], [`providers`]: the LiDAR, planar and VIO providers and their
//!   choice from the config and rig;
//! - [`lidar_source`]: the RPLIDAR and Livox drivers behind one runner;
//! - [`depth_source`]: the World Engine and RealSense depth cameras;
//! - [`voxfeed`]: the 3D voxel map the pilot reads, fed by the 3D LiDAR,
//!   the depth camera and (when asked) the planar LiDAR;
//! - [`nav`]: the north-aligned navigation map, live occupancy and sweep
//!   cloud, the flight controller's IMU feed and the scans waiting for their
//!   pose;
//! - [`settings`]: the `navigation.*` keys;
//! - [`topics`], [`publish`], [`capture_state`]: what is published, how it
//!   reaches the host, and when a capture session wants raw scans;
//! - [`http`], [`status`], [`sweep`]: the routes;
//! - [`timebase`]: the clock every stamp is on (sim time on a simulated
//!   node).

pub mod capture_state;
pub mod depth_source;
pub mod http;
pub mod lane;
pub mod lidar_source;
pub mod nav;
pub mod pose;
pub mod providers;
pub mod publish;
pub mod settings;
pub mod status;
pub mod supervisor;
pub mod sweep;
pub mod timebase;
pub mod topics;
pub mod voxfeed;

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
