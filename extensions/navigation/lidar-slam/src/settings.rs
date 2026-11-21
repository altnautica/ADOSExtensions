//! The `navigation.*` plugin config keys `navigation-lidar-slam` runs with:
//! the pose provider, the pose publish interval, the declared take-off
//! heading, the VIO and LiDAR inputs, the memory budget and where the heavy
//! map is built. [`NavConfig::from_values`] resolves a map of those keys
//! (absent keys take their defaults).

use std::collections::BTreeMap;

use ados_mapping_pose::config::{heading_deg, integer, optional_unsigned, typed, ConfigError};
use ados_mapping_protocol::capture::{ComputePlacement, PoseSource};
use serde::Deserialize;
use serde_json::Value;

use crate::pose::vio::pairing::VioScale;

pub const KEY_POSE_PROVIDER: &str = "navigation.pose_provider";
pub const KEY_POSE_PUBLISH_INTERVAL_MS: &str = "navigation.pose_publish_interval_ms";
pub const KEY_TAKEOFF_HEADING_DEG: &str = "navigation.takeoff_heading_deg";
pub const KEY_VIO: &str = "navigation.vio";
pub const KEY_LIDAR: &str = "navigation.lidar";
pub const KEY_MEMORY_BUDGET_MB: &str = "navigation.memory_budget_mb";
pub const KEY_MAP_PLACEMENT: &str = "navigation.map_placement";
pub const KEY_DEPTH: &str = "navigation.depth";
pub const KEY_VOXMAP: &str = "navigation.voxmap";

/// Every key the service reads.
pub const CONFIG_KEYS: &[&str] = &[
    KEY_POSE_PROVIDER,
    KEY_POSE_PUBLISH_INTERVAL_MS,
    KEY_TAKEOFF_HEADING_DEG,
    KEY_VIO,
    KEY_LIDAR,
    KEY_MEMORY_BUDGET_MB,
    KEY_MAP_PLACEMENT,
    KEY_DEPTH,
    KEY_VOXMAP,
];

/// The configured pose provider (`navigation.pose_provider`). `Auto` picks
/// by profile and rig (see [`resolve_pose_provider`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum PoseProviderConfig {
    #[default]
    Auto,
    Vio,
    Lidar,
    Off,
}

/// Resolve the configured provider; `None` runs none. `Auto`: LiDAR
/// odometry when the rig has the configured LiDAR (with its scan model) and
/// IMU, on a drone too (there the LiDAR pose is the navigation source the
/// flight controller fuses); else, on a visor, VIO when it has a front stereo
/// pair and an IMU; else none.
pub fn resolve_pose_provider(
    cfg: PoseProviderConfig,
    profile: &str,
    rig_has_stereo_imu: bool,
    rig_has_lidar_imu: bool,
) -> Option<PoseSource> {
    match cfg {
        PoseProviderConfig::Off => None,
        PoseProviderConfig::Vio => Some(PoseSource::Vio),
        PoseProviderConfig::Lidar => Some(PoseSource::Lidar),
        PoseProviderConfig::Auto if rig_has_lidar_imu => Some(PoseSource::Lidar),
        PoseProviderConfig::Auto if profile == "visor" && rig_has_stereo_imu => {
            Some(PoseSource::Vio)
        }
        PoseProviderConfig::Auto => None,
    }
}

/// The resolved `navigation.*` config.
#[derive(Debug, Clone, PartialEq)]
pub struct NavConfig {
    pub pose_provider: PoseProviderConfig,
    /// The pose topic's interval on the poses' own measurement time, ms.
    pub pose_publish_interval_ms: i64,
    /// The heading the vehicle faces at take-off, degrees clockwise from
    /// north; `None` takes the north from the flight controller.
    pub takeoff_heading_deg: Option<f64>,
    pub vio: VioSettings,
    pub lidar: LidarSettings,
    /// The depth camera (`navigation.depth`); see [`crate::depth_source::config`].
    pub depth: crate::depth_source::DepthSettings,
    /// The 3D voxel map (`navigation.voxmap`); see [`crate::voxfeed::config`].
    pub voxmap: crate::voxfeed::VoxmapSettings,
    /// `None`: 80 % of the unit's memory cap.
    pub memory_budget_mb: Option<u64>,
    pub map_placement: ComputePlacement,
}

impl Default for NavConfig {
    fn default() -> Self {
        Self {
            pose_provider: PoseProviderConfig::Auto,
            pose_publish_interval_ms: 100,
            takeoff_heading_deg: None,
            vio: VioSettings::default(),
            lidar: LidarSettings::default(),
            depth: crate::depth_source::DepthSettings::default(),
            voxmap: crate::voxfeed::VoxmapSettings::default(),
            memory_budget_mb: None,
            map_placement: ComputePlacement::default(),
        }
    }
}

impl NavConfig {
    pub fn from_values(values: &BTreeMap<String, Value>) -> Result<Self, ConfigError> {
        let d = Self::default();
        Ok(Self {
            pose_provider: typed(values, KEY_POSE_PROVIDER, d.pose_provider)?,
            pose_publish_interval_ms: integer(
                values,
                KEY_POSE_PUBLISH_INTERVAL_MS,
                d.pose_publish_interval_ms,
            )?,
            takeoff_heading_deg: heading_deg(values, KEY_TAKEOFF_HEADING_DEG)?,
            vio: typed(values, KEY_VIO, d.vio)?,
            lidar: typed(values, KEY_LIDAR, d.lidar)?,
            depth: crate::depth_source::config::resolve(values, KEY_DEPTH)?,
            voxmap: crate::voxfeed::config::resolve(values, KEY_VOXMAP)?,
            memory_budget_mb: optional_unsigned(values, KEY_MEMORY_BUDGET_MB)?,
            map_placement: typed(values, KEY_MAP_PLACEMENT, d.map_placement)?,
        })
    }
}

/// VIO inputs (`navigation.vio`): the Basalt binary, the rig sensors it reads
/// and the resolution it tracks at (`scale`: 1 or 0.5).
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct VioSettings {
    /// Relative paths resolve against the plugin's install directory.
    pub binary: String,
    pub left: String,
    pub right: String,
    pub imu: String,
    pub scale: VioScale,
}

impl Default for VioSettings {
    fn default() -> Self {
        Self {
            binary: format!("bin/{}-linux/basalt-ados", std::env::consts::ARCH),
            left: "cam_fl".into(),
            right: "cam_fr".into(),
            imu: "imu0".into(),
            scale: VioScale::Full,
        }
    }
}

/// LiDAR inputs (`navigation.lidar`): which rig sensor, the IMU a planar LiDAR is
/// levelled with, the 3D odometry project, and the per-driver device settings.
/// The rig sensor's `lidar.model` picks the driver.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct LidarSettings {
    /// The rig sensor id of the LiDAR.
    pub lidar_id: String,
    /// The rig IMU the planar scan matcher levels scans with.
    pub imu: String,
    /// The prebuilt KISS-ICP runtime the 3D odometry runs on (relative to the
    /// install directory; the release ships it as an unpacked payload).
    pub runtime: String,
    /// The developer fallback without a runtime: the `uv` that provisions the
    /// pinned 3D odometry project into the data dir. A bare name is looked up
    /// on `PATH`; a path is relative to the install directory.
    pub uv: String,
    /// The 3D odometry project directory (relative to the install directory).
    pub project_dir: String,
    pub rplidar: RplidarSettings,
    pub livox: LivoxSettings,
}

impl Default for LidarSettings {
    fn default() -> Self {
        Self {
            lidar_id: "lidar0".into(),
            imu: "imu0".into(),
            runtime: format!("lib/{}-linux/kiss-icp-runtime", std::env::consts::ARCH),
            uv: "uv".into(),
            project_dir: "lidar-slam/python/kiss_icp_live".into(),
            rplidar: RplidarSettings::default(),
            livox: LivoxSettings::default(),
        }
    }
}

/// An RPLIDAR on a serial port (`navigation.lidar.rplidar`).
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RplidarSettings {
    /// Serial port path, `tcp:<host>:<port>`, or `auto` (USB discovery);
    /// empty = no live driver.
    pub device: String,
    pub baud: u32,
}

impl Default for RplidarSettings {
    fn default() -> Self {
        Self {
            device: "auto".into(),
            baud: 460_800,
        }
    }
}

/// A Livox LiDAR through the SDK2 shared library (`navigation.lidar.livox`).
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct LivoxSettings {
    /// The SDK2 shared library (relative to the install directory).
    pub sdk_library: String,
    /// Host address on the LiDAR's subnet; unset = no live driver.
    pub host_ip: Option<String>,
    /// Only these LiDAR addresses; empty accepts any.
    pub lidar_ips: Vec<String>,
}

impl Default for LivoxSettings {
    fn default() -> Self {
        Self {
            sdk_library: format!(
                "lib/{}-linux/liblivox_lidar_sdk_shared.so",
                std::env::consts::ARCH
            ),
            host_ip: None,
            lidar_ips: Vec::new(),
        }
    }
}

#[cfg(test)]
#[path = "settings_tests.rs"]
mod tests;
