//! The structured `atlas.*` settings: pose provider, VIO and LiDAR inputs,
//! HUD buttons and the segment roll.

use serde::Deserialize;
use world_engine_protocol::atlas::PoseSource;

use crate::pose::vio::pairing::VioScale;

/// The configured pose provider (`atlas.pose_provider`). `Auto` picks by
/// profile and rig (see [`resolve_pose_provider`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum PoseProviderConfig {
    #[default]
    Auto,
    FcEkf,
    Vio,
    Lidar,
}

/// Resolve the configured provider. `Auto` on a visor: LiDAR odometry when
/// the rig has the configured LiDAR (with its scan model) and IMU, else VIO
/// when it has a front stereo pair and an IMU, else the FC EKF; on a drone,
/// the FC EKF.
pub fn resolve_pose_provider(
    cfg: PoseProviderConfig,
    profile: &str,
    rig_has_stereo_imu: bool,
    rig_has_lidar_imu: bool,
) -> PoseSource {
    match cfg {
        PoseProviderConfig::FcEkf => PoseSource::FcEkf,
        PoseProviderConfig::Vio => PoseSource::Vio,
        PoseProviderConfig::Lidar => PoseSource::Lidar,
        PoseProviderConfig::Auto if profile == "visor" && rig_has_lidar_imu => PoseSource::Lidar,
        PoseProviderConfig::Auto if profile == "visor" && rig_has_stereo_imu => PoseSource::Vio,
        PoseProviderConfig::Auto => PoseSource::FcEkf,
    }
}

/// VIO inputs (`atlas.vio`): the Basalt binary, the rig sensors it reads and
/// the resolution it tracks at (`scale`: 1 or 0.5).
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

/// LiDAR inputs (`atlas.lidar`): which rig sensor, the IMU a planar LiDAR is
/// levelled with, the 3D odometry project, and the per-driver device settings.
/// The rig sensor's `lidar.model` picks the driver.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct LidarSettings {
    /// The rig sensor id of the LiDAR.
    pub lidar_id: String,
    /// The rig IMU the planar scan matcher levels scans with.
    pub imu: String,
    /// The `uv` binary that provisions the pinned 3D odometry project
    /// (relative to the install directory; the release ships it as a payload).
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
            uv: format!("bin/{}-linux/uv", std::env::consts::ARCH),
            project_dir: "agent/capture/python/kiss_icp_live".into(),
            rplidar: RplidarSettings::default(),
            livox: LivoxSettings::default(),
        }
    }
}

/// An RPLIDAR on a serial port (`atlas.lidar.rplidar`).
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

/// A Livox LiDAR through the SDK2 shared library (`atlas.lidar.livox`).
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

/// Segment roll (`atlas.segment`); the contract default is 4 GiB or 10 min.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SegmentSettings {
    pub max_bytes: u64,
    pub max_duration_s: u64,
}

impl Default for SegmentSettings {
    fn default() -> Self {
        Self {
            max_bytes: 4 << 30,
            max_duration_s: 600,
        }
    }
}

impl SegmentSettings {
    pub fn limits(&self) -> ados_capture_contract::SegmentLimits {
        ados_capture_contract::SegmentLimits {
            max_bytes: self.max_bytes.max(1 << 20),
            max_duration_ns: self.max_duration_s.max(1) * 1_000_000_000,
        }
    }
}
