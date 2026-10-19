//! The structured `atlas.*` settings: pose provider, VIO and LiDAR inputs,
//! HUD buttons and the segment roll.

use serde::Deserialize;
use world_engine_protocol::atlas::PoseSource;

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

/// Resolve the configured provider. `Auto`: on a visor, VIO when the rig has
/// a front stereo pair and an IMU (the indoor default), else the FC EKF; on a
/// drone, the FC EKF. LiDAR odometry is only used when asked for.
pub fn resolve_pose_provider(
    cfg: PoseProviderConfig,
    profile: &str,
    rig_has_stereo_imu: bool,
) -> PoseSource {
    match cfg {
        PoseProviderConfig::FcEkf => PoseSource::FcEkf,
        PoseProviderConfig::Vio => PoseSource::Vio,
        PoseProviderConfig::Lidar => PoseSource::Lidar,
        PoseProviderConfig::Auto if profile == "visor" && rig_has_stereo_imu => PoseSource::Vio,
        PoseProviderConfig::Auto => PoseSource::FcEkf,
    }
}

/// VIO inputs (`atlas.vio`): the Basalt binary and the rig sensors it reads.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct VioSettings {
    /// Relative paths resolve against the plugin's install directory.
    pub binary: String,
    pub left: String,
    pub right: String,
    pub imu: String,
}

impl Default for VioSettings {
    fn default() -> Self {
        Self {
            binary: format!("bin/{}-linux/basalt-ados", std::env::consts::ARCH),
            left: "cam_fl".into(),
            right: "cam_fr".into(),
            imu: "imu0".into(),
        }
    }
}

/// LiDAR odometry inputs (`atlas.lidar`).
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct LidarSettings {
    /// The rig sensor id of the LiDAR.
    pub lidar_id: String,
    /// The `uv` binary that runs the pinned odometry project.
    pub uv: String,
    /// The odometry project directory (relative to the install directory).
    pub project_dir: String,
    /// The Livox SDK2 shared library (relative to the install directory).
    pub sdk_library: String,
    /// Host address on the LiDAR's subnet; unset = no live LiDAR driver.
    pub host_ip: Option<String>,
    /// Only these LiDAR addresses; empty accepts any.
    pub lidar_ips: Vec<String>,
}

impl Default for LidarSettings {
    fn default() -> Self {
        Self {
            lidar_id: "lidar0".into(),
            uv: "uv".into(),
            project_dir: "agent/capture/python/kiss_icp_live".into(),
            sdk_library: format!(
                "lib/{}-linux/liblivox_lidar_sdk_shared.so",
                std::env::consts::ARCH
            ),
            host_ip: None,
            lidar_ips: Vec::new(),
        }
    }
}

/// Front-panel button pins (`atlas.hud`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct HudSettings {
    pub rec_pin: u32,
    pub mark_pin: u32,
}

impl Default for HudSettings {
    fn default() -> Self {
        Self {
            rec_pin: crate::hud::DEFAULT_REC_PIN,
            mark_pin: crate::hud::DEFAULT_MARK_PIN,
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
