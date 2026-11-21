//! `navigation.voxmap`: whether the 3D voxel map runs and how.
//!
//! ```json
//! { "enabled": null, "planar": false, "window": "default", "esdf_every": 1, "depth_max_range_m": 10.0 }
//! ```
//!
//! `enabled` null (the default) runs the map when the rig's LiDAR is a 3D
//! LiDAR, when a depth camera is configured, or when the LiDAR is planar and
//! `planar` is true; `true` and `false` force it. Planar scans feed the map
//! only with `planar` true: a planar LiDAR alone sees one tilted slice, which
//! the pilot already reads directly.

use std::collections::BTreeMap;

use ados_mapping_pose::config::{typed, ConfigError};
use ados_voxmap::VoxMapConfig;
use serde::Deserialize;
use serde_json::Value;

/// The window the map keeps around the vehicle.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum VoxWindow {
    /// 0.2 m voxels over 24 x 24 x 10 m.
    #[default]
    Default,
    /// 0.15 m voxels over 30 x 30 x 12 m: an RK3588-class board with 8 GB.
    Fine,
}

/// The resolved `navigation.voxmap` key.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct VoxmapSettings {
    /// `None`: decided by the sensors (see the module docs).
    pub enabled: Option<bool>,
    /// Feed planar LiDAR scans into the map.
    pub planar: bool,
    pub window: VoxWindow,
    /// Update the obstacle-distance field every this many frames (1..=10).
    pub esdf_every: u16,
    /// Depth camera returns farther than this are left out, metres.
    pub depth_max_range_m: f32,
}

impl Default for VoxmapSettings {
    fn default() -> Self {
        Self {
            enabled: None,
            planar: false,
            window: VoxWindow::Default,
            esdf_every: 1,
            depth_max_range_m: 10.0,
        }
    }
}

/// What the run's sensors are.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Sensors {
    /// A 3D LiDAR runs.
    pub lidar3d: bool,
    /// A planar LiDAR runs.
    pub planar: bool,
    /// A depth camera is configured.
    pub depth: bool,
}

impl VoxmapSettings {
    pub fn validate(&self) -> Result<(), String> {
        if !(1..=10).contains(&self.esdf_every) {
            return Err(format!("esdf_every: {} is not 1..=10", self.esdf_every));
        }
        if !(self.depth_max_range_m.is_finite() && (0.5..=20.0).contains(&self.depth_max_range_m)) {
            return Err(format!(
                "depth_max_range_m: {} is not 0.5..=20",
                self.depth_max_range_m
            ));
        }
        Ok(())
    }

    /// Whether the map runs with these sensors.
    pub fn active(&self, s: Sensors) -> bool {
        self.enabled
            .unwrap_or(s.lidar3d || s.depth || (s.planar && self.planar))
    }

    pub fn map_config(&self) -> VoxMapConfig {
        let base = match self.window {
            VoxWindow::Default => VoxMapConfig::default(),
            VoxWindow::Fine => VoxMapConfig::fine_window(),
        };
        VoxMapConfig {
            esdf_every: self.esdf_every,
            ..base
        }
    }
}

/// `key` as [`VoxmapSettings`], validated; defaults when absent.
pub fn resolve(
    values: &BTreeMap<String, Value>,
    key: &'static str,
) -> Result<VoxmapSettings, ConfigError> {
    let s: VoxmapSettings = typed(values, key, VoxmapSettings::default())?;
    s.validate().map_err(|reason| ConfigError { key, reason })?;
    Ok(s)
}
