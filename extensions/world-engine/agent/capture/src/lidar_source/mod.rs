//! The capture service's live LiDAR input: one seam over every supported
//! device. The rig sensor's scan model (`lidar.model`) picks the driver:
//!
//! - `rplidar-c1`: an RPLIDAR over its serial protocol ([`rplidar`]),
//!   planar revolutions with z = 0;
//! - `livox-mid360`: a Livox device through the SDK2 shared library
//!   ([`livox`], feature `livox-sdk`, Unix).
//!
//! Every driver delivers [`LidarScan`]s on a tokio channel in the sensor FLU
//! frame, stamped on the host monotonic clock. A model with no driver fails
//! with [`LidarSourceError::UnsupportedModel`]; there is no fallback source.

pub mod livox;
pub mod rplidar;

#[cfg(test)]
mod tests;

use std::path::{Path, PathBuf};

use ados_capture_contract::LidarModel;
use tokio::sync::mpsc;

use crate::lidar_scan::LidarScan;
use crate::runtime::LidarSettings;

pub use livox::{scan_from_livox, LivoxSource, LivoxSourceConfig, SdkNetConfig, SDK_CONFIG_FILE};
pub use rplidar::{scan_from_rplidar, RplidarSource};

/// Scans that may wait for the consumer (~0.4 s at 10 Hz).
pub const SCAN_CHANNEL_DEPTH: usize = 4;
/// `lidar.model` of the RPLIDAR C1.
pub const MODEL_RPLIDAR_C1: &str = "rplidar-c1";
/// `lidar.model` of the Livox Mid-360.
pub const MODEL_LIVOX_MID360: &str = "livox-mid360";

#[derive(Debug, thiserror::Error)]
pub enum LidarSourceError {
    #[error("no live driver for LiDAR model `{0}`")]
    UnsupportedModel(String),
    /// The selected driver's device setting is empty: no live driver wanted.
    #[error("`atlas.lidar.{0}` is not set")]
    NotConfigured(&'static str),
    #[error("LiDAR model rate {0} Hz is not a positive scan rate")]
    InvalidRate(f64),
    #[error("this build has no Livox SDK support (feature `livox-sdk`, Linux)")]
    Unsupported,
    #[error("writing {path}: {source}")]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    #[error("starting the LiDAR forwarding thread: {0}")]
    Thread(std::io::Error),
    #[error(transparent)]
    Rplidar(#[from] ados_rplidar::RplidarError),
    #[cfg(all(feature = "livox-sdk", unix))]
    #[error(transparent)]
    Sdk(#[from] ados_livox::SdkError),
}

/// Counters a running driver reports.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LidarDriverStats {
    /// Whether the device link is up; `None` when the driver cannot tell.
    pub connected: Option<bool>,
    /// Scans handed to the capture loop.
    pub scans: u64,
    /// Scans dropped because the consumer fell behind.
    pub dropped_scans: u64,
    /// Link re-establishments after a loss.
    pub reconnects: u64,
}

/// A running LiDAR driver; dropping it stops the device.
pub trait LidarDriver: Send {
    fn stats(&self) -> LidarDriverStats;
}

/// Scan period from the model's rate.
pub fn period_ns(model: &LidarModel) -> Result<u64, LidarSourceError> {
    if model.rate_hz.is_finite() && model.rate_hz > 0.0 {
        Ok((1e9 / model.rate_hz).round() as u64)
    } else {
        Err(LidarSourceError::InvalidRate(model.rate_hz))
    }
}

/// Start the driver for `model`, its scans tagged `sensor_id`. Relative
/// paths in `settings` resolve against the install directory; `work_dir`
/// holds driver files (the Livox SDK config). An empty RPLIDAR `device` or an
/// unset Livox `host_ip` is [`LidarSourceError::NotConfigured`].
pub fn start(
    model: &LidarModel,
    settings: &LidarSettings,
    sensor_id: &str,
    work_dir: &Path,
) -> Result<(Box<dyn LidarDriver>, mpsc::Receiver<LidarScan>), LidarSourceError> {
    match model.model.as_str() {
        MODEL_RPLIDAR_C1 => {
            let device = settings.rplidar.device.trim();
            if device.is_empty() {
                return Err(LidarSourceError::NotConfigured("rplidar.device"));
            }
            let (src, rx) = RplidarSource::start(ados_rplidar::RplidarConfig {
                device: device.to_string(),
                baud: settings.rplidar.baud,
                sensor_id: sensor_id.to_string(),
            })?;
            Ok((Box::new(src), rx))
        }
        MODEL_LIVOX_MID360 => {
            let Some(host_ip) = settings.livox.host_ip.clone() else {
                return Err(LidarSourceError::NotConfigured("livox.host_ip"));
            };
            let mut net = SdkNetConfig::new(host_ip);
            net.lidar_ips = settings.livox.lidar_ips.clone();
            let (src, rx) = LivoxSource::start(LivoxSourceConfig {
                sensor_id: sensor_id.to_string(),
                sdk_library: crate::providers::install_path(&settings.livox.sdk_library),
                net,
                work_dir: work_dir.join("livox"),
                period_ns: period_ns(model)?,
            })?;
            Ok((Box::new(src), rx))
        }
        other => Err(LidarSourceError::UnsupportedModel(other.to_string())),
    }
}
