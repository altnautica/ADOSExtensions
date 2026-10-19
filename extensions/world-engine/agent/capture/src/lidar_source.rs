//! The capture service's LiDAR input: a Livox SDK2 device driven through
//! `ados-livox`, its scans delivered as [`LidarScan`]s on a tokio channel.
//!
//! The SDK is a shared library loaded at run time (feature `livox-sdk`, Unix
//! only). Without the feature, or when the library cannot be loaded, starting
//! fails with an error naming the cause; there is no fallback source.

use std::path::PathBuf;
use std::sync::atomic::AtomicU64;
use std::sync::Arc;

use tokio::sync::mpsc;

use crate::lidar_scan::LidarScan;

pub use ados_livox::SdkNetConfig;

/// Scans that may wait for the consumer (~0.4 s at 10 Hz).
pub const SCAN_CHANNEL_DEPTH: usize = 4;
/// SDK config file name inside `work_dir`.
pub const SDK_CONFIG_FILE: &str = "livox_sdk2.json";

#[derive(Debug, Clone)]
pub struct LidarSourceConfig {
    /// Rig sensor id the scans are tagged with.
    pub sensor_id: String,
    /// `liblivox_lidar_sdk_shared.so`.
    pub sdk_library: PathBuf,
    pub net: SdkNetConfig,
    /// Where the SDK config is written.
    pub work_dir: PathBuf,
    /// Scan period, ns (10 Hz: 100 ms).
    pub period_ns: u64,
}

#[derive(Debug, thiserror::Error)]
pub enum LidarSourceError {
    #[error("this build has no Livox SDK support (feature `livox-sdk`, Linux)")]
    Unsupported,
    #[error("writing {path}: {source}")]
    Io {
        path: PathBuf,
        source: std::io::Error,
    },
    #[cfg(all(feature = "livox-sdk", unix))]
    #[error(transparent)]
    Sdk(#[from] ados_livox::SdkError),
}

/// A decoded scan as the capture path carries it; the vectors move, no copy.
pub fn scan_from_livox(sensor_id: &str, s: ados_livox::Scan) -> LidarScan {
    LidarScan {
        sensor_id: sensor_id.to_string(),
        t_ns: s.t_ns,
        points: s.points,
        t_offset_ns: s.t_offset_ns,
        intensity: s.reflectivity,
    }
}

/// A running LiDAR source; dropping it stops the SDK.
pub struct LidarSource {
    /// Scans dropped on a full channel or from a second device.
    pub dropped: Arc<AtomicU64>,
    #[cfg(all(feature = "livox-sdk", unix))]
    driver: ados_livox::SdkDriver,
}

impl LidarSource {
    /// Start the SDK; scans arrive on the returned receiver.
    pub fn start(
        cfg: LidarSourceConfig,
    ) -> Result<(Self, mpsc::Receiver<LidarScan>), LidarSourceError> {
        start_impl(cfg)
    }

    #[cfg(all(feature = "livox-sdk", unix))]
    pub fn sdk_stats(&self) -> ados_livox::SdkStats {
        self.driver.stats()
    }
}

#[cfg(not(all(feature = "livox-sdk", unix)))]
fn start_impl(
    _cfg: LidarSourceConfig,
) -> Result<(LidarSource, mpsc::Receiver<LidarScan>), LidarSourceError> {
    Err(LidarSourceError::Unsupported)
}

#[cfg(all(feature = "livox-sdk", unix))]
fn start_impl(
    cfg: LidarSourceConfig,
) -> Result<(LidarSource, mpsc::Receiver<LidarScan>), LidarSourceError> {
    use std::sync::atomic::Ordering;

    use ados_livox::{DecoderConfig, DeviceEvent, Event, SdkDriver, SdkDriverConfig};

    let io = |path: &std::path::Path| {
        let path = path.to_path_buf();
        move |source| LidarSourceError::Io { path, source }
    };
    std::fs::create_dir_all(&cfg.work_dir).map_err(io(&cfg.work_dir))?;
    let config_json = cfg.work_dir.join(SDK_CONFIG_FILE);
    std::fs::write(&config_json, cfg.net.to_json().to_string()).map_err(io(&config_json))?;

    let (tx, rx) = mpsc::channel(SCAN_CHANNEL_DEPTH);
    let dropped = Arc::new(AtomicU64::new(0));
    let sink_dropped = dropped.clone();
    let sensor_id = cfg.sensor_id;
    // One rig sensor is one device: the first handle seen is kept.
    let mut device: Option<u32> = None;
    let sink = move |ev: DeviceEvent| {
        let Event::Scan(scan) = ev.event else { return };
        if *device.get_or_insert(ev.handle) != ev.handle
            || tx.try_send(scan_from_livox(&sensor_id, scan)).is_err()
        {
            sink_dropped.fetch_add(1, Ordering::Relaxed);
        }
    };
    let driver = SdkDriver::start(
        SdkDriverConfig {
            library: cfg.sdk_library,
            config_json,
            decoder: DecoderConfig {
                period_ns: cfg.period_ns,
                ..DecoderConfig::default()
            },
            enable_imu: false,
            queue: 1024,
        },
        sink,
    )?;
    Ok((LidarSource { dropped, driver }, rx))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn livox_scan_moves_into_a_lidar_scan() {
        let s = ados_livox::Scan {
            t_ns: 42,
            device_t_ns: 7,
            time_type: None,
            points: vec![[1.0, 2.0, 3.0]],
            t_offset_ns: vec![5],
            reflectivity: vec![200],
            tag: vec![0],
        };
        let l = scan_from_livox("mid360", s);
        assert_eq!(l.sensor_id, "mid360");
        assert_eq!((l.t_ns, l.len()), (42, 1));
        assert_eq!((l.t_offset_ns[0], l.intensity[0]), (5, 200));
    }

    #[test]
    fn missing_sdk_fails_closed() {
        let dir = tempfile::tempdir().unwrap();
        let err = LidarSource::start(LidarSourceConfig {
            sensor_id: "mid360".into(),
            sdk_library: dir.path().join("liblivox_lidar_sdk_shared.so"),
            net: SdkNetConfig::new("192.168.1.50"),
            work_dir: dir.path().join("work"),
            period_ns: 100_000_000,
        })
        .err()
        .expect("no SDK library here");
        let msg = err.to_string();
        assert!(
            msg.contains("liblivox_lidar_sdk_shared.so") || msg.contains("livox-sdk"),
            "{msg}"
        );
    }
}
