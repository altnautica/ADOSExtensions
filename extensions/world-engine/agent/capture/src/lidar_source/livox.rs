//! A Livox SDK2 device driven through `ados-livox`, its scans delivered as
//! [`LidarScan`]s on a tokio channel.
//!
//! The SDK is a shared library loaded at run time (feature `livox-sdk`, Unix
//! only). Without the feature, or when the library cannot be loaded, starting
//! fails with an error naming the cause; there is no fallback source.
//!
//! The SDK reports no link state and no health: the link counts as up while
//! scans keep arriving ([`LINK_STALE`]), and the health is unknown.

use std::path::PathBuf;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use tokio::sync::mpsc;

use super::{LidarDriver, LidarDriverStats, LidarSourceError, ReturnCounter};
use crate::lidar_scan::LidarScan;

pub use ados_livox::SdkNetConfig;

/// SDK config file name inside `work_dir`.
pub const SDK_CONFIG_FILE: &str = "livox_sdk2.json";

#[derive(Debug, Clone)]
pub struct LivoxSourceConfig {
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

/// A device whose last scan is older than this counts as disconnected.
pub const LINK_STALE: Duration = Duration::from_secs(1);

#[derive(Debug, Default)]
struct Counters {
    /// Scans of the rig's device, delivered or not.
    frames: AtomicU64,
    /// Scans handed to the consumer.
    delivered: AtomicU64,
    /// Scans dropped on a full channel or from a second device.
    dropped: AtomicU64,
    /// Host monotonic ns of the device's last scan; 0 before the first.
    last_scan_ns: AtomicU64,
    /// The returns of the rig's device's scans.
    returns: ReturnCounter,
}

/// A running Livox source; dropping it stops the SDK.
pub struct LivoxSource {
    counters: Arc<Counters>,
    #[cfg(all(feature = "livox-sdk", unix))]
    driver: ados_livox::SdkDriver,
}

impl LivoxSource {
    /// Start the SDK; scans go to `scans`.
    pub fn start(
        cfg: LivoxSourceConfig,
        scans: mpsc::Sender<LidarScan>,
    ) -> Result<Self, LidarSourceError> {
        start_impl(cfg, scans)
    }

    #[cfg(all(feature = "livox-sdk", unix))]
    pub fn sdk_stats(&self) -> ados_livox::SdkStats {
        self.driver.stats()
    }
}

impl LidarDriver for LivoxSource {
    fn stats(&self) -> LidarDriverStats {
        let c = &self.counters;
        let last = c.last_scan_ns.load(Ordering::Relaxed);
        LidarDriverStats {
            connected: link_fresh(last, crate::clock::mono_ns()),
            device: None,
            health: None,
            revolutions: c.frames.load(Ordering::Relaxed),
            scans: c.delivered.load(Ordering::Relaxed),
            dropped_scans: c.dropped.load(Ordering::Relaxed),
            reconnects: 0,
            returns_last_scan: c.returns.returns_last_scan(),
            empty_scans: c.returns.empty_scans(),
        }
    }
}

/// Whether a device whose last scan was at `last_ns` (0: none yet) is still
/// delivering at `now_ns`.
fn link_fresh(last_ns: u64, now_ns: u64) -> bool {
    last_ns != 0 && now_ns.saturating_sub(last_ns) < LINK_STALE.as_nanos() as u64
}

#[cfg(not(all(feature = "livox-sdk", unix)))]
fn start_impl(
    _cfg: LivoxSourceConfig,
    _scans: mpsc::Sender<LidarScan>,
) -> Result<LivoxSource, LidarSourceError> {
    Err(LidarSourceError::Unsupported)
}

#[cfg(all(feature = "livox-sdk", unix))]
fn start_impl(
    cfg: LivoxSourceConfig,
    scans: mpsc::Sender<LidarScan>,
) -> Result<LivoxSource, LidarSourceError> {
    use ados_livox::{DecoderConfig, DeviceEvent, Event, SdkDriver, SdkDriverConfig};

    let io = |path: &std::path::Path| {
        let path = path.to_path_buf();
        move |source| LidarSourceError::Io { path, source }
    };
    std::fs::create_dir_all(&cfg.work_dir).map_err(io(&cfg.work_dir))?;
    let config_json = cfg.work_dir.join(SDK_CONFIG_FILE);
    std::fs::write(&config_json, cfg.net.to_json().to_string()).map_err(io(&config_json))?;

    let counters = Arc::new(Counters::default());
    let c = counters.clone();
    let sensor_id = cfg.sensor_id;
    // One rig sensor is one device: the first handle seen is kept.
    let mut device: Option<u32> = None;
    let sink = move |ev: DeviceEvent| {
        let Event::Scan(scan) = ev.event else { return };
        if *device.get_or_insert(ev.handle) != ev.handle {
            c.dropped.fetch_add(1, Ordering::Relaxed);
            return;
        }
        c.frames.fetch_add(1, Ordering::Relaxed);
        c.last_scan_ns
            .store(crate::clock::mono_ns(), Ordering::Relaxed);
        c.returns.on_scan(scan.points.len());
        let counter = match scans.try_send(scan_from_livox(&sensor_id, scan)) {
            Ok(()) => &c.delivered,
            Err(_) => &c.dropped,
        };
        counter.fetch_add(1, Ordering::Relaxed);
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
    Ok(LivoxSource { counters, driver })
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
    fn the_link_is_up_only_while_scans_keep_arriving() {
        let stale = LINK_STALE.as_nanos() as u64;
        assert!(!link_fresh(0, 5), "no scan yet");
        assert!(link_fresh(1_000, 1_000 + stale - 1));
        assert!(!link_fresh(1_000, 1_000 + stale));
    }

    #[test]
    fn missing_sdk_fails_closed() {
        let dir = tempfile::tempdir().unwrap();
        let (tx, _rx) = mpsc::channel(1);
        let err = LivoxSource::start(
            LivoxSourceConfig {
                sensor_id: "mid360".into(),
                sdk_library: dir.path().join("liblivox_lidar_sdk_shared.so"),
                net: SdkNetConfig::new("192.168.1.50"),
                work_dir: dir.path().join("work"),
                period_ns: 100_000_000,
            },
            tx,
        )
        .err()
        .expect("no SDK library here");
        let msg = err.to_string();
        assert!(
            msg.contains("liblivox_lidar_sdk_shared.so") || msg.contains("livox-sdk"),
            "{msg}"
        );
    }
}
