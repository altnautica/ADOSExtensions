//! An RPLIDAR through `ados-rplidar`: its revolutions, already in the sensor
//! FLU frame (z = 0) and stamped on the service's timebase, forwarded as
//! [`LidarScan`]s onto a tokio channel by a small bridge thread.
//!
//! The device driver keeps reconnecting a lost link on its own; the port and
//! health reported are those of the latest connection. Dropping the source
//! stops the device (`STOP`), which ends the bridge.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::thread::{self, JoinHandle};

use ados_mapping_protocol::capture::LidarHealth;
use ados_rplidar::{Health, HealthStatus, RawScan, Rplidar, RplidarConfig};
use tokio::sync::mpsc;

use super::{LidarDriver, LidarDriverStats, LidarSourceError, ReturnCounter};
use ados_mapping_pose::lidar_scan::LidarScan;

/// A revolution as the capture path carries it; the vectors move, no copy.
pub fn scan_from_rplidar(sensor_id: &str, s: RawScan) -> LidarScan {
    LidarScan {
        sensor_id: sensor_id.to_string(),
        t_ns: s.t_ns,
        points: s.points,
        t_offset_ns: s.t_offset_ns,
        intensity: s.intensity,
    }
}

#[derive(Default)]
struct Counters {
    delivered: AtomicU64,
    dropped: AtomicU64,
    returns: ReturnCounter,
}

/// A running RPLIDAR source.
pub struct RplidarSource {
    device: Option<Rplidar>,
    counters: Arc<Counters>,
    bridge: Option<JoinHandle<()>>,
}

impl RplidarSource {
    /// Open the device (`cfg.device`: a serial port, `tcp:<host>:<port>` or
    /// `auto`), run its handshake and start scanning; revolutions go to
    /// `scans` tagged `cfg.sensor_id`.
    pub fn start(
        cfg: RplidarConfig,
        scans: mpsc::Sender<LidarScan>,
    ) -> Result<Self, LidarSourceError> {
        let sensor_id = cfg.sensor_id.clone();
        let (device, raw) = Rplidar::start(cfg)?;
        let counters = Arc::new(Counters::default());
        let c = counters.clone();
        let bridge = thread::Builder::new()
            .name(format!("rplidar-fwd-{sensor_id}"))
            .spawn(move || {
                // Ends when the device stops (its sender drops) or the
                // consumer goes away (dropping `raw` then stops the reader).
                while let Ok(scan) = raw.recv() {
                    c.returns.on_scan(scan.points.len());
                    match scans.try_send(scan_from_rplidar(&sensor_id, scan)) {
                        Ok(()) => c.delivered.fetch_add(1, Ordering::Relaxed),
                        Err(mpsc::error::TrySendError::Full(_)) => {
                            c.dropped.fetch_add(1, Ordering::Relaxed)
                        }
                        Err(mpsc::error::TrySendError::Closed(_)) => break,
                    };
                }
            })
            .map_err(LidarSourceError::Thread)?;
        Ok(Self {
            device: Some(device),
            counters,
            bridge: Some(bridge),
        })
    }

    /// The device's own counters (revolutions, resyncs, reconnects).
    pub fn device_stats(&self) -> Option<ados_rplidar::RplidarStats> {
        self.device.as_ref().map(Rplidar::stats)
    }
}

/// `GET_HEALTH` in the status vocabulary; a status byte the protocol does
/// not define is reported as a warning, as the handshake treats it.
pub fn lidar_health(h: Health) -> (LidarHealth, u16) {
    let level = match h.status {
        HealthStatus::Good => LidarHealth::Good,
        HealthStatus::Warning | HealthStatus::Unknown(_) => LidarHealth::Warning,
        HealthStatus::Error => LidarHealth::Error,
    };
    (level, h.error_code)
}

impl LidarDriver for RplidarSource {
    fn stats(&self) -> LidarDriverStats {
        let device = self.device.as_ref();
        let dev = self.device_stats().unwrap_or_default();
        LidarDriverStats {
            connected: dev.connected,
            device: device.map(Rplidar::path),
            health: device.map(|d| lidar_health(d.health())),
            revolutions: dev.revolutions,
            scans: self.counters.delivered.load(Ordering::Relaxed),
            dropped_scans: self.counters.dropped.load(Ordering::Relaxed) + dev.dropped_scans,
            reconnects: dev.reconnects,
            returns_last_scan: self.counters.returns.returns_last_scan(),
            empty_scans: self.counters.returns.empty_scans(),
        }
    }
}

impl Drop for RplidarSource {
    fn drop(&mut self) {
        // Stopping the device joins its reader, whose sender then drops and
        // ends the bridge.
        drop(self.device.take());
        if let Some(b) = self.bridge.take() {
            let _ = b.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn revolution_moves_into_a_lidar_scan() {
        let s = RawScan {
            t_ns: 9,
            points: vec![[1.0, 0.0, 0.0], [0.0, 2.0, 0.0]],
            t_offset_ns: vec![0, 200_000],
            intensity: vec![47, 60],
        };
        let l = scan_from_rplidar("lidar0", s);
        assert_eq!(l.sensor_id, "lidar0");
        assert_eq!((l.t_ns, l.len()), (9, 2));
        assert_eq!((l.t_offset_ns[1], l.intensity[1]), (200_000, 60));
        assert!(l.points.iter().all(|p| p[2] == 0.0));
    }

    #[test]
    fn device_health_maps_to_the_status_words_with_its_code() {
        let h = |status, error_code| lidar_health(Health { status, error_code });
        assert_eq!(h(HealthStatus::Good, 0), (LidarHealth::Good, 0));
        assert_eq!(h(HealthStatus::Warning, 7), (LidarHealth::Warning, 7));
        assert_eq!(h(HealthStatus::Error, 0x1234), (LidarHealth::Error, 0x1234));
        assert_eq!(h(HealthStatus::Unknown(9), 1), (LidarHealth::Warning, 1));
    }
}
