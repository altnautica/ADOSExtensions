//! The capture service's live LiDAR input: one seam over every supported
//! device. The rig sensor's scan model (`lidar.model`) picks the driver:
//!
//! - `rplidar-c1`: an RPLIDAR over its serial protocol ([`rplidar`]),
//!   planar revolutions with z = 0;
//! - `livox-mid360`: a Livox device through the SDK2 shared library
//!   ([`livox`], feature `livox-sdk`, Unix).
//!
//! Every driver delivers [`LidarScan`]s on the tokio channel it is started
//! with, in the sensor FLU frame, stamped on the host monotonic clock. A
//! model with no driver fails with [`LidarSourceError::UnsupportedModel`];
//! there is no fallback source.
//!
//! [`LidarRunner`] owns the driver for a capture run: it starts it, retries a
//! device that was absent (or did not answer) every [`START_RETRY`], and
//! reports the link as a [`LidarStatus`] every [`STATUS_PERIOD`]. The run's
//! scan channel outlives every start attempt, so the capture loop reads one
//! receiver whether the device answered at start or later.

pub mod livox;
pub mod rplidar;

#[cfg(test)]
mod tests;

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use ados_capture_contract::LidarModel;
use tokio::sync::{mpsc, watch};
use tokio::task::JoinHandle;
use tokio::time::Instant;
use world_engine_protocol::atlas::{LidarHealth, LidarStatus};

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
/// Wait between start attempts of a declared LiDAR whose driver did not start.
pub const START_RETRY: Duration = Duration::from_secs(2);
/// Period of the LiDAR link status.
pub const STATUS_PERIOD: Duration = Duration::from_secs(1);

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

impl LidarSourceError {
    /// Whether a later start may succeed with the same settings: the device
    /// was absent, did not answer, or reported a health error. An unset,
    /// unsupported or invalid setting waits for a config change instead.
    pub fn is_retryable(&self) -> bool {
        !matches!(
            self,
            Self::UnsupportedModel(_)
                | Self::NotConfigured(_)
                | Self::InvalidRate(_)
                | Self::Unsupported
        )
    }
}

/// What a running driver reports.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct LidarDriverStats {
    /// Whether the device link is up.
    pub connected: bool,
    /// The port the device is on (the resolved one under USB discovery);
    /// `None` for a driver without one.
    pub device: Option<String>,
    /// The device's health and error code from its latest (re)connect;
    /// `None` for a driver without a health report.
    pub health: Option<(LidarHealth, u16)>,
    /// Revolutions (frames) the device produced, delivered or not.
    pub revolutions: u64,
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

/// Start the driver for `model`, its scans tagged `sensor_id` and sent on
/// `scans`. Relative paths in `settings` resolve against the install
/// directory; `work_dir` holds driver files (the Livox SDK config). An empty
/// RPLIDAR `device` or an unset Livox `host_ip` is
/// [`LidarSourceError::NotConfigured`]. Blocking: it runs the device
/// handshake.
pub fn start(
    model: &LidarModel,
    settings: &LidarSettings,
    sensor_id: &str,
    work_dir: &Path,
    scans: mpsc::Sender<LidarScan>,
) -> Result<Box<dyn LidarDriver>, LidarSourceError> {
    match model.model.as_str() {
        MODEL_RPLIDAR_C1 => {
            let device = settings.rplidar.device.trim();
            if device.is_empty() {
                return Err(LidarSourceError::NotConfigured("rplidar.device"));
            }
            let src = RplidarSource::start(
                ados_rplidar::RplidarConfig {
                    device: device.to_string(),
                    baud: settings.rplidar.baud,
                    sensor_id: sensor_id.to_string(),
                },
                scans,
            )?;
            Ok(Box::new(src))
        }
        MODEL_LIVOX_MID360 => {
            let Some(host_ip) = settings.livox.host_ip.clone() else {
                return Err(LidarSourceError::NotConfigured("livox.host_ip"));
            };
            let mut net = SdkNetConfig::new(host_ip);
            net.lidar_ips = settings.livox.lidar_ips.clone();
            let src = LivoxSource::start(
                LivoxSourceConfig {
                    sensor_id: sensor_id.to_string(),
                    sdk_library: crate::providers::install_path(&settings.livox.sdk_library),
                    net,
                    work_dir: work_dir.join("livox"),
                    period_ns: period_ns(model)?,
                },
                scans,
            )?;
            Ok(Box::new(src))
        }
        other => Err(LidarSourceError::UnsupportedModel(other.to_string())),
    }
}

/// Starts a driver whose scans go to the sender it is given; blocking.
pub type Starter = Arc<
    dyn Fn(mpsc::Sender<LidarScan>) -> Result<Box<dyn LidarDriver>, LidarSourceError> + Send + Sync,
>;

/// The rig's configured LiDAR for the length of a capture run.
pub struct LidarRunner {
    pub sensor_id: String,
    /// `lidar.model`, for the log.
    pub model: String,
    pub starter: Starter,
    /// Wait after a failed start before the next one.
    pub retry: Duration,
    /// Period of the link status.
    pub status_period: Duration,
}

impl LidarRunner {
    pub fn new(sensor_id: impl Into<String>, model: impl Into<String>, starter: Starter) -> Self {
        Self {
            sensor_id: sensor_id.into(),
            model: model.into(),
            starter,
            retry: START_RETRY,
            status_period: STATUS_PERIOD,
        }
    }

    /// Start the driver with its scans on `scans` (on a blocking thread),
    /// retry a start that failed for a reason a later attempt can clear, and
    /// put the link status on `status` every period. Runs until the task is
    /// aborted, which drops the driver and so stops the device.
    pub async fn run(self, scans: mpsc::Sender<LidarScan>, status: watch::Sender<LidarStatus>) {
        let sensor = self.sensor_id.as_str();
        let mut driver: Option<Box<dyn LidarDriver>> = None;
        type Started = Result<Box<dyn LidarDriver>, LidarSourceError>;
        let mut starting: Option<JoinHandle<Started>> = None;
        // `None`: no further attempt this run.
        let mut next_start = Some(Instant::now());
        let mut failures = 0u32;
        let mut rate = RateMeter::default();
        let mut tick = tokio::time::interval(self.status_period);
        tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
        loop {
            let due = next_start.filter(|_| driver.is_none() && starting.is_none());
            tokio::select! {
                _ = tick.tick() => {
                    let stats = driver.as_ref().map(|d| d.stats());
                    let rate_hz = stats.as_ref().and_then(|s| rate.sample(Instant::now(), s));
                    status.send_replace(lidar_status(stats.as_ref(), rate_hz));
                }
                _ = tokio::time::sleep_until(due.unwrap_or_else(Instant::now)), if due.is_some() => {
                    next_start = None;
                    let (start, tx) = (self.starter.clone(), scans.clone());
                    starting = Some(tokio::task::spawn_blocking(move || (*start)(tx)));
                }
                done = async { starting.as_mut().expect("guarded by is_some").await }, if starting.is_some() => {
                    starting = None;
                    match done {
                        Ok(Ok(d)) => {
                            if failures > 0 {
                                tracing::info!(sensor, model = %self.model, attempts = failures + 1, "lidar_source_recovered");
                            }
                            rate.restart(Instant::now(), &d.stats());
                            driver = Some(d);
                        }
                        Ok(Err(LidarSourceError::NotConfigured(key))) => {
                            tracing::info!(sensor, key, "lidar_source_not_configured");
                        }
                        Ok(Err(e)) if !e.is_retryable() => {
                            tracing::warn!(sensor, model = %self.model, error = %e, "lidar_source_unavailable");
                        }
                        Ok(Err(e)) => {
                            failures += 1;
                            if failures == 1 {
                                tracing::warn!(
                                    sensor,
                                    model = %self.model,
                                    error = %e,
                                    retry_ms = self.retry.as_millis() as u64,
                                    "lidar_source_unavailable"
                                );
                            } else {
                                tracing::debug!(sensor, attempts = failures, error = %e, "lidar_source_still_unavailable");
                            }
                            next_start = Some(Instant::now() + self.retry);
                        }
                        Err(e) => {
                            failures += 1;
                            tracing::warn!(sensor, error = %e, "lidar_source_start_aborted");
                            next_start = Some(Instant::now() + self.retry);
                        }
                    }
                }
            }
        }
    }
}

/// Revolutions per second between status samples.
#[derive(Debug, Default)]
struct RateMeter {
    last: Option<(Instant, u64)>,
}

impl RateMeter {
    /// A driver just started: the first status measures from here.
    fn restart(&mut self, now: Instant, s: &LidarDriverStats) {
        self.last = Some((now, s.revolutions));
    }

    /// The rate since the previous sample; `None` while the link is down or
    /// before there is a previous sample.
    fn sample(&mut self, now: Instant, s: &LidarDriverStats) -> Option<f32> {
        let (t0, r0) = self.last.replace((now, s.revolutions))?;
        let dt = now.duration_since(t0).as_secs_f64();
        (s.connected && dt > 0.0).then(|| (s.revolutions.saturating_sub(r0) as f64 / dt) as f32)
    }
}

/// The link status from a running driver's report, or the not-running
/// status (`connected: false`, nulls, zero counters) without one.
pub fn lidar_status(stats: Option<&LidarDriverStats>, rate_hz: Option<f32>) -> LidarStatus {
    let Some(s) = stats else {
        return LidarStatus::default();
    };
    LidarStatus {
        device: s.device.clone(),
        connected: s.connected,
        health: s.health.map(|(h, _)| h),
        health_code: s.health.map(|(_, code)| code),
        rate_hz: rate_hz.filter(|_| s.connected),
        scans: s.scans,
        dropped_scans: s.dropped_scans,
        reconnects: s.reconnects,
        ..LidarStatus::default()
    }
}
