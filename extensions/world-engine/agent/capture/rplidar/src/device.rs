//! A running RPLIDAR: the start-up handshake, the reader thread that turns
//! the node stream into revolutions, and reconnection.
//!
//! Start-up is `STOP`, a 10 ms settle, discard stale input, `GET_INFO`,
//! `GET_HEALTH` (status error fails), then `SCAN`. The reader thread stamps
//! each received chunk with host monotonic time and hands it to the
//! [`RevolutionAssembler`]. A link that errors, closes or goes quiet for
//! [`STALL_TIMEOUT`] is reopened every [`RECONNECT_DELAY`], without limit,
//! until it answers again or the driver is dropped. Dropping the driver (or
//! its receiver) sends `STOP`.

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::{self, Receiver, SyncSender, TrySendError};
use std::sync::Arc;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use crate::clock::mono_ns;
use crate::discover;
use crate::error::RplidarError;
use crate::proto::{DeviceInfo, Health, HealthStatus};
use crate::revolution::{RawScan, RevolutionAssembler};
use crate::transport::{self, Link};

/// The device's UART rate.
pub const DEFAULT_BAUD: u32 = 460_800;
/// `device` value that selects USB discovery.
pub const AUTO_DEVICE: &str = "auto";
/// Wait between reconnect attempts.
pub const RECONNECT_DELAY: Duration = Duration::from_secs(2);
/// A scanning link silent this long is treated as lost.
pub const STALL_TIMEOUT: Duration = Duration::from_secs(1);
/// How long each request may take to be answered.
pub const COMMAND_TIMEOUT: Duration = Duration::from_secs(1);
/// Revolutions buffered for the consumer before new ones are dropped.
pub const CHANNEL_DEPTH: usize = 4;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RplidarConfig {
    /// Serial port path, `tcp:<host>:<port>`, or `auto` (USB discovery).
    pub device: String,
    pub baud: u32,
    /// Rig sensor id, for logs.
    pub sensor_id: String,
}

impl Default for RplidarConfig {
    fn default() -> Self {
        Self {
            device: AUTO_DEVICE.to_string(),
            baud: DEFAULT_BAUD,
            sensor_id: "lidar0".to_string(),
        }
    }
}

/// Counters since start.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct RplidarStats {
    /// The link is up and scanning.
    pub connected: bool,
    pub revolutions: u64,
    pub points: u64,
    /// Nodes with no return.
    pub no_return: u64,
    /// Bytes skipped to regain node framing.
    pub resync_bytes: u64,
    /// Revolutions dropped because the consumer fell behind.
    pub dropped_scans: u64,
    /// Successful reconnects after a lost link.
    pub reconnects: u64,
}

#[derive(Default)]
struct Shared {
    stop: AtomicBool,
    connected: AtomicBool,
    revolutions: AtomicU64,
    points: AtomicU64,
    no_return: AtomicU64,
    resync_bytes: AtomicU64,
    dropped_scans: AtomicU64,
    reconnects: AtomicU64,
}

/// A scanning device; revolutions arrive on the receiver `start` returns.
pub struct Rplidar {
    shared: Arc<Shared>,
    path: String,
    info: DeviceInfo,
    health: Health,
    thread: Option<JoinHandle<()>>,
}

struct Session {
    link: Box<dyn Link>,
    path: String,
    info: DeviceInfo,
    health: Health,
}

impl Rplidar {
    /// Open the device, run the handshake and start scanning. Fails when the
    /// device cannot be opened, does not answer, or reports a health error;
    /// once started, a lost link is retried in the background.
    pub fn start(cfg: RplidarConfig) -> Result<(Self, Receiver<RawScan>), RplidarError> {
        let session = connect(&cfg)?;
        let (tx, rx) = mpsc::sync_channel(CHANNEL_DEPTH);
        let shared = Arc::new(Shared::default());
        shared.connected.store(true, Ordering::Relaxed);
        let (path, info, health) = (session.path.clone(), session.info, session.health);
        let thread = {
            let shared = shared.clone();
            thread::Builder::new()
                .name(format!("rplidar-{}", cfg.sensor_id))
                .spawn(move || run(cfg, session.link, &shared, &tx))?
        };
        Ok((
            Self {
                shared,
                path,
                info,
                health,
                thread: Some(thread),
            },
            rx,
        ))
    }

    /// The port the device was first found on.
    pub fn path(&self) -> &str {
        &self.path
    }

    /// `GET_INFO` at start-up.
    pub fn info(&self) -> &DeviceInfo {
        &self.info
    }

    /// `GET_HEALTH` at start-up.
    pub fn health(&self) -> &Health {
        &self.health
    }

    pub fn stats(&self) -> RplidarStats {
        let s = &self.shared;
        let ld = |a: &AtomicU64| a.load(Ordering::Relaxed);
        RplidarStats {
            connected: s.connected.load(Ordering::Relaxed),
            revolutions: ld(&s.revolutions),
            points: ld(&s.points),
            no_return: ld(&s.no_return),
            resync_bytes: ld(&s.resync_bytes),
            dropped_scans: ld(&s.dropped_scans),
            reconnects: ld(&s.reconnects),
        }
    }
}

impl Drop for Rplidar {
    fn drop(&mut self) {
        self.shared.stop.store(true, Ordering::Relaxed);
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
    }
}

fn connect(cfg: &RplidarConfig) -> Result<Session, RplidarError> {
    let path = if cfg.device == AUTO_DEVICE {
        discover::find(cfg.baud)?.0
    } else {
        cfg.device.clone()
    };
    let mut link = transport::open(&path, cfg.baud)?;
    transport::stop_and_drain(link.as_mut())?;
    let info = transport::query_info(link.as_mut(), COMMAND_TIMEOUT)?;
    let health = transport::query_health(link.as_mut(), COMMAND_TIMEOUT)?;
    match health.status {
        HealthStatus::Good => {}
        HealthStatus::Error => return Err(RplidarError::Health(health.error_code)),
        HealthStatus::Warning | HealthStatus::Unknown(_) => tracing::warn!(
            sensor = %cfg.sensor_id,
            status = ?health.status,
            code = health.error_code,
            "rplidar_health_warning"
        ),
    }
    transport::start_scan(link.as_mut(), COMMAND_TIMEOUT)?;
    tracing::info!(
        sensor = %cfg.sensor_id,
        path = %path,
        model = info.model,
        firmware = %info.firmware(),
        hardware = info.hardware,
        serial = %info.serial_hex(),
        "rplidar_scan_started"
    );
    Ok(Session {
        link,
        path,
        info,
        health,
    })
}

enum End {
    /// Driver or receiver dropped.
    Stopped,
    Lost(String),
}

fn run(cfg: RplidarConfig, mut link: Box<dyn Link>, shared: &Shared, tx: &SyncSender<RawScan>) {
    let mut asm = RevolutionAssembler::new(cfg.baud);
    loop {
        match stream(link.as_mut(), &mut asm, shared, tx) {
            End::Stopped => {
                // Draining after STOP leaves nothing unread, so the link
                // closes cleanly behind the request.
                let _ = transport::stop_and_drain(link.as_mut());
                return;
            }
            End::Lost(why) => {
                tracing::warn!(sensor = %cfg.sensor_id, reason = %why, "rplidar_link_lost");
            }
        }
        shared.connected.store(false, Ordering::Relaxed);
        drop(link);
        asm.reset();
        link = loop {
            if !sleep_unless_stopped(RECONNECT_DELAY, shared) {
                return;
            }
            match connect(&cfg) {
                Ok(s) => break s.link,
                Err(e) => tracing::warn!(
                    sensor = %cfg.sensor_id,
                    error = %e,
                    "rplidar_reconnect_failed"
                ),
            }
        };
        shared.reconnects.fetch_add(1, Ordering::Relaxed);
        shared.connected.store(true, Ordering::Relaxed);
    }
}

fn stream(
    link: &mut dyn Link,
    asm: &mut RevolutionAssembler,
    shared: &Shared,
    tx: &SyncSender<RawScan>,
) -> End {
    let mut buf = [0u8; 4096];
    let mut last_rx = Instant::now();
    loop {
        if shared.stop.load(Ordering::Relaxed) {
            return End::Stopped;
        }
        match link.read(&mut buf) {
            Ok(0) => return End::Lost("link closed".into()),
            Ok(n) => {
                let t_rx = mono_ns();
                last_rx = Instant::now();
                let mut receiver_gone = false;
                asm.push(&buf[..n], t_rx, |scan| {
                    shared
                        .points
                        .fetch_add(scan.len() as u64, Ordering::Relaxed);
                    match tx.try_send(scan) {
                        Ok(()) => {}
                        Err(TrySendError::Full(_)) => {
                            shared.dropped_scans.fetch_add(1, Ordering::Relaxed);
                        }
                        Err(TrySendError::Disconnected(_)) => receiver_gone = true,
                    }
                });
                let st = asm.stats();
                shared.revolutions.store(st.revolutions, Ordering::Relaxed);
                shared.no_return.store(st.no_return, Ordering::Relaxed);
                shared
                    .resync_bytes
                    .store(st.resync_bytes, Ordering::Relaxed);
                if receiver_gone {
                    return End::Stopped;
                }
            }
            Err(e) if transport::is_timeout(&e) || e.kind() == std::io::ErrorKind::Interrupted => {
                if last_rx.elapsed() >= STALL_TIMEOUT {
                    return End::Lost(format!("no data for {} ms", STALL_TIMEOUT.as_millis()));
                }
            }
            Err(e) => return End::Lost(e.to_string()),
        }
    }
}

/// Sleep `d` in short steps; `false` when the driver was dropped meanwhile.
fn sleep_unless_stopped(d: Duration, shared: &Shared) -> bool {
    let step = Duration::from_millis(50);
    let until = Instant::now() + d;
    while Instant::now() < until {
        if shared.stop.load(Ordering::Relaxed) {
            return false;
        }
        thread::sleep(step.min(until.saturating_duration_since(Instant::now())));
    }
    !shared.stop.load(Ordering::Relaxed)
}
