//! Rangefinder drivers.
//!
//! A rangefinder gives the optical-flow modes a metric scale. Two
//! sources are supported:
//!
//! * `fc_relay` — the flight controller already publishes
//!   `DISTANCE_SENSOR`; the plugin relays the latest reading. This is
//!   the default topology and the universal path (no extra wiring).
//! * `tfluna_uart` — Benewake TF-Luna on a companion serial port. A
//!   blocking reader task owns the port (raw 8N1 at the configured
//!   baud) and fills a latest-reading cell; the frame worker only reads
//!   the cell, so it never blocks on the TTY.
//!
//! An I2C rangefinder is wired to the flight controller and read through
//! `fc_relay`. The pipeline reads [`Rangefinder::read`] per frame.

use std::io::Read;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// One distance measurement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RangeReading {
    pub distance_m: f32,
    /// 0..100; 0 means invalid.
    pub quality: i32,
    pub timestamp_monotonic_ns: i64,
}

/// Common contract for downward distance sources.
pub trait Rangefinder: Send {
    /// Latest reading, or `None` when no fresh data is available.
    fn read(&mut self) -> Option<RangeReading>;
    fn min_range_m(&self) -> f32;
    fn max_range_m(&self) -> f32;
    fn name(&self) -> &'static str;
    /// Why the source cannot produce readings (for example the serial
    /// port does not open), or `None` when it is healthy.
    fn error(&self) -> Option<String> {
        None
    }
}

// ---------------------------------------------------------------------------
// FC relay (the default + universal path)
// ---------------------------------------------------------------------------

const RELAY_STALE_AFTER_NS: i64 = 200_000_000; // 200 ms
const RELAY_FALLBACK_MIN_M: f32 = 0.0;
const RELAY_FALLBACK_MAX_M: f32 = 40.0;

/// Republishes the FC's `DISTANCE_SENSOR` stream through the common
/// contract. The pipeline subscribes to `DISTANCE_SENSOR` over
/// `ctx.mavlink.subscribe` and feeds decoded fields into
/// [`RelayDistanceSensor::on_distance`]; `read` returns the latest
/// reading if fresh.
pub struct RelayDistanceSensor {
    inner: Mutex<RelayInner>,
}

struct RelayInner {
    latest: Option<RangeReading>,
    last_min_m: Option<f32>,
    last_max_m: Option<f32>,
}

impl Default for RelayDistanceSensor {
    fn default() -> Self {
        Self {
            inner: Mutex::new(RelayInner {
                latest: None,
                last_min_m: None,
                last_max_m: None,
            }),
        }
    }
}

impl RelayDistanceSensor {
    pub fn new() -> Self {
        Self::default()
    }

    /// Read the latest reading through a shared `&self` (the inner
    /// state is a mutex, so a shared reference is enough). Used by the
    /// pipeline's `Arc<RelayDistanceSensor>` wrapper.
    pub fn read_shared(&self) -> Option<RangeReading> {
        let g = self.inner.lock().expect("relay lock");
        let latest = g.latest?;
        let age = crate::mavlink_emit::monotonic_ns() - latest.timestamp_monotonic_ns;
        if age > RELAY_STALE_AFTER_NS {
            return None;
        }
        Some(latest)
    }

    /// Feed a decoded `DISTANCE_SENSOR`: distances in centimetres,
    /// `covariance` 0..255 (quality = 100 - covariance, clamped).
    pub fn on_distance(
        &self,
        current_cm: i64,
        min_cm: Option<i64>,
        max_cm: Option<i64>,
        covariance: i64,
        now_ns: i64,
    ) {
        let distance_m = current_cm as f32 / 100.0;
        let quality = (100 - covariance).clamp(0, 100) as i32;
        let mut g = self.inner.lock().expect("relay lock");
        if let Some(c) = min_cm {
            g.last_min_m = Some(c as f32 / 100.0);
        }
        if let Some(c) = max_cm {
            g.last_max_m = Some(c as f32 / 100.0);
        }
        g.latest = Some(RangeReading {
            distance_m,
            quality,
            timestamp_monotonic_ns: now_ns,
        });
    }
}

impl Rangefinder for RelayDistanceSensor {
    fn read(&mut self) -> Option<RangeReading> {
        self.read_shared()
    }
    fn min_range_m(&self) -> f32 {
        self.inner
            .lock()
            .expect("relay lock")
            .last_min_m
            .unwrap_or(RELAY_FALLBACK_MIN_M)
    }
    fn max_range_m(&self) -> f32 {
        self.inner
            .lock()
            .expect("relay lock")
            .last_max_m
            .unwrap_or(RELAY_FALLBACK_MAX_M)
    }
    fn name(&self) -> &'static str {
        "fc_relay"
    }
}

// ---------------------------------------------------------------------------
// TF-Luna UART
// ---------------------------------------------------------------------------

pub const TFLUNA_FRAME_LEN: usize = 9;
const TFLUNA_HEADER: u8 = 0x59;
const TFLUNA_TOP_QUALITY_SIGNAL: f32 = 20000.0;

/// Parse one 9-byte TF-Luna packet. Returns `(distance_m, quality,
/// signal_strength)` on a valid header + checksum, else `None`.
///
/// Frame: `0x59 0x59 | dist_lo dist_hi | sig_lo sig_hi | temp_lo
/// temp_hi | checksum`, checksum = sum(bytes 0..8) & 0xFF.
pub fn parse_tfluna_frame(frame: &[u8]) -> Option<(f32, i32, i32)> {
    if frame.len() != TFLUNA_FRAME_LEN {
        return None;
    }
    if frame[0] != TFLUNA_HEADER || frame[1] != TFLUNA_HEADER {
        return None;
    }
    let checksum: u8 = frame[0..8].iter().fold(0u8, |a, &b| a.wrapping_add(b));
    if checksum != frame[8] {
        return None;
    }
    let distance_cm = frame[2] as u16 | ((frame[3] as u16) << 8);
    let signal = frame[4] as u16 | ((frame[5] as u16) << 8);
    let distance_m = distance_cm as f32 / 100.0;
    let quality = ((signal as f32 / (TFLUNA_TOP_QUALITY_SIGNAL / 100.0)) as i32).clamp(0, 100);
    Some((distance_m, quality, signal as i32))
}

/// Scan a byte buffer for the next valid frame. Returns
/// `(frame, remaining_after)`; on no frame, preserves a short tail so a
/// header straddling the read boundary matches next call.
pub fn find_tfluna_frame(buffer: &[u8]) -> (Option<[u8; TFLUNA_FRAME_LEN]>, Vec<u8>) {
    let n = buffer.len();
    let mut i = 0;
    while i + TFLUNA_FRAME_LEN <= n {
        if buffer[i] == TFLUNA_HEADER && buffer[i + 1] == TFLUNA_HEADER {
            let candidate = &buffer[i..i + TFLUNA_FRAME_LEN];
            if parse_tfluna_frame(candidate).is_some() {
                let mut f = [0u8; TFLUNA_FRAME_LEN];
                f.copy_from_slice(candidate);
                return (Some(f), buffer[i + TFLUNA_FRAME_LEN..].to_vec());
            }
        }
        i += 1;
    }
    let tail_start = n.saturating_sub(TFLUNA_FRAME_LEN - 1);
    (None, buffer[tail_start..].to_vec())
}

/// Retry interval for a TF-Luna port that is absent or failed.
pub const TFLUNA_RETRY: Duration = Duration::from_secs(2);
const TFLUNA_READ_TIMEOUT: Duration = Duration::from_millis(100);
const TFLUNA_STALE_AFTER: Duration = Duration::from_millis(200);

/// The latest TF-Luna reading: distance (m), quality (0..100), and when
/// it arrived.
pub type TfLunaCell = Arc<Mutex<Option<(f32, i32, Instant)>>>;

/// TF-Luna over a serial port. [`TfLunaUart::spawn`] starts the reader
/// on a blocking task; dropping the handle stops it.
pub struct TfLunaUart {
    cell: TfLunaCell,
    error: Arc<Mutex<Option<String>>>,
    attempts: Arc<AtomicU64>,
    stop: Arc<AtomicBool>,
}

impl TfLunaUart {
    /// Start the reader for `device` at `baud`. Opening is retried every
    /// [`TFLUNA_RETRY`] with no cap; a failure is reported by
    /// [`Rangefinder::error`] until the port opens.
    pub fn spawn(device: impl Into<String>, baud: u32) -> Self {
        let device = device.into();
        let me = Self {
            cell: Arc::new(Mutex::new(None)),
            error: Arc::new(Mutex::new(None)),
            attempts: Arc::new(AtomicU64::new(0)),
            stop: Arc::new(AtomicBool::new(false)),
        };
        let (cell, error, attempts, stop) = (
            me.cell.clone(),
            me.error.clone(),
            me.attempts.clone(),
            me.stop.clone(),
        );
        tokio::task::spawn_blocking(move || {
            while !stop.load(Ordering::Relaxed) {
                attempts.fetch_add(1, Ordering::Relaxed);
                let opened = serialport::new(device.as_str(), baud)
                    .data_bits(serialport::DataBits::Eight)
                    .parity(serialport::Parity::None)
                    .stop_bits(serialport::StopBits::One)
                    .flow_control(serialport::FlowControl::None)
                    .timeout(TFLUNA_READ_TIMEOUT)
                    .open();
                let why = match opened {
                    Ok(mut port) => {
                        eprintln!("vision-nav: tfluna opened {device} at {baud} baud 8N1");
                        *error.lock().expect("tfluna error lock") = None;
                        let e = read_tfluna_stream(&mut port, &cell, &stop);
                        format!("tfluna {device} read failed: {e}")
                    }
                    Err(e) => format!("tfluna {device} open failed: {e}"),
                };
                if stop.load(Ordering::Relaxed) {
                    return;
                }
                let mut slot = error.lock().expect("tfluna error lock");
                if slot.as_deref() != Some(why.as_str()) {
                    eprintln!(
                        "vision-nav: {why}; retrying every {}s",
                        TFLUNA_RETRY.as_secs()
                    );
                }
                *slot = Some(why);
                drop(slot);
                std::thread::sleep(TFLUNA_RETRY);
            }
        });
        me
    }

    /// How many times the reader has tried to open the port.
    pub fn open_attempts(&self) -> u64 {
        self.attempts.load(Ordering::Relaxed)
    }
}

impl Drop for TfLunaUart {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
    }
}

/// Drain `port` into `cell` until a read fails (a read timeout is not a
/// failure) or `stop` is set. Returns the error that ended the stream.
pub fn read_tfluna_stream<R: Read + ?Sized>(
    port: &mut R,
    cell: &TfLunaCell,
    stop: &AtomicBool,
) -> std::io::Error {
    let mut buffer: Vec<u8> = Vec::with_capacity(64);
    let mut chunk = [0u8; 64];
    loop {
        if stop.load(Ordering::Relaxed) {
            return std::io::Error::new(std::io::ErrorKind::Interrupted, "stopped");
        }
        match port.read(&mut chunk) {
            Ok(0) => std::thread::sleep(TFLUNA_READ_TIMEOUT),
            Ok(n) => {
                buffer.extend_from_slice(&chunk[..n]);
                loop {
                    let (frame, remaining) = find_tfluna_frame(&buffer);
                    buffer = remaining;
                    let Some(f) = frame else { break };
                    if let Some((d, q, _sig)) = parse_tfluna_frame(&f) {
                        *cell.lock().expect("tfluna cell lock") = Some((d, q, Instant::now()));
                    }
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
            Err(e) => return e,
        }
    }
}

impl Rangefinder for TfLunaUart {
    fn read(&mut self) -> Option<RangeReading> {
        let (distance_m, quality, at) = (*self.cell.lock().expect("tfluna cell lock"))?;
        if at.elapsed() > TFLUNA_STALE_AFTER {
            return None;
        }
        let age_ns = at.elapsed().as_nanos() as i64;
        Some(RangeReading {
            distance_m,
            quality,
            timestamp_monotonic_ns: crate::mavlink_emit::monotonic_ns() - age_ns,
        })
    }
    fn min_range_m(&self) -> f32 {
        0.2
    }
    fn max_range_m(&self) -> f32 {
        8.0
    }
    fn name(&self) -> &'static str {
        "tfluna_uart"
    }
    fn error(&self) -> Option<String> {
        self.error.lock().expect("tfluna error lock").clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tfluna_packet(distance_cm: u16, signal: u16) -> [u8; 9] {
        let mut f = [0u8; 9];
        f[0] = 0x59;
        f[1] = 0x59;
        f[2] = (distance_cm & 0xFF) as u8;
        f[3] = (distance_cm >> 8) as u8;
        f[4] = (signal & 0xFF) as u8;
        f[5] = (signal >> 8) as u8;
        f[6] = 0;
        f[7] = 0;
        f[8] = f[0..8].iter().fold(0u8, |a, &b| a.wrapping_add(b));
        f
    }

    #[test]
    fn tfluna_parses_a_valid_frame() {
        let f = tfluna_packet(150, 10000);
        let (dist, q, sig) = parse_tfluna_frame(&f).unwrap();
        assert!((dist - 1.5).abs() < 1e-4);
        assert_eq!(sig, 10000);
        assert!(q > 0 && q <= 100);
    }

    #[test]
    fn tfluna_rejects_bad_checksum() {
        let mut f = tfluna_packet(150, 10000);
        f[8] = f[8].wrapping_add(1);
        assert!(parse_tfluna_frame(&f).is_none());
    }

    #[test]
    fn tfluna_resyncs_in_a_noisy_stream() {
        // Garbage, then a valid frame, then a trailing partial header.
        let valid = tfluna_packet(200, 5000);
        let mut stream = vec![0x00, 0xAB, 0x59];
        stream.extend_from_slice(&valid);
        stream.push(0x59); // partial next header
        let (frame, remaining) = find_tfluna_frame(&stream);
        assert!(frame.is_some());
        let (dist, _, _) = parse_tfluna_frame(&frame.unwrap()).unwrap();
        assert!((dist - 2.0).abs() < 1e-4);
        // The partial header tail is preserved.
        assert!(remaining.contains(&0x59));
    }

    #[test]
    fn relay_goes_stale() {
        let relay = RelayDistanceSensor::new();
        let now = crate::mavlink_emit::monotonic_ns();
        relay.on_distance(150, Some(20), Some(800), 10, now - RELAY_STALE_AFTER_NS - 1);
        let mut r = relay;
        assert!(r.read().is_none(), "stale reading dropped");
    }

    #[test]
    fn relay_fresh_reading_passes() {
        let relay = RelayDistanceSensor::new();
        relay.on_distance(
            150,
            Some(20),
            Some(800),
            10,
            crate::mavlink_emit::monotonic_ns(),
        );
        let mut r = relay;
        let reading = r.read().unwrap();
        assert!((reading.distance_m - 1.5).abs() < 1e-4);
        assert_eq!(reading.quality, 90);
    }

    /// A reader that yields a few chunks and then fails.
    struct Scripted(Vec<Vec<u8>>);
    impl Read for Scripted {
        fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
            if self.0.is_empty() {
                return Err(std::io::Error::new(std::io::ErrorKind::BrokenPipe, "gone"));
            }
            let chunk = self.0.remove(0);
            buf[..chunk.len()].copy_from_slice(&chunk);
            Ok(chunk.len())
        }
    }

    #[test]
    fn tfluna_stream_fills_the_cell_across_split_reads() {
        let packet = tfluna_packet(250, 8000);
        let mut port = Scripted(vec![packet[..4].to_vec(), packet[4..].to_vec()]);
        let cell: TfLunaCell = Arc::new(Mutex::new(None));
        let err = read_tfluna_stream(&mut port, &cell, &AtomicBool::new(false));
        assert_eq!(err.kind(), std::io::ErrorKind::BrokenPipe);
        let (d, _q, _at) = cell.lock().unwrap().unwrap();
        assert!((d - 2.5).abs() < 1e-4);
    }

    #[cfg(unix)]
    #[test]
    fn tfluna_reader_pty() {
        use std::io::Write;
        let (mut master, mut slave) = serialport::TTYPort::pair().expect("pty pair");
        let cell: TfLunaCell = Arc::new(Mutex::new(None));
        let stop = Arc::new(AtomicBool::new(false));
        let (c, s) = (cell.clone(), stop.clone());
        let reader = std::thread::spawn(move || {
            serialport::SerialPort::set_timeout(&mut slave, TFLUNA_READ_TIMEOUT).unwrap();
            read_tfluna_stream(&mut slave, &c, &s)
        });
        master.write_all(&tfluna_packet(180, 9000)).unwrap();
        let deadline = Instant::now() + Duration::from_secs(2);
        while cell.lock().unwrap().is_none() && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(10));
        }
        stop.store(true, Ordering::Relaxed);
        reader.join().unwrap();
        let (d, _q, _at) = cell.lock().unwrap().expect("reading over the pty");
        assert!((d - 1.8).abs() < 1e-4);
    }

    #[tokio::test(flavor = "multi_thread", worker_threads = 2)]
    async fn tfluna_absent_retries() {
        let mut rf = TfLunaUart::spawn("/dev/ttyUSB99-absent", 115_200);
        tokio::time::sleep(TFLUNA_RETRY + Duration::from_millis(500)).await;
        assert!(rf.open_attempts() >= 2, "attempts {}", rf.open_attempts());
        assert!(rf.error().unwrap().contains("open failed"));
        assert!(rf.read().is_none());
    }
}
