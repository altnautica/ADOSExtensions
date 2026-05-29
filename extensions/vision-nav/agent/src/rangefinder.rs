//! Rangefinder drivers.
//!
//! A rangefinder gives the optical-flow modes a metric scale. Four
//! sources are supported:
//!
//! * `fc_relay` — the flight controller already publishes
//!   `DISTANCE_SENSOR`; the plugin relays the latest reading. This is
//!   the default topology and the universal path (no extra wiring).
//! * `tfluna_uart` — Benewake TF-Luna over UART. The 9-byte frame
//!   parser is pure and fully ported.
//! * `garmin_lidarlite_i2c` / `vl53l1x_i2c` — I2C sensors. The register
//!   protocol needs raw `/dev/i2c-*` ioctls (the plugin SDK does not
//!   yet expose an I2C facade), so these are documented stubs that fail
//!   safe by returning no reading; see [`I2cRangefinder`]. The
//!   `fc_relay` path covers I2C sensors wired to the FC instead.
//!
//! The pipeline reads [`Rangefinder::read`] per frame.

use std::sync::Mutex;

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
        self.inner.lock().expect("relay lock").last_min_m.unwrap_or(RELAY_FALLBACK_MIN_M)
    }
    fn max_range_m(&self) -> f32 {
        self.inner.lock().expect("relay lock").last_max_m.unwrap_or(RELAY_FALLBACK_MAX_M)
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

/// TF-Luna over a UART device. The frame parser above is the tested
/// core; the live driver opens the serial node and drains it per read.
pub struct TfLunaUart {
    device: String,
    baud: u32,
    file: Option<std::fs::File>,
    buffer: Vec<u8>,
}

impl TfLunaUart {
    pub fn new(device: impl Into<String>, baud: u32) -> Self {
        Self {
            device: device.into(),
            baud,
            file: None,
            buffer: Vec::new(),
        }
    }

    /// Open the serial node. The serial line discipline (baud,
    /// 8N1, raw) is left to the system default `stty`/agent provisioning
    /// for the device; the TF-Luna ships 115200 8N1 out of the box,
    /// which is the common bench config. Returns whether the node
    /// opened.
    pub fn open(&mut self) -> bool {
        match std::fs::OpenOptions::new().read(true).open(&self.device) {
            Ok(f) => {
                eprintln!(
                    "vision-nav: tfluna opened {} (expecting {} baud 8N1)",
                    self.device, self.baud
                );
                self.file = Some(f);
                true
            }
            Err(e) => {
                eprintln!("vision-nav: tfluna open {} failed: {e}", self.device);
                false
            }
        }
    }
}

impl Rangefinder for TfLunaUart {
    fn read(&mut self) -> Option<RangeReading> {
        use std::io::Read;
        let file = self.file.as_mut()?;
        let mut chunk = [0u8; 64];
        if let Ok(n) = file.read(&mut chunk) {
            self.buffer.extend_from_slice(&chunk[..n]);
            // Bound the buffer.
            if self.buffer.len() > 256 {
                let drop = self.buffer.len() - 256;
                self.buffer.drain(0..drop);
            }
        }
        let mut latest: Option<(f32, i32, i32)> = None;
        loop {
            let (frame, remaining) = find_tfluna_frame(&self.buffer);
            self.buffer = remaining;
            match frame {
                Some(f) => {
                    if let Some(parsed) = parse_tfluna_frame(&f) {
                        latest = Some(parsed);
                    }
                }
                None => break,
            }
        }
        let (distance_m, quality, _sig) = latest?;
        Some(RangeReading {
            distance_m,
            quality,
            timestamp_monotonic_ns: crate::mavlink_emit::monotonic_ns(),
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
    // baud is captured for future termios configuration once the
    // hardware.uart facade exposes line-discipline control.
}

// ---------------------------------------------------------------------------
// I2C rangefinders (documented stubs — fail safe)
// ---------------------------------------------------------------------------

/// Garmin LIDAR-Lite / ST VL53L1X over I2C.
///
/// TODO: Rust-native I2C register access. The Garmin LIDAR-Lite and the
/// VL53L1X both need raw `/dev/i2c-*` register reads (SMBus ioctls); the
/// plugin SDK does not yet expose an I2C facade, and the prior Python
/// drivers leaned on `smbus2` / the upstream `VL53L1X` library. Until an
/// I2C facade lands, an I2C-wired rangefinder should be wired to the FC
/// instead and consumed through the `fc_relay` topology, which is fully
/// supported. This stub fails safe: [`read`](Rangefinder::read) always
/// returns `None`, so the OF estimator falls back to the
/// `optical_flow_degraded` scale ladder rather than feeding a wrong
/// distance into the EKF.
pub struct I2cRangefinder {
    name: &'static str,
    min_m: f32,
    max_m: f32,
}

impl I2cRangefinder {
    pub fn garmin_lidarlite(_bus: u32) -> Self {
        Self {
            name: "garmin_lidarlite_i2c",
            min_m: 0.05,
            max_m: 40.0,
        }
    }
    pub fn vl53l1x(_bus: u32) -> Self {
        Self {
            name: "vl53l1x_i2c",
            min_m: 0.04,
            max_m: 4.0,
        }
    }
}

impl Rangefinder for I2cRangefinder {
    fn read(&mut self) -> Option<RangeReading> {
        // Fail safe: no reading until the I2C facade lands. Never a
        // fabricated distance into the EKF.
        None
    }
    fn min_range_m(&self) -> f32 {
        self.min_m
    }
    fn max_range_m(&self) -> f32 {
        self.max_m
    }
    fn name(&self) -> &'static str {
        self.name
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
        relay.on_distance(150, Some(20), Some(800), 10, crate::mavlink_emit::monotonic_ns());
        let mut r = relay;
        let reading = r.read().unwrap();
        assert!((reading.distance_m - 1.5).abs() < 1e-4);
        assert_eq!(reading.quality, 90);
    }

    #[test]
    fn i2c_stub_fails_safe() {
        let mut g = I2cRangefinder::garmin_lidarlite(1);
        assert!(g.read().is_none());
        assert_eq!(g.name(), "garmin_lidarlite_i2c");
    }
}
