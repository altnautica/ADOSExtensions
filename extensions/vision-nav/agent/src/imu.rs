//! IMU source + frame/IMU time alignment.
//!
//! Estimators do their best work when each frame is paired with the
//! IMU sample taken at the same instant. The IMU source keeps a
//! fixed-capacity ring of recent SI samples; the [`TimeAligner`] pairs
//! a camera-frame timestamp with the closest sample (linearly
//! interpolated between the two bracketing samples) and tracks the
//! rolling residual drift the GCS surfaces and the VIO pre-arm gate
//! consults.
//!
//! The universal IMU path is MAVLink `RAW_IMU` (#27): every FC
//! publishes it, in milli-g acceleration and milli-rad/s gyro. The
//! pipeline subscribes via `ctx.mavlink.subscribe("RAW_IMU", ..)` and
//! feeds decoded fields into [`ImuBuffer::record_raw_imu`].
//!
//! A higher-rate direct-I2C BMI088 source (~400 Hz, bypassing the FC
//! rate cap) is a documented TODO: it needs raw `/dev/i2c-*` register
//! reads, which the plugin SDK does not yet expose a facade for. Until
//! that lands the MAVLink path is the only source; see the module note.

use std::collections::VecDeque;
use std::sync::Mutex;

/// One IMU sample reduced to SI units. Gyro rad/s, accel m/s², body
/// frame. `ts_ns` is the agent monotonic timestamp at ingestion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuSample {
    pub ts_ns: i64,
    pub xgyro: f32,
    pub ygyro: f32,
    pub zgyro: f32,
    pub xacc: f32,
    pub yacc: f32,
    pub zacc: f32,
}

const MILLIRAD_PER_S_TO_RAD_PER_S: f32 = 0.001;
const MG_TO_M_PER_S2: f32 = 9.80665 * 0.001;

/// Source identifier surfaced on the heartbeat. The MAVLink path is the
/// only source today; the direct-I2C path will add its own id.
pub const SOURCE_ID_MAVLINK_RAW_IMU: &str = "mavlink-raw-imu";

/// Fixed-capacity ring of recent samples + a smoothed rate estimate.
/// Shared between the MAVLink subscription callback (writer) and the
/// pipeline (reader) behind a mutex.
pub struct ImuBuffer {
    inner: Mutex<Inner>,
}

struct Inner {
    buffer: VecDeque<ImuSample>,
    capacity: usize,
    last_seen_ns: Option<i64>,
    last_rate_hz: Option<f32>,
}

impl Default for ImuBuffer {
    fn default() -> Self {
        Self::with_capacity(400)
    }
}

impl ImuBuffer {
    /// A buffer holding `capacity` samples (default 400 ~= 2 s at 200 Hz).
    pub fn with_capacity(capacity: usize) -> Self {
        Self {
            inner: Mutex::new(Inner {
                buffer: VecDeque::with_capacity(capacity),
                capacity,
                last_seen_ns: None,
                last_rate_hz: None,
            }),
        }
    }

    /// Record a decoded `RAW_IMU` sample, converting the wire units
    /// (milli-g accel, milli-rad/s gyro) to SI. `ts_ns` is the agent
    /// monotonic time at ingestion.
    #[allow(clippy::too_many_arguments)]
    pub fn record_raw_imu(
        &self,
        ts_ns: i64,
        xgyro_mrad: f32,
        ygyro_mrad: f32,
        zgyro_mrad: f32,
        xacc_mg: f32,
        yacc_mg: f32,
        zacc_mg: f32,
    ) {
        self.record(ImuSample {
            ts_ns,
            xgyro: xgyro_mrad * MILLIRAD_PER_S_TO_RAD_PER_S,
            ygyro: ygyro_mrad * MILLIRAD_PER_S_TO_RAD_PER_S,
            zgyro: zgyro_mrad * MILLIRAD_PER_S_TO_RAD_PER_S,
            xacc: xacc_mg * MG_TO_M_PER_S2,
            yacc: yacc_mg * MG_TO_M_PER_S2,
            zacc: zacc_mg * MG_TO_M_PER_S2,
        });
    }

    /// Append a pre-converted SI sample (used by tests and any future
    /// direct-bus source) and refresh the rate EMA.
    pub fn record(&self, sample: ImuSample) {
        let mut g = self.inner.lock().expect("imu buffer lock");
        if let Some(last) = g.last_seen_ns {
            let dt_s = ((sample.ts_ns - last) as f32 / 1e9).max(1e-6);
            let inst = 1.0 / dt_s;
            g.last_rate_hz = Some(match g.last_rate_hz {
                None => inst,
                Some(prev) => 0.9 * prev + 0.1 * inst,
            });
        }
        g.last_seen_ns = Some(sample.ts_ns);
        if g.buffer.len() == g.capacity {
            g.buffer.pop_front();
        }
        g.buffer.push_back(sample);
    }

    /// The most recent sample, or `None` if no data yet.
    pub fn latest(&self) -> Option<ImuSample> {
        self.inner.lock().expect("imu buffer lock").buffer.back().copied()
    }

    /// A snapshot of in-buffer samples, oldest first.
    pub fn recent(&self) -> Vec<ImuSample> {
        self.inner
            .lock()
            .expect("imu buffer lock")
            .buffer
            .iter()
            .copied()
            .collect()
    }

    /// Smoothed sample rate in Hz, or `None` until two samples land.
    pub fn rate_hz(&self) -> Option<f32> {
        self.inner.lock().expect("imu buffer lock").last_rate_hz
    }
}

/// Camera-IMU sync drift band. Green ≤10 ms, yellow ≤30 ms, red above.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriftBand {
    Green,
    Yellow,
    Red,
}

const DRIFT_BAND_GREEN_MS: f32 = 10.0;
const DRIFT_BAND_YELLOW_MS: f32 = 30.0;

/// An IMU sample paired with a camera frame timestamp.
#[derive(Debug, Clone, Copy)]
pub struct AlignedSample {
    pub frame_ts_ns: i64,
    pub imu_sample: ImuSample,
    pub residual_ms: f32,
}

/// Pairs frame timestamps with IMU samples and watches drift over a
/// sliding window. The static `timeshift_cam_imu` offset (from the
/// camchain.yaml the calibration helper produces) is applied to the
/// frame timestamp before the search.
pub struct TimeAligner {
    timeshift_ns: i64,
    residuals: VecDeque<f32>,
    window: usize,
}

impl TimeAligner {
    pub fn new(timeshift_cam_imu_s: f64, window: usize) -> Self {
        Self {
            timeshift_ns: (timeshift_cam_imu_s * 1e9) as i64,
            residuals: VecDeque::with_capacity(window),
            window,
        }
    }

    /// Replace the static offset (a fresh calibration result). Clears
    /// the residual window since old residuals were against the prior
    /// offset.
    pub fn set_timeshift(&mut self, timeshift_cam_imu_s: f64) {
        self.timeshift_ns = (timeshift_cam_imu_s * 1e9) as i64;
        self.residuals.clear();
    }

    /// Pair `frame_ts_ns` with the closest IMU sample from `samples`
    /// (oldest first), interpolating between the two bracketing
    /// samples when both exist. Records the residual into the rolling
    /// window. `None` when the IMU buffer is empty.
    pub fn lookup(&mut self, frame_ts_ns: i64, samples: &[ImuSample]) -> Option<AlignedSample> {
        if samples.is_empty() {
            return None;
        }
        let target = frame_ts_ns + self.timeshift_ns;
        let mut before: Option<ImuSample> = None;
        let mut after: Option<ImuSample> = None;
        for &s in samples {
            if s.ts_ns <= target {
                before = Some(s);
            } else {
                after = Some(s);
                break;
            }
        }
        let (picked, residual_ns) = match (before, after) {
            (Some(b), Some(a)) => interpolate(b, a, target),
            (Some(b), None) => (b, (target - b.ts_ns).abs()),
            (None, Some(a)) => (a, (target - a.ts_ns).abs()),
            (None, None) => return None,
        };
        let residual_ms = residual_ns as f32 / 1e6;
        if self.residuals.len() == self.window {
            self.residuals.pop_front();
        }
        self.residuals.push_back(residual_ms);
        Some(AlignedSample {
            frame_ts_ns,
            imu_sample: picked,
            residual_ms,
        })
    }

    /// Rolling average residual in ms, or `None` when empty.
    pub fn mean_residual_ms(&self) -> Option<f32> {
        if self.residuals.is_empty() {
            return None;
        }
        Some(self.residuals.iter().sum::<f32>() / self.residuals.len() as f32)
    }

    /// Band for the current rolling residual. `Green` when no samples
    /// yet (the GCS reads this as "not running" rather than a failure).
    pub fn drift_band(&self) -> DriftBand {
        match self.mean_residual_ms() {
            None => DriftBand::Green,
            Some(m) if m <= DRIFT_BAND_GREEN_MS => DriftBand::Green,
            Some(m) if m <= DRIFT_BAND_YELLOW_MS => DriftBand::Yellow,
            Some(_) => DriftBand::Red,
        }
    }
}

/// Linearly interpolate gyro + accel between two bracketing samples,
/// returning the interpolated sample at `target` and the residual to
/// the nearest real sample.
fn interpolate(before: ImuSample, after: ImuSample, target: i64) -> (ImuSample, i64) {
    let span = (after.ts_ns - before.ts_ns).max(1);
    let w = ((target - before.ts_ns) as f64 / span as f64).clamp(0.0, 1.0) as f32;
    let lerp = |a: f32, b: f32| a + (b - a) * w;
    let s = ImuSample {
        ts_ns: target,
        xgyro: lerp(before.xgyro, after.xgyro),
        ygyro: lerp(before.ygyro, after.ygyro),
        zgyro: lerp(before.zgyro, after.zgyro),
        xacc: lerp(before.xacc, after.xacc),
        yacc: lerp(before.yacc, after.yacc),
        zacc: lerp(before.zacc, after.zacc),
    };
    let residual = (target - before.ts_ns).abs().min((target - after.ts_ns).abs());
    (s, residual)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(ts_ns: i64, g: f32) -> ImuSample {
        ImuSample {
            ts_ns,
            xgyro: g,
            ygyro: g,
            zgyro: g,
            xacc: 0.0,
            yacc: 0.0,
            zacc: 9.81,
        }
    }

    #[test]
    fn raw_imu_units_convert_to_si() {
        let buf = ImuBuffer::default();
        // 1000 mrad/s -> 1 rad/s; 1000 mg -> 9.80665 m/s².
        buf.record_raw_imu(0, 1000.0, 0.0, 0.0, 0.0, 0.0, 1000.0);
        let s = buf.latest().unwrap();
        assert!((s.xgyro - 1.0).abs() < 1e-6);
        assert!((s.zacc - 9.80665).abs() < 1e-4);
    }

    #[test]
    fn buffer_is_capacity_bounded() {
        let buf = ImuBuffer::with_capacity(3);
        for i in 0..5 {
            buf.record(sample(i * 1000, i as f32));
        }
        let recent = buf.recent();
        assert_eq!(recent.len(), 3);
        assert_eq!(recent[0].ts_ns, 2000); // oldest two evicted
    }

    #[test]
    fn rate_emerges_after_two_samples() {
        let buf = ImuBuffer::default();
        assert!(buf.rate_hz().is_none());
        buf.record(sample(0, 0.0));
        buf.record(sample(5_000_000, 0.0)); // 5 ms -> 200 Hz
        let rate = buf.rate_hz().unwrap();
        assert!((rate - 200.0).abs() < 1.0);
    }

    #[test]
    fn aligner_interpolates_between_bracket() {
        let mut a = TimeAligner::new(0.0, 60);
        let samples = vec![sample(0, 0.0), sample(10_000_000, 10.0)];
        let aligned = a.lookup(5_000_000, &samples).unwrap();
        // Halfway -> gyro ~5.
        assert!((aligned.imu_sample.xgyro - 5.0).abs() < 1e-3);
    }

    #[test]
    fn drift_band_thresholds() {
        let mut a = TimeAligner::new(0.0, 60);
        // No samples -> green.
        assert_eq!(a.drift_band(), DriftBand::Green);
        // Frame far from any IMU sample -> large residual -> red.
        let samples = vec![sample(0, 0.0)];
        a.lookup(100_000_000, &samples); // 100 ms residual
        assert_eq!(a.drift_band(), DriftBand::Red);
    }

    #[test]
    fn empty_buffer_lookup_is_none() {
        let mut a = TimeAligner::new(0.0, 60);
        assert!(a.lookup(0, &[]).is_none());
    }
}
