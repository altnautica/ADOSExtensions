//! Mains flicker: detection from the per-frame mean intensity, and the
//! exposure rule that avoids it.
//!
//! Lamps on 50 Hz mains flicker at 100 Hz. A camera sampling at `fs` sees
//! that flicker aliased to `|100 − k·fs|` (folded into `0..fs/2`); at the
//! default 20 Hz trigger rate the alias is DC, which is why that rate is the
//! default. The detector keeps a 2 s series of frame mean intensities, takes
//! its amplitude spectrum by direct DFT at the real frame times (frames are not
//! perfectly periodic), and reports the strongest non-DC peak as a percentage
//! of the mean: over [`FLICKER_WARN_PCT`] raises `flicker`.
//!
//! The exposure rule (A9): an exposure that is a whole multiple of the flicker
//! period (10 ms) integrates whole cycles and is flicker-free; one at or under
//! 2 ms is short enough that the per-frame swing is bounded. Anything between
//! is moved to the nearest safe value with the gain changed to keep the
//! product (the image brightness) the same.

use std::collections::VecDeque;

use world_engine_protocol::capture_ctl::{ExposureMode, ExposureView};

/// Window of the mean-intensity series.
pub const WINDOW_NS: u64 = 2_000_000_000;
/// Flicker amplitude (percent of the mean) that raises the warning.
pub const FLICKER_WARN_PCT: f64 = 3.0;
/// Amplitude at which the chip turns bad.
pub const FLICKER_BAD_PCT: f64 = 10.0;
/// Lowest frequency the detector scans (below is scene change, not flicker).
const F_MIN_HZ: f64 = 1.0;
const F_STEP_HZ: f64 = 0.25;

/// Per-camera flicker detector.
#[derive(Debug, Default, Clone)]
pub struct FlickerDetector {
    series: VecDeque<(u64, f64)>,
}

impl FlickerDetector {
    pub fn push(&mut self, t_ns: u64, mean: f64) {
        self.series.push_back((t_ns, mean));
        let horizon = t_ns.saturating_sub(WINDOW_NS);
        while self.series.front().is_some_and(|(t, _)| *t < horizon) {
            self.series.pop_front();
        }
    }

    /// The strongest non-DC spectral amplitude, percent of the mean, or `None`
    /// before a usable window (16 frames) has been seen.
    pub fn amplitude_pct(&self) -> Option<f64> {
        let n = self.series.len();
        if n < 16 {
            return None;
        }
        let t0 = self.series.front()?.0;
        let span_s = (self.series.back()?.0 - t0) as f64 * 1e-9;
        if span_s <= 0.0 {
            return None;
        }
        let fs = (n - 1) as f64 / span_s;
        let mean = self.series.iter().map(|(_, v)| v).sum::<f64>() / n as f64;
        if mean <= 0.0 {
            return None;
        }
        // Remove the linear trend: a light switched on mid-window is a ramp,
        // and a ramp leaks into every low bin.
        let ts: Vec<f64> = self
            .series
            .iter()
            .map(|(t, _)| (t - t0) as f64 * 1e-9)
            .collect();
        let tm = ts.iter().sum::<f64>() / n as f64;
        let (mut sxy, mut sxx) = (0.0, 0.0);
        for (t, (_, v)) in ts.iter().zip(self.series.iter()) {
            sxy += (t - tm) * (v - mean);
            sxx += (t - tm) * (t - tm);
        }
        let slope = if sxx > 0.0 { sxy / sxx } else { 0.0 };
        let resid: Vec<f64> = ts
            .iter()
            .zip(self.series.iter())
            .map(|(t, (_, v))| v - mean - slope * (t - tm))
            .collect();
        let mut best = 0.0f64;
        let mut f = F_MIN_HZ.max(2.0 / span_s);
        while f <= fs / 2.0 {
            let w = std::f64::consts::TAU * f;
            let (mut re, mut im) = (0.0, 0.0);
            for (t, r) in ts.iter().zip(resid.iter()) {
                re += r * (w * t).cos();
                im -= r * (w * t).sin();
            }
            best = best.max(2.0 * (re * re + im * im).sqrt() / n as f64);
            f += F_STEP_HZ;
        }
        Some(100.0 * best / mean)
    }
}

/// The flicker period for a mains frequency (half the mains period).
pub fn flicker_period_us(mains_hz: u32) -> u32 {
    1_000_000 / (2 * mains_hz.max(1))
}

/// Exposure cap per mode, microseconds.
pub fn exposure_cap_us(mode: ExposureMode) -> u32 {
    match mode {
        ExposureMode::Walking => 4_000,
        ExposureMode::Slow => 10_000,
        ExposureMode::Short => 2_000,
    }
}

/// Longest exposure that is flicker-safe by being short.
pub const SHORT_EXPOSURE_US: u32 = 2_000;
/// Highest analogue gain the rule will ask for.
pub const MAX_GAIN: f32 = 16.0;

/// The flicker-safe `(exposure_us, gain)` closest to a requested pair,
/// keeping `exposure × gain` where the gain range allows. The safe values
/// under the mode cap are anything up to [`SHORT_EXPOSURE_US`] and whole
/// flicker periods; the nearest one to the (capped) request wins, the
/// shorter on a tie.
pub fn quantise_exposure(
    req_us: u32,
    req_gain: f32,
    mode: ExposureMode,
    mains_hz: u32,
) -> (u32, f32) {
    let period = flicker_period_us(mains_hz).max(1);
    let cap = exposure_cap_us(mode).max(1);
    let wanted = req_us.clamp(1, cap);
    let product = req_us.max(1) as f64 * req_gain.max(1.0) as f64;
    let exposure = if wanted <= SHORT_EXPOSURE_US {
        wanted
    } else {
        let below = (wanted / period) * period;
        let above = below + period;
        [SHORT_EXPOSURE_US.min(cap), below, above]
            .into_iter()
            .filter(|&c| c > 0 && c <= cap)
            .min_by_key(|&c| (c.abs_diff(wanted), c))
            .unwrap_or(SHORT_EXPOSURE_US.min(cap))
    };
    let gain = ((product / exposure as f64) as f32).clamp(1.0, MAX_GAIN);
    (exposure, gain)
}

/// Fold an exposure request into the current view: missing fields keep
/// their current values and the result is re-quantised against the
/// (possibly new) mode cap. The camera control path does not exist yet, so
/// the view is never `locked` or `applied`; the values are what was asked.
pub fn settle_exposure(cur: ExposureView, req: ExposureView, mains_hz: u32) -> ExposureView {
    let mode = req.mode.or(cur.mode).unwrap_or_default();
    let gain = req.gain.or(cur.gain);
    let (exposure_us, gain) = match req.exposure_us.or(cur.exposure_us) {
        Some(us) => {
            let (e, g) = quantise_exposure(us, gain.unwrap_or(1.0), mode, mains_hz);
            (Some(e), Some(g))
        }
        None => (None, gain),
    };
    ExposureView {
        locked: false,
        exposure_us,
        gain,
        mode: Some(mode),
        applied: false,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Mean intensity of a frame exposed `exp_s` starting at `t` under a light
    /// whose output is `1 + a·sin(2π·100·t)`.
    fn exposed_mean(t: f64, exp_s: f64, a: f64) -> f64 {
        let w = std::f64::consts::TAU * 100.0;
        let integral = exp_s - a / w * ((w * (t + exp_s)).cos() - (w * t).cos());
        120.0 * integral / exp_s
    }

    fn run(fps: f64, exp_s: f64, a: f64) -> f64 {
        let mut d = FlickerDetector::default();
        for i in 0..(2.0 * fps) as u64 {
            let t = i as f64 / fps;
            d.push((t * 1e9) as u64 + 1_000_000_000, exposed_mean(t, exp_s, a));
        }
        d.amplitude_pct().unwrap()
    }

    #[test]
    fn hundred_hertz_flicker_is_seen_through_its_alias() {
        // 30 fps aliases 100 Hz to 10 Hz; a 1 ms exposure keeps most of it.
        let amp = run(30.0, 0.001, 0.3);
        assert!(amp > FLICKER_WARN_PCT * 3.0, "{amp}");
        // A steady light has none.
        assert!(run(30.0, 0.001, 0.0) < 0.5);
    }

    #[test]
    fn a_ten_ms_exposure_integrates_the_flicker_away() {
        let amp = run(30.0, 0.010, 0.3);
        assert!(amp < FLICKER_WARN_PCT, "{amp}");
    }

    #[test]
    fn exposures_move_to_flicker_safe_values_keeping_brightness() {
        // Walking caps at 4 ms; 3 ms is unsafe, so 2 ms at 1.5x gain.
        assert_eq!(
            quantise_exposure(3_000, 1.0, ExposureMode::Walking, 50),
            (2_000, 1.5)
        );
        // Slow allows 10 ms: 12 ms request → 10 ms at 1.2x.
        let (e, g) = quantise_exposure(12_000, 1.0, ExposureMode::Slow, 50);
        assert_eq!(e, 10_000);
        assert!((g - 1.2).abs() < 1e-6);
        // 7 ms in slow mode: 10 ms is nearer than 2 ms.
        assert_eq!(
            quantise_exposure(7_000, 1.0, ExposureMode::Slow, 50).0,
            10_000
        );
        // 9 ms slow → 10 ms, not 2 ms.
        let (e, g) = quantise_exposure(9_000, 1.0, ExposureMode::Slow, 50);
        assert_eq!(e, 10_000);
        assert_eq!(g, 1.0);
        // 5 ms is under one period: the nearest safe value is 2 ms.
        assert_eq!(
            quantise_exposure(5_000, 1.0, ExposureMode::Slow, 50).0,
            2_000
        );
        // Short mode never exceeds 2 ms; gain is capped.
        assert_eq!(
            quantise_exposure(30_000, 2.0, ExposureMode::Short, 50),
            (2_000, MAX_GAIN)
        );
        // Already safe values are kept.
        assert_eq!(
            quantise_exposure(1_500, 2.0, ExposureMode::Walking, 50),
            (1_500, 2.0)
        );
        // 60 Hz mains: period 8333 µs.
        assert_eq!(
            quantise_exposure(9_000, 1.0, ExposureMode::Slow, 60).0,
            8_333
        );
    }

    #[test]
    fn a_mode_change_alone_reapplies_the_cap() {
        let slow = settle_exposure(
            ExposureView::default(),
            ExposureView {
                locked: true,
                exposure_us: Some(10_000),
                mode: Some(ExposureMode::Slow),
                ..Default::default()
            },
            50,
        );
        assert_eq!(slow.exposure_us, Some(10_000));
        assert!(!slow.locked && !slow.applied);
        let walking = settle_exposure(
            slow,
            ExposureView {
                mode: Some(ExposureMode::Walking),
                ..Default::default()
            },
            50,
        );
        assert!(walking.exposure_us.unwrap() <= 4_000, "{walking:?}");
        assert_eq!(walking.gain, Some(5.0));
        assert_eq!(walking.mode, Some(ExposureMode::Walking));
    }
}
