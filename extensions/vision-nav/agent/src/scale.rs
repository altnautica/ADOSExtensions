//! Rangefinder-free altitude ladder.
//!
//! `optical_flow_degraded` runs the same flow tracker but pulls scale
//! from a four-rung ladder instead of a dedicated rangefinder. The
//! pipeline feeds the ladder decoded MAVLink fields
//! (`GLOBAL_POSITION_INT.relative_alt`, `VFR_HUD.alt`,
//! `GPS_RAW_INT.{alt,fix_type,eph}`); each [`pick`](ScaleLadder::pick)
//! walks the rungs and returns the first healthy one, falling back to a
//! static value so the estimator can always emit at the lowest quality
//! rather than refuse to feed the EKF.
//!
//! Per-rung quality multipliers (the EKF auto-de-weights degraded
//! rungs): relative_alt 0.7, raw baro 0.6, GPS 0.4, static 0.2.
//! Distance is clamped to `[0.3, 50.0]` m so a glitch reading does not
//! produce a runaway scale.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;

pub const STALE_THRESHOLD_NS: i64 = 2_000_000_000;
pub const MIN_DISTANCE_M: f32 = 0.3;
pub const MAX_DISTANCE_M: f32 = 50.0;
pub const DEFAULT_STATIC_FALLBACK_M: f32 = 1.5;

pub const QM_RELATIVE_ALT: f32 = 0.7;
pub const QM_RAW_BARO: f32 = 0.6;
pub const QM_GPS: f32 = 0.4;
pub const QM_STATIC: f32 = 0.2;

pub const GPS_FIX_TYPE_3D: i32 = 3;
pub const GPS_EPH_MAX_CM: i32 = 200;

/// Which rung produced a scale value. The GCS sensors card surfaces it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScaleRung {
    Baro,
    Gps,
    Static,
}

impl ScaleRung {
    pub fn as_str(self) -> &'static str {
        match self {
            ScaleRung::Baro => "baro",
            ScaleRung::Gps => "gps",
            ScaleRung::Static => "static",
        }
    }
}

/// One ladder evaluation: the chosen distance, its rung, and the
/// quality multiplier applied to the flow tracker's quality.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScalePick {
    pub distance_m: f32,
    pub source: ScaleRung,
    pub quality_multiplier: f32,
}

/// The MAVLink-derived ladder. `now_ns` is injected into `pick` so the
/// staleness gates are deterministic in tests.
pub struct ScaleLadder {
    outdoor: AtomicBool,
    static_fallback_m: f32,
    inner: Mutex<Inner>,
}

#[derive(Default)]
struct Inner {
    relative_alt: Option<(f32, i64)>,
    vfr_alt: Option<(f32, i64)>,
    gps_alt: Option<(f32, i64)>,
    takeoff_alt: Option<f32>,
    gps_meta: Option<(i32, i32)>, // (fix_type, eph_cm)
}

impl ScaleLadder {
    pub fn new(outdoor: bool) -> Self {
        Self {
            outdoor: AtomicBool::new(outdoor),
            static_fallback_m: DEFAULT_STATIC_FALLBACK_M,
            inner: Mutex::new(Inner::default()),
        }
    }

    /// GCS toggle: enables / disables the GPS rung at runtime.
    pub fn set_outdoor(&self, value: bool) {
        self.outdoor.store(value, Ordering::Relaxed);
    }

    /// `GLOBAL_POSITION_INT.relative_alt` in millimetres.
    pub fn on_global_position(&self, relative_alt_mm: i64, now_ns: i64) {
        let metres = relative_alt_mm as f32 / 1000.0;
        let mut g = self.inner.lock().expect("ladder lock");
        g.relative_alt = Some((metres, now_ns));
    }

    /// `VFR_HUD.alt` in metres (AMSL). First sample captures the
    /// take-off reference so later samples report AGL.
    pub fn on_vfr_hud(&self, alt_m: f32, now_ns: i64) {
        let mut g = self.inner.lock().expect("ladder lock");
        let metres = match g.takeoff_alt {
            None => {
                g.takeoff_alt = Some(alt_m);
                0.0
            }
            Some(t) => alt_m - t,
        };
        g.vfr_alt = Some((metres, now_ns));
    }

    /// `GPS_RAW_INT.{alt(mm), fix_type, eph(cm)}`.
    pub fn on_gps_raw(&self, alt_mm: i64, fix_type: i32, eph_cm: i32, now_ns: i64) {
        let alt_m = alt_mm as f32 / 1000.0;
        let mut g = self.inner.lock().expect("ladder lock");
        let metres = match g.takeoff_alt {
            Some(t) => alt_m - t,
            None => alt_m,
        };
        g.gps_alt = Some((metres, now_ns));
        g.gps_meta = Some((fix_type, eph_cm));
    }

    /// Walk the ladder at `now_ns`; return the first healthy rung or
    /// the static fallback.
    pub fn pick(&self, now_ns: i64) -> ScalePick {
        let g = self.inner.lock().expect("ladder lock");

        // Rung 1: relative_alt (baro-EKF AGL).
        if let Some((v, t)) = g.relative_alt {
            if now_ns - t <= STALE_THRESHOLD_NS {
                return ScalePick {
                    distance_m: clamp(v),
                    source: ScaleRung::Baro,
                    quality_multiplier: QM_RELATIVE_ALT,
                };
            }
        }
        // Rung 2: raw baro from VFR_HUD minus take-off.
        if let Some((v, t)) = g.vfr_alt {
            if now_ns - t <= STALE_THRESHOLD_NS {
                return ScalePick {
                    distance_m: clamp(v),
                    source: ScaleRung::Baro,
                    quality_multiplier: QM_RAW_BARO,
                };
            }
        }
        // Rung 3: GPS alt, gated on outdoor flag + 3D fix + HDOP.
        if self.outdoor.load(Ordering::Relaxed) {
            if let (Some((v, t)), Some((fix, eph))) = (g.gps_alt, g.gps_meta) {
                if now_ns - t <= STALE_THRESHOLD_NS
                    && fix >= GPS_FIX_TYPE_3D
                    && eph <= GPS_EPH_MAX_CM
                {
                    return ScalePick {
                        distance_m: clamp(v),
                        source: ScaleRung::Gps,
                        quality_multiplier: QM_GPS,
                    };
                }
            }
        }
        // Rung 4: static fallback. The GCS surfaces the red banner.
        ScalePick {
            distance_m: clamp(self.static_fallback_m),
            source: ScaleRung::Static,
            quality_multiplier: QM_STATIC,
        }
    }
}

fn clamp(v: f32) -> f32 {
    v.clamp(MIN_DISTANCE_M, MAX_DISTANCE_M)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn relative_alt_is_primary_rung() {
        let l = ScaleLadder::new(false);
        l.on_global_position(1500, 0);
        let p = l.pick(0);
        assert_eq!(p.source, ScaleRung::Baro);
        assert!((p.distance_m - 1.5).abs() < 1e-4);
        assert_eq!(p.quality_multiplier, QM_RELATIVE_ALT);
    }

    #[test]
    fn vfr_hud_captures_takeoff_then_reports_agl() {
        let l = ScaleLadder::new(false);
        l.on_vfr_hud(100.0, 0); // take-off reference, reports 0 (clamped up)
        let p = l.pick(0);
        assert_eq!(p.source, ScaleRung::Baro);
        assert!((p.distance_m - MIN_DISTANCE_M).abs() < 1e-4);
        assert_eq!(p.quality_multiplier, QM_RAW_BARO);
        l.on_vfr_hud(101.2, 0);
        let p = l.pick(0);
        assert!((p.distance_m - 1.2).abs() < 1e-3);
    }

    #[test]
    fn gps_rung_requires_outdoor_and_3d_fix() {
        let indoor = ScaleLadder::new(false);
        indoor.on_gps_raw(5000, 3, 150, 0);
        assert_eq!(indoor.pick(0).source, ScaleRung::Static);

        let outdoor = ScaleLadder::new(true);
        outdoor.on_gps_raw(5000, 3, 150, 0);
        let p = outdoor.pick(0);
        assert_eq!(p.source, ScaleRung::Gps);
        assert_eq!(p.quality_multiplier, QM_GPS);
    }

    #[test]
    fn gps_rejected_when_fix_poor() {
        let l = ScaleLadder::new(true);
        l.on_gps_raw(5000, 2, 100, 0); // 2D fix
        assert_eq!(l.pick(0).source, ScaleRung::Static);
        l.on_gps_raw(5000, 3, 500, 0); // HDOP too high
        assert_eq!(l.pick(0).source, ScaleRung::Static);
    }

    #[test]
    fn static_fallback_with_no_messages() {
        let l = ScaleLadder::new(false);
        let p = l.pick(0);
        assert_eq!(p.source, ScaleRung::Static);
        assert!((p.distance_m - 1.5).abs() < 1e-4);
        assert_eq!(p.quality_multiplier, QM_STATIC);
    }

    #[test]
    fn distance_clamped_to_valid_range() {
        let l = ScaleLadder::new(false);
        l.on_global_position(100_000_000, 0); // 100 km
        assert!((l.pick(0).distance_m - MAX_DISTANCE_M).abs() < 1e-4);
        l.on_global_position(-5000, 0); // below take-off
        assert!((l.pick(0).distance_m - MIN_DISTANCE_M).abs() < 1e-4);
    }

    #[test]
    fn stale_relative_alt_drops_to_static() {
        let l = ScaleLadder::new(false);
        l.on_global_position(1500, 0);
        assert_eq!(l.pick(0).source, ScaleRung::Baro);
        // Past the staleness threshold.
        let later = STALE_THRESHOLD_NS + 1;
        assert_eq!(l.pick(later).source, ScaleRung::Static);
    }
}
