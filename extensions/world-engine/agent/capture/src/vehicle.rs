//! The vehicle-state feed (`telemetry.read`): GNSS fixes for the session
//! anchor (the mean of the first 100 3D fixes) and the `/gnss/<id>/fix`
//! channel. Subscribed once per process; capture runs read it.

use std::sync::Arc;

use ados_capture_contract::GeoAnchor;
use ados_sdk::{ClientError, EventCallback, PluginContext};
use parking_lot::Mutex;

use crate::geo::{geoid_undulation_m, AnchorLatch};

/// One 3D fix from the vehicle state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Fix {
    /// Host monotonic ns the fix was decoded at (arrival minus its age).
    pub t_ns: u64,
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub h_ellipsoid_m: f64,
    /// DOP (unitless) when reported.
    pub dop: Option<f64>,
}

#[derive(Default)]
struct Inner {
    latch: AnchorLatch,
    latest: Option<Fix>,
    seq: u64,
    /// Host ns since which every snapshot has carried a 3D fix.
    fix3d_since: Option<u64>,
}

/// A fix older than this does not count as GNSS now, ns.
pub const FIX_STALE_NS: u64 = 2_000_000_000;

#[derive(Clone, Default)]
pub struct VehicleFeed {
    inner: Arc<Mutex<Inner>>,
}

impl VehicleFeed {
    pub fn new() -> Self {
        Self::default()
    }

    pub async fn subscribe(&self, ctx: &PluginContext) -> Result<(), ClientError> {
        let feed = self.clone();
        let cb: EventCallback = Arc::new(move |state| {
            if let Ok(v) = rmpv::ext::from_value::<serde_json::Value>(state) {
                feed.on_state(&v, crate::clock::mono_ns());
            }
        });
        ctx.telemetry.subscribe(cb).await
    }

    /// Fold one vehicle-state snapshot received at host `now_ns`. Only a 3D
    /// fix at a real position counts.
    pub fn on_state(&self, v: &serde_json::Value, now_ns: u64) {
        let fix_type = v
            .pointer("/gps/fix_type")
            .and_then(serde_json::Value::as_i64)
            .unwrap_or(0);
        let age_ms = v.get("position_age_ms").and_then(serde_json::Value::as_u64);
        let lat = v
            .pointer("/position/lat")
            .and_then(serde_json::Value::as_f64);
        let lon = v
            .pointer("/position/lon")
            .and_then(serde_json::Value::as_f64);
        let alt = v
            .pointer("/position/alt_msl")
            .and_then(serde_json::Value::as_f64);
        let pos = match (age_ms, lat, lon, alt) {
            (Some(age), Some(la), Some(lo), Some(al)) if !(la == 0.0 && lo == 0.0) => {
                Some((age, la, lo, al))
            }
            _ => None,
        };
        let mut g = self.inner.lock();
        let Some((age_ms, lat, lon, alt)) = pos.filter(|_| fix_type >= 3) else {
            g.fix3d_since = None;
            return;
        };
        g.fix3d_since.get_or_insert(now_ns);
        let dop = v
            .pointer("/gps/eph")
            .and_then(serde_json::Value::as_f64)
            .filter(|d| d.is_finite() && *d > 0.0);
        g.latch.observe(lat, lon, alt);
        g.seq += 1;
        g.latest = Some(Fix {
            t_ns: now_ns.saturating_sub(age_ms * 1_000_000),
            lat_deg: lat,
            lon_deg: lon,
            h_ellipsoid_m: alt + geoid_undulation_m(lat, lon),
            dop,
        });
    }

    /// The averaged GNSS anchor, once latched.
    pub fn anchor(&self) -> Option<GeoAnchor> {
        self.inner.lock().latch.anchor()
    }

    /// The newest fix with a sequence number that changes per fix.
    pub fn latest(&self) -> Option<(u64, Fix)> {
        let g = self.inner.lock();
        g.latest.map(|f| (g.seq, f))
    }

    /// Whether GNSS has held a 3D fix for at least `settle_ns` up to `now_ns`
    /// (host monotonic), with the newest fix no older than [`FIX_STALE_NS`].
    pub fn gnss_steady(&self, now_ns: u64, settle_ns: u64) -> bool {
        let g = self.inner.lock();
        let held = g
            .fix3d_since
            .is_some_and(|t| now_ns.saturating_sub(t) >= settle_ns);
        let fresh = g
            .latest
            .is_some_and(|f| now_ns.saturating_sub(f.t_ns) <= FIX_STALE_NS);
        held && fresh
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn state(fix: i64, lat: f64) -> serde_json::Value {
        json!({"position_age_ms": 20, "position": {"lat": lat, "lon": 77.5, "alt_msl": 900.0},
               "gps": {"fix_type": fix, "eph": 0.9}})
    }

    #[test]
    fn three_d_fixes_latch_the_anchor_after_a_hundred() {
        let f = VehicleFeed::new();
        f.on_state(&state(2, 12.0), 1_000_000_000);
        assert!(f.latest().is_none(), "a 2D fix is not used");
        for _ in 0..99 {
            f.on_state(&state(3, 12.0), 1_000_000_000);
        }
        assert!(f.anchor().is_none());
        f.on_state(&state(3, 12.0), 1_000_000_000);
        let a = f.anchor().unwrap();
        assert!((a.lat_deg - 12.0).abs() < 1e-12);
        let (seq, fix) = f.latest().unwrap();
        assert_eq!(seq, 100);
        assert_eq!(fix.t_ns, 980_000_000);
        assert_eq!(fix.dop, Some(0.9));
        assert!((fix.h_ellipsoid_m - (900.0 + a.geoid_undulation_m)).abs() < 1e-9);
    }

    #[test]
    fn gnss_is_steady_after_the_settle_time_and_not_after_a_dropout() {
        let f = VehicleFeed::new();
        let s = 1_000_000_000u64;
        assert!(!f.gnss_steady(0, 0), "no fix yet");
        f.on_state(&state(3, 12.0), s);
        f.on_state(&state(3, 12.0), 5 * s);
        assert!(!f.gnss_steady(5 * s, 10 * s), "held 4 s of 10");
        f.on_state(&state(3, 12.0), 11 * s);
        assert!(f.gnss_steady(11 * s, 10 * s));
        assert!(!f.gnss_steady(14 * s, 10 * s), "the newest fix is 3 s old");
        f.on_state(&state(1, 12.0), 12 * s);
        assert!(!f.gnss_steady(12 * s, 10 * s), "the fix dropped");
        f.on_state(&state(3, 12.0), 13 * s);
        assert!(!f.gnss_steady(13 * s, 10 * s), "the settle time restarts");
        assert!(f.gnss_steady(13 * s, 0));
    }
}
