//! Whether the device is idle or charging, for the auto-upload gate.
//!
//! Read from the vehicle-state snapshot the host delivers on
//! `telemetry.subscribe` (cap `telemetry.read`, at most 10 Hz):
//!
//! - **idle**: the vehicle is not armed (`armed` is not `true`), or no
//!   snapshot arrived in the last [`STALE_AFTER`] (a node with no flight
//!   controller, such as a Visor, never gets one: nothing on it flies).
//! - **charging**: a fresh pack in `batteries[]` (`age_ms` under
//!   [`PACK_FRESH_MS`]) reports `current_a` below [`CHARGE_CURRENT_A`]
//!   (current flowing into the pack).

use std::sync::Arc;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use serde_json::Value;

use crate::host::{to_json, Host};

/// A snapshot older than this no longer describes the vehicle.
pub const STALE_AFTER: Duration = Duration::from_secs(10);
/// A pack reading older than this is not a reading.
pub const PACK_FRESH_MS: u64 = 5_000;
/// Pack current below this (amps, negative into the pack) is charging.
pub const CHARGE_CURRENT_A: f64 = -0.05;

/// Whether `snapshot` describes an idle or charging device.
pub fn idle_or_charging(snapshot: Option<&Value>) -> bool {
    let Some(s) = snapshot else { return true };
    if s.get("armed").and_then(Value::as_bool) != Some(true) {
        return true;
    }
    s.get("batteries")
        .and_then(Value::as_array)
        .is_some_and(|packs| {
            packs.iter().any(|p| {
                let fresh = p
                    .get("age_ms")
                    .and_then(Value::as_u64)
                    .is_some_and(|a| a < PACK_FRESH_MS);
                let current = p.get("current_a").and_then(Value::as_f64);
                fresh && current.is_some_and(|c| c < CHARGE_CURRENT_A)
            })
        })
}

/// The latest vehicle-state snapshot.
#[derive(Clone, Default)]
pub struct PowerWatch {
    latest: Arc<Mutex<Option<(Instant, Value)>>>,
}

impl PowerWatch {
    /// Record one snapshot.
    pub fn note(&self, snapshot: Value) {
        *self.latest.lock() = Some((Instant::now(), snapshot));
    }

    /// Subscribe to the vehicle state. A host that refuses leaves the watch
    /// empty, which reads as idle (logged: the charging signal is then lost).
    pub async fn subscribe(&self, host: &dyn Host) {
        let watch = self.clone();
        let callback: Arc<dyn Fn(rmpv::Value) + Send + Sync> =
            Arc::new(move |v| watch.note(to_json(v)));
        if let Err(e) = host.telemetry_subscribe(callback).await {
            tracing::warn!(error = %e, "vehicle-state subscribe failed; auto-upload treats the device as idle");
        }
    }

    pub fn idle_or_charging(&self) -> bool {
        let latest = self.latest.lock();
        let fresh = latest
            .as_ref()
            .filter(|(at, _)| at.elapsed() < STALE_AFTER)
            .map(|(_, v)| v);
        idle_or_charging(fresh)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn disarmed_or_absent_is_idle_and_armed_needs_charging() {
        assert!(idle_or_charging(None));
        assert!(idle_or_charging(Some(&json!({"armed": false}))));
        let armed = json!({"armed": true, "batteries": [{"current_a": 12.0, "age_ms": 100}]});
        assert!(!idle_or_charging(Some(&armed)));
        let charging = json!({"armed": true, "batteries": [{"current_a": -1.2, "age_ms": 100}]});
        assert!(idle_or_charging(Some(&charging)));
        let stale = json!({"armed": true, "batteries": [{"current_a": -1.2, "age_ms": 60000}]});
        assert!(!idle_or_charging(Some(&stale)));
        let w = PowerWatch::default();
        assert!(w.idle_or_charging());
        w.note(armed);
        assert!(!w.idle_or_charging());
    }
}
