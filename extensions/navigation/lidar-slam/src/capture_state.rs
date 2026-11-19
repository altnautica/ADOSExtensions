//! Whether a capture session is recording now, from the capture service's
//! `plugin.capture.state`: the lane streams `plugin.navigation.scan_raw`
//! only then. A state older than [`FRESH`] counts as not recording, so with
//! Capture absent or stopped nothing is streamed.

use std::sync::Arc;
use std::time::{Duration, Instant};

use ados_mapping_protocol::atlas::CaptureState;
use parking_lot::Mutex;
use serde::Deserialize;

/// How long a capture state holds.
pub const FRESH: Duration = Duration::from_secs(2);

/// The fields read from a capture state (the rest are ignored).
#[derive(Deserialize)]
struct StateOnly {
    state: CaptureState,
}

/// The newest capture state and when it arrived.
#[derive(Clone, Default)]
pub struct CaptureWatch(Arc<Mutex<Option<(Instant, CaptureState)>>>);

impl CaptureWatch {
    /// Fold one `plugin.capture.state` payload; one that does not decode is
    /// ignored.
    pub fn on_payload(&self, payload: &rmpv::Value) {
        // Through JSON: rmpv's own deserializer takes no enum from a string.
        let state = rmpv::ext::from_value::<serde_json::Value>(payload.clone())
            .ok()
            .and_then(|v| serde_json::from_value::<StateOnly>(v).ok());
        if let Some(s) = state {
            self.set(Instant::now(), s.state);
        }
    }

    pub fn set(&self, at: Instant, state: CaptureState) {
        *self.0.lock() = Some((at, state));
    }

    /// A session is recording, by a state no older than [`FRESH`] at `now`.
    pub fn recording(&self, now: Instant) -> bool {
        self.0
            .lock()
            .is_some_and(|(at, s)| s == CaptureState::Capturing && now.duration_since(at) < FRESH)
    }

    /// The callback for `event.subscribe`: deliveries carry the payload
    /// under `payload`.
    pub fn callback(&self) -> ados_sdk::EventCallback {
        let watch = self.clone();
        Arc::new(move |args: rmpv::Value| {
            let payload = args.as_map().and_then(|m| {
                m.iter()
                    .find(|(k, _)| k.as_str() == Some("payload"))
                    .map(|(_, v)| v.clone())
            });
            if let Some(p) = payload {
                watch.on_payload(&p);
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn payload(state: &str) -> rmpv::Value {
        rmpv::Value::Map(vec![
            ("session_id".into(), "s1".into()),
            ("state".into(), state.into()),
            ("keyframes".into(), 4.into()),
        ])
    }

    #[test]
    fn only_a_fresh_capturing_state_records() {
        let w = CaptureWatch::default();
        let now = Instant::now();
        assert!(!w.recording(now), "no capture state: Capture absent");
        w.on_payload(&payload("capturing"));
        assert!(w.recording(Instant::now()));
        assert!(!w.recording(Instant::now() + FRESH), "stale");
        w.on_payload(&payload("paused"));
        assert!(!w.recording(Instant::now()));
        w.on_payload(&rmpv::Value::from("garbage"));
        assert!(
            !w.recording(Instant::now()),
            "an undecodable payload changes nothing"
        );
    }

    #[test]
    fn the_callback_reads_the_delivered_payload() {
        let w = CaptureWatch::default();
        (w.callback())(rmpv::Value::Map(vec![
            ("topic".into(), "plugin.capture.state".into()),
            ("payload".into(), payload("capturing")),
        ]));
        assert!(w.recording(Instant::now()));
    }
}
