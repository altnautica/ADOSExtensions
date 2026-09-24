//! The drone's second capture-bus subscriber and the `status` telemetry writer.
//!
//! - Shared topics: every bus event on a [`PLUGIN_ATLAS_TOPICS`] topic is
//!   republished on the plugin event bus (`event.publish`), its payload decoded
//!   as a msgpack value, so a plugin that declared the shared topic receives the
//!   pose and the world-model descriptors. Capture-internal topics (keyframes,
//!   capture state, offloaded pose) never cross.
//! - Capture state: the latest `atlas.capture.state` status is kept with its
//!   arrival time.
//! - Status: the link is the single writer of the `status` telemetry channel on
//!   a drone: `{"atlas": <atlas state, or null>, "compute": null}`, sent on
//!   change and at least every [`STATUS_REFRESH`]. The atlas half is null once no
//!   capture-state event arrived for [`CAPTURE_STATE_STALE`].
//!
//! It runs on its own bus connection so a slow LAN forward never delays the
//! pose republish or the status.

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use ados_protocol::frame::PLUGIN_MAX_FRAME;
use ados_protocol::ipc::{connect_with_retry, read_length_prefixed};
use parking_lot::Mutex;
use rmpv::Value;
use serde_json::json;
use tokio::sync::{watch, Notify};
use tokio::time::Instant;
use world_engine_protocol::atlas::{
    AtlasEvent, AtlasForwardStatus, CaptureStatus, ATLAS_CAPTURE_STATE_TOPIC, PLUGIN_ATLAS_TOPICS,
};
use world_engine_protocol::atlas_state::atlas_state_json;
use world_engine_protocol::STATUS_CHANNEL;

use crate::forward::{now_ms, SharedForward, CONNECT_RETRIES, CONNECT_RETRY_DELAY};
use crate::host::Host;
use crate::sleep_or_shutdown;

/// A capture state older than this is no longer reported.
pub const CAPTURE_STATE_STALE: Duration = Duration::from_secs(10);
/// The status is re-sent at least this often.
const STATUS_REFRESH: Duration = Duration::from_secs(5);
/// How often the writer checks for a change it was not woken for (the
/// forwarder's facts, the capture state ageing out).
const STATUS_TICK: Duration = Duration::from_secs(1);
/// Backoff before reconnecting the bus.
const RECONNECT_DELAY: Duration = Duration::from_secs(2);

/// The latest capture status and when it arrived.
#[derive(Default)]
pub struct CaptureState {
    latest: Mutex<Option<(CaptureStatus, Instant)>>,
    changed: Notify,
}

impl CaptureState {
    fn set(&self, status: CaptureStatus) {
        *self.latest.lock() = Some((status, Instant::now()));
        self.changed.notify_one();
    }

    /// The latest status, unless it is older than [`CAPTURE_STATE_STALE`].
    fn fresh(&self) -> Option<CaptureStatus> {
        self.latest
            .lock()
            .as_ref()
            .filter(|(_, at)| at.elapsed() < CAPTURE_STATE_STALE)
            .map(|(s, _)| s.clone())
    }
}

/// What one bus frame asks of the monitor.
#[derive(Debug, PartialEq)]
enum BusEvent {
    /// Republish on the plugin bus.
    Shared(String, Value),
    /// A new capture status.
    CaptureState(CaptureStatus),
}

/// Classify one bus frame, or `None` for a frame the monitor ignores (a
/// capture-internal topic, or one that does not decode).
fn bus_event(body: &[u8]) -> Option<BusEvent> {
    let event = AtlasEvent::decode(body).ok()?;
    if event.topic == ATLAS_CAPTURE_STATE_TOPIC {
        return CaptureStatus::from_msgpack(&event.payload)
            .ok()
            .map(BusEvent::CaptureState);
    }
    if !PLUGIN_ATLAS_TOPICS.contains(&event.topic.as_str()) {
        return None;
    }
    let payload = rmpv::decode::read_value(&mut event.payload.as_slice()).ok()?;
    Some(BusEvent::Shared(event.topic, payload))
}

/// The `status` channel payload.
fn status_json(
    capture: Option<&CaptureStatus>,
    forward: &AtlasForwardStatus,
    now_ms: i64,
) -> serde_json::Value {
    json!({
        "atlas": capture.map(|s| atlas_state_json(s, Some(forward), now_ms)),
        "compute": null,
    })
}

/// Subscribe to the capture bus until `shutdown`, republishing shared topics and
/// recording the capture state.
pub async fn run_bus_monitor(
    host: Arc<dyn Host>,
    bus_socket: PathBuf,
    capture: Arc<CaptureState>,
    mut shutdown: watch::Receiver<bool>,
) {
    loop {
        if *shutdown.borrow() {
            return;
        }
        let mut stream =
            match connect_with_retry(&bus_socket, CONNECT_RETRIES, CONNECT_RETRY_DELAY).await {
                Ok(s) => s,
                Err(e) => {
                    tracing::debug!(error = %e, "atlas bus not reachable; retrying");
                    if sleep_or_shutdown(&mut shutdown, RECONNECT_DELAY).await {
                        return;
                    }
                    continue;
                }
            };
        loop {
            let read = tokio::select! {
                r = shutdown.changed() => {
                    if r.is_err() || *shutdown.borrow() { return; }
                    continue;
                }
                r = read_length_prefixed(&mut stream, PLUGIN_MAX_FRAME, false) => r,
            };
            match read {
                Ok(Some(body)) => match bus_event(&body) {
                    Some(BusEvent::Shared(topic, payload)) => {
                        if let Err(e) = host.event_publish(&topic, payload).await {
                            tracing::debug!(topic, error = %e, "shared topic republish failed");
                        }
                    }
                    Some(BusEvent::CaptureState(status)) => capture.set(status),
                    None => {}
                },
                Ok(None) => break,
                Err(e) => {
                    tracing::debug!(error = %e, "atlas bus read failed; reconnecting");
                    break;
                }
            }
        }
        if sleep_or_shutdown(&mut shutdown, RECONNECT_DELAY).await {
            return;
        }
    }
}

/// Write the `status` telemetry channel until `shutdown`: on a capture-state
/// change, on any other change seen at the next tick, and at least every
/// [`STATUS_REFRESH`].
pub async fn run_status_writer(
    host: Arc<dyn Host>,
    capture: Arc<CaptureState>,
    forward: SharedForward,
    mut shutdown: watch::Receiver<bool>,
) {
    let mut tick = tokio::time::interval(STATUS_TICK);
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    let mut last: Option<(Option<CaptureStatus>, AtlasForwardStatus)> = None;
    let mut last_sent: Option<Instant> = None;
    loop {
        tokio::select! {
            r = shutdown.changed() => {
                if r.is_err() || *shutdown.borrow() { return; }
                continue;
            }
            _ = capture.changed.notified() => {}
            _ = tick.tick() => {}
        }
        // Compared without the write stamp, so only a real change forces a send.
        let inputs = (capture.fresh(), forward.lock().snapshot(0));
        let due = last_sent.is_none_or(|t| t.elapsed() >= STATUS_REFRESH);
        if !due && last.as_ref() == Some(&inputs) {
            continue;
        }
        let now = now_ms();
        let mut fwd = inputs.1.clone();
        fwd.generated_at_ms = now;
        let body = status_json(inputs.0.as_ref(), &fwd, now);
        match rmpv::ext::to_value(&body) {
            Ok(payload) => {
                if let Err(e) = host.telemetry_extend(STATUS_CHANNEL, payload).await {
                    tracing::debug!(error = %e, "status telemetry write failed");
                }
            }
            Err(e) => tracing::warn!(error = %e, "status telemetry encode failed"),
        }
        last = Some(inputs);
        last_sent = Some(Instant::now());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::tests::capturing_status;
    use crate::forward::ForwardStatus;
    use crate::testing::FakeHost;
    use tokio::io::AsyncWriteExt;
    use tokio::net::UnixListener;
    use world_engine_protocol::atlas::{ATLAS_KEYFRAME_TOPIC, PLUGIN_ATLAS_POSE_TOPIC};

    fn frame(topic: &str, payload: &Value) -> Vec<u8> {
        let mut inner = Vec::new();
        rmpv::encode::write_value(&mut inner, payload).unwrap();
        AtlasEvent::new(topic, None, inner).encode().unwrap()
    }

    #[test]
    fn only_shared_topics_cross_onto_the_plugin_bus() {
        let payload = Value::Map(vec![(Value::from("ts_ms"), Value::from(7))]);
        for topic in PLUGIN_ATLAS_TOPICS {
            assert_eq!(
                bus_event(&frame(topic, &payload)),
                Some(BusEvent::Shared(topic.to_string(), payload.clone()))
            );
        }
        assert_eq!(bus_event(&frame(ATLAS_KEYFRAME_TOPIC, &payload)), None);
        assert_eq!(bus_event(&frame("atlas.pose.offload", &payload)), None);
        assert_eq!(bus_event(b"not msgpack"), None);
    }

    #[test]
    fn a_capture_state_event_is_recorded_not_republished() {
        let status = capturing_status();
        let body = AtlasEvent::new(
            ATLAS_CAPTURE_STATE_TOPIC,
            None,
            status.to_msgpack().unwrap(),
        )
        .encode()
        .unwrap();
        assert_eq!(bus_event(&body), Some(BusEvent::CaptureState(status)));
    }

    #[tokio::test(start_paused = true)]
    async fn the_atlas_status_goes_null_after_ten_seconds_without_capture_state() {
        let state = CaptureState::default();
        let fwd = ForwardStatus::default().snapshot(1);
        state.set(capturing_status());

        let live = status_json(state.fresh().as_ref(), &fwd, 1);
        assert_eq!(live["atlas"]["sessionId"], "sess-1");
        assert!(live["compute"].is_null());

        tokio::time::advance(CAPTURE_STATE_STALE).await;
        let stale = status_json(state.fresh().as_ref(), &fwd, 2);
        assert!(stale["atlas"].is_null());
        assert!(stale["compute"].is_null());
    }

    /// End to end: a pose on the bus reaches the plugin bus, and a capture state
    /// on the bus reaches the `status` channel.
    #[tokio::test]
    async fn the_monitor_republishes_the_pose_and_the_writer_sends_the_status() {
        let dir = tempfile::tempdir().unwrap();
        let sock = dir.path().join("atlas.sock");
        let listener = UnixListener::bind(&sock).unwrap();
        let host = FakeHost::new();
        let capture = Arc::new(CaptureState::default());
        let (_stop, shutdown) = watch::channel(false);
        tokio::spawn(run_bus_monitor(
            host.clone(),
            sock,
            capture.clone(),
            shutdown.clone(),
        ));
        tokio::spawn(run_status_writer(
            host.clone(),
            capture,
            Arc::new(Mutex::new(ForwardStatus::default())),
            shutdown,
        ));

        let (mut conn, _) = listener.accept().await.unwrap();
        let state = AtlasEvent::new(
            ATLAS_CAPTURE_STATE_TOPIC,
            None,
            capturing_status().to_msgpack().unwrap(),
        )
        .encode()
        .unwrap();
        for body in [frame(PLUGIN_ATLAS_POSE_TOPIC, &Value::from("pose")), state] {
            let mut wire = (body.len() as u32).to_be_bytes().to_vec();
            wire.extend_from_slice(&body);
            conn.write_all(&wire).await.unwrap();
        }

        host.wait_for(|h| !h.events.lock().is_empty()).await;
        assert_eq!(
            host.events.lock()[0],
            (PLUGIN_ATLAS_POSE_TOPIC.to_string(), Value::from("pose"))
        );
        host.wait_for(|h| {
            h.telemetry
                .lock()
                .iter()
                .any(|(_, p)| crate::host::to_json(p.clone())["atlas"]["state"] == "capturing")
        })
        .await;
        assert!(host
            .telemetry
            .lock()
            .iter()
            .all(|(c, _)| c == STATUS_CHANNEL));
    }
}
