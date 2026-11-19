//! The drone's second private-bus subscriber, the shared capture state and
//! the `status` telemetry writer.
//!
//! - Capture state: the latest `capture.state` from the capture service's
//!   private bus is kept with its arrival time; keyframes are the
//!   forwarder's.
//! - `plugin.capture.state`: the capture status plus the session's upload
//!   progress, published on the plugin event bus on change and at least every
//!   [`REFRESH`] for every plugin holding `telemetry.read` (Navigation streams
//!   the scans a session records only while it reads `capturing` here). The
//!   link publishes it because it holds both halves: the capture service's
//!   state and the uploader's progress.
//! - Status: the link is the single writer of the `status` telemetry channel
//!   on a drone: `{"capture": <capture state, or null>}`, on the same cadence.
//!
//! Both go quiet (null status, nothing published) once no capture-state event
//! arrived for [`CAPTURE_STATE_STALE`]. The bus reader runs on its own
//! connection so a slow LAN forward never delays the state.

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use ados_mapping_protocol::capture::{
    CaptureStatus, KeyframeForwardStatus, NodeEvent, CAPTURE_STATE_TOPIC,
};
use ados_mapping_protocol::capture_state::capture_state_json;
use ados_mapping_protocol::STATUS_CHANNEL;
use ados_protocol::frame::PLUGIN_MAX_FRAME;
use ados_protocol::ipc::{connect_with_retry, read_length_prefixed};
use parking_lot::Mutex;
use serde::Serialize;
use serde_json::json;
use tokio::sync::{watch, Notify};
use tokio::time::Instant;

use crate::forward::{now_ms, SharedForward, CONNECT_RETRIES, CONNECT_RETRY_DELAY};
use crate::sleep_or_shutdown;
use crate::upload::{UploadManager, UploadView};
use ados_mapping_transport::host::Host;

/// The shared topic the capture state is published on.
pub const CAPTURE_STATE_SHARED_TOPIC: &str = "plugin.capture.state";
/// A capture state older than this is no longer reported.
pub const CAPTURE_STATE_STALE: Duration = Duration::from_secs(10);
/// The state is re-sent at least this often (Navigation counts a state older
/// than 2 s as not recording).
const REFRESH: Duration = Duration::from_secs(1);
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

/// The capture status in one bus frame, or `None` for any other topic or a
/// frame that does not decode.
fn capture_status(body: &[u8]) -> Option<CaptureStatus> {
    let event = NodeEvent::decode(body).ok()?;
    (event.topic == CAPTURE_STATE_TOPIC)
        .then(|| CaptureStatus::from_msgpack(&event.payload).ok())
        .flatten()
}

/// The `plugin.capture.state` payload.
#[derive(Debug, Serialize)]
struct SharedState<'a> {
    #[serde(flatten)]
    status: &'a CaptureStatus,
    #[serde(skip_serializing_if = "Option::is_none")]
    upload: Option<UploadView>,
}

/// The `status` channel payload.
fn status_json(
    capture: Option<&CaptureStatus>,
    forward: &KeyframeForwardStatus,
    now_ms: i64,
) -> serde_json::Value {
    json!({ "capture": capture.map(|s| capture_state_json(s, Some(forward), now_ms)) })
}

/// Read the capture bus until `shutdown`, recording the capture state.
pub async fn run_bus_monitor(
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
                    tracing::debug!(error = %e, "capture bus not reachable; retrying");
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
                Ok(Some(body)) => {
                    if let Some(status) = capture_status(&body) {
                        capture.set(status);
                    }
                }
                Ok(None) => break,
                Err(e) => {
                    tracing::debug!(error = %e, "capture bus read failed; reconnecting");
                    break;
                }
            }
        }
        if sleep_or_shutdown(&mut shutdown, RECONNECT_DELAY).await {
            return;
        }
    }
}

/// Publish the shared capture state and write the `status` telemetry until
/// `shutdown`: on a capture-state change, on any other change seen at the
/// next tick, and at least every [`REFRESH`].
pub async fn run_state_writer(
    host: Arc<dyn Host>,
    capture: Arc<CaptureState>,
    forward: SharedForward,
    uploads: Option<UploadManager>,
    mut shutdown: watch::Receiver<bool>,
) {
    let mut tick = tokio::time::interval(REFRESH);
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
    loop {
        tokio::select! {
            r = shutdown.changed() => {
                if r.is_err() || *shutdown.borrow() { return; }
                continue;
            }
            _ = capture.changed.notified() => {}
            _ = tick.tick() => {}
        }
        let status = capture.fresh();
        if let Some(s) = &status {
            let shared = SharedState {
                status: s,
                upload: uploads.as_ref().and_then(|u| u.job_view(&s.session_id)),
            };
            match serde_json::to_value(&shared).map(|v| rmpv::ext::to_value(&v)) {
                Ok(Ok(payload)) => {
                    if let Err(e) = host
                        .event_publish(CAPTURE_STATE_SHARED_TOPIC, payload)
                        .await
                    {
                        tracing::debug!(error = %e, "capture state publish failed");
                    }
                }
                _ => tracing::warn!("capture state encode failed"),
            }
        }
        let now = now_ms();
        let mut fwd = forward.lock().snapshot(0);
        fwd.generated_at_ms = now;
        match rmpv::ext::to_value(status_json(status.as_ref(), &fwd, now)) {
            Ok(payload) => {
                if let Err(e) = host.telemetry_extend(STATUS_CHANNEL, payload).await {
                    tracing::debug!(error = %e, "status telemetry write failed");
                }
            }
            Err(e) => tracing::warn!(error = %e, "status telemetry encode failed"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::tests::capturing_status;
    use crate::forward::ForwardStatus;
    use ados_mapping_protocol::capture::CAPTURE_KEYFRAME_TOPIC;
    use ados_mapping_transport::host::to_json;
    use ados_mapping_transport::testing::FakeHost;
    use tokio::io::AsyncWriteExt;
    use tokio::net::UnixListener;

    fn state_frame() -> Vec<u8> {
        NodeEvent::new(
            CAPTURE_STATE_TOPIC,
            None,
            capturing_status().to_msgpack().unwrap(),
        )
        .encode()
        .unwrap()
    }

    #[test]
    fn only_the_capture_state_is_read_off_the_bus() {
        assert_eq!(capture_status(&state_frame()), Some(capturing_status()));
        let kf = NodeEvent::new(CAPTURE_KEYFRAME_TOPIC, None, vec![1, 2])
            .encode()
            .unwrap();
        assert_eq!(capture_status(&kf), None);
        assert_eq!(capture_status(b"not msgpack"), None);
    }

    #[tokio::test(start_paused = true)]
    async fn the_capture_status_goes_null_after_ten_seconds_without_capture_state() {
        let state = CaptureState::default();
        let fwd = ForwardStatus::default().snapshot(1);
        state.set(capturing_status());
        let live = status_json(state.fresh().as_ref(), &fwd, 1);
        assert_eq!(live["capture"]["sessionId"], "sess-1");
        tokio::time::advance(CAPTURE_STATE_STALE).await;
        assert!(status_json(state.fresh().as_ref(), &fwd, 2)["capture"].is_null());
    }

    /// End to end: a capture state on the bus is published as
    /// `plugin.capture.state` (its `state` the field Navigation reads) and
    /// reaches the `status` channel.
    #[tokio::test]
    async fn a_bus_capture_state_is_shared_and_reaches_the_status() {
        let dir = tempfile::tempdir().unwrap();
        let sock = dir.path().join("capture.sock");
        let listener = UnixListener::bind(&sock).unwrap();
        let host = FakeHost::new();
        let capture = Arc::new(CaptureState::default());
        let (_stop, shutdown) = watch::channel(false);
        tokio::spawn(run_bus_monitor(sock, capture.clone(), shutdown.clone()));
        tokio::spawn(run_state_writer(
            host.clone(),
            capture,
            Arc::new(Mutex::new(ForwardStatus::default())),
            None,
            shutdown,
        ));
        let (mut conn, _) = listener.accept().await.unwrap();
        let body = state_frame();
        let mut wire = (body.len() as u32).to_be_bytes().to_vec();
        wire.extend_from_slice(&body);
        conn.write_all(&wire).await.unwrap();

        host.wait_for(|h| !h.events.lock().is_empty()).await;
        let (topic, payload) = host.events.lock()[0].clone();
        assert_eq!(topic, CAPTURE_STATE_SHARED_TOPIC);
        let shared = to_json(payload);
        assert_eq!(shared["state"], "capturing");
        assert_eq!(shared["session_id"], "sess-1");
        assert!(shared.get("upload").is_none());
        host.wait_for(|h| {
            h.telemetry
                .lock()
                .iter()
                .any(|(_, p)| to_json(p.clone())["capture"]["state"] == "capturing")
        })
        .await;
        assert!(host
            .telemetry
            .lock()
            .iter()
            .all(|(c, _)| c == STATUS_CHANNEL));
    }
}
