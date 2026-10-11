//! The task-level supervisor around one streaming session, and its bounded
//! restart budget. See the parent module for the session lifecycle.

use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio::sync::{mpsc, Notify};

use super::SessionSpec;
use crate::offload::Detector;
use crate::offload_stream::{run_offload_session, OffloadFrameStream, SessionExit};
use crate::session_registry::{SessionProgress, SessionRegistry};

/// How many times a faulting session is restarted within [`RESTART_WINDOW`] before
/// it is given up cleanly (Closed). Bounds a persistently-failing session so it
/// never restart-loops forever.
pub(super) const MAX_RESTARTS: u32 = 3;
/// The rolling window the restart budget is counted over. A fault after a quiet
/// stretch (window elapsed) resets the budget, so an occasional transient fault
/// never permanently exhausts a session's restarts.
const RESTART_WINDOW: Duration = Duration::from_secs(60);
/// The first restart waits this long; it doubles per consecutive restart, capped
/// at [`RESTART_BACKOFF_CAP_MS`], so a tight fault loop backs off between attempts.
const RESTART_BACKOFF_BASE_MS: u64 = 100;
const RESTART_BACKOFF_CAP_MS: u64 = 2_000;

/// The outer task-level supervisor around one streaming session. The runner has
/// its own inner frame-reconnect (an RTSP blip is retried); this supervises the
/// runner TASK: a backend fault or a panic restarts the session under a bounded
/// budget over fresh streams; a clean end (cancel / stream ended / sink closed /
/// source lost) closes it. On close, the session is removed from the registry and
/// `tx` is dropped, ending the pump (and the drone's WS subscriber).
pub(super) async fn supervise_session<S, F>(
    spec: SessionSpec,
    mut factory: F,
    detector: Arc<dyn Detector>,
    tx: mpsc::Sender<world_engine_protocol::offload::OffloadDetectionBatch>,
    cancel: Arc<Notify>,
    registry: Arc<SessionRegistry>,
) where
    S: OffloadFrameStream + 'static,
    F: FnMut() -> Option<S> + Send,
{
    let progress = SessionProgress::bound(registry.clone(), spec.id.clone());
    let mut restarts: u32 = 0;
    let mut window_start = Instant::now();

    tracing::info!(session = %spec.id, camera = %spec.camera_id, "offload session started");

    let reason: &str = loop {
        let Some(stream) = factory() else {
            break "frame source exhausted";
        };

        // Run the session as a CHILD task so a panic is catchable — a panic is a
        // fault to restart under budget, never a silent loss of the session.
        let runner = {
            let id = spec.id.clone();
            let cam = spec.camera_id.clone();
            let det = detector.clone();
            let sink = tx.clone();
            let rc = cancel.clone();
            let prog = progress.clone();
            tokio::spawn(async move {
                run_offload_session(&id, &cam, stream, det, sink, rc, prog).await
            })
        };

        match runner.await {
            Ok(SessionExit::Cancelled) => break "cancelled",
            Ok(SessionExit::StreamEnded) => break "frame stream ended",
            Ok(SessionExit::SinkClosed) => break "detection sink closed",
            Ok(SessionExit::SourceLost) => break "source lost past the reconnect budget",
            Ok(SessionExit::BackendFault) => {
                if !restart_allowed(&mut restarts, &mut window_start) {
                    break "gave up after repeated backend faults";
                }
                registry.note_restart(&spec.id);
                tracing::warn!(session = %spec.id, restarts, "offload session backend faulted; restarting");
                if backoff_or_cancel(restarts, &cancel).await {
                    break "cancelled";
                }
            }
            Err(join_err) => {
                // The runner task panicked (a bug, not a handled fault). Restart it
                // under the same budget rather than silently losing the session.
                if !restart_allowed(&mut restarts, &mut window_start) {
                    break "gave up after repeated task panics";
                }
                registry.note_restart(&spec.id);
                tracing::warn!(session = %spec.id, restarts, error = %join_err, "offload session task panicked; restarting");
                if backoff_or_cancel(restarts, &cancel).await {
                    break "cancelled";
                }
            }
        }
    };

    registry.close(&spec.id);
    tracing::info!(session = %spec.id, reason, "offload session closed");
    // `tx` drops here → the pump's channel closes → the drone's WS subscribers
    // close (the drone re-opens if it still wants the session).
    drop(tx);
}

/// Whether a fault may be restarted: reset the budget if the rolling window has
/// elapsed since it started, then allow up to [`MAX_RESTARTS`] within the window.
/// Returns `false` (give up) once the budget is exhausted.
fn restart_allowed(restarts: &mut u32, window_start: &mut Instant) -> bool {
    if window_start.elapsed() >= RESTART_WINDOW {
        *restarts = 0;
        *window_start = Instant::now();
    }
    if *restarts >= MAX_RESTARTS {
        return false;
    }
    *restarts += 1;
    true
}

/// Back off before a restart (escalating, capped), cancellable. Returns `true`
/// when a cancel landed during the wait (the session should stop, not restart).
async fn backoff_or_cancel(restarts: u32, cancel: &Notify) -> bool {
    let backoff = RESTART_BACKOFF_BASE_MS
        .saturating_mul(1u64 << restarts.saturating_sub(1).min(5))
        .min(RESTART_BACKOFF_CAP_MS);
    // During the backoff no runner is running, so this is the only waiter on
    // `cancel`; a `notify_one` fired here (or a stored permit) is observed.
    tokio::select! {
        _ = tokio::time::sleep(Duration::from_millis(backoff)) => false,
        _ = cancel.notified() => true,
    }
}
