//! Node-side streaming-session manager: the auto-start + supervise half of the
//! offload path.
//!
//! A one-shot [`crate::ComputeJobKind::PerceptionOffload`] job runs the detector
//! over ONE frame carried in its params. A *streaming session* is different: an
//! NPU-less drone opens a continuous frames→detections lane, streaming its live
//! camera (RTSP) to the node and subscribing to the node's per-session detection
//! WebSocket ([`crate::offload_ws`]). This manager owns that lane on the node.
//!
//! The trigger is still a `PerceptionOffload` job, but one whose params carry a
//! `session` block ([`SessionSpec::from_job_params`]) instead of (or as well as)
//! a single `frame`. The daemon's worker loop, on claiming such a job, hands the
//! spec here — [`OffloadSessionManager::start`] registers a
//! [`crate::StreamingSession`] record and spawns a *supervisor* around
//! [`crate::run_offload_session`]. The supervisor pumps emitted batches into the
//! shared [`DetectionBroadcaster`] the WS router fans out, and it treats a
//! streaming session as a first-class, supervised, restartable unit — not a
//! fire-and-forget task.
//!
//! Session vs. job: a streaming session runs on its own tokio task, OUTSIDE the
//! batch worker pool that drains reconstruction jobs. So a session never occupies
//! a worker slot and never queues behind a reconstruction — batch cannot starve
//! streaming of a worker slot. The one shared resource is raw CPU/GPU; the worker
//! yields to a live session before starting a heavy backend
//! ([`OffloadSessionManager::should_yield_to_streaming`]), but fine-grained
//! CPU/GPU preemption of an in-flight reconstruction is out of scope.
//!
//! Lifecycle: sessions are keyed by id, so a re-submit of a live session is a
//! no-op (idempotent), and a session self-removes from the registry when it ends.
//! A session that dies from a backend fault or a panic is restarted a bounded
//! number of times before it is given up cleanly (never silently dropped, never
//! restart-looping). The registry is the source of truth for `/api/compute/status`
//! (`active_sessions` + a state breakdown) and `/api/compute/sessions` (the live
//! records). [`OffloadSessionManager::stop_all`] tears every session down on
//! shutdown.

use std::sync::Arc;
use std::time::Duration;

use serde::Deserialize;
use tokio::sync::{mpsc, Notify};

use crate::offload::Detector;
use crate::offload_stream::{OffloadFrameStream, RtspFrameStream};
use crate::offload_ws::{pump_to_broadcaster, DetectionBroadcaster};
use crate::session_registry::{now_ms, SessionRegistry, WorkPriority};

use self::supervisor::{supervise_session, MAX_RESTARTS};

/// The default RGB24 frame size assumed when a session spec omits it. The drone
/// normally advertises its camera size in the spec; this is the safety default
/// so a spec missing it still names a concrete fixed-frame size for the decoder.
const DEFAULT_WIDTH: u32 = 1280;
const DEFAULT_HEIGHT: u32 = 720;
/// Per-session channel + broadcaster buffer depth (batches). A slow WS subscriber
/// past the buffer lags and skips (never blocking the detector); the pump keeps
/// pace with the detector on this bound.
const SESSION_CHANNEL_CAP: usize = 64;

/// How often the stall watchdog scans for `Live` sessions that went quiet.
const STALL_WATCHDOG_TICK: Duration = Duration::from_secs(1);

fn default_width() -> u32 {
    DEFAULT_WIDTH
}
fn default_height() -> u32 {
    DEFAULT_HEIGHT
}

/// The description of a streaming offload session, lifted from a job's params.
///
/// The wire shape is `params.session = { id, rtsp_url, camera_id, width?, height? }`.
/// `id` is the session id (the WS path + the batch tag), `rtsp_url` is the drone's
/// live feed the node pulls (e.g. `rtsp://drone.local:8554/main`), `camera_id`
/// tags each detection, and `width`/`height` size the decoded RGB24 frames.
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
pub struct SessionSpec {
    pub id: String,
    pub rtsp_url: String,
    pub camera_id: String,
    #[serde(default = "default_width")]
    pub width: u32,
    #[serde(default = "default_height")]
    pub height: u32,
}

impl SessionSpec {
    /// Extract a session spec from a job's `params`, or `None` when the job
    /// carries no `session` block (a one-shot per-frame offload job, which takes
    /// the unchanged path). A present-but-malformed `session` is treated as
    /// absent — the caller falls back to the per-frame path rather than failing.
    pub fn from_job_params(params: &serde_json::Value) -> Option<Self> {
        let session = params.get("session")?;
        match serde_json::from_value::<SessionSpec>(session.clone()) {
            Ok(spec) if !spec.id.is_empty() && !spec.rtsp_url.is_empty() => Some(spec),
            _ => None,
        }
    }
}

/// Owns the node's streaming-session registry + the shared detection broadcaster
/// the WS router fans out. Held by the daemon for its lifetime.
pub struct OffloadSessionManager {
    broadcaster: Arc<DetectionBroadcaster>,
    detector: Arc<dyn Detector>,
    /// Whether the node serves offload sessions at all (the
    /// `perception.serving.enabled` toggle). When false, `start` declines so the
    /// node reconstructs but never runs perception offload for a drone.
    serve_offload: bool,
    /// The live streaming-session registry — the source of truth for every
    /// session's state, throughput, and reconnect / restart history, plus the
    /// lock-free `active_sessions` projection shared into the engine's heartbeat.
    registry: Arc<SessionRegistry>,
}

impl OffloadSessionManager {
    /// The priority a live streaming offload session runs at.
    pub const STREAMING_PRIORITY: WorkPriority = WorkPriority::Streaming;
    /// The priority best-effort batch reconstruction runs at.
    pub const BATCH_PRIORITY: WorkPriority = WorkPriority::Batch;
    /// The restart budget (exposed for tests + observability).
    pub const MAX_RESTARTS: u32 = MAX_RESTARTS;

    /// A manager fanning sessions out over `broadcaster`, running `detector` on
    /// every streamed frame. `serve_offload` is the config toggle: when false,
    /// every `start` is declined (the node still does reconstruction).
    pub fn new(
        broadcaster: Arc<DetectionBroadcaster>,
        detector: Arc<dyn Detector>,
        serve_offload: bool,
    ) -> Self {
        Self {
            broadcaster,
            detector,
            serve_offload,
            registry: Arc::new(SessionRegistry::new()),
        }
    }

    /// The live-session projection (Opening|Live count), shared into the engine so
    /// its heartbeat reflects active offload sessions with no lock.
    pub fn session_counter(&self) -> Arc<std::sync::atomic::AtomicU32> {
        self.registry.active_counter()
    }

    /// The session registry handle, so the job API can read the live records
    /// (`/api/compute/sessions`) + the state breakdown (`/api/compute/status`).
    pub fn registry(&self) -> Arc<SessionRegistry> {
        self.registry.clone()
    }

    /// Whether any streaming session is live (the batch-yields-to-streaming
    /// signal).
    pub fn any_live(&self) -> bool {
        self.registry.any_live()
    }

    /// Whether best-effort batch reconstruction should yield to a live streaming
    /// session before starting its heavy backend. Streaming is latency-critical
    /// ([`Self::STREAMING_PRIORITY`]) and outranks best-effort batch
    /// ([`Self::BATCH_PRIORITY`]); sessions run outside the batch worker pool, so
    /// this only nudges shared-CPU/GPU scheduling, never a worker slot.
    pub fn should_yield_to_streaming(&self) -> bool {
        self.any_live() && Self::STREAMING_PRIORITY > Self::BATCH_PRIORITY
    }

    /// Start (or re-affirm) the session described by `spec`, pulling the drone's
    /// RTSP feed. Idempotent: a spec whose id is already live is a no-op. The
    /// session is supervised — a backend fault / panic restarts it under a bounded
    /// budget over fresh RTSP streams.
    pub async fn start(&self, spec: SessionSpec) {
        let (cam, url, w, h) = (
            spec.camera_id.clone(),
            spec.rtsp_url.clone(),
            spec.width,
            spec.height,
        );
        // A fresh RTSP stream per (re)start: a supervised restart re-spawns ffmpeg.
        let factory = move || Some(RtspFrameStream::new(cam.clone(), url.clone(), w, h));
        self.start_with_factory(spec, factory).await;
    }

    /// Start a session over a single injected frame stream. Tests call this with a
    /// synthetic [`crate::VecFrameStream`] so the manager's lifecycle is exercised
    /// with no ffmpeg / camera / RTSP. The single stream is consumed once; a
    /// supervised restart (a backend fault) has no fresh stream to re-run, so the
    /// session closes rather than restarting.
    pub async fn start_with_stream<S>(&self, spec: SessionSpec, stream: S)
    where
        S: OffloadFrameStream + 'static,
    {
        let mut held = Some(stream);
        self.start_with_factory(spec, move || held.take()).await;
    }

    /// The core supervised start: register the session, spawn the detection pump,
    /// and spawn the supervisor around the runner. `factory` yields a fresh frame
    /// stream per (re)start; `None` means no more streams to run (a consumed
    /// single-shot stream), so the supervisor closes rather than restarting.
    pub async fn start_with_factory<S, F>(&self, spec: SessionSpec, factory: F)
    where
        S: OffloadFrameStream + 'static,
        F: FnMut() -> Option<S> + Send + 'static,
    {
        if !self.serve_offload {
            tracing::info!(session = %spec.id, "offload serving disabled by config; session declined");
            return;
        }
        // Register the session (Opening) synchronously up front, so the active
        // count reflects it before the supervisor task is even scheduled and a
        // concurrent re-submit of the same id is deduped.
        let cancel = Arc::new(Notify::new());
        if !self.registry.register(
            &spec.id,
            &spec.camera_id,
            &spec.rtsp_url,
            cancel.clone(),
            now_ms(),
        ) {
            tracing::info!(session = %spec.id, "offload session already running; re-submit ignored");
            return;
        }

        // The detection return lane: the runner emits one batch per frame onto
        // `tx`; the pump forwards them into the broadcaster the WS router fans out.
        // The SUPERVISOR owns `tx` and hands a clone to each runner iteration, so a
        // restart is transparent to the drone's WS subscriber — the pump (and the
        // WS with it) closes only when the session truly ends and the supervisor
        // drops its `tx`.
        let (tx, rx) = mpsc::channel(SESSION_CHANNEL_CAP);
        tokio::spawn(pump_to_broadcaster(
            spec.id.clone(),
            rx,
            self.broadcaster.clone(),
        ));

        let detector = self.detector.clone();
        let registry = self.registry.clone();
        tokio::spawn(supervise_session(
            spec, factory, detector, tx, cancel, registry,
        ));
    }

    /// Stop the named session (best-effort). Returns whether it was live. The
    /// session's supervisor observes the cancel, closes the session, and removes
    /// it from the registry as it unwinds.
    pub async fn stop(&self, session_id: &str) -> bool {
        match self.registry.cancel_of(session_id) {
            Some(c) => {
                c.notify_one();
                true
            }
            None => false,
        }
    }

    /// Cancel every live session (daemon shutdown). Each supervisor reaps its own
    /// registry record as it stops.
    pub async fn stop_all(&self) {
        for c in self.registry.all_cancels() {
            c.notify_one();
        }
    }

    /// How many sessions are currently live (any of Opening/Live/Stalled).
    pub async fn active_count(&self) -> usize {
        self.registry.len()
    }

    /// The shared broadcaster (the WS router mounts on it).
    pub fn broadcaster(&self) -> Arc<DetectionBroadcaster> {
        self.broadcaster.clone()
    }

    /// Spawn the stall watchdog: on a tick, flip any `Live` session with no batch
    /// within the stall window to `Stalled`, so a session whose source went quiet
    /// without a reader error reads honestly rather than as a stale `Live`. The
    /// daemon calls this once at startup; it runs for the process lifetime.
    pub fn spawn_stall_watchdog(&self) {
        let registry = self.registry.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(STALL_WATCHDOG_TICK);
            tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
            loop {
                tick.tick().await;
                for id in registry
                    .stall_quiet_sessions(now_ms(), crate::session_registry::STALL_WINDOW_MS)
                {
                    tracing::info!(session = %id, "offload session stalled: no detections within the stall window");
                }
            }
        });
    }
}

mod supervisor;
#[cfg(test)]
mod tests;
