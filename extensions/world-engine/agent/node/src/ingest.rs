//! Atlas capture ingest: turn the keyframe + capture-state events a drone
//! forwards to the compute node into a reconstructor-ingestible dataset and a
//! reconstruct job.
//!
//! A compute node receives framed `AtlasEvent`s off a bearer (LAN / WFB / cloud)
//! into its event router. This is the seam between that receive side and the job
//! queue. Two things happen here:
//!
//! - A keyframe event carries the image bytes plus the pose and intrinsics; the
//!   [`KeyframePersister`] streams the image to disk and records its frame. This
//!   touches no store state, so it runs WITHOUT the engine lock.
//! - The terminal `Bagged` capture-state finalizes the on-disk dataset (writes
//!   `transforms.json`) and produces the dataset + `Reconstruct` job for the
//!   caller to submit under its store lock. The dataset carries `input_path` so a
//!   backend trains on the real images, not the empty default.
//!
//! A malformed frame is dropped + logged (never an error that could tear down the
//! receive loop); only a real filesystem fault propagates from [`AtlasIngest::step`].

use std::collections::{HashMap, HashSet};

use world_engine_protocol::atlas::{
    AtlasEvent, CaptureState, CaptureStatus, KeyframeBudget, KeyframeEnvelope, PoseSource,
    VioHealth, ATLAS_CAPTURE_STATE_TOPIC, ATLAS_KEYFRAME_TOPIC,
};
use world_engine_protocol::compute::{ComputeJobKind, ComputeJobState};

use crate::keyframe_persister::{dataset_id_for, KeyframePersister};
use crate::session::LiveReconstructConfig;
use crate::store::JobStore;
use crate::ComputeError;
use crate::{Dataset, JobRecord};

#[path = "ingest_live.rs"]
mod live;
use live::SessionLive;

/// The reconstruct backend a captured session defaults to. `auto` lets the
/// compute node pick per host: the seamless COLMAP-seed → native-Metal `msplat`
/// path on Apple Silicon, a CUDA trainer or the portable Brush trainer elsewhere,
/// and the mock backend on a node with no trainer (CI / no-GPU). The keyframes
/// carry VIO / FC poses, so a node without a COLMAP seed still trains directly on
/// the written `transforms.json` (Brush random-init, no pose search).
const DEFAULT_RECONSTRUCT_BACKEND: &str = "auto";

/// The default gaussian-splat training length (`steps` job param) when nothing
/// overrides it. A splat trainer's quality-vs-time knob; the backend reads it as
/// `--total-steps` (Brush) / `-n` (msplat).
const DEFAULT_TRAIN_ITERS: u64 = 30_000;

/// Resolve the reconstruct training length: the `ADOS_COMPUTE_TRAIN_ITERS`
/// override when set to a valid `u64`, else [`DEFAULT_TRAIN_ITERS`]. Read once at
/// ingest-construction time (the daemon builds one `AtlasIngest`), so a shorter
/// training run for a fast node is a single env var, not a per-job edit.
fn default_train_iters() -> u64 {
    std::env::var("ADOS_COMPUTE_TRAIN_ITERS")
        .ok()
        .and_then(|v| v.trim().parse::<u64>().ok())
        .unwrap_or(DEFAULT_TRAIN_ITERS)
}

/// A session silent this long with no `Bagged` frame is finalized as-is.
pub const SESSION_IDLE_MS: i64 = 10 * 60 * 1000;

/// One capture session's reconstruction-input accounting.
#[derive(Debug, Clone, Default)]
struct SessionAccount {
    seen: u64,
    rejected: u64,
    undisciplined_clock: u64,
    persisted: u64,
    cameras: HashSet<String>,
    last_event_ms: i64,
}

/// Accumulates a capture session's events: persists each keyframe's image and, on
/// the bagged terminal state, finalizes the dataset and yields the reconstruct
/// job. When live reconstruction is enabled, it also tracks a per-session cadence
/// and, on a tick, snapshots the growing dataset into fresh periodic reconstruct
/// jobs ([`due_reconstructs`](Self::due_reconstructs)) so the world model updates
/// during the flight, not only at the end.
#[derive(Debug)]
pub struct AtlasIngest {
    /// Keyframe events seen this session (received-side delivery count).
    keyframes_seen: u64,
    /// Keyframe events REFUSED as reconstruction input (see
    /// [`AtlasIngest::keyframes_rejected`]).
    keyframes_rejected: u64,
    /// Keyframes accepted whose clock offset was never measured, so the dataset
    /// can carry the honest fact rather than the reconstruction quietly
    /// inheriting an undisciplined clock.
    keyframes_undisciplined_clock: u64,
    /// What a keyframe must satisfy to enter a dataset. Defaults to
    /// [`KeyframeBudget::default`]: covariance and anchor are gated hard; the
    /// timebase sigma is carried and reported rather than gated, because a node
    /// without a disciplined clock honestly reports it
    /// unmeasured and rejecting every frame there would yield zero
    /// reconstructions with no operator-visible cause.
    budget: KeyframeBudget,
    persister: KeyframePersister,
    /// Live-reconstruction cadence config (disabled by default → final bag only).
    live_config: LiveReconstructConfig,
    /// Per-session live state, keyed by session id. Empty when live reconstruction
    /// is disabled (the sessions are never tracked, so the path is byte-unchanged).
    sessions: HashMap<String, SessionLive>,
    /// The capturing drone's device id, keyed by session id, learned from each
    /// event's `device_id` (the drone-side forwarder stamps it on egress). Stamped
    /// into the dataset + reconstruct job so the compute→cloud producer can
    /// attribute the world model to the drone that captured it. Empty when no event
    /// carried an id (a pre-attribution capture), and the job then omits it rather
    /// than asserting a wrong one.
    session_devices: HashMap<String, String>,
    /// Per-session reconstruction-input accounting and last activity, keyed by
    /// session id. A dataset carries ITS session's numbers, never the node-wide
    /// totals that mix in other sessions and other drones; the last-event time
    /// lets a session whose `Bagged` frame never arrived be finalized anyway.
    accounts: HashMap<String, SessionAccount>,
    /// The training length stamped onto every reconstruct job's `steps` param, so
    /// the backend trains for a configurable number of iterations instead of the
    /// hardcoded default. Resolved once from [`default_train_iters`].
    train_iters: u64,
}

impl AtlasIngest {
    /// An ingest that persists datasets under `work_root` (the same root the
    /// reconstructor reads `input_path` from and the artifact server serves), with
    /// live reconstruction disabled (final-bag reconstruct only).
    pub fn new(work_root: impl Into<std::path::PathBuf>) -> Self {
        Self::with_live_config(work_root, LiveReconstructConfig::default())
    }

    /// An ingest with an explicit live-reconstruction cadence. When
    /// `live_config.enabled`, an active session is snapshotted and reconstructed
    /// on the cadence (see [`due_reconstructs`](Self::due_reconstructs)); when
    /// disabled, the node reconstructs only the final bag.
    pub fn with_live_config(
        work_root: impl Into<std::path::PathBuf>,
        live_config: LiveReconstructConfig,
    ) -> Self {
        Self {
            keyframes_seen: 0,
            keyframes_rejected: 0,
            keyframes_undisciplined_clock: 0,
            budget: KeyframeBudget::default(),
            persister: KeyframePersister::new(work_root),
            live_config,
            sessions: HashMap::new(),
            session_devices: HashMap::new(),
            accounts: HashMap::new(),
            train_iters: default_train_iters(),
        }
    }

    /// Override what a keyframe must satisfy to enter a dataset — the opt-in for
    /// a rig with a PPS-disciplined clock ([`KeyframeBudget::strict`]).
    pub fn with_budget(mut self, budget: KeyframeBudget) -> Self {
        self.budget = budget;
        self
    }

    /// Keyframes refused as reconstruction input. Every one of these would
    /// otherwise have entered a dataset and baked a wrong pose into the
    /// reconstruction with no other symptom — a mis-posed keyframe does not make
    /// the output visibly broken, it makes it subtly, confidently wrong.
    pub fn keyframes_rejected(&self) -> u64 {
        self.keyframes_rejected
    }

    /// Accepted keyframes whose clock offset was never measured.
    pub fn keyframes_undisciplined_clock(&self) -> u64 {
        self.keyframes_undisciplined_clock
    }

    /// Keyframe events received so far (the received-side delivery proof — the
    /// drone's send is fire-and-forget, so only what the node decodes counts).
    pub fn keyframes_seen(&self) -> u64 {
        self.keyframes_seen
    }

    /// Process one received Atlas event WITHOUT touching the store. A keyframe is
    /// persisted to disk (and the received counter bumped); a `Bagged`
    /// capture-state finalizes the dataset and returns the dataset + reconstruct
    /// job for the caller to submit under its store lock (see
    /// [`submit_reconstruct_job`]). Other topics, non-terminal states, and
    /// malformed payloads return `None`. Only a filesystem fault on the manifest
    /// write propagates.
    pub fn step(
        &mut self,
        event: &AtlasEvent,
        now_ms: i64,
    ) -> std::io::Result<Option<(Dataset, JobRecord)>> {
        match event.topic.as_str() {
            ATLAS_KEYFRAME_TOPIC => {
                self.keyframes_seen += 1;
                match KeyframeEnvelope::from_msgpack(&event.payload) {
                    Ok(kf) => {
                        self.note_device(&kf.session_id, event.device_id.as_deref());
                        let acct = self.accounts.entry(kf.session_id.clone()).or_default();
                        acct.seen += 1;
                        acct.last_event_ms = now_ms;
                        // Refuse a frame the reconstructor cannot trust BEFORE it
                        // reaches disk. A frame with no position-prior sigma
                        // destabilises the bundle adjustment, one whose pose was
                        // paired too far from its exposure instant is placed where
                        // the aircraft was not, and one captured before the geo
                        // anchor latched sits in a different frame of reference
                        // from every other frame in the same session.
                        if let Err(reason) = kf.validate_with(&self.budget) {
                            self.keyframes_rejected += 1;
                            if let Some(acct) = self.accounts.get_mut(&kf.session_id) {
                                acct.rejected += 1;
                            }
                            tracing::warn!(
                                session = %kf.session_id,
                                kf_id = kf.kf_id,
                                camera = %kf.camera_id,
                                rejected_total = self.keyframes_rejected,
                                reason = %reason,
                                "atlas_keyframe_rejected"
                            );
                            return Ok(None);
                        }
                        if !kf.clock_is_disciplined() {
                            self.keyframes_undisciplined_clock += 1;
                            if let Some(acct) = self.accounts.get_mut(&kf.session_id) {
                                acct.undisciplined_clock += 1;
                            }
                        }
                        match self.persister.persist(&kf) {
                            Ok(()) => {
                                if let Some(acct) = self.accounts.get_mut(&kf.session_id) {
                                    acct.persisted += 1;
                                    acct.cameras.insert(kf.camera_id.clone());
                                }
                                self.note_persisted(&kf, now_ms)
                            }
                            Err(e) => tracing::warn!(error = %e, "atlas_keyframe_persist_failed"),
                        }
                    }
                    Err(e) => tracing::debug!(error = %e, "atlas_keyframe_decode_failed"),
                }
                Ok(None)
            }
            ATLAS_CAPTURE_STATE_TOPIC => match CaptureStatus::from_msgpack(&event.payload) {
                Ok(status) if status.state == CaptureState::Bagged => {
                    self.note_device(&status.session_id, event.device_id.as_deref());
                    // `bag` yields `None` for a session that persisted zero
                    // keyframes (no reconstruct to run); the Option propagates.
                    let result = self.bag(&status, now_ms)?;
                    // The session is over: drop its live + device state regardless of
                    // whether a job was produced. Any periodic cycle still running
                    // finishes on its own in the worker; the final bag reconstruct is
                    // the authoritative full-set output.
                    self.forget_session(&status.session_id);
                    Ok(result)
                }
                Ok(status) => {
                    self.note_device(&status.session_id, event.device_id.as_deref());
                    self.accounts
                        .entry(status.session_id.clone())
                        .or_default()
                        .last_event_ms = now_ms;
                    Ok(None)
                }
                Err(e) => {
                    tracing::debug!(error = %e, "atlas_ingest_bad_capture_state");
                    Ok(None)
                }
            },
            _ => Ok(None),
        }
    }

    /// Drop every piece of per-session state once the session is over.
    fn forget_session(&mut self, session_id: &str) {
        self.sessions.remove(session_id);
        self.session_devices.remove(session_id);
        self.accounts.remove(session_id);
    }

    /// Finalize every session that has persisted keyframes but has been silent
    /// for [`SESSION_IDLE_MS`]: its `Bagged` frame was lost (power-off, link loss
    /// at landing), so without this the capture would never be reconstructed and
    /// its state would stay in memory for the life of the node. Returns the
    /// dataset + reconstruct job for each, exactly as a received bag would.
    pub fn idle_bags(&mut self, now_ms: i64) -> std::io::Result<Vec<(Dataset, JobRecord)>> {
        let idle: Vec<String> = self
            .accounts
            .iter()
            .filter(|(_, a)| now_ms.saturating_sub(a.last_event_ms) >= SESSION_IDLE_MS)
            .map(|(id, _)| id.clone())
            .collect();
        let mut out = Vec::new();
        for session_id in idle {
            let (keyframes, cameras) = self
                .accounts
                .get(&session_id)
                .map(|a| (a.persisted, a.cameras.len() as u32))
                .unwrap_or((0, 0));
            tracing::info!(session = %session_id, keyframes, "atlas session idle without a bag; finalizing");
            let status = CaptureStatus {
                session_id: session_id.clone(),
                state: CaptureState::Bagged,
                keyframes,
                vio_health: VioHealth::Lost,
                camera_count: cameras,
                ingest_rate_hz: 0.0,
                capped: false,
                anchored: true,
                pose_tier: PoseSource::FcEkf,
                dropped_keyframes: 0,
            };
            let result = self.bag(&status, now_ms)?;
            self.forget_session(&session_id);
            out.extend(result);
        }
        Ok(out)
    }

    /// Record the capturing drone's device id for a session from an event's
    /// `device_id`. A no-op for an absent or empty id, so a session is never
    /// attributed to an empty drone (the job then omits `device_id` rather than
    /// asserting a wrong one).
    fn note_device(&mut self, session_id: &str, device_id: Option<&str>) {
        if let Some(dev) = device_id {
            if !dev.is_empty() {
                self.session_devices
                    .insert(session_id.to_string(), dev.to_string());
            }
        }
    }

    /// Finalize a bagged session: write `transforms.json` and build the dataset +
    /// reconstruct job. Returns `None` when the session persisted zero keyframes —
    /// there is nothing to reconstruct, so no job is enqueued (a reconstruct over
    /// an empty dataset only dies at the backend with a "no such directory" error).
    /// When at least one keyframe was persisted the dataset carries `input_path` so
    /// the backend trains on the real images.
    fn bag(
        &mut self,
        status: &CaptureStatus,
        now_ms: i64,
    ) -> std::io::Result<Option<(Dataset, JobRecord)>> {
        let dataset_id = dataset_id_for(&status.session_id);
        let input_path = self.persister.finalize(&dataset_id)?;
        // No keyframes persisted → no dataset on disk → no reconstruct to run. Skip
        // enqueueing a job that would only fail at the backend with a missing input.
        if input_path.is_none() {
            tracing::info!(
                session = %status.session_id,
                "atlas capture bagged with no keyframes; skipping reconstruct"
            );
            return Ok(None);
        }
        let device_id = self.session_devices.get(&status.session_id).cloned();
        let acct = self
            .accounts
            .get(&status.session_id)
            .cloned()
            .unwrap_or_default();
        // The final bag is the generation AFTER every periodic cycle, so a viewer
        // holding cycle N-1 sees the full-set artifact as strictly newer.
        let generation = self
            .sessions
            .get(&status.session_id)
            .map(|l| l.driver.cycles())
            .unwrap_or(0);

        let mut meta = serde_json::json!({
            "keyframes": status.keyframes,
            "cameras": status.camera_count,
            "received_keyframes": acct.seen,
            // Honest reconstruction-input accounting: how many frames were
            // refused, and how many were accepted from an undisciplined clock.
            // Both are properties of the dataset a quality gate must weigh, and
            // neither is visible from the keyframe count alone.
            "rejected_keyframes": acct.rejected,
            "undisciplined_clock_keyframes": acct.undisciplined_clock,
        });
        if let Some(path) = &input_path {
            meta["input_path"] = serde_json::Value::String(path.to_string_lossy().into_owned());
        }

        // The job carries its capturing session AND the capturing drone so the
        // compute→cloud producer can attribute the world model to the drone
        // (the cloud job record's subject). Both stay absent when unknown rather than
        // asserting a wrong id.
        let mut params = serde_json::json!({
            "backend": DEFAULT_RECONSTRUCT_BACKEND,
            "session_id": status.session_id.clone(),
            "steps": self.train_iters,
            "generation": generation,
        });
        if let Some(dev) = &device_id {
            meta["device_id"] = serde_json::Value::String(dev.clone());
            params["device_id"] = serde_json::Value::String(dev.clone());
        }

        let dataset = Dataset {
            id: dataset_id.clone(),
            kind: "bag".into(),
            created_ms: now_ms,
            meta,
        };
        let job = JobRecord {
            id: format!("recon-{}", status.session_id),
            kind: ComputeJobKind::Reconstruct,
            dataset_id: Some(dataset_id),
            state: ComputeJobState::Queued,
            progress: 0.0,
            params,
            result_ref: None,
            error: None,
            created_ms: now_ms,
            updated_ms: now_ms,
        };
        Ok(Some((dataset, job)))
    }
}

/// Submit a bagged session's dataset + reconstruct job to the store, idempotently.
/// The dataset and job ids are deterministic per session, so a re-sent `Bagged`
/// (plausible on the lossy fire-and-forget lane) finds them already present and is
/// swallowed (a `Conflict` is success, not a store fault) rather than erroring the
/// receive loop. Run under the engine lock; the store write is brief. Returns the
/// reconstruct job id.
pub fn submit_reconstruct_job(
    store: &JobStore,
    dataset: &Dataset,
    job: &JobRecord,
) -> Result<String, ComputeError> {
    if let Err(e) = store.insert_dataset(dataset) {
        if !matches!(e, ComputeError::Conflict(_)) {
            return Err(e);
        }
    }
    if let Err(e) = store.submit_job(job) {
        if !matches!(e, ComputeError::Conflict(_)) {
            return Err(e);
        }
        tracing::debug!(job = %job.id, "atlas_ingest_duplicate_bagged");
    }
    Ok(job.id.clone())
}

#[cfg(test)]
#[path = "ingest_tests.rs"]
mod tests;
