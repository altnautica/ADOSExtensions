//! Live reconstruction for [`AtlasIngest`]: the per-session cadence and the
//! periodic snapshot reconstruct jobs it yields during a flight.

use std::collections::HashSet;

use world_engine_protocol::atlas::KeyframeEnvelope;
use world_engine_protocol::compute::{ComputeJobKind, ComputeJobState};

use super::{AtlasIngest, DEFAULT_RECONSTRUCT_BACKEND};
use crate::keyframe_persister::dataset_id_for;
use crate::session::{LiveReconstructConfig, LiveReconstructDriver};
use crate::{Dataset, JobRecord};

/// Per-session live-reconstruction state: the cadence driver plus the in-flight
/// cycle's job id (so the caller can poll the store and release the
/// skip-while-running guard) and the distinct cameras seen (for the snapshot's
/// camera count).
#[derive(Debug)]
pub(super) struct SessionLive {
    pub(super) driver: LiveReconstructDriver,
    /// The job id of the periodic cycle currently in flight, if any.
    pub(super) in_flight_job: Option<String>,
    /// Distinct camera ids seen this session (the snapshot's `cameras` count).
    pub(super) camera_ids: HashSet<String>,
}

impl SessionLive {
    pub(super) fn new(config: LiveReconstructConfig, now_ms: i64) -> Self {
        Self {
            driver: LiveReconstructDriver::new(config, now_ms),
            in_flight_job: None,
            camera_ids: HashSet::new(),
        }
    }
}

impl AtlasIngest {
    /// Record a successfully-persisted keyframe against its session's live
    /// cadence. A no-op when live reconstruction is disabled (no per-session state
    /// is tracked, so the disabled path is byte-unchanged).
    pub(super) fn note_persisted(&mut self, kf: &KeyframeEnvelope, now_ms: i64) {
        if !self.live_config.enabled {
            return;
        }
        let live = self
            .sessions
            .entry(kf.session_id.clone())
            .or_insert_with(|| SessionLive::new(self.live_config, now_ms));
        live.driver.note_keyframe();
        live.camera_ids.insert(kf.camera_id.clone());
    }

    /// The `(session_id, job_id)` of every session with a periodic cycle in
    /// flight. The caller polls the store for each job's state and, for the ones
    /// that reached a terminal state (or vanished), calls
    /// [`note_cycle_finished`](Self::note_cycle_finished) to release the
    /// skip-while-running guard.
    pub fn in_flight_cycles(&self) -> Vec<(String, String)> {
        self.sessions
            .iter()
            .filter_map(|(session, live)| {
                live.in_flight_job.clone().map(|job| (session.clone(), job))
            })
            .collect()
    }

    /// Release the skip-while-running guard for `session`: its in-flight cycle's
    /// job reached a terminal state (or was purged). A no-op for an unknown or
    /// idle session.
    pub fn note_cycle_finished(&mut self, session_id: &str) {
        if let Some(live) = self.sessions.get_mut(session_id) {
            live.driver.note_cycle_finished();
            live.in_flight_job = None;
        }
    }

    /// For every active session whose cadence is due, snapshot the keyframes
    /// captured so far (a real, full reconstruct of the growing set — NOT
    /// incremental training) and return the `(Dataset, JobRecord)` for the caller
    /// to submit. A due session's skip-while-running guard is armed here, so it
    /// cannot become due again until its job finishes (coalesce, never pile up).
    ///
    /// Each cycle gets its own dataset id (`<dataset>-c<cycle>`) so its artifact
    /// lands in its own directory, while the dataset's `input_path` points at the
    /// shared, growing image directory + the latest snapshot manifest. The worker
    /// runs these exactly like a final-bag reconstruct, so each completed cycle is
    /// a fresh world-model artifact the GCS polls off the per-job output list.
    ///
    /// Only a filesystem fault from writing a snapshot manifest propagates.
    pub fn due_reconstructs(&mut self, now_ms: i64) -> std::io::Result<Vec<(Dataset, JobRecord)>> {
        // Two phases so the persister snapshot (an immutable borrow) and the
        // driver update (a mutable borrow) do not overlap on `self`.
        let due: Vec<String> = self
            .sessions
            .iter()
            .filter(|(_, live)| live.driver.due(now_ms))
            .map(|(session, _)| session.clone())
            .collect();

        let mut jobs = Vec::new();
        for session_id in due {
            let dataset_id = dataset_id_for(&session_id);
            let Some(snapshot_dir) = self.persister.snapshot(&dataset_id)? else {
                // No frames on disk yet (the cadence gate makes this unlikely);
                // skip without arming a cycle.
                continue;
            };
            // Read the capturing drone before the mutable session borrow (disjoint
            // fields; sequential keeps it obvious).
            let device_id = self.session_devices.get(&session_id).cloned();
            let live = self
                .sessions
                .get_mut(&session_id)
                .expect("the due session is present");
            let cycle = live.driver.begin_cycle(now_ms);
            let keyframes = live.driver.persisted();
            let cameras = live.camera_ids.len() as u64;

            let cycle_dataset_id = format!("{dataset_id}-c{cycle}");
            let job_id = format!("recon-{session_id}-c{cycle}");
            live.in_flight_job = Some(job_id.clone());

            let mut meta = serde_json::json!({
                "session_id": session_id.clone(),
                "input_path": snapshot_dir.to_string_lossy(),
                "keyframes": keyframes,
                "cameras": cameras,
                "live": true,
                "cycle": cycle,
            });
            // The job carries its capturing session AND the capturing drone so the
            // compute→cloud producer can attribute the world model to the drone
            // (the cloud job record's subject). Both stay absent when unknown rather than
            // asserting a wrong id.
            // The generation is the world-model artifact set's monotonic
            // version, and the cycle index IS that version for a live session:
            // each cycle publishes an immutable set the viewer can diff against
            // the last. The worker reads it back off the job to stamp the
            // descriptors it publishes.
            let mut params = serde_json::json!({
                "backend": DEFAULT_RECONSTRUCT_BACKEND,
                "session_id": session_id.clone(),
                "steps": self.train_iters,
                "generation": cycle,
            });
            if let Some(dev) = &device_id {
                meta["device_id"] = serde_json::Value::String(dev.clone());
                params["device_id"] = serde_json::Value::String(dev.clone());
            }
            let dataset = Dataset {
                id: cycle_dataset_id.clone(),
                kind: "live_snapshot".into(),
                created_ms: now_ms,
                meta,
            };
            let job = JobRecord {
                id: job_id,
                kind: ComputeJobKind::Reconstruct,
                dataset_id: Some(cycle_dataset_id),
                state: ComputeJobState::Queued,
                progress: 0.0,
                params,
                result_ref: None,
                error: None,
                created_ms: now_ms,
                updated_ms: now_ms,
            };
            jobs.push((dataset, job));
        }
        Ok(jobs)
    }
}
