//! The compute node's reconstruct jobs as cloud records.
//!
//! The node mirrors its reconstruct jobs into the extension's cloud records
//! (collection [`JOBS_COLLECTION`], one record per job id, subject = the drone
//! that captured it) through the plugin host's `cloud.records.put`, so the GCS
//! can show a drone's world models when no compute node is reachable on the
//! LAN. The GCS reads the reconstruction LOCAL-FIRST off the node; the cloud
//! record is the secondary/remote path.
//!
//! One representative entry per capture session: the newest COMPLETED
//! reconstruct (else the newest in-flight one), so live cycles collapse
//! cleanly. A job with no capturing drone id is skipped: the record's subject is
//! the drone that captured it, so an unattributable job is never written with a
//! wrong/empty id.
//!
//! [`JobsCloudSync`] remembers what the cloud last accepted, so a record is put
//! only when it is new or changed, and a failed put (the relay is down on an
//! unpaired or offline node) is retried on the next tick.

use std::collections::HashMap;

use serde::{Deserialize, Serialize};

use crate::{ComputeError, ComputeJobKind, ComputeJobState, JobRecord, JobStore};

/// The cloud records collection the jobs are written to (the GCS reads the
/// same name).
pub const JOBS_COLLECTION: &str = "jobs";

/// One reconstruct job in the camelCase wire shape the GCS reads.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct AtlasJobEntry {
    /// The capturing drone; never empty (a job without one is skipped upstream).
    pub device_id: String,
    /// The capture session the world model reconstructs.
    pub session_id: String,
    /// The job kind (always `reconstruct` for a world model).
    pub kind: String,
    /// The status vocabulary the GCS reads: `queued` / `running` / `done` /
    /// `error` / `cancelled` (mapped from the engine state).
    pub status: String,
    /// The dataset/bag the job ran on (lineage).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub input_bag: Option<String>,
    /// The reconstruction artifact URL when an output exists.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub output_url: Option<String>,
    /// Opaque metadata the GCS reads: `{ backend?, viewerHint?, gaussianCount? }`.
    /// `backend` drives the reconstruction-honesty badge.
    pub metadata: serde_json::Value,
    /// Job creation time (epoch ms).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub started_at: Option<i64>,
    /// Job completion time (epoch ms) when the job reached a terminal state.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub finished_at: Option<i64>,
}

/// A session's representative job: the job id (the record key) and its entry.
#[derive(Debug, Clone, PartialEq)]
pub struct AtlasJob {
    pub job_id: String,
    pub entry: AtlasJobEntry,
}

impl AtlasJob {
    /// The record body: the entry's fields plus this node's identity as both
    /// the poster and the compute node (the reconstructor is this node).
    pub fn record_data(&self, compute_node_id: &str) -> serde_json::Value {
        let mut v = serde_json::to_value(&self.entry).unwrap_or(serde_json::Value::Null);
        if let Some(obj) = v.as_object_mut() {
            obj.insert("posterDeviceId".into(), compute_node_id.into());
            obj.insert("computeNodeId".into(), compute_node_id.into());
        }
        v
    }
}

/// Map the engine's job state to the status vocabulary the GCS reads
/// (`done`/`error`, not `completed`/`failed`).
fn cmd_atlas_status(state: ComputeJobState) -> &'static str {
    match state {
        ComputeJobState::Queued => "queued",
        ComputeJobState::Running => "running",
        ComputeJobState::Completed => "done",
        ComputeJobState::Failed => "error",
        ComputeJobState::Cancelled => "cancelled",
    }
}

/// The viewer a completed artifact prefers, keyed on the output kind, matching the
/// GCS `viewerForKind` mapping. `None` lets the GCS fall back to its default world
/// viewer.
fn viewer_hint_for_kind(kind: &str) -> Option<&'static str> {
    match kind {
        "splat" => Some("splat"),
        "cloud" | "ply" | "pointcloud" => Some("cloud"),
        _ => None,
    }
}

/// A non-empty string param, or `None`.
fn str_param<'a>(job: &'a JobRecord, key: &str) -> Option<&'a str> {
    job.params
        .get(key)
        .and_then(|v| v.as_str())
        .filter(|s| !s.is_empty())
}

/// Prefer a completed job over a non-completed one; among equal completeness, the
/// newer update wins. So a session's row reflects its latest completed world model
/// (else the latest in-flight job, so the operator still sees progress).
fn is_better(new: &JobRecord, cur: &JobRecord) -> bool {
    let rank = |s: ComputeJobState| (s == ComputeJobState::Completed) as u8;
    match rank(new.state).cmp(&rank(cur.state)) {
        std::cmp::Ordering::Greater => true,
        std::cmp::Ordering::Less => false,
        std::cmp::Ordering::Equal => new.updated_ms >= cur.updated_ms,
    }
}

/// Build the reconstruct-job entries from the store: one representative per
/// session that carries a capturing drone id, with the honest backend + artifact
/// lifted from the job's first output. Sorted by session id.
pub fn build_atlas_jobs(store: &JobStore) -> Result<Vec<AtlasJob>, ComputeError> {
    // Collapse each session to one representative reconstruct job.
    let mut by_session: HashMap<String, JobRecord> = HashMap::new();
    for job in store.list_jobs()? {
        if job.kind != ComputeJobKind::Reconstruct {
            continue;
        }
        let Some(session) = str_param(&job, "session_id").map(str::to_string) else {
            continue;
        };
        // Never write a wrong/empty deviceId: a job with no capturing drone is
        // not surfaced to the cloud (it would be unattributable).
        if str_param(&job, "device_id").is_none() {
            continue;
        }
        match by_session.get(&session) {
            Some(cur) if !is_better(&job, cur) => {}
            _ => {
                by_session.insert(session, job);
            }
        }
    }

    let mut jobs = Vec::with_capacity(by_session.len());
    for (session_id, job) in by_session {
        let device_id = str_param(&job, "device_id").unwrap_or_default().to_string();

        // The reconstruction artifact (the first output, mirroring the local
        // path's `outputs[0]`) carries the honest backend + gaussian count; a job
        // with no output yet has none.
        let outputs = store.outputs_for_job(&job.id)?;
        let first = outputs.first();
        let output_url = first.map(|o| o.uri.clone());

        let mut metadata = serde_json::Map::new();
        // The honest reconstruction backend: the output's stamped
        // backend, else the requested hint before an output exists.
        let backend = first
            .and_then(|o| o.meta.get("backend").and_then(|v| v.as_str()))
            .or_else(|| job.params.get("backend").and_then(|v| v.as_str()));
        if let Some(b) = backend {
            metadata.insert("backend".into(), serde_json::Value::String(b.to_string()));
        }
        if let Some(vh) = first.and_then(|o| viewer_hint_for_kind(&o.kind)) {
            metadata.insert(
                "viewerHint".into(),
                serde_json::Value::String(vh.to_string()),
            );
        }
        if let Some(gc) = first.and_then(|o| o.meta.get("gaussian_count").and_then(|v| v.as_u64()))
        {
            metadata.insert("gaussianCount".into(), serde_json::json!(gc));
        }

        let terminal = job.state.is_terminal();
        jobs.push(AtlasJob {
            entry: AtlasJobEntry {
                device_id,
                session_id,
                kind: "reconstruct".into(),
                status: cmd_atlas_status(job.state).into(),
                input_bag: job.dataset_id.clone(),
                output_url,
                metadata: serde_json::Value::Object(metadata),
                started_at: Some(job.created_ms),
                finished_at: terminal.then_some(job.updated_ms),
            },
            job_id: job.id,
        });
    }
    // Deterministic order (a stable put order + deterministic tests).
    jobs.sort_by(|a, b| a.entry.session_id.cmp(&b.entry.session_id));
    Ok(jobs)
}

/// One record due for a put.
#[derive(Debug, Clone, PartialEq)]
pub struct PendingRecord {
    /// The record key (the job id).
    pub key: String,
    /// The record subject (the capturing drone).
    pub device_id: String,
    /// The record body.
    pub data: serde_json::Value,
}

/// What the cloud last accepted, per record key, so only new or changed
/// records are put.
#[derive(Debug, Default)]
pub struct JobsCloudSync {
    synced: HashMap<String, serde_json::Value>,
}

impl JobsCloudSync {
    pub fn new() -> Self {
        Self::default()
    }

    /// The records among `jobs` that are new or differ from what the cloud
    /// last accepted.
    pub fn pending(&self, jobs: &[AtlasJob], compute_node_id: &str) -> Vec<PendingRecord> {
        jobs.iter()
            .filter_map(|job| {
                let data = job.record_data(compute_node_id);
                (self.synced.get(&job.job_id) != Some(&data)).then(|| PendingRecord {
                    key: job.job_id.clone(),
                    device_id: job.entry.device_id.clone(),
                    data,
                })
            })
            .collect()
    }

    /// Record that the cloud accepted `record`; it is not put again until it
    /// changes.
    pub fn mark_synced(&mut self, record: PendingRecord) {
        self.synced.insert(record.key, record.data);
    }
}

/// True when a `cloud.records.put` reply says the cloud accepted the write
/// (`{ok: true}`). The relay-down `{error: "not_available", ...}` map, or any
/// other reply, is not an acceptance, so the record stays due.
pub fn put_accepted(reply: &rmpv::Value) -> bool {
    reply.as_map().is_some_and(|m| {
        m.iter()
            .any(|(k, v)| k.as_str() == Some("ok") && v.as_bool() == Some(true))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Output;

    fn recon_job(id: &str, session: &str, device: &str, state: ComputeJobState) -> JobRecord {
        JobRecord {
            id: id.into(),
            kind: ComputeJobKind::Reconstruct,
            dataset_id: Some(format!("ds-{session}")),
            state,
            progress: if state == ComputeJobState::Completed {
                1.0
            } else {
                0.0
            },
            params: serde_json::json!({
                "backend": "brush",
                "session_id": session,
                "device_id": device,
            }),
            result_ref: None,
            error: None,
            created_ms: 100,
            updated_ms: 200,
        }
    }

    fn store_with(jobs: &[JobRecord]) -> JobStore {
        let s = JobStore::open_in_memory().unwrap();
        for j in jobs {
            s.submit_job(j).unwrap();
            // Move it to its intended terminal/running state (submit inserts queued).
            if j.state != ComputeJobState::Queued {
                s.set_job_state(
                    &j.id,
                    j.state,
                    j.progress,
                    j.result_ref.as_deref(),
                    None,
                    j.updated_ms,
                )
                .unwrap();
            }
        }
        s
    }

    #[test]
    fn a_completed_reconstruct_becomes_a_done_entry_with_backend_and_output() {
        let store = store_with(&[recon_job(
            "recon-s1",
            "s1",
            "drone-1",
            ComputeJobState::Completed,
        )]);
        // The real backend + artifact ride the job's first output.
        let mut out = Output::new(
            "o1".into(),
            "recon-s1".into(),
            "splat".into(),
            "http://node:8092/artifacts/s1/world.spz".into(),
            200,
        );
        out.meta = serde_json::json!({ "gaussian_count": 250000, "backend": "brush" });
        store.insert_output(&out).unwrap();

        let jobs = build_atlas_jobs(&store).unwrap();
        assert_eq!(jobs.len(), 1);
        assert_eq!(jobs[0].job_id, "recon-s1");
        let e = &jobs[0].entry;
        assert_eq!(e.device_id, "drone-1");
        assert_eq!(e.session_id, "s1");
        assert_eq!(e.kind, "reconstruct");
        // completed → done (the vocabulary the GCS reads).
        assert_eq!(e.status, "done");
        assert_eq!(e.input_bag.as_deref(), Some("ds-s1"));
        assert_eq!(
            e.output_url.as_deref(),
            Some("http://node:8092/artifacts/s1/world.spz")
        );
        // The honest backend badge + viewer hint + gaussian count.
        assert_eq!(e.metadata["backend"], "brush");
        assert_eq!(e.metadata["viewerHint"], "splat");
        assert_eq!(e.metadata["gaussianCount"], 250000);
        assert_eq!(e.finished_at, Some(200));
    }

    #[test]
    fn the_record_body_is_the_camelcase_entry_plus_the_node_identity() {
        let store = store_with(&[recon_job(
            "recon-s1",
            "s1",
            "drone-1",
            ComputeJobState::Running,
        )]);
        let jobs = build_atlas_jobs(&store).unwrap();
        let job = jobs[0].record_data("ws-7");
        // The keys the GCS cloud-jobs reader reads, camelCased.
        assert_eq!(job["deviceId"], "drone-1");
        assert_eq!(job["sessionId"], "s1");
        assert_eq!(job["status"], "running");
        assert_eq!(job["inputBag"], "ds-s1");
        assert_eq!(job["computeNodeId"], "ws-7");
        assert_eq!(job["posterDeviceId"], "ws-7");
        // No output yet → no outputUrl key; backend falls back to the requested hint.
        assert!(job.get("outputUrl").is_none());
        assert_eq!(job["metadata"]["backend"], "brush");
    }

    #[test]
    fn a_job_without_a_capturing_drone_is_never_emitted() {
        // No device_id in params → unattributable → skipped (never a wrong/empty id).
        let mut job = recon_job("recon-x", "sx", "drone-x", ComputeJobState::Completed);
        job.params = serde_json::json!({ "backend": "brush", "session_id": "sx" });
        let store = store_with(&[job]);
        assert!(build_atlas_jobs(&store).unwrap().is_empty());
    }

    #[test]
    fn offload_jobs_and_sessionless_jobs_are_excluded() {
        let store = JobStore::open_in_memory().unwrap();
        // An offload job (no world model, no session) is not a reconstruct.
        let offload = JobRecord {
            id: "off-1".into(),
            kind: ComputeJobKind::PerceptionOffload,
            dataset_id: None,
            state: ComputeJobState::Completed,
            progress: 1.0,
            params: serde_json::json!({ "device_id": "drone-1" }),
            result_ref: None,
            error: None,
            created_ms: 1,
            updated_ms: 2,
        };
        store.submit_job(&offload).unwrap();
        assert!(build_atlas_jobs(&store).unwrap().is_empty());
    }

    #[test]
    fn the_representative_prefers_the_completed_cycle_over_a_running_one() {
        // A live session with two cycles: c0 completed, c1 running. The row shows
        // the completed world model, not the in-flight cycle (no done→running
        // regression).
        let mut c0 = recon_job("recon-s-c0", "s", "drone-1", ComputeJobState::Completed);
        c0.updated_ms = 300;
        let mut c1 = recon_job("recon-s-c1", "s", "drone-1", ComputeJobState::Running);
        c1.updated_ms = 400; // newer, but not completed
        let store = store_with(&[c0, c1]);
        let jobs = build_atlas_jobs(&store).unwrap();
        assert_eq!(jobs.len(), 1, "one row per session");
        assert_eq!(jobs[0].job_id, "recon-s-c0");
        assert_eq!(jobs[0].entry.status, "done");
    }

    #[test]
    fn a_failed_job_maps_to_error_and_carries_its_finish_time() {
        let store = store_with(&[recon_job(
            "recon-f",
            "sf",
            "drone-1",
            ComputeJobState::Failed,
        )]);
        let jobs = build_atlas_jobs(&store).unwrap();
        assert_eq!(jobs[0].entry.status, "error");
        assert_eq!(jobs[0].entry.finished_at, Some(200));
    }

    #[test]
    fn a_record_is_put_when_new_or_changed_and_retried_until_accepted() {
        let store = store_with(&[recon_job(
            "recon-s1",
            "s1",
            "drone-1",
            ComputeJobState::Running,
        )]);
        let mut sync = JobsCloudSync::new();
        let jobs = build_atlas_jobs(&store).unwrap();

        // New: due, with the capturing drone as the subject.
        let due = sync.pending(&jobs, "ws-7");
        assert_eq!(due.len(), 1);
        assert_eq!(due[0].key, "recon-s1");
        assert_eq!(due[0].device_id, "drone-1");

        // A failed put is not marked, so the record is still due next tick.
        assert_eq!(sync.pending(&jobs, "ws-7").len(), 1);

        // Accepted: nothing due while it is unchanged.
        sync.mark_synced(due.into_iter().next().unwrap());
        assert!(sync.pending(&jobs, "ws-7").is_empty());

        // The job completes: the changed record is due again.
        store
            .set_job_state("recon-s1", ComputeJobState::Completed, 1.0, None, None, 900)
            .unwrap();
        let jobs = build_atlas_jobs(&store).unwrap();
        let due = sync.pending(&jobs, "ws-7");
        assert_eq!(due.len(), 1);
        assert_eq!(due[0].data["status"], "done");
    }

    #[test]
    fn only_an_ok_reply_counts_as_accepted() {
        use rmpv::Value;
        assert!(put_accepted(&Value::Map(vec![(
            Value::from("ok"),
            Value::from(true)
        )])));
        // The relay-down shape is not an acceptance.
        assert!(!put_accepted(&Value::Map(vec![
            (Value::from("error"), Value::from("not_available")),
            (Value::from("method"), Value::from("cloud.records.put")),
        ])));
        assert!(!put_accepted(&Value::Map(vec![(
            Value::from("ok"),
            Value::from(false)
        )])));
        assert!(!put_accepted(&Value::Nil));
    }
}
