//! The job and output shapes the REST API serves: a job record plus its
//! staged-pipeline state (current stage, per-stage rows, GPU, ETA, tool
//! versions, structured error), and an output plus its deliverable facts.

use serde::Serialize;

use crate::recon::{self, StageError};
use crate::store_stages::{StageRow, StageState};
use crate::worker::NvidiaGpu;
use crate::{ComputeError, ComputeJobKind, ComputeJobState, JobRecord, JobStore, Output};

/// A job as served over the REST API.
#[derive(Debug, Serialize)]
pub struct JobView {
    pub id: String,
    pub kind: ComputeJobKind,
    pub dataset_id: Option<String>,
    pub state: ComputeJobState,
    pub progress: f32,
    pub params: serde_json::Value,
    pub result_ref: Option<String>,
    pub created_ms: i64,
    pub updated_ms: i64,
    /// The capturing session a job belongs to, lifted from `params`.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub session_id: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub preset: Option<String>,
    /// `queued`, the running stage id, `done`, `failed` or `cancelled`.
    pub stage: String,
    pub stage_progress: f32,
    pub stages: Vec<StageRow>,
    pub gpu: Option<NvidiaGpu>,
    pub eta_s: Option<u64>,
    pub tools: std::collections::BTreeMap<String, String>,
    pub error: Option<StageError>,
}

fn now_ns() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_nanos() as i64)
        .unwrap_or(0)
}

/// The pinned tools whose versions a job reports.
fn job_tools(job: &JobRecord) -> std::collections::BTreeMap<String, String> {
    let pins = crate::tools::pins();
    let mut names = recon::required_tools(job);
    if job.kind == ComputeJobKind::Calibrate {
        names.extend(["kalibr", "dvlc"]);
    }
    names
        .into_iter()
        .filter_map(|n| pins.get(n).map(|p| (n.to_string(), p.version.clone())))
        .collect()
}

impl JobView {
    /// Build the view; `gpu` is the current GPU sample (shown on running jobs).
    pub fn build(
        store: &JobStore,
        job: JobRecord,
        gpu: Option<&NvidiaGpu>,
    ) -> Result<Self, ComputeError> {
        let stages = store.stages(&job.id)?;
        let running = stages.iter().find(|r| r.state == StageState::Running);
        let stage = match job.state {
            ComputeJobState::Queued => "queued".to_string(),
            ComputeJobState::Running => running
                .map(|r| r.id.clone())
                .unwrap_or_else(|| "running".to_string()),
            ComputeJobState::Completed => "done".to_string(),
            ComputeJobState::Failed => "failed".to_string(),
            ComputeJobState::Cancelled => "cancelled".to_string(),
        };
        let stage_progress = running.map(|r| r.progress).unwrap_or(match job.state {
            ComputeJobState::Completed => 1.0,
            _ => 0.0,
        });
        let eta_s = (job.state == ComputeJobState::Running && job.progress > 0.02)
            .then(|| stages.iter().filter_map(|r| r.started_ns).min())
            .flatten()
            .map(|start| {
                let elapsed = (now_ns() - start).max(0) as f64 / 1e9;
                (elapsed * (1.0 - job.progress as f64) / job.progress as f64) as u64
            });
        let error = match store.job_error(&job.id)? {
            Some(e) => Some(e),
            None if job.state == ComputeJobState::Failed => Some(StageError::new(
                "E_JOB_FAILED",
                job.error.clone().unwrap_or_default(),
            )),
            None => None,
        };
        let text = |k: &str| {
            job.params
                .get(k)
                .and_then(|v| v.as_str())
                .map(str::to_string)
        };
        Ok(Self {
            session_id: text("session_id"),
            preset: text("preset"),
            tools: job_tools(&job),
            gpu: (job.state == ComputeJobState::Running)
                .then(|| gpu.cloned())
                .flatten(),
            stage,
            stage_progress,
            stages,
            eta_s,
            error,
            id: job.id,
            kind: job.kind,
            dataset_id: job.dataset_id,
            state: job.state,
            progress: job.progress,
            params: job.params,
            result_ref: job.result_ref,
            created_ms: job.created_ms,
            updated_ms: job.updated_ms,
        })
    }
}

/// An output as served over the REST API: the record plus the deliverable's
/// `file`, `format`, `size_bytes` and `place_json` lifted from its metadata.
#[derive(Debug, Serialize)]
pub struct OutputView {
    #[serde(flatten)]
    pub output: Output,
    pub file: Option<String>,
    pub format: Option<String>,
    pub size_bytes: Option<u64>,
    pub place_json: Option<String>,
}

impl OutputView {
    pub fn of(output: Output) -> Self {
        let s = |k: &str| {
            output
                .meta
                .get(k)
                .and_then(|v| v.as_str())
                .map(str::to_string)
        };
        Self {
            file: s("file"),
            format: s("format"),
            size_bytes: output.meta.get("size_bytes").and_then(|v| v.as_u64()),
            place_json: s("place_json"),
            output,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn job(state: ComputeJobState) -> JobRecord {
        JobRecord {
            id: "j".into(),
            kind: ComputeJobKind::Reconstruct,
            dataset_id: Some("s1".into()),
            state,
            progress: 0.5,
            params: serde_json::json!({"session_id": "s1", "preset": "preview"}),
            result_ref: None,
            error: Some("boom".into()),
            created_ms: 1,
            updated_ms: 2,
        }
    }

    #[test]
    fn a_running_staged_job_reports_its_stage_gpu_and_tools() {
        let s = JobStore::open_in_memory().unwrap();
        let mut j = job(ComputeJobState::Queued);
        j.error = None;
        s.submit_job(&j).unwrap();
        s.init_stages("j", &["prepare", "sfm"]).unwrap();
        s.claim_next_queued(3).unwrap();
        s.set_stage("j", "prepare", StageState::Done, 10, None)
            .unwrap();
        s.set_stage(
            "j",
            "sfm",
            StageState::Running,
            now_ns() - 5_000_000_000,
            None,
        )
        .unwrap();
        s.set_stage_progress("j", "sfm", 0.25, 0.5, 4).unwrap();
        let gpu = NvidiaGpu {
            name: "gpu".into(),
            util_pct: Some(50.0),
            vram_used_mb: Some(1000),
            vram_total_mb: Some(8000),
        };
        let v = JobView::build(&s, s.get_job("j").unwrap().unwrap(), Some(&gpu)).unwrap();
        let json = serde_json::to_value(&v).unwrap();
        assert_eq!(json["stage"], "sfm");
        assert_eq!(json["stage_progress"], 0.25);
        assert_eq!(json["stages"][0]["state"], "done");
        assert_eq!(json["gpu"]["name"], "gpu");
        assert!(json["gpu"].get("vram_total_mb").is_none());
        assert!(json["eta_s"].as_u64().is_some());
        assert_eq!(json["session_id"], "s1");
        assert!(json["error"].is_null());
        assert!(json["tools"].is_object());
    }

    #[test]
    fn a_failed_job_without_a_structured_error_reports_its_message() {
        let s = JobStore::open_in_memory().unwrap();
        let v = JobView::build(&s, job(ComputeJobState::Failed), None).unwrap();
        let json = serde_json::to_value(&v).unwrap();
        assert_eq!(json["stage"], "failed");
        assert_eq!(json["error"]["code"], "E_JOB_FAILED");
        assert_eq!(json["error"]["message"], "boom");
        assert!(json["gpu"].is_null());
    }

    #[test]
    fn output_views_lift_the_deliverable_facts() {
        let mut o = Output::new("o".into(), "j".into(), "splat".into(), "u".into(), 1);
        o.meta = serde_json::json!({"file": "scene.spz", "format": "spz", "size_bytes": 9, "place_json": null});
        let v = serde_json::to_value(OutputView::of(o)).unwrap();
        assert_eq!(v["file"], "scene.spz");
        assert_eq!(v["size_bytes"], 9);
        assert!(v["place_json"].is_null());
        assert_eq!(v["kind"], "splat");
    }
}
