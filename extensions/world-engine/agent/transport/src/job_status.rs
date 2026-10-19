//! A compute job's status as the node's job API serves it
//! (`GET /api/compute/jobs/{id}`): the job record plus the staged pipeline's
//! current stage, per-stage rows, GPU sample, ETA, pinned tool versions and a
//! structured error `{code, message, tool?}`.

use std::collections::BTreeMap;

use serde::{Deserialize, Deserializer, Serialize};
use world_engine_protocol::compute::{ComputeJobKind, ComputeJobState};

/// Why a job failed or was refused.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct JobError {
    /// A stable code (`E_TOOL_MISSING`, `E_NOT_UPLOADED`, `E_JOB_FAILED`, ...).
    pub code: String,
    #[serde(default, alias = "detail")]
    pub message: String,
    /// The pinned tool the error is about, when there is one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool: Option<String>,
}

/// The error as an object, or the plain string an older node sends.
fn job_error<'de, D: Deserializer<'de>>(d: D) -> Result<Option<JobError>, D::Error> {
    #[derive(Deserialize)]
    #[serde(untagged)]
    enum Wire {
        Object(JobError),
        Text(String),
    }
    Ok(Option::<Wire>::deserialize(d)?.map(|w| match w {
        Wire::Object(e) => e,
        Wire::Text(message) => JobError {
            code: "E_JOB_FAILED".to_string(),
            message,
            tool: None,
        },
    }))
}

/// One pipeline stage of a job.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JobStage {
    /// `prepare`, `sfm`, `train`, `mesh`, `products`.
    pub id: String,
    /// `pending`, `running`, `done`, `failed`, `skipped`.
    pub state: String,
    #[serde(default)]
    pub progress: f32,
    #[serde(default)]
    pub started_ns: Option<i64>,
    #[serde(default)]
    pub ended_ns: Option<i64>,
    /// Stage facts (tool versions used, counts).
    #[serde(default)]
    pub detail: serde_json::Value,
}

/// The node's GPU sample while the job runs.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JobGpu {
    pub name: String,
    #[serde(default)]
    pub util_pct: Option<f32>,
    #[serde(default)]
    pub vram_used_mb: Option<u64>,
}

/// `GET /api/compute/jobs/{id}`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct JobStatus {
    pub id: String,
    pub kind: ComputeJobKind,
    #[serde(default)]
    pub dataset_id: Option<String>,
    pub state: ComputeJobState,
    #[serde(default)]
    pub progress: f32,
    #[serde(default)]
    pub params: serde_json::Value,
    #[serde(default)]
    pub result_ref: Option<String>,
    #[serde(default)]
    pub created_ms: i64,
    #[serde(default)]
    pub updated_ms: i64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub session_id: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub preset: Option<String>,
    /// `queued`, the running stage id, `done`, `failed` or `cancelled`.
    #[serde(default)]
    pub stage: String,
    #[serde(default)]
    pub stage_progress: f32,
    #[serde(default)]
    pub stages: Vec<JobStage>,
    #[serde(default)]
    pub gpu: Option<JobGpu>,
    #[serde(default)]
    pub eta_s: Option<u64>,
    #[serde(default)]
    pub tools: BTreeMap<String, String>,
    #[serde(default, deserialize_with = "job_error")]
    pub error: Option<JobError>,
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A failed reconstruction as the node's `JobView` serializes it.
    const FAILED: &str = r#"{
        "id": "j-1", "kind": "reconstruct", "dataset_id": "visor-1-20261019T101500Z-3fa2",
        "state": "failed", "progress": 0.4,
        "params": {"session_id": "visor-1-20261019T101500Z-3fa2", "preset": "tour", "generation": 1},
        "result_ref": null, "created_ms": 1760868900000, "updated_ms": 1760868960000,
        "session_id": "visor-1-20261019T101500Z-3fa2", "preset": "tour",
        "stage": "failed", "stage_progress": 0.0,
        "stages": [
            {"id": "prepare", "state": "done", "progress": 1.0, "started_ns": 10, "ended_ns": 20, "detail": {"frames": 1200}},
            {"id": "sfm", "state": "failed", "progress": 0.3, "started_ns": 21, "ended_ns": null, "detail": null}
        ],
        "gpu": {"name": "RTX 4090", "util_pct": 87.0, "vram_used_mb": 18000},
        "eta_s": null,
        "tools": {"colmap": "4.2.1"},
        "error": {"code": "E_TOOL_MISSING", "message": "colmap is not installed", "tool": "colmap"}
    }"#;

    #[test]
    fn a_node_job_view_decodes_with_its_structured_error() {
        let s: JobStatus = serde_json::from_str(FAILED).unwrap();
        assert_eq!(s.state, ComputeJobState::Failed);
        assert_eq!(s.stage, "failed");
        assert_eq!(s.stages[1].id, "sfm");
        assert_eq!(s.gpu.as_ref().unwrap().vram_used_mb, Some(18000));
        assert_eq!(s.tools["colmap"], "4.2.1");
        let e = s.error.as_ref().unwrap();
        assert_eq!(
            (e.code.as_str(), e.message.as_str(), e.tool.as_deref()),
            ("E_TOOL_MISSING", "colmap is not installed", Some("colmap"))
        );
        // It serializes back to the same field names.
        let back = serde_json::to_value(&s).unwrap();
        assert_eq!(back["error"]["code"], "E_TOOL_MISSING");
        assert_eq!(back["stages"][0]["detail"]["frames"], 1200);
    }

    #[test]
    fn a_running_job_and_a_plain_string_error_decode() {
        let running = r#"{"id": "j-2", "kind": "reconstruct", "state": "running", "progress": 0.5,
            "params": null, "created_ms": 1, "updated_ms": 2, "stage": "train",
            "stage_progress": 0.42, "stages": [], "gpu": null, "eta_s": 120, "tools": {}, "error": null}"#;
        let s: JobStatus = serde_json::from_str(running).unwrap();
        assert_eq!(
            (s.stage.as_str(), s.eta_s, s.error),
            ("train", Some(120), None)
        );
        let legacy = r#"{"id": "j-3", "kind": "reconstruct", "state": "failed", "error": "boom"}"#;
        let s: JobStatus = serde_json::from_str(legacy).unwrap();
        assert_eq!(s.error.unwrap().message, "boom");
        let detail = r#"{"id": "j-4", "kind": "change", "state": "failed", "error": {"code": "E_X", "detail": "d"}}"#;
        assert_eq!(
            serde_json::from_str::<JobStatus>(detail)
                .unwrap()
                .error
                .unwrap()
                .message,
            "d"
        );
    }
}
