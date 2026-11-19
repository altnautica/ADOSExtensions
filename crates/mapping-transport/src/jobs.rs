//! Reconstruction jobs on the paired node, over the LAN: submit one for an
//! uploaded session, read a job's view, and find a job's tour bundle. The job
//! client is pinned to the LAN address [`resolve_lan`] checked and follows no
//! redirects; it authenticates with this device's `jobs.submit` credential.

use std::sync::Arc;

use ados_mapping_protocol::compute::{ComputeJobKind, ComputeJobState};
use axum::http::StatusCode;
use serde_json::{json, Value};

use crate::credentials::{CredentialStore, JOB_SUBMIT_LANE};
use crate::host::Host;
use crate::lan_node::resolve_lan;
use crate::reply::{codes, ApiError};
use crate::ComputeClient;

/// The reconstruction presets the node knows.
pub const PRESETS: [&str; 4] = ["preview", "hq", "tour", "warehouse"];

/// The paired node's job API, as this device reaches it.
#[derive(Clone)]
pub struct NodeJobs {
    pub host: Arc<dyn Host>,
    pub credentials: CredentialStore,
}

impl NodeJobs {
    /// A client of the paired node's job API, and the node's label.
    async fn client(&self) -> Result<(ComputeClient, String), ApiError> {
        let node = resolve_lan(self.host.as_ref(), &self.credentials, &JOB_SUBMIT_LANE).await?;
        let credential = node.credential(&self.credentials, &JOB_SUBMIT_LANE).ok();
        let label = node.label();
        Ok((
            ComputeClient::pinned(node.base_url(), credential, Some(node.pin)),
            label,
        ))
    }

    /// Submit a reconstruction of a session the node holds. `job_id`
    /// (optional) makes the submit idempotent: a tool that stops waiting
    /// already holds the id the job will have. The node refuses a session it
    /// has not committed (`E_NOT_UPLOADED`).
    pub async fn reconstruct(
        &self,
        session_id: &str,
        preset: &str,
        job_id: Option<String>,
    ) -> Result<Value, ApiError> {
        if !PRESETS.contains(&preset) {
            return Err(ApiError::bad_request(format!(
                "unknown preset `{preset}` (expected one of {})",
                PRESETS.join(" / ")
            )));
        }
        let (client, node) = self.client().await?;
        let submitted = client
            .submit_job(
                ComputeJobKind::Reconstruct,
                None,
                json!({ "session_id": session_id, "preset": preset }),
                job_id,
            )
            .await?;
        Ok(json!({ "ok": true, "node": node, "job_id": submitted.job_id }))
    }

    /// The node's view of a job (stage, progress, structured error) plus the
    /// node's label.
    pub async fn job_status(&self, job_id: &str) -> Result<Value, ApiError> {
        let (client, node) = self.client().await?;
        let status = client.job_status(job_id).await?;
        let mut view = serde_json::to_value(status).map_err(ApiError::io)?;
        view["node"] = Value::String(node);
        Ok(view)
    }

    /// The job's walkthrough tour bundle: where to fetch it. A job that
    /// failed answers with its own error code; one still running with
    /// `E_NO_TOUR` naming its stage.
    pub async fn tour_bundle(&self, job_id: &str) -> Result<Value, ApiError> {
        let (client, node) = self.client().await?;
        let status = client.job_status(job_id).await?;
        if let Some(e) = status.error {
            return Err(ApiError::new(StatusCode::CONFLICT, &e.code, e.message));
        }
        if status.state != ComputeJobState::Completed {
            return Err(ApiError::new(
                StatusCode::CONFLICT,
                codes::NO_TOUR,
                format!("job `{job_id}` is not finished (stage `{}`)", status.stage),
            ));
        }
        let outputs = client.job_outputs(job_id).await?;
        let tour = outputs
            .into_iter()
            .find(|o| o.kind == "tour")
            .ok_or_else(|| {
                ApiError::new(
                    StatusCode::NOT_FOUND,
                    codes::NO_TOUR,
                    format!("job `{job_id}` produced no tour bundle (not a tour preset)"),
                )
            })?;
        Ok(json!({
            "job_id": job_id,
            "node": node,
            "uri": tour.uri,
            "meta": tour.meta,
        }))
    }
}

#[cfg(test)]
#[path = "jobs_tests.rs"]
mod tests;
