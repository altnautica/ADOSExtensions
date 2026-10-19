//! Reconstruction on the paired node, over the LAN:
//!
//! - `POST /atlas/sessions/{id}/reconstruct` `{"preset": "preview|hq|tour|warehouse"}`
//!   → `{"ok":true,"node":"..","job_id":".."}`. The job (`kind: reconstruct`,
//!   `session_id`, `preset`) is submitted to the node's `POST
//!   /api/compute/jobs` with this device's `jobs.submit` credential. `409
//!   E_NOT_UPLOADED` when the session is still on this device without a
//!   commit, or when the node has not committed it; the node's other
//!   refusals pass through with their code.
//! - [`DroneApi::job_status`] / [`DroneApi::tour_bundle`] (MCP tools): the
//!   node's job view and the job's `tour` artifact.

use axum::extract::{Path, State};
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::routing::post;
use axum::Router;
use serde::Deserialize;
use serde_json::{json, Value};
use world_engine_capture::storage::safe_session_id;
use world_engine_protocol::compute::ComputeJobKind;
use world_engine_transport::ComputeClient;

use crate::api::DroneApi;
use crate::credentials::JOB_SUBMIT_LANE;
use crate::lan_node::{node_label, resolve_lan};
use crate::reply::{codes, respond, ApiError};
use crate::sessions::SessionDir;

/// The reconstruction presets the node knows.
pub const PRESETS: [&str; 4] = ["preview", "hq", "tour", "warehouse"];

pub fn routes() -> Router<DroneApi> {
    Router::new().route("/atlas/sessions/:id/reconstruct", post(reconstruct))
}

impl DroneApi {
    /// A client of the paired node's job API, and the node's label.
    async fn job_client(&self) -> Result<(ComputeClient, String), ApiError> {
        let node = resolve_lan(self.host.as_ref(), &self.credentials, &JOB_SUBMIT_LANE).await?;
        let credential = node.credential(&self.credentials, &JOB_SUBMIT_LANE);
        Ok((
            ComputeClient::new(node.base_url(), credential),
            node_label(&node),
        ))
    }

    /// Submit a reconstruction of an uploaded session.
    pub async fn reconstruct(&self, session_id: &str, preset: &str) -> Result<Value, ApiError> {
        if !PRESETS.contains(&preset) {
            return Err(ApiError::bad_request(format!(
                "unknown preset `{preset}` (expected one of {})",
                PRESETS.join(" / ")
            )));
        }
        if !safe_session_id(session_id) {
            return Err(ApiError::bad_request(format!(
                "invalid session id `{session_id}`"
            )));
        }
        let local = self
            .uploads
            .root()
            .and_then(|root| SessionDir::open(&root, session_id));
        if local.is_some_and(|d| !d.uploaded) {
            return Err(ApiError::new(
                StatusCode::CONFLICT,
                codes::NOT_UPLOADED,
                "the session has not been uploaded to a node",
            ));
        }
        let (client, node) = self.job_client().await?;
        let submitted = client
            .submit_job(
                ComputeJobKind::Reconstruct,
                None,
                json!({ "session_id": session_id, "preset": preset }),
                None,
            )
            .await?;
        Ok(json!({ "ok": true, "node": node, "job_id": submitted.job_id }))
    }

    /// The node's view of a job (stage, progress, error, outputs-to-come).
    pub async fn job_status(&self, job_id: &str) -> Result<Value, ApiError> {
        let (client, node) = self.job_client().await?;
        let mut view = client.job_view(job_id).await?;
        if let Some(obj) = view.as_object_mut() {
            obj.insert("node".to_string(), Value::String(node));
        }
        Ok(view)
    }

    /// The job's walkthrough tour bundle: where to fetch it.
    pub async fn tour_bundle(&self, job_id: &str) -> Result<Value, ApiError> {
        let (client, node) = self.job_client().await?;
        let outputs = client.job_outputs(job_id).await?;
        let tour =
            outputs
                .into_iter()
                .find(|o| o.kind == "tour")
                .ok_or_else(|| {
                    ApiError::new(
                StatusCode::NOT_FOUND,
                codes::NO_TOUR,
                format!("job `{job_id}` has no tour bundle (not finished, or not a tour preset)"),
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

#[derive(Debug, Deserialize)]
struct ReconstructBody {
    preset: String,
}

async fn reconstruct(
    State(api): State<DroneApi>,
    Path(id): Path<String>,
    body: axum::body::Bytes,
) -> Response {
    match serde_json::from_slice::<ReconstructBody>(&body) {
        Ok(b) => respond(api.reconstruct(&id, &b.preset).await),
        Err(e) => ApiError::bad_request(e.to_string()).into_response(),
    }
}

#[cfg(test)]
#[path = "jobs_api_tests.rs"]
mod tests;
