//! Reconstruction on the paired node, over the LAN:
//!
//! - `POST /atlas/sessions/{id}/reconstruct` `{"preset": "preview|hq|tour|warehouse"}`
//!   → `{"ok":true,"node":"..","job_id":".."}`. The job (`kind: reconstruct`,
//!   `session_id`, `preset`) is submitted to the node's `POST
//!   /api/compute/jobs` with this device's `jobs.submit` credential
//!   ([`NodeJobs`]). `409 E_NOT_UPLOADED` when the session is still on this
//!   device without a commit, or when the node has not committed it; the
//!   node's other refusals pass through with their code.

use ados_mapping_transport::jobs::NodeJobs;
use axum::extract::{Path, State};
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::routing::post;
use axum::Router;
use capture_service::storage::safe_session_id;
use serde::Deserialize;
use serde_json::Value;

use crate::api::DroneApi;
use crate::sessions::SessionDir;
use ados_mapping_transport::reply::{codes, respond, ApiError};

pub fn routes() -> Router<DroneApi> {
    Router::new().route("/atlas/sessions/:id/reconstruct", post(reconstruct))
}

impl DroneApi {
    /// Submit a reconstruction of an uploaded session. `job_id` (optional)
    /// makes the submit idempotent.
    pub async fn reconstruct(
        &self,
        session_id: &str,
        preset: &str,
        job_id: Option<String>,
    ) -> Result<Value, ApiError> {
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
        let jobs = NodeJobs {
            host: self.host.clone(),
            credentials: self.credentials.clone(),
        };
        jobs.reconstruct(session_id, preset, job_id).await
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
        Ok(b) => respond(api.reconstruct(&id, &b.preset, None).await),
        Err(e) => ApiError::bad_request(e.to_string()).into_response(),
    }
}

#[cfg(test)]
#[path = "jobs_api_tests.rs"]
mod tests;
