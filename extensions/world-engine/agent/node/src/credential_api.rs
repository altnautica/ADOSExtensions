//! Owner routes on the job API for the credentials this node issues:
//!
//! - `POST /api/compute/node-credentials` `{ peer_device_id, lanes? }` issues a
//!   credential for another node (every lane by default), replacing any earlier
//!   one for that peer, and returns the secret once.
//! - `GET /api/compute/node-credentials` lists what was issued (no secrets).
//! - `POST /api/compute/node-credentials/:id/revoke` revokes one.
//!
//! All three sit behind [`crate::auth::require_job_api`] with no node lane, so
//! only the operator (on the plugin's `http.sock`) reaches them: a node
//! credential can never issue another.

use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use axum::extract::Path;
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::{Extension, Json};
use serde::Deserialize;

use world_engine_protocol::node_credential::NodeLane;

use crate::auth::ComputeAuth;
use crate::node_credentials::CredentialError;

fn now_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0)
}

fn error(status: StatusCode, message: &str) -> Response {
    (status, Json(serde_json::json!({ "error": message }))).into_response()
}

#[derive(Debug, Deserialize)]
pub(crate) struct MintRequest {
    peer_device_id: String,
    #[serde(default)]
    lanes: Option<Vec<NodeLane>>,
}

pub(crate) async fn mint(
    Extension(auth): Extension<Arc<ComputeAuth>>,
    Json(req): Json<MintRequest>,
) -> Response {
    let lanes = req.lanes.unwrap_or_else(|| crate::lanes::ALL.to_vec());
    // The store fsyncs its file; keep that off the async workers.
    let minted = tokio::task::spawn_blocking(move || {
        auth.credentials.mint(&req.peer_device_id, &lanes, now_ms())
    })
    .await;
    match minted {
        Ok(Ok(m)) => (StatusCode::CREATED, Json(m)).into_response(),
        Ok(Err(e @ (CredentialError::BadPeer | CredentialError::NoLanes))) => {
            error(StatusCode::BAD_REQUEST, &e.to_string())
        }
        Ok(Err(e)) => {
            tracing::error!(error = %e, "node credential issue failed");
            error(StatusCode::INTERNAL_SERVER_ERROR, &e.to_string())
        }
        Err(e) => error(StatusCode::INTERNAL_SERVER_ERROR, &e.to_string()),
    }
}

pub(crate) async fn list(Extension(auth): Extension<Arc<ComputeAuth>>) -> Response {
    Json(serde_json::json!({
        "workstation_node_id": auth.credentials.node_id(),
        "credentials": auth.credentials.list(),
    }))
    .into_response()
}

pub(crate) async fn revoke(
    Extension(auth): Extension<Arc<ComputeAuth>>,
    Path(id): Path<String>,
) -> Response {
    match tokio::task::spawn_blocking(move || auth.credentials.revoke(&id)).await {
        Ok(Ok(revoked)) => Json(serde_json::json!({ "revoked": revoked })).into_response(),
        Ok(Err(e)) => {
            // Revoked in memory, but it would come back after a restart.
            tracing::error!(error = %e, "node credential revoke not persisted");
            error(StatusCode::INTERNAL_SERVER_ERROR, &e.to_string())
        }
        Err(e) => error(StatusCode::INTERNAL_SERVER_ERROR, &e.to_string()),
    }
}
