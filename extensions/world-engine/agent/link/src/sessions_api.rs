//! The recorded sessions on this device:
//!
//! - `GET /atlas/sessions` → `{"sessions":[row...]}` (rows per
//!   [`crate::sessions`], newest first); `[]` while the capture root is
//!   unknown.
//! - `POST /atlas/sessions/{id}/upload` → `{"ok":true}`: queue it for the
//!   paired node (`409 E_RECORDING` while it records).
//! - `DELETE /atlas/sessions/{id}[?force=true]` → `{"ok":true}`: refused with
//!   `409 E_RECORDING` while it records, `409 E_UPLOADING` while queued or
//!   uploading, and `409 E_NOT_UPLOADED` before the node committed it unless
//!   `force=true`.

use axum::extract::{Path, Query, State};
use axum::http::StatusCode;
use axum::response::Response;
use axum::routing::{delete, get, post};
use axum::Router;
use serde::Deserialize;
use serde_json::{json, Value};

use crate::api::DroneApi;
use crate::reply::{codes, respond, ApiError};
use crate::sessions::{row, scan, SessionDir, SessionRow};
use crate::upload::{no_storage, unknown_session};

pub fn routes() -> Router<DroneApi> {
    Router::new()
        .route("/atlas/sessions", get(list))
        .route("/atlas/sessions/:id", delete(remove))
        .route("/atlas/sessions/:id/upload", post(upload))
}

impl DroneApi {
    /// Every session row, newest first.
    pub async fn sessions_list(&self) -> Result<Value, ApiError> {
        let Some(root) = self.uploads.root() else {
            return Ok(json!({ "sessions": [] }));
        };
        let uploads = self.uploads.clone();
        let rows = tokio::task::spawn_blocking(move || {
            let mut rows: Vec<SessionRow> = scan(&root)
                .iter()
                .map(|dir| {
                    let size = crate::sessions::dir_size(&dir.path);
                    row(dir, uploads.view(dir, size))
                })
                .collect();
            rows.sort_by(|a, b| {
                (&b.started_at_utc, &b.session_id).cmp(&(&a.started_at_utc, &a.session_id))
            });
            rows
        })
        .await
        .map_err(ApiError::io)?;
        Ok(json!({ "sessions": rows }))
    }

    /// Queue a session for upload.
    pub fn session_upload(&self, id: &str) -> Result<Value, ApiError> {
        self.uploads.request(id)?;
        Ok(json!({ "ok": true }))
    }

    /// Delete a session directory.
    pub async fn session_delete(&self, id: &str, force: bool) -> Result<Value, ApiError> {
        let root = self.uploads.root().ok_or_else(no_storage)?;
        let dir = SessionDir::open(&root, id).ok_or_else(|| unknown_session(id))?;
        let conflict =
            |code: &str, detail: &str| Err(ApiError::new(StatusCode::CONFLICT, code, detail));
        if dir.recording {
            return conflict(codes::RECORDING, "the session is still recording");
        }
        if self.uploads.busy(id) {
            return conflict(codes::UPLOADING, "the session is being uploaded");
        }
        if !dir.uploaded && !force {
            return conflict(
                codes::NOT_UPLOADED,
                "the session is not committed on a node; pass force=true to delete it anyway",
            );
        }
        let path = dir.path.clone();
        tokio::task::spawn_blocking(move || std::fs::remove_dir_all(path))
            .await
            .map_err(ApiError::io)?
            .map_err(ApiError::io)?;
        self.uploads.forget(id);
        tracing::info!(session = id, force, "session deleted");
        Ok(json!({ "ok": true }))
    }
}

async fn list(State(api): State<DroneApi>) -> Response {
    respond(api.sessions_list().await)
}

async fn upload(State(api): State<DroneApi>, Path(id): Path<String>) -> Response {
    respond(api.session_upload(&id))
}

#[derive(Debug, Default, Deserialize)]
struct DeleteQuery {
    #[serde(default)]
    force: Option<String>,
}

async fn remove(
    State(api): State<DroneApi>,
    Path(id): Path<String>,
    Query(q): Query<DeleteQuery>,
) -> Response {
    let force = matches!(q.force.as_deref(), Some("true" | "1"));
    respond(api.session_delete(&id, force).await)
}

#[cfg(test)]
#[path = "sessions_api_tests.rs"]
mod tests;
