//! HTTP surface of the upload lane: manifest post, ranged file upload, resume
//! probe and session status. The commit handler lives in `commit.rs`.

use std::collections::BTreeMap;
use std::io;
use std::path::PathBuf;
use std::sync::Arc;

use ados_capture_contract::Manifest;
use axum::body::{Body, Bytes};
use axum::extract::{DefaultBodyLimit, Path, State};
use axum::http::{header, HeaderMap, HeaderName, HeaderValue, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post, put};
use axum::{Json, Router};
use serde_json::{json, Value};

use super::disk::{self, FileState, StreamFail};
use super::paths;
use super::{commit, CommitHook, SessionStore, MANIFEST_FILE};

/// Largest request body one upload request may carry.
pub(crate) const MAX_REQUEST_BYTES: u64 = 1 << 30;

const UPLOAD_OFFSET: HeaderName = HeaderName::from_static("upload-offset");
const UPLOAD_LENGTH: HeaderName = HeaderName::from_static("upload-length");

#[derive(Clone)]
pub(crate) struct Ctx {
    pub store: Arc<SessionStore>,
    pub hook: CommitHook,
}

/// The session upload routes under `/api/compute/sessions/:id`.
pub fn bulk_router(store: Arc<SessionStore>, on_commit: CommitHook) -> Router {
    Router::new()
        .route(
            "/api/compute/sessions/:id",
            get(get_session).post(post_manifest),
        )
        .route("/api/compute/sessions/:id/commit", post(commit::commit))
        .route(
            "/api/compute/sessions/:id/files/*rel",
            put(put_file)
                .head(head_file)
                .layer(DefaultBodyLimit::max(MAX_REQUEST_BYTES as usize)),
        )
        .with_state(Ctx {
            store,
            hook: on_commit,
        })
}

/// A JSON error reply.
#[derive(Debug)]
pub(crate) struct ApiError {
    status: StatusCode,
    body: Value,
}

impl ApiError {
    pub(crate) fn new(status: StatusCode, code: &str) -> Self {
        Self::with(status, json!({ "error": code }))
    }

    pub(crate) fn with(status: StatusCode, body: Value) -> Self {
        Self { status, body }
    }
}

impl From<io::Error> for ApiError {
    fn from(e: io::Error) -> Self {
        tracing::warn!(error = %e, "session upload I/O error");
        Self::with(
            StatusCode::INTERNAL_SERVER_ERROR,
            json!({ "error": "E_IO", "message": e.to_string() }),
        )
    }
}

impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        (self.status, Json(self.body)).into_response()
    }
}

pub(crate) type ApiResult<T> = Result<T, ApiError>;

pub(crate) fn check_id(id: &str) -> ApiResult<()> {
    if paths::valid_session_id(id) {
        Ok(())
    } else {
        Err(ApiError::new(StatusCode::BAD_REQUEST, "E_BAD_ID"))
    }
}

fn check_rel(rel: &str) -> ApiResult<()> {
    if paths::valid_rel(rel) {
        Ok(())
    } else {
        Err(ApiError::new(StatusCode::BAD_REQUEST, "E_BAD_PATH"))
    }
}

/// The directory of a posted session; 404 before the manifest arrives.
pub(crate) fn require_session(store: &SessionStore, id: &str) -> ApiResult<PathBuf> {
    let dir = store.session_dir(id);
    if dir.join(MANIFEST_FILE).is_file() {
        Ok(dir)
    } else {
        Err(ApiError::new(StatusCode::NOT_FOUND, "E_UNKNOWN_SESSION"))
    }
}

fn already_committed() -> ApiError {
    ApiError::new(StatusCode::CONFLICT, "E_ALREADY_COMMITTED")
}

fn offset_mismatch(received: u64) -> ApiError {
    ApiError::with(
        StatusCode::CONFLICT,
        json!({ "error": "E_OFFSET_MISMATCH", "received": received }),
    )
}

fn storage_full(headroom: u64) -> ApiError {
    ApiError::with(
        StatusCode::INSUFFICIENT_STORAGE,
        json!({ "error": "E_STORAGE_FULL", "free_bytes": headroom }),
    )
}

/// Run blocking filesystem work off the async workers.
pub(crate) async fn blocking<T, F>(f: F) -> ApiResult<T>
where
    T: Send + 'static,
    F: FnOnce() -> io::Result<T> + Send + 'static,
{
    match tokio::task::spawn_blocking(f).await {
        Ok(result) => result.map_err(ApiError::from),
        Err(e) => Err(ApiError::from(io::Error::other(e.to_string()))),
    }
}

async fn session_files(dir: PathBuf) -> ApiResult<BTreeMap<String, FileState>> {
    blocking(move || disk::list_files(&dir)).await
}

/// `POST /api/compute/sessions/:id`: create or resume a session from its
/// manifest. The body is stored verbatim (`checksums.b3` covers it).
async fn post_manifest(
    State(ctx): State<Ctx>,
    Path(id): Path<String>,
    body: Bytes,
) -> ApiResult<(StatusCode, Json<Value>)> {
    check_id(&id)?;
    // `contract` deserializes into a one-value enum, so a parsed manifest is
    // an `ados.capture/1` manifest.
    let manifest: Manifest = serde_json::from_slice(&body).map_err(|e| {
        ApiError::with(
            StatusCode::BAD_REQUEST,
            json!({ "error": "E_BAD_MANIFEST", "message": e.to_string() }),
        )
    })?;
    if manifest.session_id != id {
        return Err(ApiError::new(StatusCode::BAD_REQUEST, "E_SESSION_MISMATCH"));
    }

    let lock = ctx.store.lock(&id);
    let _guard = lock.lock().await;
    let dir = ctx.store.session_dir(&id);
    if ctx.store.case_variant_of(&id).is_some() {
        return Err(ApiError::new(StatusCode::CONFLICT, "E_SESSION_EXISTS"));
    }
    if disk::is_committed(&dir) {
        return Err(already_committed());
    }
    let status = match tokio::fs::read(dir.join(MANIFEST_FILE)).await {
        Ok(existing) if existing == body => StatusCode::OK,
        Ok(_) => {
            disk::write_atomic(&dir, MANIFEST_FILE, &body).await?;
            StatusCode::OK
        }
        Err(e) if e.kind() == io::ErrorKind::NotFound => {
            tokio::fs::create_dir_all(&dir).await?;
            disk::write_atomic(&dir, MANIFEST_FILE, &body).await?;
            StatusCode::CREATED
        }
        Err(e) => return Err(e.into()),
    };
    let files = session_files(dir).await?;
    Ok((
        status,
        Json(json!({ "ok": true, "session_id": id, "files": files })),
    ))
}

/// `GET /api/compute/sessions/:id`: commit state and per-file progress.
async fn get_session(State(ctx): State<Ctx>, Path(id): Path<String>) -> ApiResult<Json<Value>> {
    check_id(&id)?;
    let dir = require_session(&ctx.store, &id)?;
    let committed = disk::is_committed(&dir);
    let files = session_files(dir).await?;
    Ok(Json(
        json!({ "session_id": id, "committed": committed, "files": files }),
    ))
}

/// `PUT /api/compute/sessions/:id/files/*rel`: append one chunk. Without a
/// `Content-Range` the body is the whole file.
async fn put_file(
    State(ctx): State<Ctx>,
    Path((id, rel)): Path<(String, String)>,
    headers: HeaderMap,
    body: Body,
) -> ApiResult<Json<Value>> {
    check_id(&id)?;
    check_rel(&rel)?;
    let range = match headers.get(header::CONTENT_RANGE) {
        None => None,
        Some(v) => Some(
            v.to_str()
                .ok()
                .and_then(paths::parse_content_range)
                .ok_or_else(|| ApiError::new(StatusCode::BAD_REQUEST, "E_BAD_RANGE"))?,
        ),
    };

    let lock = ctx.store.lock(&id);
    let _guard = lock.lock().await;
    let dir = require_session(&ctx.store, &id)?;
    if disk::is_committed(&dir) {
        return Err(already_committed());
    }
    if disk::blocked_by_existing(&dir, &rel) {
        return Err(ApiError::new(StatusCode::BAD_REQUEST, "E_BAD_PATH"));
    }
    let received = match disk::file_state(&dir, &rel)? {
        Some(state) if state.complete => return Err(offset_mismatch(state.received)),
        Some(state) => state.received,
        None => 0,
    };
    let start = range.map_or(0, |r| r.start);
    if start != received {
        return Err(offset_mismatch(received));
    }
    let recorded = disk::read_total(&dir, &rel)?;
    let (expected, total) = match range {
        Some(r) => {
            if let (Some(a), Some(b)) = (recorded, r.total) {
                if a != b {
                    return Err(ApiError::new(StatusCode::BAD_REQUEST, "E_BAD_RANGE"));
                }
            }
            let total = r.total.or(recorded);
            if total.is_some_and(|t| r.end >= t) {
                return Err(ApiError::new(StatusCode::BAD_REQUEST, "E_BAD_RANGE"));
            }
            (Some(r.len()), total)
        }
        None => (None, None),
    };
    if expected.is_some_and(|n| n > MAX_REQUEST_BYTES) {
        return Err(ApiError::new(StatusCode::PAYLOAD_TOO_LARGE, "E_TOO_LARGE"));
    }
    // The declared size, else the transport length; either way the stream is
    // capped at the headroom so an undeclared body cannot overrun it.
    let declared = expected.or_else(|| {
        headers
            .get(header::CONTENT_LENGTH)
            .and_then(|v| v.to_str().ok())
            .and_then(|v| v.parse::<u64>().ok())
    });
    let headroom = {
        let store = ctx.store.clone();
        blocking(move || store.headroom()).await?
    };
    if declared.is_some_and(|n| n > headroom) {
        return Err(storage_full(headroom));
    }
    let cap = MAX_REQUEST_BYTES.min(headroom);

    let part = disk::part_path(&dir, &rel);
    if let Some(parent) = part.parent() {
        tokio::fs::create_dir_all(parent).await?;
    }
    let written = disk::stream_to_part(&part, start, body, expected, cap)
        .await
        .map_err(|fail| match fail {
            StreamFail::Length => ApiError::new(StatusCode::BAD_REQUEST, "E_LENGTH_MISMATCH"),
            StreamFail::TooLarge if cap < MAX_REQUEST_BYTES => storage_full(headroom),
            StreamFail::TooLarge => ApiError::new(StatusCode::PAYLOAD_TOO_LARGE, "E_TOO_LARGE"),
            StreamFail::Body(message) => ApiError::with(
                StatusCode::BAD_REQUEST,
                json!({ "error": "E_BODY", "message": message }),
            ),
            StreamFail::Io(e) => ApiError::from(e),
        })?;

    let received = start + written;
    // A whole-file upload declares its total by its length.
    let total = if range.is_none() {
        Some(received)
    } else {
        total
    };
    let complete = total == Some(received);
    let total_file = disk::total_path(&dir, &rel);
    if complete {
        let dest = dir.join(&rel);
        if let Some(parent) = dest.parent() {
            tokio::fs::create_dir_all(parent).await?;
        }
        tokio::fs::rename(&part, &dest).await?;
        disk::remove_if_exists(&total_file).await?;
    } else if let Some(t) = total {
        if recorded != Some(t) {
            tokio::fs::write(&total_file, t.to_string()).await?;
        }
    }
    Ok(Json(
        json!({ "received": received, "total": total, "complete": complete }),
    ))
}

/// `HEAD /api/compute/sessions/:id/files/*rel`: the resume offset.
async fn head_file(
    State(ctx): State<Ctx>,
    Path((id, rel)): Path<(String, String)>,
) -> ApiResult<Response> {
    check_id(&id)?;
    check_rel(&rel)?;
    let dir = require_session(&ctx.store, &id)?;
    let Some(state) = disk::file_state(&dir, &rel)? else {
        return Ok(StatusCode::NOT_FOUND.into_response());
    };
    let mut headers = HeaderMap::new();
    headers.insert(UPLOAD_OFFSET, HeaderValue::from(state.received));
    if let Some(total) = state.total {
        headers.insert(UPLOAD_LENGTH, HeaderValue::from(total));
    }
    headers.insert(header::CONTENT_LENGTH, HeaderValue::from(state.received));
    Ok((StatusCode::OK, headers).into_response())
}
