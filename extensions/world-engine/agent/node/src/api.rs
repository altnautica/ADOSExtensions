//! The compute node's REST job API (native Rust, axum over tokio). A drone or
//! GCS submits reconstruction and offload jobs here and reads their status and
//! results. Handlers lock the engine (a single-writer SQLite store) briefly per
//! request. Gated by [`crate::auth::require_job_api`]: the owner (on the
//! plugin's `http.sock`, or the standalone node's bearer) reaches every route;
//! over TCP a node credential reaches only the calls another node makes, under
//! a rate limiter.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};

use axum::extract::{Path, State};
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post};
use axum::{Extension, Json, Router};
use serde::{Deserialize, Serialize};
use tokio::sync::Mutex;
use world_engine_protocol::compute::{
    CancelResponse, NodeIdentity, SubmitResponse, NODE_IDENTITY_PATH,
};

use crate::artifacts::rewrite_artifact_host;
use crate::auth::{require_job_api, ComputeAuth, NodeCaller};
use crate::credential_api;
use crate::job_views::{JobView, OutputView};
use crate::recon::{self, StageError};
use crate::session_registry::{SessionRegistry, SessionStateCounts};
use crate::{
    ComputeError, ComputeHeartbeat, ComputeJobKind, ComputeJobState, Dataset, Engine, JobRecord,
};

/// Shared engine handle. One mutex serializes access to the single-writer store,
/// shared by the API handlers and the worker loop.
pub type ApiState = Arc<Mutex<Engine>>;

/// Build the job-API router over a shared engine, gated by
/// [`require_job_api`].
///
/// Build the router with a default public base (loopback) and an empty session
/// registry (`/api/compute/sessions` reads `[]`). The daemon uses
/// [`build_router_with_base`] with its live base + the live registry; this
/// shorthand is for callers (tests) where
/// artifact-host rewriting is a no-op and no session is live.
pub fn build_router(state: ApiState, auth: Arc<ComputeAuth>) -> Router {
    build_router_with_base(
        state,
        auth,
        Arc::from("http://127.0.0.1:8092"),
        Arc::new(SessionRegistry::new()),
    )
}

pub fn build_router_with_base(
    state: ApiState,
    auth: Arc<ComputeAuth>,
    public_base: Arc<str>,
    sessions: Arc<SessionRegistry>,
) -> Router {
    Router::new()
        .route("/api/compute/status", get(status))
        // Which node this is: served to any caller (see `identity`).
        .route(NODE_IDENTITY_PATH, get(identity))
        .route("/api/compute/datasets", post(create_dataset))
        .route("/api/compute/jobs", get(list_jobs).post(submit_job))
        .route("/api/compute/jobs/:id", get(job_status))
        .route("/api/compute/jobs/:id/cancel", post(cancel_job))
        .route("/api/compute/jobs/:id/outputs", get(job_outputs))
        // The live streaming perception-offload sessions (state / throughput /
        // reconnect + restart history), read from the registry, not the job store.
        .route("/api/compute/sessions", get(list_sessions))
        // Issuing credentials to other nodes: owner-only (no node lane reaches
        // them).
        .route(
            "/api/compute/node-credentials",
            get(credential_api::list).post(credential_api::mint),
        )
        .route(
            "/api/compute/node-credentials/:id/revoke",
            post(credential_api::revoke),
        )
        .layer(axum::middleware::from_fn_with_state(
            auth.clone(),
            require_job_api,
        ))
        // The credential handlers read the issued store.
        .layer(Extension(auth))
        // The live public base rewrites each stored artifact URL's host on read,
        // so a URL frozen at an earlier (drifting) hostname stays reachable.
        .layer(Extension(public_base))
        // The session registry the status + sessions handlers read.
        .layer(Extension(sessions))
        .with_state(state)
}

fn now_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0)
}

fn next_id(prefix: &str) -> String {
    static COUNTER: AtomicU64 = AtomicU64::new(0);
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos())
        .unwrap_or(0);
    let n = COUNTER.fetch_add(1, Ordering::Relaxed);
    format!("{prefix}-{nanos}-{n}")
}

// A ComputeError renders as a JSON error with a fitting status. NotFound is a
// 404, a wrong kind is a 400, everything else is a 500.
impl IntoResponse for ComputeError {
    fn into_response(self) -> Response {
        let status = match self {
            ComputeError::NotFound(_) => StatusCode::NOT_FOUND,
            ComputeError::Conflict(_) => StatusCode::CONFLICT,
            ComputeError::WrongKind(_) => StatusCode::BAD_REQUEST,
            _ => StatusCode::INTERNAL_SERVER_ERROR,
        };
        let body = Json(serde_json::json!({ "error": self.to_string() }));
        (status, body).into_response()
    }
}

/// A refused submission: `{error: <code>, detail, tool?}`.
fn refuse(status: StatusCode, e: &StageError) -> Response {
    let mut body = serde_json::json!({ "error": e.code, "detail": e.message });
    if let Some(t) = &e.tool {
        body["tool"] = t.clone().into();
    }
    (status, Json(body)).into_response()
}

#[derive(Debug, Deserialize)]
struct CreateDatasetRequest {
    id: Option<String>,
    kind: String,
    meta: Option<serde_json::Value>,
}

/// A job submission. The staged kinds may carry their inputs at the top level
/// (`{"kind":"reconstruct","session_id":..,"preset":..}`,
/// `{"kind":"change","a":..,"b":..}`,
/// `{"kind":"calibrate","session_id":..,"target":..}`); they are folded into
/// `params`.
#[derive(Debug, Deserialize)]
struct SubmitRequest {
    job_id: Option<String>,
    kind: ComputeJobKind,
    dataset_id: Option<String>,
    params: Option<serde_json::Value>,
    #[serde(flatten)]
    extra: serde_json::Map<String, serde_json::Value>,
}

/// The top-level fields a staged submission may carry.
const STAGED_FIELDS: [&str; 11] = [
    "session_id",
    "preset",
    "a",
    "b",
    "target",
    "board",
    "chunk_frames",
    "glim_odometry",
    "products",
    "appearance",
    "mesh_method",
];

/// The node status: the engine heartbeat (role / cluster / queue / active
/// sessions count) plus a per-state breakdown of the live streaming sessions.
/// `session_states` is additive — an older reader deserializing into
/// `ComputeHeartbeat` ignores it (`active_sessions` is the stable count).
#[derive(Debug, Serialize)]
struct StatusResponse {
    #[serde(flatten)]
    heartbeat: ComputeHeartbeat,
    session_states: SessionStateCounts,
}

async fn status(
    State(state): State<ApiState>,
    Extension(sessions): Extension<Arc<SessionRegistry>>,
) -> Result<Response, ComputeError> {
    let heartbeat = {
        let engine = state.lock().await;
        engine.heartbeat()?
    };
    let resp = StatusResponse {
        heartbeat,
        session_states: sessions.state_counts(),
    };
    Ok(Json(resp).into_response())
}

/// This node's id, the one it advertises over mDNS and names in the
/// credentials it issues. Served without a credential, so a node that reaches
/// this one by a pinned address can present only the credential this node
/// issued it.
async fn identity(Extension(auth): Extension<Arc<ComputeAuth>>) -> Json<NodeIdentity> {
    Json(NodeIdentity {
        node_id: auth.credentials.node_id().to_string(),
    })
}

/// The live streaming perception-offload sessions: state, throughput, and the
/// reconnect and restart history. Read straight from the registry, so it reflects
/// the true live state of each session, not a stale job row.
async fn list_sessions(
    Extension(sessions): Extension<Arc<SessionRegistry>>,
) -> Result<Response, ComputeError> {
    Ok(Json(sessions.snapshot(now_ms())).into_response())
}

async fn create_dataset(
    State(state): State<ApiState>,
    Json(req): Json<CreateDatasetRequest>,
) -> Result<Response, ComputeError> {
    let dataset = Dataset {
        id: req.id.unwrap_or_else(|| next_id("ds")),
        kind: req.kind,
        created_ms: now_ms(),
        meta: req.meta.unwrap_or(serde_json::Value::Null),
    };
    let engine = state.lock().await;
    engine.scheduler().store().insert_dataset(&dataset)?;
    Ok((StatusCode::CREATED, Json(dataset)).into_response())
}

async fn submit_job(
    State(state): State<ApiState>,
    caller: Option<Extension<NodeCaller>>,
    Json(req): Json<SubmitRequest>,
) -> Result<Response, ComputeError> {
    let now = now_ms();
    let mut params = req.params.unwrap_or(serde_json::Value::Null);
    for key in STAGED_FIELDS {
        if let Some(v) = req.extra.get(key) {
            if !params.is_object() {
                params = serde_json::json!({});
            }
            params[key] = v.clone();
        }
    }
    // Who submitted it: a node's credential id scopes its later reads; the
    // caller never chooses this field.
    if let Some(map) = params.as_object_mut() {
        map.remove(SUBMITTED_BY);
    }
    if let Some(Extension(NodeCaller(cred))) = caller {
        if !params.is_object() {
            params = serde_json::json!({});
        }
        params[SUBMITTED_BY] = cred.into();
    }
    let id = req.job_id.unwrap_or_else(|| next_id("job"));
    // The job id names the job's working tree, so it must be one plain path
    // component.
    if !recon::valid_job_id(&id) {
        return Ok(refuse(
            StatusCode::BAD_REQUEST,
            &StageError::input(format!("invalid job id {id:?}")),
        ));
    }
    let mut job = JobRecord {
        id,
        kind: req.kind,
        dataset_id: req.dataset_id,
        state: ComputeJobState::Queued,
        progress: 0.0,
        params,
        result_ref: None,
        error: None,
        created_ms: now,
        updated_ms: now,
    };
    let engine = state.lock().await;
    // A staged job is checked up front so the caller hears why it cannot run
    // (not uploaded, unknown preset, a pinned tool missing) instead of a job
    // that fails at once.
    if recon::is_staged(&job) {
        let scheduler = engine.scheduler();
        let staged = match recon::prepare_staged(scheduler.store(), scheduler.sessions_root(), &job)
        {
            Ok(s) => s,
            Err(e) => {
                let status = match e.code.as_str() {
                    "E_NOT_UPLOADED" | "E_NOT_RECONSTRUCTED" => StatusCode::CONFLICT,
                    _ => StatusCode::BAD_REQUEST,
                };
                return Ok(refuse(status, &e));
            }
        };
        let runner = engine.runner();
        if let Err(e) = recon::check_staged_tools(runner.as_ref(), &job, &staged) {
            return Ok(refuse(StatusCode::UNPROCESSABLE_ENTITY, &e));
        }
        // The placement generation is fixed at submit, so a resumed job keeps
        // the generation its first run stamped on its sidecars.
        if let recon::StagedJob::Reconstruct { generation, .. } = staged {
            job.params["generation"] = generation.into();
        }
        if let Some(sid) = job.params.get("session_id").and_then(|v| v.as_str()) {
            job.dataset_id.get_or_insert_with(|| sid.to_string());
        }
    }
    engine.scheduler().store().submit_job(&job)?;
    Ok((
        StatusCode::CREATED,
        Json(SubmitResponse {
            job_id: job.id,
            state: job.state,
        }),
    )
        .into_response())
}

/// Rewrite a job record's `result_ref` artifact host to the live public base.
fn rehost_job(mut job: JobRecord, public_base: &str) -> JobRecord {
    if let Some(r) = job.result_ref.take() {
        job.result_ref = Some(rewrite_artifact_host(&r, public_base));
    }
    job
}

/// The current GPU sample, read off the async runtime.
async fn gpu_sample() -> Option<crate::worker::NvidiaGpu> {
    tokio::task::spawn_blocking(crate::worker::nvidia_gpu)
        .await
        .ok()
        .flatten()
}

async fn list_jobs(
    State(state): State<ApiState>,
    Extension(public_base): Extension<Arc<str>>,
) -> Result<Response, ComputeError> {
    let gpu = gpu_sample().await;
    let engine = state.lock().await;
    let store = engine.scheduler().store();
    let views = store
        .list_jobs()?
        .into_iter()
        .map(|job| JobView::build(store, rehost_job(job, &public_base), gpu.as_ref()))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(Json(views).into_response())
}

/// The job param naming the node credential that submitted a job.
const SUBMITTED_BY: &str = "submitted_by";

/// Whether `caller` may read `job`: the owner reads every job, a node only
/// the jobs it submitted.
fn may_read(caller: &Option<Extension<NodeCaller>>, job: &JobRecord) -> bool {
    match caller {
        None => true,
        Some(Extension(NodeCaller(cred))) => {
            job.params.get(SUBMITTED_BY).and_then(|v| v.as_str()) == Some(cred.as_str())
        }
    }
}

async fn job_status(
    State(state): State<ApiState>,
    Extension(public_base): Extension<Arc<str>>,
    caller: Option<Extension<NodeCaller>>,
    Path(id): Path<String>,
) -> Result<Response, ComputeError> {
    let gpu = gpu_sample().await;
    let engine = state.lock().await;
    let store = engine.scheduler().store();
    match store.get_job(&id)? {
        Some(job) if may_read(&caller, &job) => Ok(Json(JobView::build(
            store,
            rehost_job(job, &public_base),
            gpu.as_ref(),
        )?)
        .into_response()),
        _ => Err(ComputeError::NotFound(format!("job {id}"))),
    }
}

async fn cancel_job(
    State(state): State<ApiState>,
    Path(id): Path<String>,
) -> Result<Response, ComputeError> {
    let engine = state.lock().await;
    let cancelled = engine.scheduler().store().cancel_job(&id, now_ms())?;
    Ok(Json(CancelResponse { cancelled }).into_response())
}

async fn job_outputs(
    State(state): State<ApiState>,
    Extension(public_base): Extension<Arc<str>>,
    caller: Option<Extension<NodeCaller>>,
    Path(id): Path<String>,
) -> Result<Response, ComputeError> {
    let engine = state.lock().await;
    let store = engine.scheduler().store();
    match store.get_job(&id)? {
        Some(job) if may_read(&caller, &job) => {}
        _ => return Err(ComputeError::NotFound(format!("job {id}"))),
    }
    let outputs: Vec<OutputView> = store
        .outputs_for_job(&id)?
        .into_iter()
        .map(|mut o| {
            o.uri = rewrite_artifact_host(&o.uri, &public_base);
            OutputView::of(o)
        })
        .collect();
    Ok(Json(outputs).into_response())
}

// These tests reach the job API as the operator (the unix operator socket)
// and through issued node credentials.
#[cfg(all(test, unix))]
#[path = "api_tests.rs"]
mod tests;
