//! The live capture session over the control socket:
//!
//! - `POST /atlas/capture/start` (body optional [`StartArgs`]) →
//!   `{"ok":true,"session_id":".."}`; `409 E_STORAGE_FULL` /
//!   `E_ALREADY_RECORDING` / `E_CLOCK_NOT_READY`.
//! - `POST /atlas/capture/{stop,pause,resume}` → the resulting state view
//!   with `"ok": true`.
//! - `POST /atlas/capture/mark` `{room, floor}` → `{"ok":true,"segment":n}`.
//! - `GET /atlas/capture/state`: the [`CaptureStateView`]; while the capture
//!   service is not running, an idle view with every field present and empty
//!   (storage free space from the capture-storage sidecar when readable).
//! - `GET /atlas/capture/coverage`: the [`CoverageView`]; an empty grid
//!   (`cell_m` 0) while the service is not running.
//! - `PUT /atlas/capture/exposure` [`ExposureView`] → the exposure the
//!   service reports, verbatim (`applied` says whether it reached the sensor).
//!
//! The WHEP and rig routes are in [`crate::device_api`].

use axum::body::Bytes;
use axum::extract::State;
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post, put};
use axum::{Json, Router};
use serde_json::{json, Value};
use world_engine_capture::storage::{read_sidecar, sidecar_path};
use world_engine_protocol::capture_ctl::{
    CaptureStateView, CoverageView, CtlRequest, ExposureView, MarkArgs, PoseView, StartArgs,
    StorageView,
};

use crate::api::DroneApi;
use crate::reply::{respond, ApiError};
use crate::sessions::LiveSession;

/// The capture routes.
pub fn routes() -> Router<DroneApi> {
    Router::new()
        .route("/atlas/capture/start", post(start))
        .route("/atlas/capture/stop", post(stop))
        .route("/atlas/capture/pause", post(pause))
        .route("/atlas/capture/resume", post(resume))
        .route("/atlas/capture/mark", post(mark))
        .route("/atlas/capture/state", get(state))
        .route("/atlas/capture/coverage", get(coverage))
        .route("/atlas/capture/exposure", put(exposure))
}

/// The view while no capture service runs: present, empty fields.
pub fn idle_view(storage: StorageView) -> CaptureStateView {
    CaptureStateView {
        state: "idle".to_string(),
        pose: PoseView {
            source: "none".to_string(),
            health: "lost".to_string(),
            ..PoseView::default()
        },
        exposure: ExposureView::default(),
        storage,
        ..CaptureStateView::default()
    }
}

/// Free space of the capture volume from the sidecar, if readable.
fn sidecar_storage() -> StorageView {
    StorageView {
        free_bytes: read_sidecar(&sidecar_path()).and_then(|s| s.volume.map(|v| v.free_bytes)),
        est_remaining_s: None,
    }
}

/// `value` as an object with `"ok": true` added.
fn with_ok(value: Value) -> Value {
    let mut out = match value {
        Value::Object(m) => Value::Object(m),
        Value::Null => json!({}),
        other => json!({ "result": other }),
    };
    out["ok"] = Value::Bool(true);
    out
}

impl DroneApi {
    /// Start a session.
    pub async fn capture_start(&self, args: StartArgs) -> Result<Value, ApiError> {
        let result = self.control.request(&CtlRequest::Start(args)).await?;
        Ok(
            json!({ "ok": true, "session_id": result.get("session_id").cloned().unwrap_or(Value::Null) }),
        )
    }

    /// `stop` / `pause` / `resume`: the resulting state view.
    pub async fn capture_command(&self, req: CtlRequest) -> Result<Value, ApiError> {
        Ok(with_ok(self.control.request(&req).await?))
    }

    /// Mark the room/floor the operator walked into.
    pub async fn capture_mark(&self, args: MarkArgs) -> Result<Value, ApiError> {
        let result = self.control.request(&CtlRequest::Mark(args)).await?;
        Ok(json!({ "ok": true, "segment": result.get("segment").cloned().unwrap_or(Value::Null) }))
    }

    /// The live state, or the idle view while the service is not running.
    pub async fn capture_state(&self) -> Result<Value, ApiError> {
        match self.control.request(&CtlRequest::State).await {
            Ok(v) => Ok(v),
            Err(e) if e.not_running() => {
                Ok(serde_json::to_value(idle_view(sidecar_storage())).unwrap_or(Value::Null))
            }
            Err(e) => Err(e.into()),
        }
    }

    /// The session the capture service has open now (recording, paused or
    /// closing), from its `state`.
    pub async fn open_session(&self) -> LiveSession {
        match self.control.request(&CtlRequest::State).await {
            Ok(v) => {
                let state = v.get("state").and_then(Value::as_str).unwrap_or("idle");
                match v.get("session_id").and_then(Value::as_str) {
                    Some(id) if state != "idle" => LiveSession::Open(id.to_string()),
                    _ => LiveSession::None,
                }
            }
            Err(e) if e.not_running() => LiveSession::None,
            Err(_) => LiveSession::Unknown,
        }
    }
}

/// A JSON body that may be absent.
fn optional_body<T: serde::de::DeserializeOwned + Default>(body: &Bytes) -> Result<T, ApiError> {
    if body.iter().all(u8::is_ascii_whitespace) {
        return Ok(T::default());
    }
    serde_json::from_slice(body).map_err(|e| ApiError::bad_request(e.to_string()))
}

async fn start(State(api): State<DroneApi>, body: Bytes) -> Response {
    match optional_body::<StartArgs>(&body) {
        Ok(args) => respond(api.capture_start(args).await),
        Err(e) => e.into_response(),
    }
}

async fn stop(State(api): State<DroneApi>) -> Response {
    respond(api.capture_command(CtlRequest::Stop).await)
}

async fn pause(State(api): State<DroneApi>) -> Response {
    respond(api.capture_command(CtlRequest::Pause).await)
}

async fn resume(State(api): State<DroneApi>) -> Response {
    respond(api.capture_command(CtlRequest::Resume).await)
}

async fn mark(State(api): State<DroneApi>, body: Bytes) -> Response {
    match serde_json::from_slice::<MarkArgs>(&body) {
        Ok(args) => respond(api.capture_mark(args).await),
        Err(e) => ApiError::bad_request(e.to_string()).into_response(),
    }
}

async fn state(State(api): State<DroneApi>) -> Response {
    respond(api.capture_state().await)
}

async fn coverage(State(api): State<DroneApi>) -> Response {
    match api.control.request(&CtlRequest::Coverage).await {
        Ok(v) => Json(v).into_response(),
        Err(e) if e.not_running() => Json(CoverageView::default()).into_response(),
        Err(e) => ApiError::from(e).into_response(),
    }
}

async fn exposure(State(api): State<DroneApi>, body: Bytes) -> Response {
    match serde_json::from_slice::<ExposureView>(&body) {
        Ok(view) => respond(
            api.control
                .request(&CtlRequest::Exposure(view))
                .await
                .map_err(ApiError::from),
        ),
        Err(e) => ApiError::bad_request(e.to_string()).into_response(),
    }
}

#[cfg(test)]
#[path = "capture_api_tests.rs"]
mod tests;
