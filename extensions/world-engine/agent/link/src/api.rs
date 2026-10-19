//! The link's HTTP API on the plugin's `http.sock` (the control plane forwards
//! `/api/plugins/<id>/x/<rest>` here as `/<rest>`, already authenticated).
//!
//! Drone and Visor:
//! - `GET /atlas/readiness`: whether this device can build a world model and
//!   whether it is capturing now: `enabled` / `cameras_configured` /
//!   `capture_profile` / `pose_source` (the configured `atlas.pose_provider`:
//!   `auto|fc_ekf|vio|lidar`) / `reconstruct_steps` from the plugin config,
//!   the live `state` / `capturing` / `session_id` / `service_running` from
//!   the capture service's control socket, and the `compute_node_id` the
//!   forwarder is streaming to.
//! - `PUT /atlas/config`: patch the Atlas plugin config (enable, capture
//!   profile, reconstruction steps, camera set). Nothing is restarted: the
//!   capture service re-reads its config every 5 s.
//! - The capture session ([`crate::capture_api`]), recorded sessions and
//!   their upload ([`crate::sessions_api`]), reconstruction on the paired node
//!   ([`crate::jobs_api`]) and the calibration store ([`crate::calib_api`]).
//! - `GET /compute/status`: a drone has no compute heartbeat, so always the
//!   not-a-compute-node 404.
//! - `POST /node-credential`: see [`crate::credentials`].
//!
//! Ground station:
//! - `GET /atlas-relay/status`: the relay's live counters.
//! - `POST /node-credential`.

use std::path::PathBuf;
use std::sync::Arc;

use axum::extract::State;
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::routing::{get, put};
use axum::{Json, Router};
use rmpv::Value as Mpv;
use serde::Deserialize;
use serde_json::{json, Value};
use world_engine_protocol::atlas::CaptureState;

use crate::control::{ControlClient, LiveCapture};
use crate::credentials::{self, CredentialStore};
use crate::forward::SharedForward;
use crate::host::{config_json, Host};
use crate::relay::{status_body, SharedSnapshot};
use crate::upload::UploadManager;
use crate::{calib_api, capture_api, jobs_api, sessions_api};

/// The capture profiles the capture service accepts.
const VALID_CAPTURE_PROFILES: [&str; 4] = ["orbit", "lawnmower", "freeform", "inspection"];
/// The camera roles the capture service accepts.
const VALID_CAMERA_ROLES: [&str; 7] = ["primary", "aux", "down", "left", "right", "back", "up"];
/// The default reconstruction detail level (training steps).
const DEFAULT_RECONSTRUCT_STEPS: u64 = 30_000;
/// The pose providers `atlas.pose_provider` selects between.
const POSE_PROVIDERS: [&str; 4] = ["auto", "fc_ekf", "vio", "lidar"];

/// An error body: `{"detail": message}`.
pub fn detail(status: StatusCode, message: impl Into<String>) -> Response {
    (status, Json(json!({ "detail": message.into() }))).into_response()
}

/// What the drone routes and the MCP tools read.
#[derive(Clone)]
pub struct DroneApi {
    pub host: Arc<dyn Host>,
    pub control: ControlClient,
    pub forward: SharedForward,
    /// This node's profile, reported verbatim.
    pub profile: String,
    /// The installed node credentials (the bulk lanes present them).
    pub credentials: CredentialStore,
    /// The session uploader; it also knows the capture root.
    pub uploads: UploadManager,
    /// The device calibration store root; `None` without a data directory.
    pub calib_root: Option<PathBuf>,
}

/// The drone router.
pub fn drone_router(api: DroneApi) -> Router {
    let credentials = api.credentials.clone();
    Router::new()
        .route("/atlas/readiness", get(get_readiness))
        .route("/atlas/config", put(put_config))
        .route("/compute/status", get(get_compute_status))
        .merge(capture_api::routes())
        .merge(sessions_api::routes())
        .merge(jobs_api::routes())
        .merge(calib_api::routes())
        .with_state(api)
        .merge(credentials::router(credentials))
}

/// The ground-station router.
pub fn ground_router(snapshot: SharedSnapshot, credentials: CredentialStore) -> Router {
    Router::new()
        .route("/atlas-relay/status", get(get_relay_status))
        .with_state(snapshot)
        .merge(credentials::router(credentials))
}

async fn get_relay_status(State(snapshot): State<SharedSnapshot>) -> Response {
    Json(status_body(snapshot.lock().as_ref())).into_response()
}

async fn get_compute_status() -> Response {
    detail(
        StatusCode::NOT_FOUND,
        "no compute status (not a compute node, or the compute daemon is not running)",
    )
}

/// The stable wire string for a capture state.
fn capture_state_str(state: CaptureState) -> &'static str {
    match state {
        CaptureState::Idle => "idle",
        CaptureState::Capturing => "capturing",
        CaptureState::Paused => "paused",
        CaptureState::Finalizing => "finalizing",
        CaptureState::Bagged => "bagged",
    }
}

async fn get_readiness(State(api): State<DroneApi>) -> Response {
    let host = api.host.as_ref();
    let enabled = config_json(host, "atlas.enabled")
        .await
        .as_bool()
        .unwrap_or(false);
    let capture_profile = config_json(host, "atlas.capture_profile")
        .await
        .as_str()
        .unwrap_or("freeform")
        .to_string();
    let reconstruct_steps = config_json(host, "atlas.reconstruct_steps")
        .await
        .as_u64()
        .filter(|n| *n > 0)
        .unwrap_or(DEFAULT_RECONSTRUCT_STEPS);
    let pose_provider = config_json(host, "atlas.pose_provider")
        .await
        .as_str()
        .filter(|p| POSE_PROVIDERS.contains(p))
        .unwrap_or("auto")
        .to_string();
    let cameras_configured = config_json(host, "atlas.cameras")
        .await
        .as_array()
        .map(|cams| {
            cams.iter()
                .filter(|c| c.get("enabled").and_then(Value::as_bool).unwrap_or(false))
                .count()
        })
        .unwrap_or(0);

    let live = LiveCapture::from(api.control.status().await);
    let (service_running, capturing, state, session_id, camera_count, keyframes, ingest_rate_hz) =
        match &live {
            LiveCapture::Running(s) => (
                json!(true),
                json!(matches!(
                    s.state,
                    CaptureState::Capturing | CaptureState::Paused
                )),
                json!(capture_state_str(s.state)),
                json!((!s.session_id.is_empty()).then(|| s.session_id.clone())),
                json!(s.camera_count),
                json!(s.keyframes),
                json!(s.ingest_rate_hz),
            ),
            // No service, no session: idle and not capturing, but no live
            // counters either.
            LiveCapture::NotRunning => (
                json!(false),
                json!(false),
                json!("idle"),
                Value::Null,
                Value::Null,
                Value::Null,
                Value::Null,
            ),
            // The service did not answer: every live field is unknown.
            LiveCapture::Unknown => (
                Value::Null,
                Value::Null,
                Value::Null,
                Value::Null,
                Value::Null,
                Value::Null,
                Value::Null,
            ),
        };
    let compute_node_id = api.forward.lock().compute_node_id().map(str::to_string);

    Json(json!({
        "enabled": enabled,
        "profile": api.profile,
        "capture_profile": capture_profile,
        "reconstruct_steps": reconstruct_steps,
        "cameras_configured": cameras_configured,
        "pose_source": pose_provider,
        "service_running": service_running,
        "capturing": capturing,
        "state": state,
        "session_id": session_id,
        "camera_count": camera_count,
        "keyframes": keyframes,
        "ingest_rate_hz": ingest_rate_hz,
        "compute_node_id": compute_node_id,
    }))
    .into_response()
}

/// The `PUT /atlas/config` body. Every field is optional (patch semantics):
/// only the provided keys are written.
#[derive(Debug, Deserialize)]
struct AtlasConfigBody {
    enabled: Option<bool>,
    /// `orbit` / `lawnmower` / `freeform` / `inspection`.
    capture_profile: Option<String>,
    /// Default reconstruction detail level, in training steps. Read by the GCS
    /// at reconstruct-submit time; the capture service does not consume it.
    reconstruct_steps: Option<u32>,
    /// The camera set, each `{id, role, enabled, reconstruct}`, written verbatim.
    cameras: Option<Value>,
}

/// Validate the patch before any write: an out-of-set profile or camera role
/// would fail the capture service's config parse, so it is caught here. `Err`
/// carries the 400 detail.
fn validate_config_body(body: &AtlasConfigBody) -> Result<(), String> {
    if let Some(profile) = body.capture_profile.as_deref() {
        if !profile.is_empty() && !VALID_CAPTURE_PROFILES.contains(&profile) {
            return Err(format!(
                "invalid capture_profile `{profile}` (expected one of {})",
                VALID_CAPTURE_PROFILES.join(" / ")
            ));
        }
    }
    if let Some(steps) = body.reconstruct_steps {
        if !(1000..=200_000).contains(&steps) {
            return Err(format!(
                "invalid reconstruct_steps `{steps}` (expected 1000..=200000)"
            ));
        }
    }
    if let Some(cameras) = body.cameras.as_ref().and_then(Value::as_array) {
        for cam in cameras {
            if let Some(role) = cam.get("role") {
                let ok = role
                    .as_str()
                    .is_some_and(|r| VALID_CAMERA_ROLES.contains(&r));
                if !ok {
                    return Err(format!(
                        "invalid camera role {role} (expected one of {})",
                        VALID_CAMERA_ROLES.join(" / ")
                    ));
                }
            }
        }
    }
    Ok(())
}

/// The config writes a patch makes, in order.
fn config_writes(body: &AtlasConfigBody) -> Result<Vec<(&'static str, Mpv)>, String> {
    let mut writes = Vec::new();
    if let Some(enabled) = body.enabled {
        writes.push(("atlas.enabled", Mpv::Boolean(enabled)));
    }
    if let Some(profile) = body.capture_profile.as_deref().filter(|s| !s.is_empty()) {
        writes.push(("atlas.capture_profile", Mpv::from(profile)));
    }
    if let Some(steps) = body.reconstruct_steps {
        writes.push(("atlas.reconstruct_steps", Mpv::from(steps)));
    }
    if let Some(cameras) = &body.cameras {
        writes.push((
            "atlas.cameras",
            rmpv::ext::to_value(cameras).map_err(|e| e.to_string())?,
        ));
    }
    Ok(writes)
}

async fn put_config(State(api): State<DroneApi>, Json(body): Json<AtlasConfigBody>) -> Response {
    if let Err(msg) = validate_config_body(&body) {
        return detail(StatusCode::BAD_REQUEST, msg);
    }
    let writes = match config_writes(&body) {
        Ok(w) => w,
        Err(msg) => return detail(StatusCode::INTERNAL_SERVER_ERROR, msg),
    };
    for (key, value) in writes {
        if let Err(e) = api.host.config_set(key, value).await {
            return detail(
                StatusCode::INTERNAL_SERVER_ERROR,
                format!("could not write {key}: {e}"),
            );
        }
    }
    let enabled = match body.enabled {
        Some(e) => e,
        None => config_json(api.host.as_ref(), "atlas.enabled")
            .await
            .as_bool()
            .unwrap_or(false),
    };
    Json(json!({
        "status": "ok",
        "enabled": enabled,
        "persisted": true,
        // Nothing is restarted: the capture service applies the change on its
        // next config poll.
        "restart": {"status": "ok", "message": "applied on the capture service's next config poll"},
    }))
    .into_response()
}

#[cfg(test)]
#[path = "api_tests.rs"]
pub(crate) mod tests;
