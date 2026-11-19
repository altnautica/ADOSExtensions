//! The link's HTTP API on the plugin's `http.sock` (the control plane forwards
//! `/api/plugins/<id>/x/<rest>` here as `/<rest>`, already authenticated).
//!
//! Drone and Visor:
//! - `GET /capture/readiness`: whether this device can capture and whether it
//!   is capturing now: `enabled` / `cameras_configured` / `capture_profile` /
//!   `pose_source` (`capture.pose_source`: `auto|fc_ekf|navigation`) /
//!   `reconstruct_steps` from the plugin config, the live `state` /
//!   `capturing` / `session_id` / `service_running` from the capture
//!   service's control socket, and the `compute_node_id` the forwarder is
//!   streaming to.
//! - `PUT /capture/config`: patch Capture's `capture.*` keys (a flat map of
//!   dotted keys). A key another extension owns is refused naming its owner.
//!   Nothing is restarted: the capture service re-reads its config every 5 s.
//! - The capture session ([`crate::capture_api`]), the WHEP preview and rig
//!   relays ([`crate::device_api`]), recorded sessions and their upload
//!   ([`crate::sessions_api`]), reconstruction on the paired node
//!   ([`crate::jobs_api`]) and the calibration store ([`crate::calib_api`]).
//! - `POST /capture/node-credential`: see
//!   [`ados_mapping_transport::credentials`].
//!
//! Ground station:
//! - `GET /capture/relay/status`: the relay's live counters.
//! - `POST /capture/node-credential`.

use std::collections::BTreeMap;
use std::path::PathBuf;
use std::sync::Arc;

use ados_mapping_protocol::atlas::CaptureState;
use axum::extract::State;
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::routing::{get, put};
use axum::{Json, Router};
use rmpv::Value as Mpv;
use serde_json::{json, Value};

use crate::control::{ControlClient, LiveCapture};
use crate::forward::SharedForward;
use crate::relay::{status_body, SharedSnapshot};
use crate::upload::UploadManager;
use crate::{calib_api, capture_api, device_api, jobs_api, sessions_api};
use ados_mapping_transport::credentials::{self, CredentialStore};
use ados_mapping_transport::host::{config_json, Host};

/// The capture profiles the capture service accepts.
const VALID_CAPTURE_PROFILES: [&str; 4] = ["orbit", "lawnmower", "freeform", "inspection"];
/// The camera roles the capture service accepts.
const VALID_CAMERA_ROLES: [&str; 7] = ["primary", "aux", "down", "left", "right", "back", "up"];
/// The default reconstruction detail level (training steps).
const DEFAULT_RECONSTRUCT_STEPS: u64 = 30_000;
/// The pose sources `capture.pose_source` selects between.
const POSE_SOURCES: [&str; 3] = ["auto", "fc_ekf", "navigation"];

/// Every key `PUT /capture/config` writes (the leaves of an object-valued key
/// such as `capture.selection` are written whole under it).
pub const CAPTURE_KEYS: [&str; 22] = [
    "capture.enabled",
    "capture.capture_profile",
    "capture.reconstruct_steps",
    "capture.cameras",
    "capture.pose_prior",
    "capture.selection.min_translation_m",
    "capture.selection.min_rotation_rad",
    "capture.selection.max_interval_ms",
    "capture.selection.max_keyframes",
    "capture.intrinsics",
    "capture.record_on_start",
    "capture.image_encoding",
    "capture.mains_hz",
    "capture.exposure",
    "capture.segment",
    "capture.auto_upload",
    "capture.rig_id",
    "capture.pose_source",
    "capture.memory_budget_mb",
    "capture.compute_node_addr",
    "capture.relay.enabled",
    "capture.relay.compute_base_url",
];

/// The extension a key outside Capture's set belongs to, by namespace.
const OWNERS: [(&str, &str); 2] = [
    ("navigation.", "Navigation"),
    ("reconstruct.", "Reconstruct"),
];

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
        .route("/capture/readiness", get(get_readiness))
        .route("/capture/config", put(put_config))
        .merge(capture_api::routes())
        .merge(sessions_api::routes())
        .merge(jobs_api::routes())
        .merge(calib_api::routes())
        .merge(device_api::routes())
        .with_state(api)
        .merge(credentials::router("/capture/node-credential", credentials))
}

/// The ground-station router.
pub fn ground_router(snapshot: SharedSnapshot, credentials: CredentialStore) -> Router {
    Router::new()
        .route("/capture/relay/status", get(get_relay_status))
        .with_state(snapshot)
        .merge(credentials::router("/capture/node-credential", credentials))
}

async fn get_relay_status(State(snapshot): State<SharedSnapshot>) -> Response {
    Json(status_body(snapshot.lock().as_ref())).into_response()
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
    let enabled = config_json(host, "capture.enabled")
        .await
        .as_bool()
        .unwrap_or(false);
    let capture_profile = config_json(host, "capture.capture_profile")
        .await
        .as_str()
        .unwrap_or("freeform")
        .to_string();
    let reconstruct_steps = config_json(host, "capture.reconstruct_steps")
        .await
        .as_u64()
        .filter(|n| *n > 0)
        .unwrap_or(DEFAULT_RECONSTRUCT_STEPS);
    let pose_source = config_json(host, "capture.pose_source")
        .await
        .as_str()
        .filter(|p| POSE_SOURCES.contains(p))
        .unwrap_or("auto")
        .to_string();
    let cameras_configured = config_json(host, "capture.cameras")
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
        "pose_source": pose_source,
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

/// Why `key` with `value` cannot be written, or `None` when it can. A key
/// another extension owns names that owner.
fn refusal(key: &str, value: &Value) -> Option<String> {
    if !CAPTURE_KEYS.contains(&key) {
        return Some(match OWNERS.iter().find(|(ns, _)| key.starts_with(ns)) {
            Some((_, owner)) => format!("`{key}` belongs to {owner}; set it on that extension"),
            None => format!("`{key}` is not a Capture setting"),
        });
    }
    let one_of = |set: &[&str]| {
        value
            .as_str()
            .filter(|s| set.contains(s))
            .map(|_| ())
            .ok_or_else(|| format!("`{key}` is one of {}, got {value}", set.join(" / ")))
    };
    let checked = match key {
        "capture.enabled" | "capture.record_on_start" | "capture.auto_upload" => value
            .as_bool()
            .map(|_| ())
            .ok_or_else(|| format!("`{key}` is true or false, got {value}")),
        "capture.capture_profile" => one_of(&VALID_CAPTURE_PROFILES),
        "capture.pose_source" => one_of(&POSE_SOURCES),
        "capture.reconstruct_steps" => value
            .as_u64()
            .filter(|n| (1000..=200_000).contains(n))
            .map(|_| ())
            .ok_or_else(|| format!("`{key}` is 1000..=200000, got {value}")),
        "capture.cameras" => cameras_ok(value),
        _ => Ok(()),
    };
    checked.err()
}

/// A camera set is an array whose roles the capture service accepts.
fn cameras_ok(value: &Value) -> Result<(), String> {
    let cams = value
        .as_array()
        .ok_or_else(|| format!("`capture.cameras` is an array, got {value}"))?;
    for cam in cams {
        if let Some(role) = cam.get("role") {
            if !role
                .as_str()
                .is_some_and(|r| VALID_CAMERA_ROLES.contains(&r))
            {
                return Err(format!(
                    "invalid camera role {role} (expected one of {})",
                    VALID_CAMERA_ROLES.join(" / ")
                ));
            }
        }
    }
    Ok(())
}

/// The config writes a patch makes, every key checked before any is
/// written. `Err` carries the 400 detail.
pub fn config_writes(patch: &BTreeMap<String, Value>) -> Result<Vec<(String, Mpv)>, String> {
    let mut writes = Vec::with_capacity(patch.len());
    for (key, value) in patch {
        if let Some(why) = refusal(key, value) {
            return Err(why);
        }
        writes.push((
            key.clone(),
            rmpv::ext::to_value(value).map_err(|e| e.to_string())?,
        ));
    }
    Ok(writes)
}

async fn put_config(
    State(api): State<DroneApi>,
    Json(patch): Json<BTreeMap<String, Value>>,
) -> Response {
    let writes = match config_writes(&patch) {
        Ok(w) => w,
        Err(msg) => return detail(StatusCode::BAD_REQUEST, msg),
    };
    for (key, value) in writes {
        if let Err(e) = api.host.config_set(&key, value).await {
            return detail(
                StatusCode::INTERNAL_SERVER_ERROR,
                format!("could not write {key}: {e}"),
            );
        }
    }
    let enabled = config_json(api.host.as_ref(), "capture.enabled")
        .await
        .as_bool()
        .unwrap_or(false);
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
