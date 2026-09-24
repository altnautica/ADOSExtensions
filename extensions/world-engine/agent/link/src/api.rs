//! The link's HTTP API on the plugin's `http.sock` (the control plane forwards
//! `/api/plugins/<id>/x/<rest>` here as `/<rest>`, already authenticated).
//!
//! Drone:
//! - `GET /atlas/readiness`: whether this drone can build a world model and
//!   whether it is capturing now: `enabled` / `cameras_configured` /
//!   `capture_profile` / `pose_source` / `reconstruct_steps` from the plugin
//!   config, the live `state` / `capturing` / `session_id` / `service_running`
//!   from the capture service's control socket, and the `compute_node_id` the
//!   forwarder is streaming to.
//! - `PUT /atlas/config`: patch the Atlas plugin config (enable, capture
//!   profile, reconstruction steps, camera set). Nothing is restarted: the
//!   capture service re-reads its config every 5 s.
//! - `POST /atlas/capture/{start,stop,pause,resume}`: drive the live session
//!   through the control socket; the reply is the resulting capture status.
//! - `GET /compute/status`: a drone has no compute heartbeat, so always the
//!   not-a-compute-node 404.
//! - `POST /node-credential`: see [`crate::credentials`].
//!
//! Ground station:
//! - `GET /atlas-relay/status`: the relay's live counters.
//! - `POST /node-credential`.

use std::sync::Arc;

use axum::extract::State;
use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::routing::{get, post, put};
use axum::{Json, Router};
use rmpv::Value as Mpv;
use serde::Deserialize;
use serde_json::{json, Value};
use world_engine_protocol::atlas::{CaptureState, CaptureStatus};

use crate::control::{ControlClient, ControlError, LiveCapture};
use crate::credentials::{self, CredentialStore};
use crate::forward::SharedForward;
use crate::host::{config_json, Host};
use crate::relay::{status_body, SharedSnapshot};

/// The capture profiles the capture service accepts.
const VALID_CAPTURE_PROFILES: [&str; 4] = ["orbit", "lawnmower", "freeform", "inspection"];
/// The camera roles the capture service accepts.
const VALID_CAMERA_ROLES: [&str; 7] = ["primary", "aux", "down", "left", "right", "back", "up"];
/// The default reconstruction detail level (training steps).
const DEFAULT_RECONSTRUCT_STEPS: u64 = 30_000;

/// An error body: `{"detail": message}`.
pub fn detail(status: StatusCode, message: impl Into<String>) -> Response {
    (status, Json(json!({ "detail": message.into() }))).into_response()
}

/// What the drone routes read.
#[derive(Clone)]
pub struct DroneApi {
    pub host: Arc<dyn Host>,
    pub control: ControlClient,
    pub forward: SharedForward,
    /// This node's profile, reported verbatim.
    pub profile: String,
}

/// The drone router.
pub fn drone_router(api: DroneApi, credentials: CredentialStore) -> Router {
    Router::new()
        .route("/atlas/readiness", get(get_readiness))
        .route("/atlas/config", put(put_config))
        .route(
            "/atlas/capture/start",
            post(|s: State<DroneApi>| capture(s, "start")),
        )
        .route(
            "/atlas/capture/stop",
            post(|s: State<DroneApi>| capture(s, "stop")),
        )
        .route(
            "/atlas/capture/pause",
            post(|s: State<DroneApi>| capture(s, "pause")),
        )
        .route(
            "/atlas/capture/resume",
            post(|s: State<DroneApi>| capture(s, "resume")),
        )
        .route("/compute/status", get(get_compute_status))
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

/// Map the configured pose tier to the pose source the drone will use.
fn pose_source_for(tier: &str) -> &'static str {
    match tier {
        "offload" => "offloaded_slam",
        "hybrid" => "hybrid",
        _ => "local_vio",
    }
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
    let pose_tier = config_json(host, "atlas.pose_tier")
        .await
        .as_str()
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

    let live = LiveCapture::from(api.control.command("status").await);
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
        "pose_source": pose_source_for(&pose_tier),
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

/// Forward a capture command and shape the reply: the post-command status; a
/// 503 when nothing is listening (the command did not apply); a 504 when the
/// command was written but not answered in time (a slow stop may still apply).
fn capture_reply(result: Result<CaptureStatus, ControlError>) -> Response {
    match LiveCapture::from(result) {
        LiveCapture::Running(status) => Json(status).into_response(),
        LiveCapture::NotRunning => detail(
            StatusCode::SERVICE_UNAVAILABLE,
            "atlas capture service is not running",
        ),
        LiveCapture::Unknown => detail(
            StatusCode::GATEWAY_TIMEOUT,
            "atlas capture service did not confirm the command in time; it may still apply",
        ),
    }
}

async fn capture(State(api): State<DroneApi>, cmd: &'static str) -> Response {
    capture_reply(api.control.command(cmd).await)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::control::tests::{capturing_status, fake_control_socket};
    use crate::forward::ForwardStatus;
    use crate::testing::FakeHost;
    use axum::body::Body;
    use axum::http::Request;
    use parking_lot::Mutex;
    use std::path::Path;
    use tower::ServiceExt;

    fn api(host: Arc<FakeHost>, control: &Path) -> DroneApi {
        DroneApi {
            host,
            control: ControlClient::new(control.to_path_buf()),
            forward: Arc::new(Mutex::new(ForwardStatus::default())),
            profile: "drone".into(),
        }
    }

    fn router(api: DroneApi, dir: &Path) -> Router {
        drone_router(
            api,
            CredentialStore {
                path: Some(dir.join("creds.json")),
            },
        )
    }

    async fn call(
        router: Router,
        method: &str,
        uri: &str,
        body: Option<Value>,
    ) -> (StatusCode, Value) {
        let mut req = Request::builder().method(method).uri(uri);
        let body = match body {
            Some(b) => {
                req = req.header("content-type", "application/json");
                Body::from(b.to_string())
            }
            None => Body::empty(),
        };
        let resp = router.oneshot(req.body(body).unwrap()).await.unwrap();
        let status = resp.status();
        let bytes = axum::body::to_bytes(resp.into_body(), usize::MAX)
            .await
            .unwrap();
        (
            status,
            serde_json::from_slice(&bytes).unwrap_or(Value::Null),
        )
    }

    #[tokio::test]
    async fn readiness_reports_idle_when_the_service_is_down() {
        let dir = tempfile::tempdir().unwrap();
        let host = FakeHost::new();
        host.set_config("atlas.enabled", json!(true));
        host.set_config(
            "atlas.cameras",
            json!([{"id": "front", "enabled": true}, {"id": "down", "enabled": false}]),
        );
        let (st, body) = call(
            router(api(host, &dir.path().join("none.sock")), dir.path()),
            "GET",
            "/atlas/readiness",
            None,
        )
        .await;
        assert_eq!(st, StatusCode::OK);
        assert_eq!(body["enabled"], json!(true));
        assert_eq!(body["profile"], json!("drone"));
        assert_eq!(body["cameras_configured"], json!(1));
        assert_eq!(body["capture_profile"], json!("freeform"));
        assert_eq!(body["reconstruct_steps"], json!(30000));
        assert_eq!(body["pose_source"], json!("local_vio"));
        assert_eq!(body["service_running"], json!(false));
        assert_eq!(body["capturing"], json!(false));
        assert_eq!(body["state"], json!("idle"));
        assert!(body["session_id"].is_null());
        assert!(body["compute_node_id"].is_null());
        // No service means no live counters, not zero of them.
        assert!(body["keyframes"].is_null());
        assert!(body["camera_count"].is_null());
    }

    #[tokio::test]
    async fn readiness_reads_the_live_session_and_the_forwarders_node() {
        let dir = tempfile::tempdir().unwrap();
        let sock = dir.path().join("atlas-control.sock");
        let recorded = fake_control_socket(&sock, capturing_status());
        let host = FakeHost::new();
        host.set_config("atlas.pose_tier", json!("offload"));
        let api = api(host, &sock);
        // The forwarder resolved a node.
        api.forward.lock().note_node(Some("workstation-01".into()));
        let (_, body) = call(router(api, dir.path()), "GET", "/atlas/readiness", None).await;
        assert_eq!(body["service_running"], json!(true));
        assert_eq!(body["capturing"], json!(true));
        assert_eq!(body["state"], json!("capturing"));
        assert_eq!(body["session_id"], json!("sess-1"));
        assert_eq!(body["keyframes"], json!(4));
        assert_eq!(body["pose_source"], json!("offloaded_slam"));
        assert_eq!(body["compute_node_id"], json!("workstation-01"));
        assert_eq!(recorded.lock()[0], r#"{"cmd":"status"}"#);
    }

    #[tokio::test]
    async fn a_config_patch_writes_only_the_provided_keys() {
        let dir = tempfile::tempdir().unwrap();
        let host = FakeHost::new();
        host.set_config("atlas.capture_profile", json!("orbit"));
        host.set_config("atlas.reconstruct_steps", json!(7000));
        let app = router(api(host.clone(), &dir.path().join("none.sock")), dir.path());
        let (st, body) = call(
            app.clone(),
            "PUT",
            "/atlas/config",
            Some(json!({"enabled": true, "cameras": [
                {"id": "front", "role": "primary", "enabled": true, "reconstruct": true}
            ]})),
        )
        .await;
        assert_eq!(st, StatusCode::OK);
        assert_eq!(body["status"], json!("ok"));
        assert_eq!(body["enabled"], json!(true));
        assert_eq!(body["persisted"], json!(true));
        assert_eq!(body["restart"]["status"], json!("ok"));
        assert_eq!(host.config_value("atlas.enabled"), json!(true));
        assert_eq!(
            host.config_value("atlas.cameras")[0]["role"],
            json!("primary")
        );
        // Untouched keys survive.
        assert_eq!(host.config_value("atlas.capture_profile"), json!("orbit"));
        assert_eq!(host.config_value("atlas.reconstruct_steps"), json!(7000));

        // A patch without `enabled` reports the stored value.
        let (_, body) = call(
            app,
            "PUT",
            "/atlas/config",
            Some(json!({"reconstruct_steps": 9000})),
        )
        .await;
        assert_eq!(body["enabled"], json!(true));
        assert_eq!(host.config_value("atlas.reconstruct_steps"), json!(9000));
    }

    #[tokio::test]
    async fn invalid_config_patches_are_refused_before_any_write() {
        let dir = tempfile::tempdir().unwrap();
        let host = FakeHost::new();
        let app = router(api(host.clone(), &dir.path().join("none.sock")), dir.path());
        for bad in [
            json!({"reconstruct_steps": 500}),
            json!({"enabled": true, "capture_profile": "balanced"}),
            json!({"enabled": true, "cameras": [{"id": "front", "role": "navigation"}]}),
            json!({"cameras": [{"id": "a", "role": 7}]}),
        ] {
            let (st, body) = call(app.clone(), "PUT", "/atlas/config", Some(bad)).await;
            assert_eq!(st, StatusCode::BAD_REQUEST);
            assert!(body["detail"].is_string());
        }
        assert!(host.config.lock().is_empty());
        // A camera with no role is left to the schema (a partial patch).
        let ok = AtlasConfigBody {
            enabled: None,
            capture_profile: None,
            reconstruct_steps: None,
            cameras: Some(json!([{"id": "a", "enabled": true}])),
        };
        assert!(validate_config_body(&ok).is_ok());
    }

    #[tokio::test]
    async fn capture_commands_reach_the_control_socket_or_report_why_not() {
        let dir = tempfile::tempdir().unwrap();
        let sock = dir.path().join("atlas-control.sock");
        let recorded = fake_control_socket(&sock, capturing_status());
        let (st, body) = call(
            router(api(FakeHost::new(), &sock), dir.path()),
            "POST",
            "/atlas/capture/stop",
            None,
        )
        .await;
        assert_eq!(st, StatusCode::OK);
        assert_eq!(body["session_id"], json!("sess-1"));
        assert_eq!(recorded.lock()[0], r#"{"cmd":"stop"}"#);

        let (st, body) = call(
            router(
                api(FakeHost::new(), &dir.path().join("none.sock")),
                dir.path(),
            ),
            "POST",
            "/atlas/capture/start",
            None,
        )
        .await;
        assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE);
        assert_eq!(
            body["detail"],
            json!("atlas capture service is not running")
        );
        assert_eq!(
            capture_reply(Err(ControlError::Timeout)).status(),
            StatusCode::GATEWAY_TIMEOUT
        );
    }

    #[tokio::test]
    async fn a_drone_has_no_compute_status() {
        let dir = tempfile::tempdir().unwrap();
        let (st, body) = call(
            router(
                api(FakeHost::new(), &dir.path().join("none.sock")),
                dir.path(),
            ),
            "GET",
            "/compute/status",
            None,
        )
        .await;
        assert_eq!(st, StatusCode::NOT_FOUND);
        assert_eq!(
            body,
            json!({"detail": "no compute status (not a compute node, or the compute daemon is not running)"})
        );
    }
}
