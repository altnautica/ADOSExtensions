//! Readiness, config and compute-status routes, plus the helpers every route
//! group's tests share.

use super::*;
use crate::control::tests::{capturing_status, fake_status_socket};
use crate::forward::ForwardStatus;
use crate::testing::FakeHost;
use crate::upload::RootFn;
use axum::body::Body;
use axum::http::Request;
use parking_lot::Mutex;
use std::path::Path;
use tower::ServiceExt;
use world_engine_transport::BulkConfig;

/// A drone API over `host`, the control socket at `control`, sessions under
/// `root` and the calib store at `<root>/calib`.
pub fn test_api(host: Arc<FakeHost>, control: &Path, root: &Path) -> DroneApi {
    let credentials = CredentialStore {
        path: Some(root.join("creds.json")),
    };
    let sessions = root.join("sessions");
    let root_fn: RootFn = Arc::new(move || Some(sessions.clone()));
    DroneApi {
        host: host.clone(),
        control: ControlClient::new(control.to_path_buf()),
        forward: Arc::new(Mutex::new(ForwardStatus::default())),
        profile: "drone".into(),
        credentials: credentials.clone(),
        uploads: UploadManager::new(host, credentials, root_fn, BulkConfig::default()),
        calib_root: Some(root.join("calib")),
    }
}

/// One request through the router: status and JSON body (`null` when none).
pub async fn call(
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
    let api = test_api(host, &dir.path().join("none.sock"), dir.path());
    let (st, body) = call(drone_router(api), "GET", "/atlas/readiness", None).await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(body["enabled"], json!(true));
    assert_eq!(body["profile"], json!("drone"));
    assert_eq!(body["cameras_configured"], json!(1));
    assert_eq!(body["capture_profile"], json!("freeform"));
    assert_eq!(body["reconstruct_steps"], json!(30000));
    assert_eq!(body["pose_source"], json!("auto"));
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
async fn readiness_reads_the_live_session_the_provider_and_the_forwarders_node() {
    let dir = tempfile::tempdir().unwrap();
    let sock = dir.path().join("atlas-control.sock");
    let recorded = fake_status_socket(&sock, capturing_status());
    let host = FakeHost::new();
    host.set_config("atlas.pose_provider", json!("vio"));
    let api = test_api(host.clone(), &sock, dir.path());
    api.forward.lock().note_node(Some("workstation-01".into()));
    let (_, body) = call(drone_router(api.clone()), "GET", "/atlas/readiness", None).await;
    assert_eq!(body["service_running"], json!(true));
    assert_eq!(body["capturing"], json!(true));
    assert_eq!(body["state"], json!("capturing"));
    assert_eq!(body["session_id"], json!("sess-1"));
    assert_eq!(body["keyframes"], json!(4));
    assert_eq!(body["pose_source"], json!("vio"));
    assert_eq!(body["compute_node_id"], json!("workstation-01"));
    assert_eq!(recorded.lock()[0], r#"{"cmd":"status"}"#);

    // An out-of-set provider (a retired tier name) reads as auto.
    host.set_config("atlas.pose_provider", json!("offload"));
    let (_, body) = call(drone_router(api), "GET", "/atlas/readiness", None).await;
    assert_eq!(body["pose_source"], json!("auto"));
}

#[tokio::test]
async fn a_config_patch_writes_only_the_provided_keys() {
    let dir = tempfile::tempdir().unwrap();
    let host = FakeHost::new();
    host.set_config("atlas.capture_profile", json!("orbit"));
    host.set_config("atlas.reconstruct_steps", json!(7000));
    let app = drone_router(test_api(
        host.clone(),
        &dir.path().join("none.sock"),
        dir.path(),
    ));
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
    let app = drone_router(test_api(
        host.clone(),
        &dir.path().join("none.sock"),
        dir.path(),
    ));
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
async fn a_drone_has_no_compute_status() {
    let dir = tempfile::tempdir().unwrap();
    let app = drone_router(test_api(
        FakeHost::new(),
        &dir.path().join("none.sock"),
        dir.path(),
    ));
    let (st, body) = call(app, "GET", "/compute/status", None).await;
    assert_eq!(st, StatusCode::NOT_FOUND);
    assert_eq!(
        body,
        json!({"detail": "no compute status (not a compute node, or the compute daemon is not running)"})
    );
}
