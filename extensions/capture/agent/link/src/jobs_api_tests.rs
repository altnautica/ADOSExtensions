use std::sync::Arc;

use axum::Json;
use parking_lot::Mutex;
use serde_json::json;

use super::*;
use crate::api::drone_router;
use crate::api::tests::{call, test_api};
use crate::upload::tests::finished_session;
use crate::PIN_KEY;
use ados_mapping_transport::testing::FakeHost;
use capture_service::storage::UPLOADED_MARKER;

type Submitted = Arc<Mutex<Vec<Value>>>;

/// A stand-in job API: accepts `reconstruct` for a session ending `0001`,
/// refuses others as not uploaded.
async fn serve_jobs(submitted: Submitted) -> String {
    let app = axum::Router::new().route(
        "/api/compute/jobs",
        post(move |Json(body): Json<Value>| {
            let submitted = submitted.clone();
            async move {
                let sid = body["params"]["session_id"].as_str().unwrap_or("").to_string();
                submitted.lock().push(body);
                if sid.ends_with("0001") {
                    (StatusCode::CREATED, Json(json!({"job_id": "j-1", "state": "queued"})))
                } else {
                    (
                        StatusCode::CONFLICT,
                        Json(json!({"error": "E_NOT_UPLOADED", "detail": format!("session {sid} is not committed")})),
                    )
                }
            }
        }),
    );
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });
    addr.to_string()
}

#[tokio::test]
async fn a_reconstruct_is_submitted_to_the_paired_node() {
    let dir = tempfile::tempdir().unwrap();
    let submitted: Submitted = Arc::default();
    let addr = serve_jobs(submitted.clone()).await;
    let host = FakeHost::new();
    host.set_config(PIN_KEY, json!(addr));
    let api = test_api(host, &dir.path().join("none.sock"), dir.path());
    let app = drone_router(api.clone());
    let root = dir.path().join("sessions");

    let up = finished_session(&root, "v-20261019T101500Z-0001");
    std::fs::write(up.join(UPLOADED_MARKER), b"{}").unwrap();
    let uri = "/capture/sessions/v-20261019T101500Z-0001/reconstruct";
    let (st, body) = call(app.clone(), "POST", uri, Some(json!({"preset": "tour"}))).await;
    assert_eq!(st, StatusCode::OK, "{body}");
    assert_eq!(body, json!({"ok": true, "node": addr, "job_id": "j-1"}));
    let sent = submitted.lock()[0].clone();
    assert_eq!(sent["kind"], "reconstruct");
    assert_eq!(
        sent["params"],
        json!({"session_id": "v-20261019T101500Z-0001", "preset": "tour"})
    );

    // Still on the device without a commit: refused before the node is asked.
    finished_session(&root, "v-20261019T101500Z-0002");
    let (st, body) = call(
        app.clone(),
        "POST",
        "/capture/sessions/v-20261019T101500Z-0002/reconstruct",
        Some(json!({"preset": "hq"})),
    )
    .await;
    assert_eq!(st, StatusCode::CONFLICT);
    assert_eq!(body["error"], "E_NOT_UPLOADED");
    assert_eq!(submitted.lock().len(), 1);

    // Gone from the device: the node decides, and its refusal passes through.
    let (st, body) = call(
        app.clone(),
        "POST",
        "/capture/sessions/v-20261019T101500Z-0009/reconstruct",
        Some(json!({"preset": "hq"})),
    )
    .await;
    assert_eq!(st, StatusCode::CONFLICT);
    assert_eq!(body["error"], "E_NOT_UPLOADED");
    assert_eq!(
        body["detail"],
        "session v-20261019T101500Z-0009 is not committed"
    );

    let (st, body) = call(app, "POST", uri, Some(json!({"preset": "ultra"}))).await;
    assert_eq!(st, StatusCode::BAD_REQUEST);
    assert_eq!(body["error"], "E_BAD_REQUEST");
}

#[tokio::test]
async fn without_a_node_the_submit_reports_it() {
    let dir = tempfile::tempdir().unwrap();
    let api = test_api(FakeHost::new(), &dir.path().join("none.sock"), dir.path());
    let err = api
        .reconstruct("v-20261019T101500Z-0001", "preview", Some("rc-1".into()))
        .await
        .unwrap_err();
    assert_eq!(err.code, "E_NO_NODE");
    assert_eq!(err.status, StatusCode::SERVICE_UNAVAILABLE);
}
