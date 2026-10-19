use std::sync::Arc;

use axum::extract::Path as UrlPath;
use axum::routing::get;
use axum::Json;
use parking_lot::Mutex;

use super::*;
use crate::api::drone_router;
use crate::api::tests::{call, test_api};
use crate::compute_node::PIN_KEY;
use crate::testing::FakeHost;
use crate::upload::tests::finished_session;
use world_engine_capture::storage::UPLOADED_MARKER;

type Submitted = Arc<Mutex<Vec<Value>>>;

/// A stand-in job API: accepts `reconstruct` for `s-committed`, refuses
/// others as not uploaded, and serves one finished job with a tour.
async fn serve_jobs(submitted: Submitted) -> String {
    let app = axum::Router::new()
        .route(
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
        )
        .route(
            "/api/compute/jobs/:id",
            get(|UrlPath(id): UrlPath<String>| async move {
                Json(json!({"id": id, "kind": "reconstruct", "state": "running", "stage": "train", "stage_progress": 0.42}))
            }),
        )
        .route(
            "/api/compute/jobs/:id/outputs",
            get(|UrlPath(id): UrlPath<String>| async move {
                Json(json!([
                    {"id": "o1", "job_id": id, "kind": "splat", "uri": "http://192.168.1.50:8092/artifacts/j-1/scene.spz", "meta": null, "created_ms": 1},
                    {"id": "o2", "job_id": id, "kind": "tour", "uri": "http://192.168.1.50:8092/artifacts/j-1/tour.zip", "meta": {"rooms": 3}, "created_ms": 2, "file": "tour.zip"}
                ]))
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
    let uri = "/atlas/sessions/v-20261019T101500Z-0001/reconstruct";
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
        "/atlas/sessions/v-20261019T101500Z-0002/reconstruct",
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
        "/atlas/sessions/v-20261019T101500Z-0009/reconstruct",
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

    let status = api.job_status("j-1").await.unwrap();
    assert_eq!(status["stage"], "train");
    assert_eq!(status["node"], json!(addr));
    let tour = api.tour_bundle("j-1").await.unwrap();
    assert_eq!(
        tour["uri"],
        "http://192.168.1.50:8092/artifacts/j-1/tour.zip"
    );
    assert_eq!(tour["meta"]["rooms"], 3);
}

#[tokio::test]
async fn without_a_node_the_submit_reports_it() {
    let dir = tempfile::tempdir().unwrap();
    let api = test_api(FakeHost::new(), &dir.path().join("none.sock"), dir.path());
    let err = api
        .reconstruct("v-20261019T101500Z-0001", "preview")
        .await
        .unwrap_err();
    assert_eq!(err.code, "E_NO_NODE");
    assert_eq!(err.status, StatusCode::SERVICE_UNAVAILABLE);
}
