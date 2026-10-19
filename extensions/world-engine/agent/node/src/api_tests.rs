//! Tests of `api`.

use super::*;
use crate::{Cluster, JobStore, MockDetector, MockReconstructor, Output, Scheduler};
use axum::body::Body;
use axum::http::Request;
use tower::ServiceExt;

fn test_state() -> ApiState {
    let store = JobStore::open_in_memory().unwrap();
    let scheduler = Scheduler::new(store, Arc::new(MockReconstructor), Arc::new(MockDetector));
    let engine = Engine::new(scheduler, Cluster::new_master("node-a"), 2);
    Arc::new(Mutex::new(engine))
}

/// Auth over an empty credential store at a path nothing is written to.
fn test_auth() -> Arc<ComputeAuth> {
    Arc::new(ComputeAuth::new(crate::NodeCredentialStore::open(
        "/nonexistent/we-node-test-creds.json".into(),
        "node-a",
    )))
}

/// Auth over a credential store in `dir` (for tests that issue).
fn auth_in(dir: &std::path::Path) -> Arc<ComputeAuth> {
    Arc::new(ComputeAuth::new(crate::NodeCredentialStore::open(
        dir.join("creds.json"),
        "node-a",
    )))
}

/// One request, as the operator (marked as arriving on `http.sock`) or as
/// a TCP caller, with optional headers.
async fn request(
    router: &Router,
    operator: bool,
    method: &str,
    path: &str,
    headers: &[(&str, &str)],
    body: serde_json::Value,
) -> (StatusCode, serde_json::Value) {
    let mut builder = Request::builder()
        .method(method)
        .uri(path)
        .header("content-type", "application/json");
    for (k, v) in headers {
        builder = builder.header(*k, *v);
    }
    let mut req = builder
        .body(Body::from(serde_json::to_vec(&body).unwrap()))
        .unwrap();
    if operator {
        req.extensions_mut()
            .insert(world_engine_transport::UnixPeer);
    }
    let resp = router.clone().oneshot(req).await.unwrap();
    let status = resp.status();
    let bytes = axum::body::to_bytes(resp.into_body(), usize::MAX)
        .await
        .unwrap();
    let json = if bytes.is_empty() {
        serde_json::Value::Null
    } else {
        serde_json::from_slice(&bytes).unwrap_or(serde_json::Value::Null)
    };
    (status, json)
}

/// A TCP request with optional headers.
async fn send_tcp(
    router: &Router,
    method: &str,
    path: &str,
    headers: &[(&str, &str)],
    body: serde_json::Value,
) -> (StatusCode, serde_json::Value) {
    request(router, false, method, path, headers, body).await
}

#[tokio::test]
async fn an_issued_drone_credential_submits_offload_over_tcp_but_reaches_nothing_else() {
    let dir = tempfile::tempdir().unwrap();
    let router = build_router(test_state(), auth_in(dir.path()));

    // Issuing is the operator's; over TCP it is refused.
    let (st, _) = send_tcp(
        &router,
        "POST",
        "/api/compute/node-credentials",
        &[],
        serde_json::json!({ "peer_device_id": "drone-1" }),
    )
    .await;
    assert_eq!(st, StatusCode::UNAUTHORIZED);
    let (st, minted) = send(
        &router,
        "POST",
        "/api/compute/node-credentials",
        serde_json::json!({ "peer_device_id": "drone-1" }),
    )
    .await;
    assert_eq!(st, StatusCode::CREATED);
    assert_eq!(minted["workstation_node_id"], "node-a");
    let cred = minted["credential"].as_str().unwrap().to_string();
    let as_drone = [("x-ados-node-credential", cred.as_str())];

    // The drone's offload submit and session health are admitted over TCP.
    let submit = serde_json::json!({ "kind": "perception_offload",
        "params": { "session": { "id": "s1", "rtsp_url": "rtsp://d:8554/main", "camera_id": "front" } } });
    let (st, _) = send_tcp(
        &router,
        "POST",
        "/api/compute/jobs",
        &as_drone,
        submit.clone(),
    )
    .await;
    assert_eq!(st, StatusCode::CREATED);
    let (st, _) = send_tcp(
        &router,
        "GET",
        "/api/compute/sessions",
        &as_drone,
        serde_json::Value::Null,
    )
    .await;
    assert_eq!(st, StatusCode::OK);
    // No credential, or a pairing key, is nothing over TCP.
    for headers in [&[][..], &[("x-ados-key", "any-key")][..]] {
        let (st, _) = send_tcp(
            &router,
            "POST",
            "/api/compute/jobs",
            headers,
            submit.clone(),
        )
        .await;
        assert_eq!(st, StatusCode::UNAUTHORIZED);
    }

    // Owner-only routes refuse it over TCP: status, listing jobs, issuing more.
    for (m, p, b) in [
        ("GET", "/api/compute/status", serde_json::Value::Null),
        ("GET", "/api/compute/jobs", serde_json::Value::Null),
        (
            "GET",
            "/api/compute/node-credentials",
            serde_json::Value::Null,
        ),
        (
            "POST",
            "/api/compute/node-credentials",
            serde_json::json!({ "peer_device_id": "x" }),
        ),
    ] {
        let (st, _) = send_tcp(&router, m, p, &as_drone, b).await;
        assert_eq!(st, StatusCode::UNAUTHORIZED, "{m} {p}");
    }

    // The operator lists and revokes it; the revoked credential stops working.
    let (_, listed) = send(
        &router,
        "GET",
        "/api/compute/node-credentials",
        serde_json::Value::Null,
    )
    .await;
    let id = listed["credentials"][0]["id"].as_str().unwrap().to_string();
    assert_eq!(listed["credentials"][0]["peer_device_id"], "drone-1");
    assert!(listed["credentials"][0].get("credential").is_none());
    let (st, revoked) = send(
        &router,
        "POST",
        &format!("/api/compute/node-credentials/{id}/revoke"),
        serde_json::Value::Null,
    )
    .await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(revoked["revoked"], true);
    let (st, _) = send_tcp(
        &router,
        "GET",
        "/api/compute/sessions",
        &as_drone,
        serde_json::Value::Null,
    )
    .await;
    assert_eq!(st, StatusCode::UNAUTHORIZED);
}

/// A request as the operator (arriving on `http.sock`).
async fn send(
    router: &Router,
    method: &str,
    path: &str,
    body: serde_json::Value,
) -> (StatusCode, serde_json::Value) {
    request(router, true, method, path, &[], body).await
}

#[tokio::test]
async fn submit_reconstruct_then_run_then_read_completed() {
    let state = test_state();
    let router = build_router(state.clone(), test_auth());

    // Create a dataset.
    let (st, ds) = send(
        &router,
        "POST",
        "/api/compute/datasets",
        serde_json::json!({ "id": "ds-1", "kind": "bag", "meta": { "cameras": 1 } }),
    )
    .await;
    assert_eq!(st, StatusCode::CREATED);
    assert_eq!(ds["id"], "ds-1");

    // Submit a reconstruct job.
    let (st, sub) = send(
        &router,
        "POST",
        "/api/compute/jobs",
        serde_json::json!({ "job_id": "job-1", "kind": "reconstruct", "dataset_id": "ds-1" }),
    )
    .await;
    assert_eq!(st, StatusCode::CREATED);
    assert_eq!(sub["job_id"], "job-1");
    assert_eq!(sub["state"], "queued");

    // It is listed as queued.
    let (_, jobs) = send(&router, "GET", "/api/compute/jobs", serde_json::Value::Null).await;
    assert_eq!(jobs.as_array().unwrap().len(), 1);

    // Run one tick (the worker).
    state.lock().await.tick(now_ms()).unwrap();

    // Now it reads as completed with the splat result.
    let (st, job) = send(
        &router,
        "GET",
        "/api/compute/jobs/job-1",
        serde_json::Value::Null,
    )
    .await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(job["state"], "completed");
    assert_eq!(job["result_ref"], "mock://splat/ds-1");

    // The output is recorded.
    let (_, outs) = send(
        &router,
        "GET",
        "/api/compute/jobs/job-1/outputs",
        serde_json::Value::Null,
    )
    .await;
    let outs: Vec<Output> = serde_json::from_value(outs).unwrap();
    assert_eq!(outs.len(), 1);
    assert_eq!(outs[0].kind, "splat");
}

#[tokio::test]
async fn submit_offload_then_run_completes() {
    let state = test_state();
    let router = build_router(state.clone(), test_auth());
    let (st, _) = send(
        &router,
        "POST",
        "/api/compute/jobs",
        serde_json::json!({ "job_id": "job-off", "kind": "perception_offload",
            "params": { "frame": { "camera_id": "front", "width": 640, "height": 480, "ts_ms": 5 } } }),
    )
    .await;
    assert_eq!(st, StatusCode::CREATED);
    state.lock().await.tick(now_ms()).unwrap();
    let (_, job) = send(
        &router,
        "GET",
        "/api/compute/jobs/job-off",
        serde_json::Value::Null,
    )
    .await;
    assert_eq!(job["state"], "completed");
    assert_eq!(job["result_ref"], "offload://detection/job-off");
    // The offload must have produced a detection ARTIFACT, not just a
    // terminal state. The worker discards JobOutcome.detections, so the
    // recorded Output is the only evidence through the REST surface.
    let (_, outs) = send(
        &router,
        "GET",
        "/api/compute/jobs/job-off/outputs",
        serde_json::Value::Null,
    )
    .await;
    let outs: Vec<Output> = serde_json::from_value(outs).unwrap();
    assert_eq!(outs.len(), 1);
    assert_eq!(outs[0].kind, "detection");
}

#[path = "api_tests_more.rs"]
mod more;
