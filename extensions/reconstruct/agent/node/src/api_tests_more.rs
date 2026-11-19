//! More tests of `api`.

use super::*;

#[tokio::test]
async fn job_api_surfaces_session_id_from_params() {
    let router = build_router(test_state(), test_auth());

    // A reconstruct job tagged with its capturing session (the shape the
    // capture ingest submits). The session must appear top-level on both the
    // list and single-job reads so the GCS can correlate the artifact.
    let (st, _) = send(
        &router,
        "POST",
        "/api/compute/jobs",
        serde_json::json!({ "job_id": "recon-s1", "kind": "reconstruct",
            "dataset_id": "ds-s1", "params": { "session_id": "s1", "backend": "brush" } }),
    )
    .await;
    assert_eq!(st, StatusCode::CREATED);

    let (_, jobs) = send(&router, "GET", "/api/compute/jobs", serde_json::Value::Null).await;
    let arr = jobs.as_array().unwrap();
    assert_eq!(arr.len(), 1);
    assert_eq!(arr[0]["session_id"], "s1");
    // The flattened JobRecord fields are still all present alongside it.
    assert_eq!(arr[0]["id"], "recon-s1");
    assert_eq!(arr[0]["state"], "queued");

    let (_, job) = send(
        &router,
        "GET",
        "/api/compute/jobs/recon-s1",
        serde_json::Value::Null,
    )
    .await;
    assert_eq!(job["session_id"], "s1");

    // A job that carries no session (an offload job) omits the field entirely,
    // so the surface stays backward-compatible.
    send(
        &router,
        "POST",
        "/api/compute/jobs",
        serde_json::json!({ "job_id": "off-1", "kind": "perception_offload" }),
    )
    .await;
    let (_, off) = send(
        &router,
        "GET",
        "/api/compute/jobs/off-1",
        serde_json::Value::Null,
    )
    .await;
    assert!(
        off.get("session_id").is_none(),
        "a sessionless job omits session_id, got: {off}"
    );
}

#[tokio::test]
async fn a_sessionless_submit_mints_unique_ids_so_a_reopen_is_never_blocked() {
    // The offload orchestrator submits its streaming-session trigger with NO
    // job_id, so the node mints a unique one. A re-open after a session ended
    // must never collide with a retained terminal job from the prior open:
    // two identical no-id submits both succeed with DIFFERENT ids.
    let router = build_router(test_state(), test_auth());
    let body = serde_json::json!({
        "kind": "perception_offload",
        "params": { "session": { "id": "s1", "rtsp_url": "rtsp://d:8554/main", "camera_id": "front" } }
    });
    let (st1, sub1) = send(&router, "POST", "/api/compute/jobs", body.clone()).await;
    assert_eq!(st1, StatusCode::CREATED);
    let (st2, sub2) = send(&router, "POST", "/api/compute/jobs", body).await;
    assert_eq!(st2, StatusCode::CREATED, "a re-open is never a 409");
    assert_ne!(
        sub1["job_id"], sub2["job_id"],
        "each trigger gets a unique node-minted id"
    );
}

#[tokio::test]
async fn duplicate_job_id_is_a_409() {
    let router = build_router(test_state(), test_auth());
    let body = serde_json::json!({ "job_id": "dup", "kind": "reconstruct", "dataset_id": "ds-x" });
    let (st1, _) = send(&router, "POST", "/api/compute/jobs", body.clone()).await;
    assert_eq!(st1, StatusCode::CREATED);
    let (st2, _) = send(&router, "POST", "/api/compute/jobs", body).await;
    assert_eq!(st2, StatusCode::CONFLICT);
}

#[tokio::test]
async fn unknown_job_is_404() {
    let router = build_router(test_state(), test_auth());
    let (st, body) = send(
        &router,
        "GET",
        "/api/compute/jobs/ghost",
        serde_json::Value::Null,
    )
    .await;
    assert_eq!(st, StatusCode::NOT_FOUND);
    assert!(body["error"].as_str().unwrap().contains("ghost"));
}

#[tokio::test]
async fn cancel_a_queued_job() {
    let router = build_router(test_state(), test_auth());
    send(
        &router,
        "POST",
        "/api/compute/jobs",
        serde_json::json!({ "job_id": "job-c", "kind": "reconstruct", "dataset_id": "ds-x" }),
    )
    .await;
    let (st, body) = send(
        &router,
        "POST",
        "/api/compute/jobs/job-c/cancel",
        serde_json::Value::Null,
    )
    .await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(body["cancelled"], true);
}

#[tokio::test]
async fn status_reports_master_role() {
    let router = build_router(test_state(), test_auth());
    let (st, body) = send(
        &router,
        "GET",
        "/api/compute/status",
        serde_json::Value::Null,
    )
    .await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(body["role"], "master");
    assert_eq!(body["workers_idle"], 2);
    assert_eq!(body["cluster"]["master_id"], "node-a");
}

/// A router over a populated session registry, to exercise the session
/// surfaces (`build_router`'s registry is empty). The engine's heartbeat
/// counter is wired to the same registry (as the daemon wires it), so
/// `active_sessions` reflects the live records.
fn router_with_sessions(sessions: Arc<SessionRegistry>) -> Router {
    let store = JobStore::open_in_memory().unwrap();
    let scheduler = Scheduler::new(store, Arc::new(MockReconstructor), Arc::new(MockDetector));
    let mut engine = Engine::new(scheduler, Cluster::new_master("node-a"), 2);
    engine.set_session_counter(sessions.active_counter());
    let state = Arc::new(Mutex::new(engine));
    build_router_with_base(
        state,
        test_auth(),
        Arc::from("http://127.0.0.1:8092"),
        sessions,
    )
}

#[tokio::test]
async fn sessions_endpoint_lists_live_sessions_with_their_fields() {
    let registry = Arc::new(SessionRegistry::new());
    registry.register(
        "s1",
        "front",
        "rtsp://drone.local:8554/main",
        Arc::new(tokio::sync::Notify::new()),
        1000,
    );
    registry.on_batch("s1", 1500); // -> Live, one frame + batch, last-batch stamped
    let router = router_with_sessions(registry);

    let (st, body) = send(
        &router,
        "GET",
        "/api/compute/sessions",
        serde_json::Value::Null,
    )
    .await;
    assert_eq!(st, StatusCode::OK);
    let arr = body.as_array().unwrap();
    assert_eq!(arr.len(), 1);
    let s = &arr[0];
    assert_eq!(s["id"], "s1");
    assert_eq!(s["state"], "live");
    assert_eq!(s["camera_id"], "front");
    assert_eq!(s["source"], "rtsp://drone.local:8554/main");
    assert_eq!(s["frames_processed"], 1);
    assert_eq!(s["batches_emitted"], 1);
    assert_eq!(s["last_batch_at_ms"], 1500);
    assert_eq!(s["reconnects"], 0);
    assert_eq!(s["restarts"], 0);
    assert!(s["uptime_ms"].is_i64(), "uptime is reported");
}

#[tokio::test]
async fn sessions_endpoint_is_empty_with_no_live_session() {
    let router = build_router(test_state(), test_auth());
    let (st, body) = send(
        &router,
        "GET",
        "/api/compute/sessions",
        serde_json::Value::Null,
    )
    .await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(body.as_array().unwrap().len(), 0);
}

#[tokio::test]
async fn status_reports_the_session_state_breakdown() {
    let registry = Arc::new(SessionRegistry::new());
    registry.register(
        "s1",
        "front",
        "rtsp://d/main",
        Arc::new(tokio::sync::Notify::new()),
        1000,
    );
    registry.on_batch("s1", 1100); // Live
    registry.register(
        "s2",
        "rear",
        "rtsp://d/rear",
        Arc::new(tokio::sync::Notify::new()),
        1000,
    ); // Opening
    let router = router_with_sessions(registry);

    let (st, body) = send(
        &router,
        "GET",
        "/api/compute/status",
        serde_json::Value::Null,
    )
    .await;
    assert_eq!(st, StatusCode::OK);
    // The heartbeat fields still flatten through.
    assert_eq!(body["role"], "master");
    // Two live sessions surface in the active count + the state breakdown.
    assert_eq!(body["active_sessions"], 2);
    assert_eq!(body["session_states"]["live"], 1);
    assert_eq!(body["session_states"]["opening"], 1);
    assert_eq!(body["session_states"]["stalled"], 0);
}

#[tokio::test]
async fn owner_routes_are_refused_over_tcp_whatever_is_presented() {
    let router = build_router(test_state(), test_auth());
    for headers in [&[][..], &[("x-ados-key", "any-key")][..]] {
        for (m, p) in [("GET", "/api/compute/status"), ("GET", "/api/compute/jobs")] {
            let (st, _) = send_tcp(&router, m, p, headers, serde_json::Value::Null).await;
            assert_eq!(st, StatusCode::UNAUTHORIZED, "{m} {p}");
            let (st, _) = send(&router, m, p, serde_json::Value::Null).await;
            assert_eq!(st, StatusCode::OK, "operator {m} {p}");
        }
    }
}

#[tokio::test]
async fn a_node_reads_only_the_jobs_it_submitted() {
    let dir = tempfile::tempdir().unwrap();
    let router = build_router(test_state(), auth_in(dir.path()));
    let mut creds = Vec::new();
    for peer in ["visor-1", "visor-2"] {
        let (st, minted) = send(
            &router,
            "POST",
            "/api/compute/node-credentials",
            serde_json::json!({ "peer_device_id": peer }),
        )
        .await;
        assert_eq!(st, StatusCode::CREATED);
        creds.push(minted["credential"].as_str().unwrap().to_string());
    }
    let one = [("x-ados-node-credential", creds[0].as_str())];
    let two = [("x-ados-node-credential", creds[1].as_str())];
    // A forged `submitted_by` is replaced by the caller's credential id.
    let submit = serde_json::json!({ "job_id": "j-own", "kind": "perception_offload",
        "params": { "submitted_by": "someone-else",
            "session": { "id": "s1", "rtsp_url": "rtsp://d:8554/main", "camera_id": "front" } } });
    let (st, _) = send_tcp(&router, "POST", "/api/compute/jobs", &one, submit).await;
    assert_eq!(st, StatusCode::CREATED);
    for path in ["/api/compute/jobs/j-own", "/api/compute/jobs/j-own/outputs"] {
        let (st, _) = send_tcp(&router, "GET", path, &one, serde_json::Value::Null).await;
        assert_eq!(st, StatusCode::OK, "{path}");
        let (st, _) = send_tcp(&router, "GET", path, &two, serde_json::Value::Null).await;
        assert_eq!(st, StatusCode::NOT_FOUND, "{path}");
        let (st, _) = send_tcp(&router, "GET", path, &[], serde_json::Value::Null).await;
        assert_eq!(st, StatusCode::UNAUTHORIZED, "{path}");
    }
    let (st, job) = send(
        &router,
        "GET",
        "/api/compute/jobs/j-own",
        serde_json::Value::Null,
    )
    .await;
    assert_eq!(st, StatusCode::OK);
    let id = creds[0].split('.').nth(1).unwrap();
    assert_eq!(job["params"]["submitted_by"], id);
    // Listing stays the owner's.
    let (st, _) = send_tcp(
        &router,
        "GET",
        "/api/compute/jobs",
        &one,
        serde_json::Value::Null,
    )
    .await;
    assert_eq!(st, StatusCode::UNAUTHORIZED);
}
