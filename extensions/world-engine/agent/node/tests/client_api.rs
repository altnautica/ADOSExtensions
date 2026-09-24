//! The compute client against the node's real job-API router on a loopback
//! port: submit, read results, cancel, idempotent retry, a 404, and the TCP
//! credential gate a drone's client meets.
//!
//! The owner-side calls (status, datasets, reads, cancel) are the operator's,
//! which reach the node on its `http.sock`; the client speaks TCP, so those
//! tests serve the router with every request marked as arriving on the
//! operator socket, exercising the real handlers and wire shapes.

use std::net::SocketAddr;
use std::sync::Arc;

use tokio::sync::Mutex;
use world_engine_node::{
    build_router, Cluster, ComputeAuth, ComputeJobKind, ComputeJobState, ComputeRole, Engine,
    JobStore, MockDetector, MockReconstructor, NodeCredentialStore, Scheduler,
};
use world_engine_protocol::node_credential::NodeLane;
use world_engine_transport::{ClientError, ComputeClient, UnixPeer};

/// Spin the real job-API router over a fresh engine on a loopback port.
/// `operator` marks every request as arriving on the operator socket (the
/// owner); otherwise the TCP gate applies. Returns the bound address, the
/// engine handle (so a test can drive `tick` directly, the way the node's
/// worker loop would), the auth (so a test can issue a credential), and the
/// credential store's directory (held for the test's life).
async fn spawn(
    operator: bool,
) -> (
    SocketAddr,
    Arc<Mutex<Engine>>,
    Arc<ComputeAuth>,
    tempfile::TempDir,
) {
    let store = JobStore::open_in_memory().unwrap();
    let scheduler = Scheduler::new(store, Arc::new(MockReconstructor), Arc::new(MockDetector));
    let engine = Arc::new(Mutex::new(Engine::new(
        scheduler,
        Cluster::new_master("node-a"),
        2,
    )));
    let dir = tempfile::tempdir().unwrap();
    let auth = Arc::new(ComputeAuth::new(NodeCredentialStore::open(
        dir.path().join("creds.json"),
        "node-a",
    )));
    let mut app = build_router(engine.clone(), auth.clone());
    if operator {
        app = app.layer(axum::middleware::from_fn(
            |mut req: axum::extract::Request, next: axum::middleware::Next| async move {
                req.extensions_mut().insert(UnixPeer);
                next.run(req).await
            },
        ));
    }
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });
    (addr, engine, auth, dir)
}

#[tokio::test]
async fn client_submits_a_job_and_reads_its_result() {
    let (addr, engine, _, _dir) = spawn(true).await;
    let client = ComputeClient::new(format!("http://{addr}"), None);

    // Status reflects the master node.
    let hb = client.status().await.unwrap();
    assert_eq!(hb.role, ComputeRole::Master);

    // Register a dataset, submit a reconstruct job over it.
    let ds = client
        .write_dataset("bag", serde_json::json!({ "cameras": 1 }))
        .await
        .unwrap();
    let sub = client
        .submit_job(
            ComputeJobKind::Reconstruct,
            Some(ds.id.clone()),
            serde_json::json!({}),
            None,
        )
        .await
        .unwrap();
    assert_eq!(sub.state, ComputeJobState::Queued);

    // The node's worker loop runs the job; drive a tick directly here.
    engine.lock().await.tick(1).unwrap();

    // The job completed and produced the mock splat output.
    let job = client.job_status(&sub.job_id).await.unwrap();
    assert_eq!(job.state, ComputeJobState::Completed);
    let outputs = client.job_outputs(&sub.job_id).await.unwrap();
    assert_eq!(outputs.len(), 1);
    assert_eq!(outputs[0].kind, "splat");
}

#[tokio::test]
async fn a_perception_offload_returns_a_detection_artifact() {
    let (addr, engine, _, _dir) = spawn(true).await;
    let client = ComputeClient::new(format!("http://{addr}"), None);
    // No dataset; the frame rides params.
    let sub = client
        .submit_job(
            ComputeJobKind::PerceptionOffload,
            None,
            serde_json::json!({ "frame": { "camera_id": "front", "width": 640, "height": 480, "ts_ms": 1 } }),
            None,
        )
        .await
        .unwrap();
    engine.lock().await.tick(1).unwrap();
    let outputs = client.job_outputs(&sub.job_id).await.unwrap();
    assert_eq!(outputs[0].kind, "detection");
}

#[tokio::test]
async fn client_cancels_a_queued_job() {
    let (addr, _, _, _dir) = spawn(true).await;
    let client = ComputeClient::new(format!("http://{addr}"), None);
    let ds = client
        .write_dataset("bag", serde_json::json!({}))
        .await
        .unwrap();
    let sub = client
        .submit_job(
            ComputeJobKind::Reconstruct,
            Some(ds.id),
            serde_json::json!({}),
            None,
        )
        .await
        .unwrap();
    // The job is still queued (no tick), so cancel succeeds.
    assert!(client.cancel_job(&sub.job_id).await.unwrap());
    assert_eq!(
        client.job_status(&sub.job_id).await.unwrap().state,
        ComputeJobState::Cancelled
    );
}

#[tokio::test]
async fn a_reused_job_id_is_a_409_so_a_retry_is_idempotent() {
    let (addr, _, _, _dir) = spawn(true).await;
    let client = ComputeClient::new(format!("http://{addr}"), None);
    let ds = client
        .write_dataset("bag", serde_json::json!({}))
        .await
        .unwrap();
    let id = Some("my-idempotent-job".to_string());
    client
        .submit_job(
            ComputeJobKind::Reconstruct,
            Some(ds.id.clone()),
            serde_json::json!({}),
            id.clone(),
        )
        .await
        .unwrap();
    // A retry with the same id does not create a second job.
    match client
        .submit_job(
            ComputeJobKind::Reconstruct,
            Some(ds.id),
            serde_json::json!({}),
            id,
        )
        .await
    {
        Err(ClientError::Http(409, _)) => {}
        other => panic!("expected Http(409) on a duplicate id, got {other:?}"),
    }
}

#[tokio::test]
async fn a_missing_job_is_a_404() {
    let (addr, _, _, _dir) = spawn(true).await;
    let client = ComputeClient::new(format!("http://{addr}"), None);
    match client.job_status("nope").await {
        Err(ClientError::Http(404, _)) => {}
        other => panic!("expected Http(404), got {other:?}"),
    }
}

#[tokio::test]
async fn over_tcp_a_lane_credential_submits_but_reads_nothing_owner_only() {
    let (addr, _, auth, _dir) = spawn(false).await;
    let submit = |client: ComputeClient| async move {
        client
            .submit_job(
                ComputeJobKind::PerceptionOffload,
                None,
                serde_json::json!({ "frame": { "camera_id": "front", "width": 64, "height": 48, "ts_ms": 1 } }),
                None,
            )
            .await
    };

    // No credential: refused.
    let anonymous = ComputeClient::new(format!("http://{addr}"), None);
    assert!(matches!(
        submit(anonymous).await,
        Err(ClientError::Http(401, _))
    ));

    // A credential issued for the submit lane: the submit is admitted.
    let minted = auth
        .credentials
        .mint("drone-1", &[NodeLane::from_static("jobs.submit")], 1)
        .unwrap();
    let drone = ComputeClient::new(format!("http://{addr}"), Some(minted.credential.clone()));
    submit(drone).await.unwrap();

    // The same credential reads nothing owner-only.
    let drone = ComputeClient::new(format!("http://{addr}"), Some(minted.credential));
    assert!(matches!(
        drone.status().await,
        Err(ClientError::Http(401, _))
    ));
    assert!(matches!(
        drone.job_status("nope").await,
        Err(ClientError::Http(401, _))
    ));
}
