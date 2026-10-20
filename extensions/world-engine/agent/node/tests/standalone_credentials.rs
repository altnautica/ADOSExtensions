//! The standalone node issues node credentials and admits them on the lanes
//! they were issued for: the owner (bearer) mints a credential for a capture
//! device, the device uploads a session and submits a reconstruction with it,
//! reads only its own job, never reaches an owner route, and is refused once
//! revoked. The issued store lives in the data directory and survives a
//! restart.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use axum::body::Body;
use axum::http::{Request, StatusCode};
use axum::Router;
use tokio::sync::Mutex;
use tower::ServiceExt;
use world_engine_node::recon::{StandInRunner, ToolRunner};
use world_engine_node::{
    standalone, ApiState, Cluster, Engine, JobStore, MockDetector, MockReconstructor, Scheduler,
};
use world_engine_protocol::node_credential::NODE_CREDENTIAL_HEADER;

const TOKEN: &str = "standalone-owner-token";
const SESSION: &str = "sim-1-20261019T000000Z-0001";

fn fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../contract/fixtures/tiny-session")
        .join(SESSION)
}

/// A standalone node over `data`, with the id, owner bearer and credential
/// store the standalone node uses; returns its router and node id.
fn node(data: &Path) -> (Router, String) {
    let node = data.join("node");
    let work = node.join("work");
    let sessions = node.join("sessions");
    std::fs::create_dir_all(&sessions).unwrap();
    let id = standalone::node_id(&node).unwrap();
    let scheduler = Scheduler::new(
        JobStore::open(&node.join("jobs.db").to_string_lossy()).unwrap(),
        Arc::new(MockReconstructor),
        Arc::new(MockDetector),
    )
    .with_sessions_root(sessions.clone());
    let mut engine = Engine::new(scheduler, Cluster::new_master(id.clone()), 1);
    let runner: Arc<dyn ToolRunner> = Arc::new(StandInRunner::default());
    engine.set_runner(runner);
    let state: ApiState = Arc::new(Mutex::new(engine));
    let auth = Arc::new(standalone::auth(&node, &id, TOKEN.into()));
    let router = standalone::router(state, auth, "http://127.0.0.1:8092", work, sessions);
    (router, id)
}

enum As<'a> {
    Owner,
    Node(&'a str),
    Nobody,
}

async fn call(
    router: &Router,
    who: &As<'_>,
    method: &str,
    uri: &str,
    body: Vec<u8>,
) -> (StatusCode, serde_json::Value) {
    let mut b = Request::builder()
        .method(method)
        .uri(uri)
        .header("content-type", "application/json");
    b = match who {
        As::Owner => b.header("authorization", format!("Bearer {TOKEN}")),
        As::Node(c) => b.header(NODE_CREDENTIAL_HEADER, *c),
        As::Nobody => b,
    };
    let resp = router
        .clone()
        .oneshot(b.body(Body::from(body)).unwrap())
        .await
        .unwrap();
    let status = resp.status();
    let bytes = axum::body::to_bytes(resp.into_body(), usize::MAX)
        .await
        .unwrap();
    (
        status,
        serde_json::from_slice(&bytes).unwrap_or(serde_json::Value::Null),
    )
}

fn json(v: serde_json::Value) -> Vec<u8> {
    serde_json::to_vec(&v).unwrap()
}

async fn mint(router: &Router, peer: &str, lanes: Option<&[&str]>) -> serde_json::Value {
    let mut req = serde_json::json!({ "peer_device_id": peer });
    if let Some(l) = lanes {
        req["lanes"] = serde_json::json!(l);
    }
    let (s, m) = call(
        router,
        &As::Owner,
        "POST",
        "/api/compute/node-credentials",
        json(req),
    )
    .await;
    assert_eq!(s, StatusCode::CREATED, "{m}");
    assert!(
        m["workstation_node_id"]
            .as_str()
            .is_some_and(|id| id.starts_with("standalone-")),
        "{m}"
    );
    m
}

async fn upload(router: &Router, who: &As<'_>) -> StatusCode {
    let dir = fixture();
    let manifest = std::fs::read(dir.join("manifest.json")).unwrap();
    let (s, _) = call(
        router,
        who,
        "POST",
        &format!("/api/compute/sessions/{SESSION}"),
        manifest,
    )
    .await;
    if !s.is_success() {
        return s;
    }
    for rel in ["rig.json", "segments/000000.mcap", "checksums.b3"] {
        let bytes = std::fs::read(dir.join(rel)).unwrap();
        let uri = format!("/api/compute/sessions/{SESSION}/files/{rel}");
        let (s, _) = call(router, who, "PUT", &uri, bytes).await;
        assert!(s.is_success(), "put {rel}: {s}");
    }
    let uri = format!("/api/compute/sessions/{SESSION}/commit");
    let (s, body) = call(router, who, "POST", &uri, json(serde_json::json!({}))).await;
    assert_eq!(s, StatusCode::OK, "commit: {body}");
    s
}

#[tokio::test]
async fn an_issued_device_credential_uploads_and_reconstructs_on_the_standalone_node() {
    let dir = tempfile::tempdir().unwrap();
    let (router, id) = node(dir.path());

    // Nothing gets in without a credential.
    assert_eq!(upload(&router, &As::Nobody).await, StatusCode::UNAUTHORIZED);

    let device = mint(&router, "visor-1", None).await;
    assert_eq!(device["workstation_node_id"], id.as_str());
    let cred = device["credential"].as_str().unwrap().to_string();
    let other = mint(&router, "drone-2", None).await;
    let other_cred = other["credential"].as_str().unwrap().to_string();
    let upload_only = mint(&router, "visor-3", Some(&["sessions.upload"])).await;
    let upload_only = upload_only["credential"].as_str().unwrap().to_string();

    // The owner lists the issued credentials, never their secrets.
    let (s, listed) = call(
        &router,
        &As::Owner,
        "GET",
        "/api/compute/node-credentials",
        vec![],
    )
    .await;
    assert_eq!(s, StatusCode::OK);
    assert_eq!(listed["credentials"].as_array().unwrap().len(), 3);
    assert!(!listed.to_string().contains(&cred));

    // The device uploads its session and submits a reconstruction.
    assert_eq!(upload(&router, &As::Node(&cred)).await, StatusCode::OK);
    let submit = json(serde_json::json!({
        "kind": "reconstruct", "session_id": SESSION, "preset": "preview"
    }));
    let (s, sub) = call(
        &router,
        &As::Node(&cred),
        "POST",
        "/api/compute/jobs",
        submit.clone(),
    )
    .await;
    assert_eq!(s, StatusCode::CREATED, "{sub}");
    let job = sub["job_id"].as_str().unwrap().to_string();
    let job_uri = format!("/api/compute/jobs/{job}");

    // It reads its own job; another node's credential does not see it.
    let (s, view) = call(&router, &As::Node(&cred), "GET", &job_uri, vec![]).await;
    assert_eq!(s, StatusCode::OK, "{view}");
    let (s, _) = call(&router, &As::Node(&other_cred), "GET", &job_uri, vec![]).await;
    assert_eq!(s, StatusCode::NOT_FOUND);
    let outputs = format!("{job_uri}/outputs");
    let (s, _) = call(&router, &As::Node(&other_cred), "GET", &outputs, vec![]).await;
    assert_eq!(s, StatusCode::NOT_FOUND);
    let (s, _) = call(&router, &As::Owner, "GET", &job_uri, vec![]).await;
    assert_eq!(s, StatusCode::OK);

    // A credential issued only for uploads cannot submit jobs.
    let (s, _) = call(
        &router,
        &As::Node(&upload_only),
        "POST",
        "/api/compute/jobs",
        submit,
    )
    .await;
    assert_eq!(s, StatusCode::UNAUTHORIZED);

    // Owner routes stay the owner's: listing jobs, cancelling, issuing.
    for (m, uri) in [
        ("GET", "/api/compute/jobs".to_string()),
        ("POST", format!("{job_uri}/cancel")),
        ("GET", "/api/compute/node-credentials".to_string()),
        ("POST", "/api/compute/node-credentials".to_string()),
    ] {
        let body = json(serde_json::json!({ "peer_device_id": "x" }));
        let (s, _) = call(&router, &As::Node(&cred), m, &uri, body).await;
        assert_eq!(s, StatusCode::UNAUTHORIZED, "{m} {uri}");
    }

    // The issued store lives in the data directory: a restarted node admits
    // the device, and a revoked credential is refused.
    drop(router);
    let (router, restarted_id) = node(dir.path());
    assert_eq!(restarted_id, id);
    let (s, _) = call(&router, &As::Node(&cred), "GET", &job_uri, vec![]).await;
    assert_eq!(s, StatusCode::OK);
    let revoke = format!(
        "/api/compute/node-credentials/{}/revoke",
        device["id"].as_str().unwrap()
    );
    let (s, r) = call(&router, &As::Owner, "POST", &revoke, vec![]).await;
    assert_eq!((s, r["revoked"].as_bool()), (StatusCode::OK, Some(true)));
    let (s, _) = call(&router, &As::Node(&cred), "GET", &job_uri, vec![]).await;
    assert_eq!(s, StatusCode::UNAUTHORIZED);
}

#[tokio::test]
async fn two_standalone_nodes_have_their_own_ids_and_credentials() {
    let (a, b) = (tempfile::tempdir().unwrap(), tempfile::tempdir().unwrap());
    let (node_a, id_a) = node(a.path());
    let (node_b, id_b) = node(b.path());
    assert_ne!(id_a, id_b);
    let from_a = mint(&node_a, "visor-1", None).await;
    let from_b = mint(&node_b, "visor-1", None).await;
    assert_eq!(from_a["workstation_node_id"], id_a.as_str());
    assert_eq!(from_b["workstation_node_id"], id_b.as_str());
    // Node A's credential means nothing to node B.
    let cred_a = from_a["credential"].as_str().unwrap();
    let (s, _) = call(
        &node_b,
        &As::Node(cred_a),
        "POST",
        &format!("/api/compute/sessions/{SESSION}"),
        std::fs::read(fixture().join("manifest.json")).unwrap(),
    )
    .await;
    assert_eq!(s, StatusCode::UNAUTHORIZED);
}
