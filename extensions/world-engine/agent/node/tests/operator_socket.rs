//! The operator socket (`http.sock`) serves the node router as the owner:
//! ados-control authenticated the operator before proxying, so a request with
//! no credential reaches an owner-only route there, while the same request over
//! the node's TCP listener (even from loopback) is refused.

use std::path::Path;
use std::sync::Arc;

use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};
use tokio::sync::Mutex;
use world_engine_node::{
    build_router, lane_router, operator_socket_router, AtlasLanes, Cluster, ComputeAuth,
    ComputeGpu, ComputeHeartbeatSidecar, DetectionBroadcaster, Engine, JobStore, LaneRoutes,
    LatestComputeStatus, MockDetector, MockReconstructor, NodeCredentialStore, Scheduler,
};
use world_engine_transport::{serve_unix, WorldBroadcaster};

/// One raw HTTP/1.1 exchange; returns the status code and the response head.
async fn exchange<S: AsyncRead + AsyncWrite + Unpin>(
    mut stream: S,
    request: &str,
) -> (u16, String) {
    stream.write_all(request.as_bytes()).await.unwrap();
    let mut buf = Vec::new();
    stream.read_to_end(&mut buf).await.unwrap();
    let text = String::from_utf8_lossy(&buf).into_owned();
    let head = text
        .split("\r\n\r\n")
        .next()
        .unwrap_or_default()
        .to_string();
    let status = head
        .split_whitespace()
        .nth(1)
        .and_then(|s| s.parse().ok())
        .unwrap();
    (status, head)
}

fn get(path: &str) -> String {
    format!("GET {path} HTTP/1.1\r\nHost: node\r\nConnection: close\r\n\r\n")
}

#[tokio::test]
async fn the_operator_socket_is_the_owner_while_tcp_without_a_key_is_refused() {
    let dir = tempfile::tempdir().unwrap();
    let work = dir.path().join("work");
    std::fs::create_dir_all(work.join("job-1")).unwrap();
    std::fs::write(work.join("job-1/cloud.ply"), b"ply-bytes").unwrap();
    let auth = Arc::new(ComputeAuth::new(NodeCredentialStore::open(
        dir.path().join("creds.json"),
        "node-a",
    )));

    let store = JobStore::open_in_memory().unwrap();
    let scheduler = Scheduler::new(store, Arc::new(MockReconstructor), Arc::new(MockDetector));
    let engine = Arc::new(Mutex::new(Engine::new(
        scheduler,
        Cluster::new_master("node-a"),
        1,
    )));
    let (events, _events_rx) = tokio::sync::mpsc::channel(8);
    let shared = build_router(engine.clone(), auth.clone()).merge(lane_router(
        auth,
        LaneRoutes {
            work_root: work,
            offload: Arc::new(DetectionBroadcaster::new(8)),
            atlas: AtlasLanes {
                events,
                world: Arc::new(WorldBroadcaster::new(8)),
            },
        },
    ));

    // TCP, served the way the node serves its drones.
    let tcp = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let tcp_addr = tcp.local_addr().unwrap();
    let tcp_app = shared.clone();
    tokio::spawn(async move {
        axum::serve(tcp, tcp_app).await.unwrap();
    });

    // The operator socket.
    let sock = dir.path().join("http.sock");
    let listener = ados_sdk::http::bind(&sock).unwrap();
    let latest = Arc::new(LatestComputeStatus::new());
    let socket_app = operator_socket_router(shared, latest.clone());
    tokio::spawn(serve_unix(listener, socket_app, std::future::pending()));

    let over_tcp = |path: &str| {
        let req = get(path);
        async move {
            exchange(
                tokio::net::TcpStream::connect(tcp_addr).await.unwrap(),
                &req,
            )
            .await
        }
    };
    let over_sock = |path: &str| {
        let req = get(path);
        let sock: &Path = &sock;
        let sock = sock.to_path_buf();
        async move { exchange(tokio::net::UnixStream::connect(sock).await.unwrap(), &req).await }
    };

    // An owner-only route: issuing credentials is never a node lane.
    assert_eq!(over_tcp("/api/compute/node-credentials").await.0, 401);
    assert_eq!(over_sock("/api/compute/node-credentials").await.0, 200);

    // A lane route: the artifact, with its Content-Length, over the socket only.
    assert_eq!(over_tcp("/artifacts/job-1/cloud.ply").await.0, 401);
    let (status, head) = over_sock("/artifacts/job-1/cloud.ply").await;
    assert_eq!(status, 200);
    assert!(
        head.to_ascii_lowercase().contains("content-length: 9"),
        "artifact carries its length: {head}"
    );

    // The compute status lives on the socket only: 404 until the first
    // heartbeat, then served; never mounted on TCP.
    assert_eq!(over_sock("/compute/status").await.0, 404);
    let hb = engine.lock().await.heartbeat().unwrap();
    latest.set(ComputeHeartbeatSidecar::from_heartbeat(
        &hb,
        ComputeGpu::default(),
        1,
    ));
    assert_eq!(over_sock("/compute/status").await.0, 200);
    assert_eq!(over_tcp("/compute/status").await.0, 404);
}
