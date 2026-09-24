//! SITL offload gate (no hardware): the whole perception-offload return path,
//! wired in-process, with a mock detector.
//!
//! frames → node streaming session → detection broadcaster → WS → drone
//! subscriber → return bridge (ados-offload safety gate) → the drone's
//! detection publisher.
//!
//! Synthetic frames go in on the node side and boxes come out of the
//! publisher the drone half hands to the plugin host, with the freshness + lock
//! safety gate exercised. No GPU, no camera, no RTSP, no RF.

use std::net::SocketAddr;
use std::sync::Arc;
use std::time::Duration;

use ados_offload::OffloadMode;
use ados_protocol::framebus::DetectionBatch;
use ados_protocol::node_credential::NodeLane;
use tokio::sync::{mpsc, Mutex, Notify};
use world_engine_node::{
    build_router, offload_ws_path, offload_ws_router, pump_to_broadcaster, run_offload_session,
    Cluster, ComputeAuth, DetectionBroadcaster, Engine, JobStore, MockDetector, MockReconstructor,
    NodeCredentialStore, Scheduler, SessionProgress, SessionSpec, VecFrameStream,
};
use world_engine_protocol::offload::{Detection, OffloadDetectionBatch};
use world_engine_transport::{
    run_offload_orchestrator, stream_offload_detections, DetectionPublisher, NodeEndpoint,
    OffloadReturnBridge, OrchestratorConfig,
};

fn now_ms() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0)
}

/// Spin the node's detection WS return-stream server on a loopback port.
async fn spawn_ws_server(broadcaster: Arc<DetectionBroadcaster>) -> std::net::SocketAddr {
    let app = offload_ws_router(broadcaster);
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });
    addr
}

/// The drone's detection publisher, standing in for the plugin host: every
/// batch the return bridge publishes lands on the returned receiver.
struct CollectingPublisher(mpsc::UnboundedSender<DetectionBatch>);

#[async_trait::async_trait]
impl DetectionPublisher for CollectingPublisher {
    async fn publish(&self, batch: &DetectionBatch) -> anyhow::Result<()> {
        self.0.send(batch.clone())?;
        Ok(())
    }
}

fn collecting_publisher() -> (
    Arc<CollectingPublisher>,
    mpsc::UnboundedReceiver<DetectionBatch>,
) {
    let (tx, rx) = mpsc::unbounded_channel();
    (Arc::new(CollectingPublisher(tx)), rx)
}

#[tokio::test]
async fn offload_frames_reach_the_drone_detection_publisher_end_to_end() {
    // --- node side: session -> broadcaster -> WS ------------------------------
    let broadcaster = Arc::new(DetectionBroadcaster::new(64));
    let ws_addr = spawn_ws_server(broadcaster.clone()).await;

    // The streaming session pumps its detection channel into the broadcaster.
    let (sess_tx, sess_rx) = tokio::sync::mpsc::channel::<OffloadDetectionBatch>(64);
    let bpump = broadcaster.clone();
    tokio::spawn(pump_to_broadcaster("sess-1".to_string(), sess_rx, bpump));

    // --- drone side: subscribe over WS ---------------------------------------
    let (publisher, mut bus) = collecting_publisher();

    let (det_tx, mut det_rx) = tokio::sync::mpsc::channel::<OffloadDetectionBatch>(64);
    let ws_url = format!("ws://{ws_addr}{}", offload_ws_path("sess-1"));
    let sub_cancel = Arc::new(Notify::new());
    let sc = sub_cancel.clone();
    let subscriber = tokio::spawn(async move {
        stream_offload_detections(&ws_url, None, det_tx, sc)
            .await
            .unwrap();
    });

    // The return bridge: gate + convert + hand to the drone's publisher.
    let mut bridge = OffloadReturnBridge::new(OffloadMode::VisionOnly, 1_000, 1_000, "offload");
    let bridge_task = tokio::spawn(async move {
        let mut published: Vec<DetectionBatch> = Vec::new();
        while let Some(batch) = det_rx.recv().await {
            let (db, _status) = bridge.ingest(&batch, now_ms());
            // Hand the batch to the drone's detection publisher.
            publisher.publish(&db).await.unwrap();
            published.push(db);
            if published.len() == 3 {
                break;
            }
        }
        published
    });

    // Wait for the WS subscriber to register before the node starts emitting, so
    // no batch is dropped in the connect window.
    for _ in 0..200 {
        if broadcaster.subscriber_count() == 1 {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }

    // --- run the node session: 3 synthetic frames ----------------------------
    let stream = VecFrameStream::solid("front", 64, 48, 3, 5000);
    let detector: Arc<dyn world_engine_node::Detector> = Arc::new(MockDetector);
    let session_cancel = Arc::new(Notify::new());
    let sc2 = session_cancel.clone();
    let session = tokio::spawn(async move {
        run_offload_session(
            "sess-1",
            "front",
            stream,
            detector,
            sess_tx,
            sc2,
            SessionProgress::detached(),
        )
        .await;
    });

    // --- assert: 3 batches reached the drone's detection publisher -----------
    let mut received = Vec::new();
    for _ in 0..3 {
        let batch = tokio::time::timeout(Duration::from_secs(10), bus.recv())
            .await
            .expect("a published detection batch within 10s")
            .expect("publisher not closed");
        received.push(batch);
    }

    assert_eq!(
        received.len(),
        3,
        "3 offloaded frames -> 3 published batches"
    );
    for (i, b) in received.iter().enumerate() {
        assert_eq!(b.model_id, "offload");
        assert_eq!(b.camera_id, "front");
        assert_eq!(
            b.frame_id, i as u64,
            "the session seq is the frame id on the bus"
        );
        assert_eq!(b.detections.len(), 1, "the mock returns one box per frame");
        // The normalized [0.4,0.4,0.2,0.2] mock box denormalized to 64x48.
        let bb = b.detections[0].bbox.as_ref().unwrap();
        assert!(
            (bb.x - 0.4 * 64.0).abs() < 1e-3,
            "x denormalized to frame width"
        );
        assert!(
            (bb.y - 0.4 * 48.0).abs() < 1e-3,
            "y denormalized to frame height"
        );
    }

    let published = bridge_task.await.unwrap();
    assert_eq!(published.len(), 3);

    // Clean shutdown.
    session.await.unwrap();
    sub_cancel.notify_waiters();
    let _ = subscriber.await;
    let _ = session_cancel; // (kept for symmetry; the vec stream ends on its own)
}

/// A mock compute node: the REAL compute job API (so the orchestrator's submit
/// hits `POST /api/compute/jobs` and gets a 201) plus the REAL per-session
/// detection WS (so the orchestrator subscribes and drains real batches), both on
/// one loopback listener — the node's own streaming session is stood in for by
/// the test publishing batches onto the broadcaster. Only discovery + the RTSP
/// frame source are mocked; the submit, the WS, and the return bridge are real.
/// Returns the node address, its engine, the credential the node issued the
/// drone for the submit lane, and the credential store's directory.
async fn spawn_mock_node(
    broadcaster: Arc<DetectionBroadcaster>,
) -> (SocketAddr, Arc<Mutex<Engine>>, String, tempfile::TempDir) {
    let store = JobStore::open_in_memory().unwrap();
    let scheduler = Scheduler::new(store, Arc::new(MockReconstructor), Arc::new(MockDetector));
    let engine = Arc::new(Mutex::new(Engine::new(
        scheduler,
        Cluster::new_master("mock-node"),
        1,
    )));
    // The drone submits over TCP, so it needs the credential the node issued
    // it for the submit lane.
    let dir = tempfile::tempdir().unwrap();
    let auth = Arc::new(ComputeAuth::new(NodeCredentialStore::open(
        dir.path().join("creds.json"),
        "mock-node",
    )));
    let credential = auth
        .credentials
        .mint("drone-1", &[NodeLane::from_static("jobs.submit")], now_ms())
        .unwrap()
        .credential;
    let app = build_router(engine.clone(), auth).merge(offload_ws_router(broadcaster));
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        axum::serve(listener, app).await.unwrap();
    });
    (addr, engine, credential, dir)
}

#[tokio::test]
async fn the_orchestrator_drives_the_offload_loop_end_to_end() {
    // --- mock node: real compute API (submit) + real WS (return stream) --------
    let broadcaster = Arc::new(DetectionBroadcaster::new(64));
    let (node_addr, _engine, credential, _dir) = spawn_mock_node(broadcaster.clone()).await;

    // --- drone side: the detection publisher the orchestrator hands batches to --
    let (publisher, mut bus) = collecting_publisher();

    // --- run the orchestrator: it submits the session job, then opens the WS ---
    let cancel = Arc::new(Notify::new());
    let cfg = OrchestratorConfig {
        session_id: "sess-orch".into(),
        camera_id: "front".into(),
        rtsp_url: "rtsp://drone.local:8554/main".into(),
        width: 64,
        height: 48,
        mode: OffloadMode::VisionOnly,
        target_budget_ms: 5_000,
        pose_budget_ms: 5_000,
        model_id: "offload".into(),
        publisher,
        detection_tee: None,
    };
    // The node is resolved up front (the SITL seam skips mDNS and points the
    // orchestrator at the mock node's base URL). Submit + WS + bridge are real.
    let node = NodeEndpoint {
        base_url: format!("http://{node_addr}"),
        credential: Some(credential),
    };
    let orch_cancel = cancel.clone();
    let orchestrator = tokio::spawn(async move {
        run_offload_orchestrator(cfg, node, orch_cancel)
            .await
            .unwrap();
    });

    // Wait for the orchestrator's WS subscriber to register (after it has
    // submitted the job + connected) before the node emits, so no batch is
    // dropped in the connect window.
    for _ in 0..400 {
        if broadcaster.subscriber_count() == 1 {
            break;
        }
        tokio::time::sleep(Duration::from_millis(5)).await;
    }
    assert_eq!(
        broadcaster.subscriber_count(),
        1,
        "the orchestrator submitted + subscribed to the node WS"
    );

    // The submit landed a real job on the node (proving step 2 ran, not just the
    // WS): the job id is node-minted, so find it by the session spec in its params.
    {
        let e = _engine.lock().await;
        let jobs = e.scheduler().store().list_jobs().unwrap();
        let session_job = jobs.iter().find(|j| {
            SessionSpec::from_job_params(&j.params)
                .map(|s| s.id == "sess-orch")
                .unwrap_or(false)
        });
        assert!(
            session_job.is_some(),
            "the orchestrator submitted a streaming-session job for this session"
        );
    }

    // --- the node's session emits 3 detection batches for this session --------
    for seq in 0..3u64 {
        broadcaster.publish(OffloadDetectionBatch::new(
            "sess-orch",
            "front",
            seq,
            5000 + seq as i64,
            64,
            48,
            vec![Detection {
                bbox: [0.4, 0.4, 0.2, 0.2],
                class: "person".into(),
                confidence: 0.9,
                track_id: Some(1),
            }],
        ));
    }

    // --- assert: 3 batches reached the drone's detection publisher ------------
    let mut received = Vec::new();
    for _ in 0..3 {
        let batch = tokio::time::timeout(Duration::from_secs(10), bus.recv())
            .await
            .expect("a published detection batch within 10s")
            .expect("publisher not closed");
        received.push(batch);
    }
    assert_eq!(received.len(), 3, "3 node batches -> 3 published batches");
    for (i, b) in received.iter().enumerate() {
        assert_eq!(b.model_id, "offload");
        assert_eq!(b.camera_id, "front");
        assert_eq!(
            b.frame_id, i as u64,
            "the batch seq is the frame id on the bus"
        );
        assert_eq!(b.detections.len(), 1);
        // The normalized [0.4,0.4,0.2,0.2] box denormalized to 64x48 by the bridge.
        let bb = b.detections[0].bbox.as_ref().unwrap();
        assert!(
            (bb.x - 0.4 * 64.0).abs() < 1e-3,
            "x denormalized to frame width"
        );
        assert!(
            (bb.y - 0.4 * 48.0).abs() < 1e-3,
            "y denormalized to frame height"
        );
        assert!((bb.width - 0.2 * 64.0).abs() < 1e-3, "w denormalized");
    }

    // Clean shutdown: cancel wakes the WS subscriber + the drain loop; the
    // orchestrator returns once both stop.
    cancel.notify_waiters();
    let _ = tokio::time::timeout(Duration::from_secs(5), orchestrator)
        .await
        .expect("the orchestrator returns on cancel");
}
