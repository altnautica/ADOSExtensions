//! Atlas SITL gate harness: the mock-runnable slices of the G0→Gn kill-gate
//! ladder, run in-process end-to-end with no real GPU / camera / RF.
//!
//! These prove the BUILT pipeline pieces COMPOSE — a simulated capture's events
//! travel drone→compute over the real LAN bearer + event router, get ingested
//! into the job queue, and are reconstructed (mock) into an output — which the
//! per-crate unit tests do not exercise together. The real-GPU / real-camera /
//! real-RF criteria of each gate are bench items (the stop boundary, M15); a
//! WFB-relay or cloud lane substitutes the same `AtlasBearer` here.
// These tests drive the unix-socket transport and the issued-credential store.
#![cfg(unix)]

use ados_capture_contract::Placement;
use tokio::net::TcpListener;
use tokio::sync::mpsc;
use world_engine_node::{
    submit_reconstruct_job, AtlasIngest, JobRecord, RerunArchetype, RerunRecording,
};
use world_engine_protocol::atlas::{SplatDescriptor, PLUGIN_ATLAS_SPLAT_TOPIC};
use world_engine_protocol::compute::{
    ComputeJobKind, ComputeJobState, ComputeRole, SlaveDescriptor,
};
use world_engine_transport::{
    atlas_event_router, AtlasBearer, AtlasEvent, BearerKind, BearerLadder, LanHttpBearer,
    LoopbackBearer, WorldBroadcaster,
};

#[path = "support/atlas_sitl.rs"]
mod atlas_support;

use atlas_support::{bagged, deliver_for, engine, keyframe, keyframe_env};

/// G0: a simulated single-camera capture flows drone→compute over the LAN bearer,
/// is ingested into a dataset, reconstructed (mock), and yields a usable splat.
#[tokio::test]
async fn g0_single_camera_capture_reconstructs_to_a_splat_end_to_end() {
    let engine = engine();

    // The compute node's event receiver on an ephemeral port.
    let (tx, mut rx) = mpsc::channel(64);
    let listener = TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move {
        let _ = axum::serve(listener, atlas_event_router(tx)).await;
    });

    // The drone forwards keyframes + a bagged state over the LAN bearer.
    let bearer = LanHttpBearer::new(format!("http://{addr}"), None);
    const N: usize = 5;
    for i in 0..N {
        bearer.send(&keyframe(i)).await.unwrap();
    }
    bearer.send(&bagged(N as u64)).await.unwrap();

    // The compute node ingests each received event; the bagged state submits a job.
    let tmp = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::new(tmp.path());
    let mut job_id = None;
    for _ in 0..(N + 1) {
        let ev = rx.recv().await.expect("the bearer delivered the event");
        if let Some((dataset, job)) = ingest.step(&ev, 200).unwrap() {
            job_id =
                Some(submit_reconstruct_job(engine.scheduler().store(), &dataset, &job).unwrap());
        }
    }
    let job_id = job_id.expect("the bagged session submitted a reconstruct job");
    assert_eq!(
        ingest.keyframes_seen(),
        N as u64,
        "every keyframe reached the node"
    );

    // The reconstruct job runs (mock) and yields a splat.
    let outcome = engine.tick(300).unwrap().expect("a job was claimed + run");
    assert_eq!(outcome.job_id, job_id);
    assert_eq!(outcome.state, ComputeJobState::Completed);
    assert!(
        outcome.outputs.iter().any(|o| o.kind == "splat"),
        "G0 yields a usable splat"
    );
    let outputs = engine.scheduler().store().outputs_for_job(&job_id).unwrap();
    assert!(outputs.iter().any(|o| o.kind == "splat"));
}

/// Integrated send-path gate: a drone publishes a capture's events over the
/// in-process [`LoopbackBearer`] via the same [`BearerLadder`] the drone-side
/// Atlas forwarder uses, and the compute node's receiver drain loop — the shape
/// the node service's `atlas_receiver_loop` runs (`while let Some(ev) =
/// rx.recv()`, terminating on channel close) — drains the bearer's channel,
/// ingests each event, and enqueues a reconstruct job on the terminal `Bagged`
/// state. Proves forwarder → bearer → receiver → [`AtlasIngest::ingest`] →
/// enqueue composes in-process with no TCP, GPU, camera, or RF.
#[tokio::test]
async fn integrated_loopback_capture_drains_into_an_enqueued_reconstruct_job() {
    let engine = engine();

    // ── Drone side: a capture's events ride the bearer ladder; the in-process
    //    loopback bearer is the local-first rung the ladder picks. ──
    let (bearer, mut rx) = LoopbackBearer::channel();
    let ladder = BearerLadder::new(vec![Box::new(bearer)]);

    const N: usize = 5;
    for i in 0..N {
        assert_eq!(
            ladder.send(&keyframe(i)).await.unwrap(),
            BearerKind::Loopback,
            "the keyframe rode the in-process loopback bearer"
        );
    }
    assert_eq!(
        ladder.send(&bagged(N as u64)).await.unwrap(),
        BearerKind::Loopback,
        "the bagged capture-state rode the loopback bearer"
    );

    // Drop the lane so the drain loop terminates on channel close, exactly as the
    // daemon's receiver loop does on shutdown.
    drop(ladder);

    // ── Compute side: the receiver drain loop. One `AtlasIngest` for the session
    //    counts keyframes and, on the `Bagged` state, submits the reconstruct job
    //    the workers pick up (mirrors `atlas_receiver_loop`, run inline against
    //    the engine's store). ──
    let store = engine.scheduler().store();
    let tmp = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::new(tmp.path());
    let mut enqueued_job = None;
    while let Some(event) = rx.recv().await {
        if let Some((dataset, job)) = ingest.step(&event, 200).unwrap() {
            enqueued_job = Some(submit_reconstruct_job(store, &dataset, &job).unwrap());
        }
    }

    // The bagged session enqueued exactly the reconstruct job, queued for a worker.
    let job_id = enqueued_job.expect("the bagged capture enqueued a reconstruct job");
    assert_eq!(
        ingest.keyframes_seen(),
        N as u64,
        "every keyframe drained into the node"
    );
    let job = store
        .get_job(&job_id)
        .unwrap()
        .expect("the enqueued job is in the store");
    assert_eq!(job.kind, ComputeJobKind::Reconstruct);
    assert_eq!(
        job.state,
        ComputeJobState::Queued,
        "the reconstruct job is queued for a worker"
    );
    assert_eq!(
        store.count_in_state(ComputeJobState::Queued).unwrap(),
        1,
        "the bagged session enqueued exactly one reconstruct job"
    );
}

/// Perception-offload gate: an NPU-less drone offloads a frame; the node runs the
/// (mock) detector and returns a detection.
#[tokio::test]
async fn perception_offload_runs_the_detector_and_returns_a_detection() {
    let engine = engine();
    engine
        .scheduler()
        .store()
        .submit_job(&JobRecord {
            id: "off-1".into(),
            kind: ComputeJobKind::PerceptionOffload,
            dataset_id: None,
            state: ComputeJobState::Queued,
            progress: 0.0,
            params: serde_json::json!({
                "frame": { "camera_id": "front", "width": 640, "height": 640, "ts_ms": 100 }
            }),
            result_ref: None,
            error: None,
            created_ms: 100,
            updated_ms: 100,
        })
        .unwrap();

    let outcome = engine.tick(200).unwrap().expect("offload job claimed");
    assert_eq!(outcome.state, ComputeJobState::Completed);
    assert!(
        !outcome.detections.is_empty(),
        "the perception offload returns at least one detection"
    );
}

/// Cluster gate: a master reports its own role and aggregates a registered slave's
/// idle capacity (the master/slave compute cluster, single-master v1).
#[test]
fn cluster_master_aggregates_a_registered_slave() {
    let mut engine = engine();
    assert_eq!(engine.heartbeat().unwrap().role, ComputeRole::Master);

    let before = engine.heartbeat().unwrap().cluster.aggregate_workers_idle;
    engine.cluster_mut().register_slave(SlaveDescriptor {
        node_id: "gpu-b".into(),
        accelerators: vec!["cuda:0".into()],
        workers_idle: 4,
        queue_depth: 0,
    });
    let hb = engine.heartbeat().unwrap();
    assert_eq!(hb.cluster.slaves.len(), 1);
    assert_eq!(
        hb.cluster.aggregate_workers_idle,
        before + 4,
        "the slave's idle workers fold into the cluster capacity"
    );
}

/// G1: the Rerun recording maps a multi-keyframe single-camera capture + the
/// reconstructed splat onto the entity tree the GCS viewer renders — the camera
/// subtree, the IMU scalars, a once-logged camera intrinsics across the run, and
/// the verbatim wire discriminators (`Transform3D`, `SplatSlab`).
#[tokio::test]
async fn g1_rerun_recording_maps_keyframes_for_the_gcs_viewer() {
    const N: usize = 4;
    let mut rec = RerunRecording::new();
    for i in 0..N {
        rec.push_keyframe(&keyframe_env("g1", "cam-front", i as u64));
    }
    rec.push_splat(
        &SplatDescriptor {
            session_id: "g0".into(),
            generation: 1,
            placement: Placement::identity(None),
            manifest_url: None,
            lod_levels: 0,
            gaussian_count: 4800,
            step: 200,
            url: Some("spz://sitl".into()),
            handle: None,
        },
        2000,
    );

    let paths: Vec<&str> = rec.entries.iter().map(|e| e.entity_path.as_str()).collect();
    assert!(
        paths.contains(&"world/camera/cam-front"),
        "the camera has a subtree the viewer renders"
    );
    assert!(
        paths.contains(&"world/camera/cam-front/rgb"),
        "the camera's image rides world/camera/<id>/rgb"
    );
    assert!(
        paths.contains(&"world/imu/accel/z"),
        "the IMU window is logged as per-axis scalars"
    );

    // The camera's static intrinsics is logged exactly once across the N
    // keyframes (the viewer needs the Pinhole once, not per frame)...
    let pinholes = rec
        .entries
        .iter()
        .filter(|e| matches!(e.archetype, RerunArchetype::Pinhole { .. }))
        .count();
    assert_eq!(
        pinholes, 1,
        "the camera intrinsics is logged once across the whole run"
    );
    // ...while every keyframe contributes its own pose transform.
    let transforms = rec
        .entries
        .iter()
        .filter(|e| matches!(e.archetype, RerunArchetype::Transform3D { .. }))
        .count();
    assert_eq!(transforms, N, "every keyframe contributes a pose transform");

    // The serialized manifest carries the verbatim Rerun archetype names the
    // GCS viewer maps (not snake_case digit-splits).
    let json = rec.to_json().unwrap();
    assert!(
        json.contains("\"Transform3D\""),
        "pose transform discriminator"
    );
    assert!(json.contains("\"SplatSlab\""), "splat slab discriminator");
    assert!(!json.contains("transform3_d"), "no mangled discriminator");
}

/// G2: a captured bag rides a bearer into the receiver/ingest, drains into one
/// reconstruct job, and — one step past the integrated-loopback gate, which
/// stops at "queued" — the worker runs it to a delivered splat output.
#[tokio::test]
async fn g2_bag_pipeline_reconstructs_to_a_delivered_output() {
    let engine = engine();

    // The PostFlightBulk bearer is not implemented; the in-process loopback
    // bearer stands in for the post-flight-bulk LAN lane here (identical
    // AtlasEvent contract, no GPU / camera / RF).
    let (bearer, mut rx) = LoopbackBearer::channel();
    let ladder = BearerLadder::new(vec![Box::new(bearer)]);

    const N: usize = 6;
    for i in 0..N {
        ladder.send(&keyframe(i)).await.unwrap();
    }
    ladder.send(&bagged(N as u64)).await.unwrap();
    drop(ladder);

    let store = engine.scheduler().store();
    let tmp = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::new(tmp.path());
    let mut job_id = None;
    while let Some(ev) = rx.recv().await {
        if let Some((dataset, job)) = ingest.step(&ev, 200).unwrap() {
            job_id = Some(submit_reconstruct_job(store, &dataset, &job).unwrap());
        }
    }
    let job_id = job_id.expect("the bagged capture enqueued a reconstruct job");

    // The dataset is a bag carrying the camera count + the received-keyframe
    // proof (the drone's send is fire-and-forget, so only decoded frames count).
    let job = store.get_job(&job_id).unwrap().expect("the enqueued job");
    let dataset_id = job
        .dataset_id
        .clone()
        .expect("the reconstruct job references a dataset");
    let dataset = store
        .get_dataset(&dataset_id)
        .unwrap()
        .expect("the bag dataset was inserted");
    assert_eq!(dataset.kind, "bag");
    assert_eq!(dataset.meta["cameras"], 1);
    assert_eq!(dataset.meta["received_keyframes"], N as u64);

    // Exactly one fused reconstruct job, queued for a worker.
    assert_eq!(
        store.count_in_state(ComputeJobState::Queued).unwrap(),
        1,
        "the bag enqueued exactly one reconstruct job"
    );

    // The worker runs it to a delivered splat output (past "queued").
    let outcome = engine.tick(300).unwrap().expect("the queued job ran");
    assert_eq!(outcome.job_id, job_id);
    assert_eq!(outcome.state, ComputeJobState::Completed);
    let outputs = store.outputs_for_job(&job_id).unwrap();
    assert!(
        outputs.iter().any(|o| o.kind == "splat"),
        "the bag reconstructs to a delivered splat output"
    );
}

/// G3: the shared-data descriptor lane isolates per device — one drone's world
/// model never crosses into another drone's plugin view — and an NPU-less
/// drone's perception offload runs the (mock) detector and returns a detection.
#[tokio::test]
async fn g3_plugin_data_share_isolates_per_device_and_offloads() {
    let broadcaster = WorldBroadcaster::new(16);
    let mut rx_one = broadcaster.subscribe();
    let mut rx_two = broadcaster.subscribe();
    assert_eq!(broadcaster.subscriber_count(), 2);

    // One drone's splat generation is published for its device only.
    let splat = AtlasEvent::new(PLUGIN_ATLAS_SPLAT_TOPIC, None, vec![1, 2, 3]);
    broadcaster.publish("drone-1", splat);

    // drone-1's plugin view receives it; drone-2's view filters it out.
    let for_one = deliver_for(&mut rx_one, "drone-1").await;
    let for_two = deliver_for(&mut rx_two, "drone-2").await;
    assert_eq!(
        for_one.as_ref().map(|e| e.topic.as_str()),
        Some(PLUGIN_ATLAS_SPLAT_TOPIC),
        "drone-1's plugin view receives its own world descriptor"
    );
    assert!(
        for_two.is_none(),
        "drone-2's plugin view never sees drone-1's world descriptor (device isolation)"
    );

    // The NPU-less offload path: the node runs the detector and returns a result.
    let engine = engine();
    engine
        .scheduler()
        .store()
        .submit_job(&JobRecord {
            id: "off-iso".into(),
            kind: ComputeJobKind::PerceptionOffload,
            dataset_id: None,
            state: ComputeJobState::Queued,
            progress: 0.0,
            params: serde_json::json!({
                "frame": { "camera_id": "front", "width": 640, "height": 640, "ts_ms": 100 }
            }),
            result_ref: None,
            error: None,
            created_ms: 100,
            updated_ms: 100,
        })
        .unwrap();
    let outcome = engine.tick(200).unwrap().expect("offload job claimed");
    assert_eq!(outcome.state, ComputeJobState::Completed);
    assert!(
        !outcome.detections.is_empty(),
        "the perception offload returns at least one detection"
    );
}
