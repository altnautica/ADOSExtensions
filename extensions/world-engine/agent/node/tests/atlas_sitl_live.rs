//! Atlas SITL gate harness, live and multi-camera slices (G4, G5) and the
//! world model as a planning input; see `atlas_sitl.rs` for the ladder.
// These tests drive the unix-socket transport and the issued-credential store.
#![cfg(unix)]

use ados_capture_contract::Placement;
use world_engine_node::{
    submit_reconstruct_job, AtlasIngest, LiveReconstructConfig, RerunArchetype, RerunRecording,
};
use world_engine_protocol::atlas::{
    CaptureState, CaptureStatus, PoseSource, VioHealth, ATLAS_CAPTURE_STATE_TOPIC,
    ATLAS_KEYFRAME_TOPIC, PLUGIN_ATLAS_OCCUPANCY_TOPIC,
};
use world_engine_protocol::compute::ComputeJobState;
use world_engine_transport::{AtlasEvent, BearerLadder, LoopbackBearer, WorldBroadcaster};

#[path = "support/atlas_sitl.rs"]
mod atlas_support;

use atlas_support::{engine, keyframe_env};

/// G4: a live (in-flight) session keeps the world model fresh by REAL periodic
/// reconstruction. On the cadence the node snapshots the keyframes captured so
/// far and runs a real reconstruct (the mock backend stands in for Brush with no
/// GPU), yielding a fresh splat artifact mid-flight; while one cycle runs the
/// cadence coalesces (skip-while-running, never piles up), then resumes over the
/// grown set after it finishes — each cycle its own job/output the GCS polls.
#[tokio::test]
async fn g4_live_session_periodic_reconstruct() {
    let engine = engine();
    let store = engine.scheduler().store();
    let tmp = tempfile::tempdir().unwrap();

    // A tight cadence for the test: a cycle every 3 new keyframes, no time gate,
    // a 2-keyframe floor (production defaults are coarser: 30 keyframes / 20 s).
    let cfg = LiveReconstructConfig {
        enabled: true,
        every_keyframes: 3,
        interval_ms: i64::MAX,
        min_keyframes: 2,
    };
    let mut ingest = AtlasIngest::with_live_config(tmp.path(), cfg);

    // Three keyframes from one moving camera; a keyframe never bags.
    for kf in 0..3u64 {
        let ev = AtlasEvent::new(
            ATLAS_KEYFRAME_TOPIC,
            None,
            keyframe_env("sitl", "cam-front", kf).to_msgpack().unwrap(),
        );
        assert!(ingest.step(&ev, 100).unwrap().is_none());
    }

    // The cadence fires a first cycle: a real reconstruct over the snapshot of
    // the keyframes captured so far (NOT synthetic deltas, NOT incremental
    // training — a real, full reconstruct of the growing set).
    let mut cycle0 = ingest.due_reconstructs(200).unwrap();
    assert_eq!(
        cycle0.len(),
        1,
        "the cadence enqueues one periodic reconstruct"
    );
    let (dataset0, job0) = cycle0.remove(0);
    assert_eq!(dataset0.meta["live"], true);
    assert_eq!(dataset0.meta["keyframes"], 3);
    // The snapshot manifest the reconstruct reads is on disk (real input).
    let input_path = dataset0.meta["input_path"].as_str().unwrap();
    assert!(std::path::Path::new(input_path)
        .join("transforms.json")
        .exists());

    // Submit + run the worker: a fresh, real world-model artifact mid-flight.
    let job_id0 = submit_reconstruct_job(store, &dataset0, &job0).unwrap();
    let outcome = engine.tick(300).unwrap().expect("the live cycle ran");
    assert_eq!(outcome.job_id, job_id0);
    assert_eq!(outcome.state, ComputeJobState::Completed);
    assert!(
        outcome.outputs.iter().any(|o| o.kind == "splat"),
        "the live cycle yields a usable splat"
    );

    // Skip-while-running: more keyframes arrive, but the cadence coalesces while
    // the cycle's guard is held, so no second job piles up on top.
    for kf in 3..6u64 {
        let ev = AtlasEvent::new(
            ATLAS_KEYFRAME_TOPIC,
            None,
            keyframe_env("sitl", "cam-front", kf).to_msgpack().unwrap(),
        );
        ingest.step(&ev, 400).unwrap();
    }
    assert!(
        ingest.due_reconstructs(500).unwrap().is_empty(),
        "a cycle in flight blocks the next"
    );

    // The cycle's job is terminal (we ran it); the reconcile releases the guard
    // (the daemon reads the store; here we assert it then release directly).
    assert_eq!(
        ingest.in_flight_cycles(),
        vec![("sitl".to_string(), job_id0.clone())]
    );
    assert_eq!(
        store.get_job(&job_id0).unwrap().unwrap().state,
        ComputeJobState::Completed
    );
    ingest.note_cycle_finished("sitl");

    // Now the next cycle, over the GROWN keyframe set, is due and runs to a fresh
    // splat — its own job/output, distinct from the first.
    let mut cycle1 = ingest.due_reconstructs(600).unwrap();
    assert_eq!(cycle1.len(), 1);
    let (dataset1, job1) = cycle1.remove(0);
    assert_eq!(
        dataset1.meta["keyframes"], 6,
        "the cycle reconstructs the grown keyframe set"
    );
    let job_id1 = submit_reconstruct_job(store, &dataset1, &job1).unwrap();
    assert_ne!(
        job_id1, job_id0,
        "each periodic cycle is its own job/output the GCS polls"
    );
    let outcome = engine
        .tick(700)
        .unwrap()
        .expect("the second live cycle ran");
    assert_eq!(outcome.state, ComputeJobState::Completed);
    assert!(outcome.outputs.iter().any(|o| o.kind == "splat"));
}

/// G5: a multi-camera capture fuses into one world — N distinct cameras' frames
/// drain into a single bag dataset whose camera count is N and a single
/// reconstruct job (not one per camera), and the same frames map onto N distinct
/// camera subtrees in the recording, each with its intrinsics logged exactly
/// once (per-camera dedup at scale).
#[tokio::test]
async fn g5_multi_cam_fuses_into_one_world() {
    let engine = engine();
    const N: usize = 4;
    const FRAMES_PER_CAM: usize = 2;

    // The same keyframes feed both the ingest path and the viewer recording:
    // each camera appears in FRAMES_PER_CAM frames, so the per-camera dedup has
    // a repeat to drop.
    let mut keyframes = Vec::new();
    for round in 0..FRAMES_PER_CAM {
        for i in 0..N {
            // The capture's session matches the terminal bag ("multicam") so the
            // persisted per-camera frames finalize into the one fused dataset.
            keyframes.push(keyframe_env(
                "multicam",
                &format!("cam-{i}"),
                (round * N + i) as u64,
            ));
        }
    }

    let mut rec = RerunRecording::new();
    let (bearer, mut rx) = LoopbackBearer::channel();
    let ladder = BearerLadder::new(vec![Box::new(bearer)]);
    for kf in &keyframes {
        ladder
            .send(&AtlasEvent::new(
                ATLAS_KEYFRAME_TOPIC,
                None,
                kf.to_msgpack().unwrap(),
            ))
            .await
            .unwrap();
        rec.push_keyframe(kf);
    }
    // The terminal bagged state declares N enabled cameras (the fusion key).
    let status = CaptureStatus {
        session_id: "multicam".into(),
        state: CaptureState::Bagged,
        keyframes: keyframes.len() as u64,
        vio_health: VioHealth::Good,
        camera_count: N as u32,
        ingest_rate_hz: 9.0,
        capped: false,
        anchored: true,
        pose_tier: PoseSource::FcEkf,
        dropped_keyframes: 0,
    };
    ladder
        .send(&AtlasEvent::new(
            ATLAS_CAPTURE_STATE_TOPIC,
            None,
            status.to_msgpack().unwrap(),
        ))
        .await
        .unwrap();
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
    let job_id = job_id.expect("the multi-cam bag enqueued a reconstruct job");

    // One fused dataset carrying all N cameras + every drained frame.
    let job = store.get_job(&job_id).unwrap().expect("the enqueued job");
    let dataset_id = job
        .dataset_id
        .clone()
        .expect("the reconstruct job references a dataset");
    let dataset = store
        .get_dataset(&dataset_id)
        .unwrap()
        .expect("the multi-cam bag dataset");
    assert_eq!(
        dataset.meta["cameras"], N as u64,
        "the fused dataset declares all N cameras"
    );
    assert_eq!(
        dataset.meta["received_keyframes"],
        (N * FRAMES_PER_CAM) as u64,
        "every camera's frames drained into the one bag"
    );
    // A single fused reconstruct job, not one per camera.
    assert_eq!(
        store.count_in_state(ComputeJobState::Queued).unwrap(),
        1,
        "the multi-cam capture fuses into one reconstruct job"
    );

    // The recording carries N distinct camera subtrees, each Pinhole logged once.
    for i in 0..N {
        let cam = format!("world/camera/cam-{i}");
        assert!(
            rec.entries.iter().any(|e| e.entity_path == cam),
            "camera {i} has its own subtree"
        );
    }
    let mut pinhole_paths: Vec<&str> = rec
        .entries
        .iter()
        .filter(|e| matches!(e.archetype, RerunArchetype::Pinhole { .. }))
        .map(|e| e.entity_path.as_str())
        .collect();
    assert_eq!(
        pinhole_paths.len(),
        N,
        "one intrinsics per distinct camera (per-camera dedup at scale)"
    );
    pinhole_paths.sort();
    pinhole_paths.dedup();
    assert_eq!(
        pinhole_paths.len(),
        N,
        "the N intrinsics sit on N distinct camera paths"
    );
}

/// G3, the part that was missing: the world model is CONSUMABLE DATA, not only a
/// picture. A completed reconstruction's geometry becomes an occupancy/ESDF
/// descriptor a planner can act on, and it crosses the per-device shared-data
/// lane to a subscriber that never touched the compute node's filesystem.
///
/// Before this, `plugin.atlas.occupancy` was a constant in the protocol with no
/// publisher and no subscriber anywhere in the tree, so a repo-wide search for a
/// consumer of the world model found a viewer and nothing else.
#[tokio::test]
async fn the_world_model_is_consumable_as_a_planning_input_over_the_shared_data_lane() {
    use world_engine_node::{derive_descriptors, derive_occupancy, Output};
    use world_engine_protocol::atlas::{OccupancyDescriptor, OccupancyField};

    let dir = tempfile::tempdir().unwrap();
    // A real reconstruction artifact: three surface points in an L, as a .ply the
    // node's own parser reads.
    let ply = dir.path().join("cloud.ply");
    std::fs::write(
        &ply,
        "ply\nformat ascii 1.0\nelement vertex 3\nproperty float x\nproperty float y\n\
         property float z\nend_header\n0 0 0\n1 0 0\n0 1 0\n",
    )
    .unwrap();
    let outputs = vec![Output {
        id: "out-cloud".into(),
        job_id: "recon-g0".into(),
        kind: "pointcloud".into(),
        uri: format!("file://{}", ply.display()),
        meta: serde_json::Value::Null,
        created_ms: 0,
    }];

    // The descriptors are derived from the real artifact, stamped with the
    // capture session and the generation a viewer diffs on.
    let placement = Placement::identity(None);
    let set = derive_descriptors("g0", 4, &placement, &outputs, dir.path());
    let cloud = set.pointcloud.expect("a point-cloud descriptor");
    assert_eq!(cloud.session_id, "g0");
    assert_eq!(cloud.generation, 4);
    assert_eq!(cloud.point_count, 3, "the count is measured, not guessed");

    // And the planning input: an ESDF, not a voxel dump.
    let (occ, grid) = derive_occupancy(
        "g0",
        4,
        &placement,
        &outputs,
        dir.path(),
        "recon-g0",
        "http://node",
    )
    .unwrap()
    .expect("real geometry yields a planning input");
    assert_eq!(occ.field, OccupancyField::Esdf);
    assert!(occ.truncation_m > 0.0);
    assert_eq!(grid.voxel_count(), grid.distances.len());
    assert!(
        grid.distances.contains(&0.0),
        "the field reaches zero at the surface"
    );

    // It crosses the per-device shared-data lane and decodes on the other side —
    // the subscriber holds a planning input having read no files at all.
    let broadcaster = WorldBroadcaster::new(16);
    let mut consumer = broadcaster.subscribe();
    assert_eq!(
        broadcaster.publish(
            "drone-1",
            AtlasEvent::new(
                PLUGIN_ATLAS_OCCUPANCY_TOPIC,
                Some("drone-1".into()),
                occ.to_msgpack().unwrap(),
            ),
        ),
        1
    );
    let (device, event) = consumer.recv().await.unwrap();
    assert_eq!(device, "drone-1");
    assert_eq!(event.topic, PLUGIN_ATLAS_OCCUPANCY_TOPIC);
    let received = OccupancyDescriptor::from_msgpack(&event.payload).unwrap();
    assert_eq!(received, occ, "the planning input crosses the lane intact");
    assert_eq!(received.field, OccupancyField::Esdf);
    assert_eq!(received.generation, 4);
    assert!(
        received.url.is_some(),
        "the descriptor names where the consumer fetches the field"
    );
}
