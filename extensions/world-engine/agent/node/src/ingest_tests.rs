use super::*;
use ados_capture_contract::{
    CameraModel, CameraModelModel, GeoAnchor, GeoAnchorDatum, GeoAnchorSource,
};
use world_engine_protocol::atlas::{
    CameraRole, ImageEncoding, KeyframeFlags, KeyframeImage, KeyframeTier, Pose, PoseSource,
    VioHealth,
};

fn real_keyframe_event(session: &str, kf_id: u64) -> AtlasEvent {
    let kf = KeyframeEnvelope {
        session_id: session.into(),
        kf_id,
        ts_unix_ms: 1_700_000_000_000 + kf_id as i64,
        camera_id: "front".into(),
        camera_role: CameraRole::Primary,
        tier: KeyframeTier::Full,
        image: KeyframeImage {
            encoding: ImageEncoding::Jpeg,
            width: 1280,
            height: 720,
            bytes: vec![0xFF, 0xD8, 0xFF, kf_id as u8],
        },
        camera: CameraModel {
            model: CameraModelModel::Pinhole,
            width: 1280,
            height: 720,
            fx: 900.0,
            fy: 900.0,
            cx: 640.0,
            cy: 360.0,
            dist: vec![],
            calibrated: true,
            readout_ns: 0,
        },
        pose: Pose {
            r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            t: [0.0, 0.0, 0.0],
            cov: None,
        },
        pose_cov: vec![0.0009; world_engine_protocol::atlas::POSE_COV_LEN],
        pose_source: PoseSource::FcEkf,
        time: world_engine_protocol::atlas::TimeAlignment::unmeasured(),
        anchor: Some(GeoAnchor {
            lat_deg: 12.97,
            lon_deg: 77.59,
            h_ellipsoid_m: 920.0,
            geoid_undulation_m: 0.0,
            datum: GeoAnchorDatum::Wgs84,
            source: GeoAnchorSource::GnssMean,
        }),
        imu_window: Vec::new(),
        flags: KeyframeFlags::default(),
    };
    AtlasEvent::new(ATLAS_KEYFRAME_TOPIC, None, kf.to_msgpack().unwrap())
}

/// A keyframe event stamped with a capturing-drone id (the egress shape).
fn keyframe_event_from(session: &str, kf_id: u64, device: &str) -> AtlasEvent {
    let mut ev = real_keyframe_event(session, kf_id);
    ev.device_id = Some(device.to_string());
    ev
}

fn capture_state(session: &str, state: CaptureState, keyframes: u64) -> AtlasEvent {
    let status = CaptureStatus {
        session_id: session.into(),
        state,
        keyframes,
        vio_health: VioHealth::Good,
        camera_count: 1,
        ingest_rate_hz: 9.0,
        capped: false,
        anchored: true,
        pose_tier: PoseSource::FcEkf,
        dropped_keyframes: 0,
    };
    AtlasEvent::new(
        ATLAS_CAPTURE_STATE_TOPIC,
        None,
        status.to_msgpack().unwrap(),
    )
}

/// A capture-state event stamped with a capturing-drone id (the egress shape).
fn capture_state_from(
    session: &str,
    state: CaptureState,
    keyframes: u64,
    device: &str,
) -> AtlasEvent {
    let mut ev = capture_state(session, state, keyframes);
    ev.device_id = Some(device.to_string());
    ev
}

#[test]
fn each_dataset_carries_its_own_sessions_accounting() {
    let dir = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::new(dir.path());
    for kf in 0..3 {
        ingest.step(&real_keyframe_event("sA", kf), 1_000).unwrap();
    }
    ingest.step(&real_keyframe_event("sB", 0), 1_000).unwrap();
    let (dataset, _) = ingest
        .step(&capture_state("sB", CaptureState::Bagged, 1), 2_000)
        .unwrap()
        .expect("sB bags");
    // sA's three keyframes must not leak into sB's dataset.
    assert_eq!(dataset.meta["received_keyframes"], 1);
}

#[test]
fn a_session_whose_bag_never_arrives_is_finalized_after_the_idle_window() {
    let dir = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::new(dir.path());
    ingest
        .step(&real_keyframe_event("sLost", 0), 1_000)
        .unwrap();
    ingest
        .step(&real_keyframe_event("sLost", 1), 1_100)
        .unwrap();
    assert!(ingest
        .idle_bags(1_100 + SESSION_IDLE_MS - 1)
        .unwrap()
        .is_empty());
    let bags = ingest.idle_bags(1_100 + SESSION_IDLE_MS).unwrap();
    assert_eq!(bags.len(), 1);
    assert_eq!(bags[0].1.id, "recon-sLost");
    assert_eq!(bags[0].0.meta["keyframes"], 2);
    // The session is released: a second sweep yields nothing.
    assert!(ingest.idle_bags(i64::MAX).unwrap().is_empty());
}

#[test]
fn keyframes_persist_then_bagged_yields_a_job_with_input_path() {
    let dir = tempfile::tempdir().unwrap();
    let store = JobStore::open(":memory:").unwrap();
    let mut ingest = AtlasIngest::new(dir.path());

    for kf_id in 0..3 {
        assert_eq!(
            ingest
                .step(&real_keyframe_event("sess1", kf_id), 100)
                .unwrap(),
            None
        );
    }
    // A non-terminal state does not bag.
    assert_eq!(
        ingest
            .step(&capture_state("sess1", CaptureState::Capturing, 3), 100)
            .unwrap(),
        None
    );

    // Bagged yields the dataset + job; the caller submits it.
    let (dataset, job) = ingest
        .step(&capture_state("sess1", CaptureState::Bagged, 3), 200)
        .unwrap()
        .expect("bagged yields a dataset + job");
    assert_eq!(job.id, "recon-sess1");
    assert_eq!(job.params["backend"], "auto");
    assert_eq!(ingest.keyframes_seen(), 3);

    // The dataset carries input_path pointing at the written dataset dir, and
    // that dir holds the manifest + the three images.
    let input_path = dataset.meta["input_path"].as_str().unwrap();
    assert_eq!(input_path, dir.path().join("ds-sess1").to_string_lossy());
    assert!(std::path::Path::new(input_path)
        .join("transforms.json")
        .exists());
    assert!(std::path::Path::new(input_path)
        .join("images/0.jpg")
        .exists());
    assert_eq!(dataset.meta["received_keyframes"], 3);

    // Submitting it lands the dataset + job in the store.
    let job_id = submit_reconstruct_job(&store, &dataset, &job).unwrap();
    assert_eq!(job_id, "recon-sess1");
    assert!(store.get_dataset("ds-sess1").unwrap().is_some());
    assert!(store.get_job("recon-sess1").unwrap().is_some());
}

#[test]
fn the_capturing_drone_device_id_reaches_the_dataset_and_job() {
    let dir = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::new(dir.path());

    // The stamped keyframe + bag events carry the drone id; it lands on both
    // the dataset meta and the job params so the compute→cloud producer can
    // attribute the world model to the capturing drone.
    ingest
        .step(&keyframe_event_from("sD", 0, "drone-42"), 100)
        .unwrap();
    let (dataset, job) = ingest
        .step(
            &capture_state_from("sD", CaptureState::Bagged, 1, "drone-42"),
            200,
        )
        .unwrap()
        .expect("bagged yields a dataset + job");
    assert_eq!(dataset.meta["device_id"], "drone-42");
    assert_eq!(job.params["device_id"], "drone-42");
    assert_eq!(job.params["session_id"], "sD");
}

#[test]
fn an_unstamped_session_omits_the_device_id_never_asserting_a_wrong_one() {
    let dir = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::new(dir.path());
    // No event carried a device id (a pre-attribution capture): the job omits
    // device_id rather than writing an empty/wrong one.
    ingest.step(&real_keyframe_event("sN", 0), 100).unwrap();
    let (dataset, job) = ingest
        .step(&capture_state("sN", CaptureState::Bagged, 1), 200)
        .unwrap()
        .unwrap();
    assert!(dataset.meta.get("device_id").is_none());
    assert!(job.params.get("device_id").is_none());
}

#[test]
fn the_live_cadence_stamps_the_capturing_drone_on_periodic_jobs() {
    let dir = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::with_live_config(dir.path(), live(3, 2));
    for kf in 0..3 {
        ingest
            .step(&keyframe_event_from("sL", kf, "drone-9"), 100)
            .unwrap();
    }
    let jobs = ingest.due_reconstructs(200).unwrap();
    assert_eq!(jobs.len(), 1);
    let (dataset, job) = &jobs[0];
    assert_eq!(dataset.meta["device_id"], "drone-9");
    assert_eq!(job.params["device_id"], "drone-9");
}

#[test]
fn a_resent_bagged_submit_is_idempotent_not_an_error() {
    let dir = tempfile::tempdir().unwrap();
    let store = JobStore::open(":memory:").unwrap();
    let mut ingest = AtlasIngest::new(dir.path());
    ingest.step(&real_keyframe_event("s2", 0), 100).unwrap();

    let (d1, j1) = ingest
        .step(&capture_state("s2", CaptureState::Bagged, 1), 200)
        .unwrap()
        .unwrap();
    assert_eq!(
        submit_reconstruct_job(&store, &d1, &j1).unwrap(),
        "recon-s2"
    );

    // A re-sent Bagged on the lossy lane: finalize is idempotent (same dir),
    // and the store submit swallows the Conflict, returning the same job id.
    let (d2, j2) = ingest
        .step(&capture_state("s2", CaptureState::Bagged, 1), 300)
        .unwrap()
        .unwrap();
    assert_eq!(
        submit_reconstruct_job(&store, &d2, &j2).unwrap(),
        "recon-s2"
    );
    assert!(store.get_dataset("ds-s2").unwrap().is_some());
}

#[test]
fn a_malformed_capture_state_is_dropped_not_an_error() {
    let dir = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::new(dir.path());
    let bad = AtlasEvent::new(ATLAS_CAPTURE_STATE_TOPIC, None, b"not msgpack".to_vec());
    assert!(ingest.step(&bad, 100).unwrap().is_none());
}

fn live(every: u64, min: u64) -> LiveReconstructConfig {
    LiveReconstructConfig {
        enabled: true,
        every_keyframes: every,
        interval_ms: i64::MAX,
        min_keyframes: min,
    }
}

#[test]
fn live_disabled_produces_no_periodic_reconstructs() {
    // The default config is disabled (opt-in), so no per-session state is
    // tracked and the cadence never fires — the node reconstructs the bag only.
    let dir = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::new(dir.path());
    for kf in 0..10 {
        ingest.step(&real_keyframe_event("off", kf), 100).unwrap();
    }
    assert!(ingest.due_reconstructs(1_000_000).unwrap().is_empty());
    assert!(ingest.in_flight_cycles().is_empty());
}

#[test]
fn live_enabled_snapshots_a_real_reconstruct_job_on_the_cadence() {
    let dir = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::with_live_config(dir.path(), live(3, 2));

    // Below the count trigger: not due.
    ingest.step(&real_keyframe_event("live", 0), 100).unwrap();
    ingest.step(&real_keyframe_event("live", 1), 100).unwrap();
    assert!(ingest.due_reconstructs(100).unwrap().is_empty());

    // The 3rd keyframe hits the cadence: one real reconstruct job over the
    // snapshot of the keyframes captured so far.
    ingest.step(&real_keyframe_event("live", 2), 100).unwrap();
    let jobs = ingest.due_reconstructs(200).unwrap();
    assert_eq!(
        jobs.len(),
        1,
        "the cadence produced one periodic reconstruct"
    );
    let (dataset, job) = &jobs[0];
    assert_eq!(dataset.id, "ds-live-c0");
    assert_eq!(dataset.kind, "live_snapshot");
    assert_eq!(dataset.meta["live"], true);
    assert_eq!(dataset.meta["keyframes"], 3);
    assert_eq!(dataset.meta["cameras"], 1);
    assert_eq!(dataset.meta["cycle"], 0);
    // The snapshot manifest the reconstruct reads exists on disk (real input).
    let input_path = dataset.meta["input_path"].as_str().unwrap();
    assert_eq!(input_path, dir.path().join("ds-live").to_string_lossy());
    assert!(std::path::Path::new(input_path)
        .join("transforms.json")
        .exists());
    assert_eq!(job.id, "recon-live-c0");
    assert_eq!(job.kind, ComputeJobKind::Reconstruct);
    assert_eq!(job.dataset_id.as_deref(), Some("ds-live-c0"));
    assert_eq!(job.params["backend"], "auto");

    // The cycle is in flight (skip-while-running armed).
    assert_eq!(
        ingest.in_flight_cycles(),
        vec![("live".to_string(), "recon-live-c0".to_string())]
    );
}

#[test]
fn live_skips_while_a_cycle_runs_then_resumes_after_it_finishes() {
    let dir = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::with_live_config(dir.path(), live(3, 2));
    for kf in 0..3 {
        ingest.step(&real_keyframe_event("s", kf), 100).unwrap();
    }
    // First cycle becomes due and is armed.
    assert_eq!(ingest.due_reconstructs(200).unwrap().len(), 1);

    // More keyframes arrive while the cycle runs: NOT due (coalesce, never
    // pile up), so no second job is submitted on top of the running one.
    for kf in 3..6 {
        ingest.step(&real_keyframe_event("s", kf), 300).unwrap();
    }
    assert!(
        ingest.due_reconstructs(400).unwrap().is_empty(),
        "a cycle in flight blocks the next"
    );

    // The job finishes (the caller polled the store and released the guard).
    ingest.note_cycle_finished("s");
    assert!(ingest.in_flight_cycles().is_empty());

    // Now the next cycle (over the new keyframes) is due.
    let jobs = ingest.due_reconstructs(500).unwrap();
    assert_eq!(jobs.len(), 1);
    assert_eq!(jobs[0].1.id, "recon-s-c1", "the cycle index advanced");
    assert_eq!(jobs[0].0.meta["keyframes"], 6);
}

#[test]
fn the_bagged_state_drops_the_live_session() {
    let dir = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::with_live_config(dir.path(), live(3, 2));
    for kf in 0..3 {
        ingest.step(&real_keyframe_event("b", kf), 100).unwrap();
    }
    // A live cycle is armed.
    assert_eq!(ingest.due_reconstructs(200).unwrap().len(), 1);
    assert!(!ingest.in_flight_cycles().is_empty());

    // The terminal bag yields the final reconstruct and clears the session,
    // so the cadence stops (no further periodic jobs for a finished session).
    let (_dataset, job) = ingest
        .step(&capture_state("b", CaptureState::Bagged, 3), 300)
        .unwrap()
        .expect("the bag yields the final reconstruct");
    assert_eq!(job.id, "recon-b");
    assert!(ingest.in_flight_cycles().is_empty());
    assert!(ingest.due_reconstructs(1_000_000).unwrap().is_empty());
}

#[test]
fn a_malformed_keyframe_is_counted_but_not_persisted() {
    let dir = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::new(dir.path());
    // An undecodable keyframe payload still counts as a received delivery, but
    // nothing is persisted. A bag over a session with zero persisted keyframes
    // has no dataset to reconstruct, so it yields NO job (rather than enqueuing
    // one that would only die at the backend with a missing input directory).
    let bad = AtlasEvent::new(ATLAS_KEYFRAME_TOPIC, None, vec![0u8; 8]);
    assert!(ingest.step(&bad, 100).unwrap().is_none());
    assert_eq!(ingest.keyframes_seen(), 1);

    assert!(
        ingest
            .step(&capture_state("badkf", CaptureState::Bagged, 1), 200)
            .unwrap()
            .is_none(),
        "an empty bag (no keyframes persisted) enqueues no reconstruct job"
    );
}

#[test]
fn a_bagged_reconstruct_job_carries_the_default_train_iters() {
    // With ADOS_COMPUTE_TRAIN_ITERS unset (the CI default), a bagged
    // reconstruct job stamps the default training length onto its `steps`
    // param so the backend trains for a real, configurable number of
    // iterations instead of an implicit backend fallback.
    let dir = tempfile::tempdir().unwrap();
    let mut ingest = AtlasIngest::new(dir.path());
    ingest.step(&real_keyframe_event("sT", 0), 100).unwrap();
    let (_dataset, job) = ingest
        .step(&capture_state("sT", CaptureState::Bagged, 1), 200)
        .unwrap()
        .expect("a session with a persisted keyframe yields a job");
    assert_eq!(job.params["steps"], 30000);
}
