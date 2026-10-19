use super::*;
use ados_capture_contract::v1::ImuSample;
use ados_capture_contract::{
    CameraModel, CameraModelModel, GeoAnchor, GeoAnchorDatum, GeoAnchorSource, Placement, Timebase,
};

fn sample_pose() -> Pose {
    Pose {
        r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        t: [1.5, -2.0, 0.5],
        cov: None,
    }
}

/// A diagonal 6x6 covariance: 0.03 m position sigma (RTK-fixed class) and
/// 0.01 rad orientation sigma, squared onto the diagonal.
fn sample_cov() -> Vec<f64> {
    let mut cov = vec![0.0; POSE_COV_LEN];
    for i in 0..3 {
        cov[i * 6 + i] = 0.03 * 0.03;
    }
    for i in 3..6 {
        cov[i * 6 + i] = 0.01 * 0.01;
    }
    cov
}

/// A measured alignment: a 20 us sigma (PPS-disciplined class) joined on a
/// trigger sequence.
fn sample_time() -> TimeAlignment {
    TimeAlignment {
        t_exposure_mid_ns: 42_000_000_000,
        timebase: Timebase::Gnss,
        sigma_ns: 20_000,
        trigger_seq: Some(19),
    }
}

fn sample_camera() -> CameraModel {
    CameraModel {
        model: CameraModelModel::Opencv,
        width: 1280,
        height: 720,
        fx: 900.0,
        fy: 900.0,
        cx: 640.0,
        cy: 360.0,
        dist: vec![0.0, 0.0, 0.0, 0.0, 0.0],
        calibrated: true,
        readout_ns: 0,
    }
}

fn sample_anchor() -> GeoAnchor {
    GeoAnchor {
        lat_deg: 12.97,
        lon_deg: 77.59,
        h_ellipsoid_m: 920.0,
        geoid_undulation_m: -86.0,
        datum: GeoAnchorDatum::Wgs84,
        source: GeoAnchorSource::GnssMean,
    }
}

fn sample_keyframe() -> KeyframeEnvelope {
    KeyframeEnvelope {
        session_id: "sess-1".into(),
        kf_id: 7,
        ts_unix_ms: 1_700_000_000_000,
        camera_id: "front".into(),
        camera_role: CameraRole::Primary,
        tier: KeyframeTier::Full,
        image: KeyframeImage {
            encoding: ImageEncoding::Jpeg,
            width: 1280,
            height: 720,
            bytes: vec![1, 2, 3, 4],
        },
        camera: sample_camera(),
        pose: sample_pose(),
        pose_cov: sample_cov(),
        pose_source: PoseSource::LocalVio,
        time: sample_time(),
        anchor: Some(sample_anchor()),
        imu_window: vec![ImuSample {
            t_ns: 41_999_000_000,
            seq: 3,
            gyro: vec![0.0, 0.0, 0.0],
            accel: vec![0.0, 0.0, 9.81],
            temp_c: None,
        }],
        flags: KeyframeFlags {
            is_session_start: true,
            ..KeyframeFlags::default()
        },
    }
}

#[test]
fn keyframe_envelope_round_trips() {
    let kf = sample_keyframe();
    let bytes = kf.to_msgpack().expect("encode");
    let back = KeyframeEnvelope::from_msgpack(&bytes).expect("decode");
    assert_eq!(kf, back);
    assert_eq!(back.camera_role, CameraRole::Primary);
    assert_eq!(back.tier, KeyframeTier::Full);
    assert_eq!(back.pose_source, PoseSource::LocalVio);
}

#[test]
fn offloaded_pose_round_trips() {
    let op = OffloadedPose {
        session_id: "sess-1".into(),
        kf_id: 7,
        camera_id: "front".into(),
        pose: sample_pose(),
        source: PoseSource::OffloadedSlam,
        ts_ms: 1_700_000_000_000,
    };
    let bytes = op.to_msgpack().expect("encode");
    let back = OffloadedPose::from_msgpack(&bytes).expect("decode");
    assert_eq!(op, back);
    assert_eq!(back.source, PoseSource::OffloadedSlam);
}

#[test]
fn world_model_descriptors_round_trip() {
    let status = CaptureStatus {
        session_id: "sess-1".into(),
        state: CaptureState::Capturing,
        keyframes: 42,
        vio_health: VioHealth::Good,
        camera_count: 1,
        ingest_rate_hz: 9.5,
        capped: false,
        anchored: true,
        pose_tier: PoseSource::LocalVio,
        dropped_keyframes: 0,
    };
    let back = CaptureStatus::from_msgpack(&status.to_msgpack().unwrap()).unwrap();
    assert_eq!(status, back);

    let cloud = PointCloudDescriptor {
        session_id: "sess-1".into(),
        generation: 3,
        placement: Placement::identity(Some(sample_anchor())),
        point_count: 10_000,
        bounds: [-1.0, -1.0, -1.0, 1.0, 1.0, 1.0],
        shm_name: Some("atlas-cloud-0".into()),
        slot: Some(2),
        seq: Some(99),
        url: None,
    };
    assert_eq!(
        cloud,
        PointCloudDescriptor::from_msgpack(&cloud.to_msgpack().unwrap()).unwrap()
    );
}

#[test]
fn wire_keys_match_the_spec() {
    // The serialized key names are the contract a Python/TS consumer reads.
    // Pin them so a serde-rename regression (which a same-struct round-trip
    // can never catch) fails the build instead of breaking the wire.
    let mut kf = sample_keyframe();
    kf.image.encoding = ImageEncoding::HevcKeyframe;
    let v = serde_json::to_value(&kf).unwrap();
    for key in [
        "session_id",
        "kf_id",
        "ts_unix_ms",
        "camera_id",
        "camera_role",
        "tier",
        "pose_source",
        "pose_cov",
        "time",
        "anchor",
        "imu_window",
        "flags",
    ] {
        assert!(v.get(key).is_some(), "keyframe key `{key}` missing");
    }
    for key in ["t_exposure_mid_ns", "timebase", "sigma_ns", "trigger_seq"] {
        assert!(v["time"].get(key).is_some(), "time key `{key}` missing");
    }
    assert_eq!(v["time"]["timebase"], "gnss");
    assert_eq!(
        v["pose_cov"].as_array().map(|a| a.len()),
        Some(POSE_COV_LEN),
        "pose_cov rides as a 36-element row-major array"
    );
    // The camera is the capture contract's camera model.
    for key in [
        "model",
        "width",
        "height",
        "fx",
        "fy",
        "cx",
        "cy",
        "dist",
        "calibrated",
        "readout_ns",
    ] {
        assert!(v["camera"].get(key).is_some(), "camera key `{key}` missing");
    }
    assert_eq!(v["camera"]["model"], "opencv");
    // The anchor is the contract's geo anchor; IMU samples carry t_ns and seq.
    assert_eq!(v["anchor"]["datum"], "WGS84");
    assert_eq!(v["anchor"]["source"], "gnss_mean");
    assert!(v["anchor"].get("h_ellipsoid_m").is_some());
    assert_eq!(v["imu_window"][0]["t_ns"], 41_999_000_000u64);
    assert_eq!(v["imu_window"][0]["seq"], 3);
    // R is capitalized per the spec's math convention; t stays lower.
    assert!(v["pose"].get("R").is_some(), "rotation key is `R`");
    assert!(v["pose"].get("t").is_some(), "translation key is `t`");
    // The HEVC encoding is hyphenated on the wire.
    assert_eq!(v["image"]["encoding"], "hevc-keyframe");
    assert_eq!(serde_json::to_value(ImageEncoding::Jpeg).unwrap(), "jpeg");
}

#[test]
fn occupancy_splat_mesh_pose_round_trip() {
    let placement = Placement {
        anchor: None,
        r_enu_artifact: [1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0],
        t_enu_artifact: [0.0, 0.0, 1.0],
        scale: 1.0,
    };
    let occ = OccupancyDescriptor {
        session_id: "sess-1".into(),
        generation: 4,
        placement: placement.clone(),
        origin: [0.0, 0.0, 0.0],
        resolution_m: 0.05,
        dims: [100, 100, 40],
        field: OccupancyField::Esdf,
        truncation_m: 2.0,
        shm_name: Some("atlas-occ-0".into()),
        slot: Some(1),
        seq: Some(7),
        url: None,
    };
    assert_eq!(
        occ,
        OccupancyDescriptor::from_msgpack(&occ.to_msgpack().unwrap()).unwrap()
    );

    let splat = SplatDescriptor {
        session_id: "sess-1".into(),
        generation: 4,
        placement: placement.clone(),
        gaussian_count: 250_000,
        step: 1500,
        url: Some("spz://session/1".into()),
        handle: None,
        manifest_url: Some("https://node.example/artifacts/sess-1/g4/manifest.json".into()),
        lod_levels: 3,
    };
    assert_eq!(
        splat,
        SplatDescriptor::from_msgpack(&splat.to_msgpack().unwrap()).unwrap()
    );

    let mesh = MeshDescriptor {
        session_id: "sess-1".into(),
        generation: 4,
        placement,
        vertex_count: 8000,
        face_count: 16000,
        url: None,
        handle: Some("mesh-0".into()),
    };
    assert_eq!(
        mesh,
        MeshDescriptor::from_msgpack(&mesh.to_msgpack().unwrap()).unwrap()
    );

    let pose_desc = PoseDescriptor {
        pose: sample_pose(),
        anchor: Some(sample_anchor()),
        ts_ms: 5,
    };
    assert_eq!(
        pose_desc,
        PoseDescriptor::from_msgpack(&pose_desc.to_msgpack().unwrap()).unwrap()
    );
}

#[test]
fn a_descriptor_without_a_placement_is_refused() {
    // Placement is required: a descriptor that does not say where its
    // coordinates sit cannot be placed in the world, and defaulting it to an
    // identity at an unknown origin would assert a georeference nobody made.
    let r: Result<OccupancyDescriptor, _> = serde_json::from_str(
        r#"{"origin":[0,0,0],"resolution_m":0.1,"dims":[4,4,4],
            "shm_name":null,"slot":null,"seq":null}"#,
    );
    assert!(r.is_err());
}

#[test]
fn occupancy_field_defaults_to_occupancy_never_esdf() {
    // A producer that omits `field` holds occupancy probabilities. Defaulting to
    // Esdf would have a planner read a probability byte as a distance in metres,
    // so the default must fall on the side that cannot be mis-measured.
    let old: OccupancyDescriptor = serde_json::from_str(
        r#"{"placement":{"anchor":null,"R_enu_artifact":[1,0,0,0,1,0,0,0,1],
            "t_enu_artifact":[0,0,0],"scale":1},
            "origin":[0,0,0],"resolution_m":0.1,"dims":[4,4,4],
            "shm_name":null,"slot":null,"seq":null}"#,
    )
    .unwrap();
    assert_eq!(old.field, OccupancyField::Occupancy);
    assert_eq!(old.truncation_m, 0.0);
    assert_eq!(old.generation, 0);
}

#[test]
fn an_unmeasured_time_alignment_is_never_trustworthy() {
    // The whole point of the sigma field is to admit uncertainty, so the
    // absent/default value must fail closed rather than read as an exact clock.
    let un = TimeAlignment::unmeasured();
    assert!(un.sigma_ns < 0);
    assert!(!un.is_trustworthy(KEYFRAME_MAX_SIGMA_NS));
    assert_eq!(
        TimeAlignment::default(),
        un,
        "Default must be the unmeasured (fail-closed) alignment, not a zeroed one"
    );
    assert!(sample_time().is_trustworthy(KEYFRAME_MAX_SIGMA_NS));
    let mut wide = sample_time();
    wide.sigma_ns = KEYFRAME_MAX_SIGMA_NS + 1;
    assert!(!wide.is_trustworthy(KEYFRAME_MAX_SIGMA_NS));
}

#[test]
fn keyframe_validate_refuses_every_untrustworthy_frame() {
    assert!(sample_keyframe().validate().is_ok());

    // No covariance: the reconstructor would have no position-prior sigma.
    let mut no_cov = sample_keyframe();
    no_cov.pose_cov.clear();
    assert_eq!(
        no_cov.validate(),
        Err(KeyframeReject::PoseCovLen { got: 0 })
    );

    // A wrong-length covariance is just as unusable as none.
    let mut short_cov = sample_keyframe();
    short_cov.pose_cov.truncate(9);
    assert_eq!(
        short_cov.validate(),
        Err(KeyframeReject::PoseCovLen { got: 9 })
    );

    // The timebase sigma is NOT a default gate: a node with no disciplined
    // clock honestly reports it unmeasured, and rejecting every frame there
    // would yield zero reconstructions with no visible cause. The sigma still
    // travels, and `clock_is_disciplined` reports the fact.
    let mut unmeasured = sample_keyframe();
    unmeasured.time = TimeAlignment::unmeasured();
    assert!(
        unmeasured.validate().is_ok(),
        "an undisciplined clock degrades the dataset's label, not its existence"
    );
    assert!(!unmeasured.clock_is_disciplined());
    assert!(sample_keyframe().clock_is_disciplined());

    // Under the strict budget (a rig that DOES have a disciplined clock) it is
    // refused.
    let strict = KeyframeBudget::strict();
    assert!(matches!(
        unmeasured.validate_with(&strict),
        Err(KeyframeReject::TimeSigma { .. })
    ));
    // A 1 ms sigma passes strict; 50 ms does not (worse than a frame period).
    let mut ok_sigma = sample_keyframe();
    ok_sigma.time.sigma_ns = 1_000_000;
    assert!(ok_sigma.validate_with(&strict).is_ok());
    let mut bad_sigma = sample_keyframe();
    bad_sigma.time.sigma_ns = 50_000_000;
    assert!(matches!(
        bad_sigma.validate_with(&strict),
        Err(KeyframeReject::TimeSigma { .. })
    ));

    // No anchor: the frame sits at the local origin while every post-anchor
    // frame sits in ENU metres, which the reconstructor reads as a teleport.
    let mut unanchored = sample_keyframe();
    unanchored.anchor = None;
    assert_eq!(unanchored.validate(), Err(KeyframeReject::NoAnchor));
}

#[test]
fn a_keyframe_without_time_or_covariance_decodes_as_untrustworthy() {
    // A frame with no pose_cov / time fields must be REFUSED, not silently
    // accepted with a zeroed covariance and a zero-sigma clock.
    let bare = serde_json::json!({
        "session_id": "old", "kf_id": 0, "ts_unix_ms": 1,
        "camera_id": "front", "camera_role": "primary", "tier": "full",
        "image": {"encoding": "jpeg", "width": 1, "height": 1, "bytes": [0]},
        "camera": {"model": "pinhole", "width": 1, "height": 1, "fx": 1.0, "fy": 1.0,
                   "cx": 0.5, "cy": 0.5, "dist": [], "calibrated": false, "readout_ns": 0},
        "pose": {"R": [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0], "t": [0.0,0.0,0.0], "cov": null},
        "pose_source": "local_vio",
        "anchor": null,
        "imu_window": [], "flags": {"is_loop_closure": false,
        "is_session_start": true, "is_session_end": false}
    });
    let kf: KeyframeEnvelope = serde_json::from_value(bare).unwrap();
    assert!(!kf.camera.calibrated);
    assert_eq!(kf.time, TimeAlignment::unmeasured());
    assert!(kf.pose_cov.is_empty());
    assert!(
        kf.validate().is_err(),
        "a frame must not pass validation on serde defaults"
    );
}
