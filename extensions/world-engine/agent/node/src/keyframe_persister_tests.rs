use super::*;
use std::path::Path;

use ados_capture_contract::{GeoAnchorDatum, GeoAnchorSource, Timebase};
use world_engine_protocol::atlas::{
    CameraRole, ImageEncoding, KeyframeFlags, KeyframeImage, KeyframeTier, Pose, PoseSource,
    TimeAlignment,
};

fn anchor() -> GeoAnchor {
    GeoAnchor {
        lat_deg: 12.97,
        lon_deg: 77.59,
        h_ellipsoid_m: 920.0,
        geoid_undulation_m: 0.0,
        datum: GeoAnchorDatum::Wgs84,
        source: GeoAnchorSource::GnssMean,
    }
}

fn keyframe(session: &str, kf_id: u64, camera: &str, fx: f64) -> KeyframeEnvelope {
    KeyframeEnvelope {
        session_id: session.into(),
        kf_id,
        ts_unix_ms: 1_700_000_000_000 + kf_id as i64,
        camera_id: camera.into(),
        camera_role: CameraRole::Primary,
        tier: KeyframeTier::Full,
        image: KeyframeImage {
            encoding: ImageEncoding::Jpeg,
            width: 1280,
            height: 720,
            bytes: vec![0xFF, 0xD8, 0xFF, kf_id as u8],
        },
        camera: CameraModel {
            model: CameraModelModel::Opencv,
            width: 1280,
            height: 720,
            fx,
            fy: fx,
            cx: 640.0,
            cy: 360.0,
            dist: vec![0.1, -0.2, 0.001, 0.002, 0.0],
            calibrated: true,
            readout_ns: 0,
        },
        // A non-trivial rotation so the OpenCV->OpenGL column flip is visible.
        pose: Pose {
            r: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0],
            t: [1.5, -2.0, 0.5],
            cov: None,
        },
        pose_cov: vec![0.0009; world_engine_protocol::atlas::POSE_COV_LEN],
        pose_source: PoseSource::LocalVio,
        time: TimeAlignment {
            t_exposure_mid_ns: 5_000_000_000 + kf_id * 100_000_000,
            timebase: Timebase::HostMonotonic,
            sigma_ns: -1,
            trigger_seq: None,
        },
        anchor: Some(anchor()),
        imu_window: Vec::new(),
        flags: KeyframeFlags::default(),
    }
}

fn read_transforms(dir: &Path) -> serde_json::Value {
    let bytes = std::fs::read(dir.join(TRANSFORMS_FILE)).expect("transforms.json written");
    serde_json::from_slice(&bytes).expect("transforms.json is valid json")
}

#[test]
fn opencv_to_opengl_negates_y_and_z_columns_keeps_position() {
    let m = opencv_c2w_to_opengl_matrix(
        &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0],
        &[10.0, 11.0, 12.0],
    );
    // Column 0 (X axis) unchanged, columns 1 and 2 (Y, Z) negated.
    assert_eq!(m[0], [1.0, -2.0, -3.0, 10.0]);
    assert_eq!(m[1], [4.0, -5.0, -6.0, 11.0]);
    assert_eq!(m[2], [7.0, -8.0, -9.0, 12.0]);
    assert_eq!(m[3], [0.0, 0.0, 0.0, 1.0]);
}

#[test]
fn single_camera_persist_then_finalize_writes_the_contract_manifest() {
    let dir = tempfile::tempdir().unwrap();
    let mut p = KeyframePersister::new(dir.path());
    for kf_id in 0..3 {
        p.persist(&keyframe("sessA", kf_id, "front", 900.0))
            .unwrap();
    }
    let dataset_id = dataset_id_for("sessA");
    assert_eq!(p.frame_count(&dataset_id), 3);

    let ds_dir = p
        .finalize(&dataset_id)
        .unwrap()
        .expect("a non-empty dataset");
    assert_eq!(ds_dir, dir.path().join(&dataset_id));

    // Each image landed under images/ with the kf_id name and the real bytes.
    for kf_id in 0..3u64 {
        let img = ds_dir.join(IMAGES_DIR).join(format!("{kf_id}.jpg"));
        let bytes = std::fs::read(&img).unwrap();
        assert_eq!(bytes, vec![0xFF, 0xD8, 0xFF, kf_id as u8]);
    }

    let t = read_transforms(&ds_dir);
    // The contract extension: id, georeference, per-frame exposure time.
    assert_eq!(t["contract"], "ados.capture/1");
    assert_eq!(t["ados_anchor"]["lat_deg"], 12.97);
    assert_eq!(t["frames"][1]["t_ns"], 5_100_000_000u64);
    assert_eq!(t["frames"][0]["camera_id"], "front");
    // Scene-level intrinsics + OPENCV camera model + Brown–Conrady coefficients.
    assert_eq!(t["camera_model"], "OPENCV");
    assert_eq!(t["fl_x"], 900.0);
    assert_eq!(t["fl_y"], 900.0);
    assert_eq!(t["cx"], 640.0);
    assert_eq!(t["cy"], 360.0);
    assert_eq!(t["w"], 1280);
    assert_eq!(t["h"], 720);
    assert_eq!(t["k1"], 0.1);
    assert_eq!(t["k2"], -0.2);
    assert_eq!(t["p1"], 0.001);
    assert_eq!(t["p2"], 0.002);
    assert_eq!(t["k3"], 0.0);

    let frames = t["frames"].as_array().unwrap();
    assert_eq!(frames.len(), 3);
    // file_path is relative to the manifest (images/<kf_id>.jpg).
    assert_eq!(frames[0]["file_path"], "images/0.jpg");
    assert_eq!(frames[2]["file_path"], "images/2.jpg");
    // transform_matrix is a 4x4 with the Y/Z columns flipped from the pose R.
    let m = &frames[0]["transform_matrix"];
    assert_eq!(m[0][0], 1.0);
    assert_eq!(m[0][1], -2.0);
    assert_eq!(m[0][2], -3.0);
    assert_eq!(m[0][3], 1.5);
    assert_eq!(m[3], serde_json::json!([0.0, 0.0, 0.0, 1.0]));
    // Per-frame intrinsics are present so a multi-camera rig is exact.
    assert_eq!(frames[0]["fl_x"], 900.0);
    assert_eq!(frames[0]["w"], 1280);

    // The dataset's georeference reads back for artifact placement.
    assert_eq!(dataset_anchor(&ds_dir), Some(anchor()));
}

#[test]
fn multi_camera_keeps_per_frame_intrinsics() {
    let dir = tempfile::tempdir().unwrap();
    let mut p = KeyframePersister::new(dir.path());
    // Two cameras with different focal lengths in one session.
    p.persist(&keyframe("multi", 0, "front", 900.0)).unwrap();
    p.persist(&keyframe("multi", 1, "down", 700.0)).unwrap();
    let ds_dir = p.finalize(&dataset_id_for("multi")).unwrap().unwrap();
    let t = read_transforms(&ds_dir);
    let frames = t["frames"].as_array().unwrap();
    // Scene-level is the first frame's; each frame carries its own intrinsics.
    assert_eq!(t["fl_x"], 900.0);
    assert_eq!(frames[0]["fl_x"], 900.0);
    assert_eq!(frames[1]["fl_x"], 700.0);
}

#[test]
fn fisheye_models_map_to_opencv_fisheye_with_four_radial_terms() {
    let dir = tempfile::tempdir().unwrap();
    let mut p = KeyframePersister::new(dir.path());
    let mut kf = keyframe("fish", 0, "front", 600.0);
    kf.camera.model = CameraModelModel::Kb4;
    kf.camera.dist = vec![0.01, -0.02, 0.003, -0.004];
    p.persist(&kf).unwrap();
    let t = read_transforms(&p.finalize(&dataset_id_for("fish")).unwrap().unwrap());
    assert_eq!(t["camera_model"], "OPENCV_FISHEYE");
    assert_eq!(t["k4"], -0.004);
    assert!(t.get("p1").is_none());
}

#[test]
fn a_coefficient_count_that_does_not_match_the_model_is_omitted() {
    let dir = tempfile::tempdir().unwrap();
    let mut p = KeyframePersister::new(dir.path());
    let mut kf = keyframe("nodist", 0, "front", 800.0);
    kf.camera.dist = vec![0.5, 0.5, 0.5, 0.5];
    p.persist(&kf).unwrap();
    let ds_dir = p.finalize(&dataset_id_for("nodist")).unwrap().unwrap();
    let t = read_transforms(&ds_dir);
    // OPENCV with no guessed distortion coefficients.
    assert_eq!(t["camera_model"], "OPENCV");
    assert!(t.get("k1").is_none());
    assert!(t["frames"][0].get("p1").is_none());
}

#[test]
fn a_local_frame_dataset_has_no_anchor() {
    let dir = tempfile::tempdir().unwrap();
    let mut p = KeyframePersister::new(dir.path());
    let mut kf = keyframe("local", 0, "front", 800.0);
    kf.anchor = None;
    p.persist(&kf).unwrap();
    let ds_dir = p.finalize(&dataset_id_for("local")).unwrap().unwrap();
    assert!(read_transforms(&ds_dir)["ados_anchor"].is_null());
    assert_eq!(dataset_anchor(&ds_dir), None);
    assert_eq!(dataset_anchor(&dir.path().join("missing")), None);
}

#[test]
fn finalize_with_no_frames_is_none() {
    let dir = tempfile::tempdir().unwrap();
    let mut p = KeyframePersister::new(dir.path());
    assert!(p.finalize(&dataset_id_for("empty")).unwrap().is_none());
}

#[test]
fn re_finalize_is_idempotent_and_returns_the_same_dir() {
    let dir = tempfile::tempdir().unwrap();
    let mut p = KeyframePersister::new(dir.path());
    p.persist(&keyframe("again", 0, "front", 900.0)).unwrap();
    let first = p.finalize(&dataset_id_for("again")).unwrap().unwrap();
    // A re-sent terminal state: the frames were consumed, but the manifest
    // exists, so finalize returns the same directory rather than None.
    let second = p.finalize(&dataset_id_for("again")).unwrap().unwrap();
    assert_eq!(first, second);
}

#[test]
fn snapshot_writes_a_manifest_without_consuming_and_grows_with_the_session() {
    let dir = tempfile::tempdir().unwrap();
    let mut p = KeyframePersister::new(dir.path());
    let id = dataset_id_for("live");

    // No frames yet: nothing to snapshot.
    assert!(p.snapshot(&id).unwrap().is_none());

    // Two keyframes, then a snapshot: a real manifest over the two frames.
    p.persist(&keyframe("live", 0, "front", 900.0)).unwrap();
    p.persist(&keyframe("live", 1, "front", 900.0)).unwrap();
    let ds_dir = p.snapshot(&id).unwrap().expect("a non-empty snapshot");
    let t = read_transforms(&ds_dir);
    assert_eq!(t["frames"].as_array().unwrap().len(), 2);

    // The snapshot did NOT consume: more keyframes accumulate, and a later
    // snapshot reflects the grown set (the live world model updates).
    assert_eq!(p.frame_count(&id), 2);
    p.persist(&keyframe("live", 2, "front", 900.0)).unwrap();
    let ds_dir = p.snapshot(&id).unwrap().expect("a grown snapshot");
    let t = read_transforms(&ds_dir);
    assert_eq!(t["frames"].as_array().unwrap().len(), 3);

    // And the final finalize still consumes the full set + writes the manifest.
    let ds_dir = p.finalize(&id).unwrap().expect("a non-empty finalize");
    let t = read_transforms(&ds_dir);
    assert_eq!(t["frames"].as_array().unwrap().len(), 3);
    assert_eq!(p.frame_count(&id), 0);
}
