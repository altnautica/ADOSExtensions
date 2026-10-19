use super::*;
use ados_capture_contract::v1::ImuSample;
use ados_capture_contract::{CameraModel, CameraModelModel, Placement, Timebase};
use world_engine_protocol::atlas::{
    CameraRole, KeyframeFlags, KeyframeImage, KeyframeTier, Pose, PoseSource, TimeAlignment,
};

fn keyframe() -> KeyframeEnvelope {
    KeyframeEnvelope {
        session_id: "s".into(),
        kf_id: 1,
        ts_unix_ms: 1000,
        camera_id: "front".into(),
        camera_role: CameraRole::Primary,
        tier: KeyframeTier::Full,
        image: KeyframeImage {
            encoding: ImageEncoding::Jpeg,
            width: 1280,
            height: 720,
            bytes: vec![],
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
            t: [1.0, 2.0, 3.0],
            cov: None,
        },
        pose_cov: vec![0.0009; world_engine_protocol::atlas::POSE_COV_LEN],
        pose_source: PoseSource::FcEkf,
        time: TimeAlignment {
            t_exposure_mid_ns: 7_000_000_000,
            timebase: Timebase::HostMonotonic,
            sigma_ns: -1,
            trigger_seq: None,
        },
        anchor: None,
        imu_window: vec![ImuSample {
            t_ns: 6_999_000_000,
            seq: 0,
            gyro: vec![0.1, 0.2, 0.3],
            accel: vec![0.0, 0.0, 9.81],
            temp_c: None,
        }],
        flags: KeyframeFlags::default(),
    }
}

#[test]
fn keyframe_maps_to_camera_subtree_and_imu_scalars() {
    let entries = log_keyframe(&keyframe());
    let paths: Vec<&str> = entries.iter().map(|e| e.entity_path.as_str()).collect();
    // The camera subtree: pinhole + transform on the camera path, image on rgb.
    assert!(paths.contains(&"world/camera/front"));
    assert!(paths.contains(&"world/camera/front/rgb"));
    // Per-axis IMU scalars.
    assert!(paths.contains(&"world/imu/accel/z"));
    assert!(paths.contains(&"world/imu/gyro/x"));

    // 3 camera entries + 6 IMU scalars (3 accel + 3 gyro) for one sample.
    assert_eq!(entries.len(), 3 + 6);

    // The pinhole carries the intrinsics K, the transform the pose.
    let pinhole = entries
        .iter()
        .find(|e| matches!(e.archetype, RerunArchetype::Pinhole { .. }))
        .unwrap();
    match &pinhole.archetype {
        RerunArchetype::Pinhole {
            image_from_camera,
            width,
            height,
        } => {
            assert_eq!(image_from_camera[0], 900.0);
            assert_eq!((*width, *height), (1280, 720));
        }
        _ => unreachable!(),
    }
    // The accel-z scalar is gravity.
    let az = entries
        .iter()
        .find(|e| e.entity_path == "world/imu/accel/z")
        .unwrap();
    assert_eq!(az.archetype, RerunArchetype::Scalar { value: 9.81 });
    // The sample is 1 ms before the exposure, so 1 ms before the frame stamp.
    assert_eq!(az.timeline_ms, 999);
}

#[test]
fn hevc_encoding_serializes_hyphenated() {
    let mut kf = keyframe();
    kf.image.encoding = ImageEncoding::HevcKeyframe;
    let img = log_keyframe(&kf)
        .into_iter()
        .find(|e| matches!(e.archetype, RerunArchetype::Image { .. }))
        .unwrap();
    match img.archetype {
        RerunArchetype::Image { encoding, .. } => assert_eq!(encoding, "hevc-keyframe"),
        _ => unreachable!(),
    }
}

#[test]
fn world_model_descriptors_map_to_world_subtree_with_payloads() {
    let splat = log_splat(
        &SplatDescriptor {
            session_id: "sess-test".into(),
            generation: 1,
            placement: Placement::identity(None),
            gaussian_count: 5,
            step: 50,
            manifest_url: None,
            lod_levels: 0,
            url: None,
            handle: None,
        },
        7,
    );
    assert_eq!(splat.entity_path, "world/splat");
    assert_eq!(splat.timeline_ms, 7);
    assert_eq!(
        splat.archetype,
        RerunArchetype::SplatSlab {
            gaussian_count: 5,
            step: 50
        }
    );

    let cloud = log_pointcloud(
        &PointCloudDescriptor {
            session_id: "sess-test".into(),
            generation: 1,
            placement: Placement::identity(None),
            point_count: 9,
            bounds: [-1.0, -1.0, -1.0, 1.0, 1.0, 1.0],
            shm_name: None,
            slot: None,
            seq: None,
            url: None,
        },
        1,
    );
    assert_eq!(cloud.entity_path, "world/points");
    assert_eq!(
        cloud.archetype,
        RerunArchetype::Points3D {
            count: 9,
            bounds: [-1.0, -1.0, -1.0, 1.0, 1.0, 1.0]
        }
    );

    let mesh = log_mesh(
        &MeshDescriptor {
            session_id: "sess-test".into(),
            generation: 1,
            placement: Placement::identity(None),
            vertex_count: 1,
            face_count: 2,
            url: None,
            handle: None,
        },
        1,
    );
    assert_eq!(mesh.entity_path, "world/mesh");
    assert_eq!(
        mesh.archetype,
        RerunArchetype::Mesh3D {
            vertex_count: 1,
            face_count: 2
        }
    );

    let occ = log_occupancy(
        &OccupancyDescriptor {
            session_id: "sess-test".into(),
            generation: 1,
            placement: Placement::identity(None),
            field: world_engine_protocol::atlas::OccupancyField::Esdf,
            truncation_m: 4.0,
            url: None,
            origin: [0.5, 0.0, 0.0],
            resolution_m: 0.05,
            dims: [10, 10, 4],
            shm_name: None,
            slot: None,
            seq: None,
        },
        1,
    );
    assert_eq!(occ.entity_path, "world/occupancy");
    assert_eq!(
        occ.archetype,
        RerunArchetype::VoxelGrid {
            dims: [10, 10, 4],
            resolution_m: 0.05,
            origin: [0.5, 0.0, 0.0]
        }
    );
}

#[test]
fn pinhole_is_logged_once_per_camera() {
    let mut rec = RerunRecording::new();
    rec.push_keyframe(&keyframe()); // first: includes the pinhole (9 entries)
    rec.push_keyframe(&keyframe()); // same camera: pinhole dropped (8 entries)
    let pinholes = rec
        .entries
        .iter()
        .filter(|e| matches!(e.archetype, RerunArchetype::Pinhole { .. }))
        .count();
    assert_eq!(pinholes, 1);
    assert_eq!(rec.len(), 9 + 8);
}

#[test]
fn recording_accumulates_and_serializes() {
    let mut rec = RerunRecording::new();
    assert!(rec.is_empty());
    rec.push_keyframe(&keyframe());
    rec.push_splat(
        &SplatDescriptor {
            session_id: "sess-test".into(),
            generation: 1,
            placement: Placement::identity(None),
            gaussian_count: 1200,
            step: 50,
            manifest_url: None,
            lod_levels: 0,
            url: None,
            handle: None,
        },
        1001,
    );
    assert_eq!(rec.len(), 9 + 1);
    let json = rec.to_json().unwrap();
    assert!(json.contains("world/splat"));
    assert!(json.contains("\"archetype\""));
    // The discriminators are the Rerun archetype names verbatim, not
    // snake_case digit-splits (would be "transform3_d" / "splat_slab").
    assert!(json.contains("\"Transform3D\""));
    assert!(json.contains("\"SplatSlab\""));
    assert!(!json.contains("transform3_d"));
    // Round-trips back to the same entries.
    let back: RerunRecording = serde_json::from_str(&json).unwrap();
    assert_eq!(back.entries, rec.entries);
}
