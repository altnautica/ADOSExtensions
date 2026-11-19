use std::path::Path;

use ados_capture_contract::{ImuNoise, Rig};

use super::*;
use crate::calib::test_util::{tiny_session, tiny_session_with};
use crate::recon::ResolvedTool;

const IMG: &str = "registry.example/calib@sha256:0123";
const SESSION: &str = "sim-1-20261019T000000Z-0001";

fn image(_: &str) -> Result<String, StageError> {
    Ok(IMG.into())
}

/// A host with no container runtime.
struct NoDocker;

impl ToolRunner for NoDocker {
    fn resolve(&self, tool: &str) -> Result<ResolvedTool, StageError> {
        Err(StageError::tool_missing(tool, "not installed"))
    }

    fn run(&self, inv: &ToolInvocation, _: &mut dyn FnMut(&str)) -> Result<(), StageError> {
        Err(self.resolve(&inv.tool).unwrap_err())
    }
}

/// Writes what the containers would, through [`stand_in`].
struct StandIn;

impl ToolRunner for StandIn {
    fn resolve(&self, tool: &str) -> Result<ResolvedTool, StageError> {
        Err(StageError::tool_missing(tool, "stand-in"))
    }

    fn run(&self, inv: &ToolInvocation, _: &mut dyn FnMut(&str)) -> Result<(), StageError> {
        match stand_in(inv) {
            Ok(true) => Ok(()),
            Ok(false) => Err(StageError::tool_failed(&inv.tool, "not a calibration run")),
            Err(e) => Err(StageError::io(e)),
        }
    }
}

fn req(target: CalibTarget) -> CalibRequest {
    CalibRequest {
        session_id: SESSION.into(),
        target,
        board: AprilGrid::default(),
    }
}

fn with_noise(root: &Path) -> std::path::PathBuf {
    tiny_session_with(root, |rig: &mut Rig| {
        rig.sensors[1].imu_noise = Some(ImuNoise {
            gyro_nd: 1.6e-4,
            accel_nd: 2.0e-3,
            gyro_rw: 2.2e-5,
            accel_rw: 3.0e-3,
        });
    })
}

#[test]
fn request_parses_with_a_default_board() {
    let r: CalibRequest =
        serde_json::from_str(r#"{"session_id":"s","target":"camera_imu"}"#).unwrap();
    assert_eq!(r.target, CalibTarget::CameraImu);
    assert_eq!(r.board, AprilGrid::default());
    assert_eq!(
        serde_json::to_value(CalibTarget::LidarCamera).unwrap(),
        "lidar_camera"
    );
}

#[test]
fn a_missing_container_runtime_is_propagated() {
    let dir = tempfile::tempdir().unwrap();
    let session = with_noise(dir.path());
    for target in [CalibTarget::Intrinsics, CalibTarget::CameraImu] {
        let err = run_with(
            &req(target),
            &session,
            &dir.path().join("work"),
            &NoDocker,
            &mut |_| {},
            &image,
        )
        .unwrap_err();
        assert_eq!(err.code, "E_TOOL_MISSING", "{target:?}");
        assert_eq!(err.tool.as_deref(), Some("docker"));
    }
}

#[test]
fn camera_imu_with_the_stand_in_calibrates_the_cameras() {
    let dir = tempfile::tempdir().unwrap();
    let session = with_noise(dir.path());
    let work = dir.path().join("work");
    let mut seen = Vec::new();
    let out = run_with(
        &req(CalibTarget::CameraImu),
        &session,
        &work,
        &StandIn,
        &mut |p| seen.push(p),
        &image,
    )
    .unwrap();
    let cam = &out.rig.sensors[0];
    let model = cam.camera.as_ref().unwrap();
    assert!(model.calibrated);
    // The fixture's cam0 extrinsics: forward camera 10 cm ahead, 5 cm left,
    // 2 cm up of the IMU (at the body origin in the tiny rig).
    let t = &cam.t_body_sensor.0;
    crate::calib::test_util::assert_close(
        &[t[2], t[3], t[4], t[7], t[9], t[11]],
        &[1.0, 0.1, -1.0, 0.05, -1.0, 0.02],
        1e-9,
    );
    assert_eq!(cam.time_offset_ns, 2_500_000);
    assert!(out.rig.calib_version.starts_with("1+camera_imu-2"));
    assert!(out.rig.calib_version.ends_with('Z'));
    assert_eq!(out.report["target"], "camera_imu");
    assert_eq!(out.report["tool"], "kalibr");
    assert!(
        (out.report["reprojection_rms_px"].as_f64().unwrap() - 0.18f64.hypot(0.12)).abs() < 1e-12
    );
    assert_eq!(out.report["cameras"][0]["frames"], 2);
    assert_eq!(seen.first(), Some(&0.0));
    assert_eq!(seen.last(), Some(&1.0));
    // The inputs Kalibr read were written beside the bag.
    let kdir = work.join("kalibr");
    let imu = std::fs::read_to_string(kdir.join("imu.yaml")).unwrap();
    assert!(imu.starts_with("rostopic: /imu0\nupdate_rate: "), "{imu}");
    assert!(kdir.join("data/cam0/1000000000.png").exists());
    assert!(kdir.join("data/imu0.csv").exists());
}

#[test]
fn intrinsics_with_the_stand_in_reads_the_image_size() {
    let dir = tempfile::tempdir().unwrap();
    let out = run_with(
        &req(CalibTarget::Intrinsics),
        &tiny_session(),
        &dir.path().join("work"),
        &StandIn,
        &mut |_| {},
        &image,
    )
    .unwrap();
    let c = out.rig.sensors[0].camera.as_ref().unwrap();
    assert_eq!((c.width, c.height), (64, 48));
    assert_eq!(c.model, ados_capture_contract::CameraModelModel::Opencv);
    assert!(c.calibrated);
    assert_eq!(
        out.rig.sensors[0].t_body_sensor,
        tiny_rig().sensors[0].t_body_sensor
    );
}

fn tiny_rig() -> Rig {
    ados_capture_contract::SessionReader::open(&tiny_session())
        .unwrap()
        .rig()
        .clone()
}

#[test]
fn camera_imu_without_imu_noise_and_short_imu_noise_runs_are_refused() {
    let dir = tempfile::tempdir().unwrap();
    for target in [
        CalibTarget::CameraImu,
        CalibTarget::ImuNoise,
        CalibTarget::LidarCamera,
    ] {
        let err = run_with(
            &req(target),
            &tiny_session(),
            &dir.path().join("work"),
            &StandIn,
            &mut |_| {},
            &image,
        )
        .unwrap_err();
        assert_eq!(err.code, "E_INPUT", "{target:?}: {err}");
    }
    let mut other = req(CalibTarget::ImuNoise);
    other.session_id = "another".into();
    let err = run_with(
        &other,
        &tiny_session(),
        dir.path(),
        &StandIn,
        &mut |_| {},
        &image,
    )
    .unwrap_err();
    assert_eq!(err.code, "E_INPUT");
}

#[test]
fn utc_stamp_formats_known_instants() {
    let at = |s: u64| utc_stamp(UNIX_EPOCH + std::time::Duration::from_secs(s));
    assert_eq!(at(0), "19700101T000000Z");
    assert_eq!(at(951_782_400), "20000229T000000Z");
    assert_eq!(at(1_792_368_000 + 3_723), "20261019T010203Z");
}

/// A session with two frames of `cam0` and three clouds of `lidar0`.
fn lidar_session(root: &Path) -> std::path::PathBuf {
    use ados_capture_contract::foxglove::packed_element_field::NumericType;
    use ados_capture_contract::foxglove::{PackedElementField, PointCloud, RawImage};
    use ados_capture_contract::{topics, Manifest, SessionWriter};

    let src = tiny_session();
    let manifest: Manifest =
        serde_json::from_slice(&std::fs::read(src.join("manifest.json")).unwrap()).unwrap();
    let mut rig = tiny_rig();
    rig.sensors.push(
        serde_json::from_value(serde_json::json!({
            "id": "lidar0", "kind": "lidar", "role": "up", "time_offset_ns": 0,
            "T_body_sensor": [1.0, 0.0, 0.0, 0.02, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.1,
                              0.0, 0.0, 0.0, 1.0],
            "lidar": crate::calib::test_util::mid360()
        }))
        .unwrap(),
    );
    let mut w = SessionWriter::create(root, manifest, &rig).unwrap();
    let field = |name: &str, offset: u32| PackedElementField {
        name: name.into(),
        offset,
        r#type: NumericType::Float32 as i32,
    };
    for k in 0..3u64 {
        let t = 1_000_000_000 + k * 50_000_000;
        if k < 2 {
            let img = RawImage {
                timestamp: None,
                frame_id: "cam0".into(),
                width: 4,
                height: 2,
                encoding: "rgb8".into(),
                step: 12,
                data: (0..24).collect(),
            };
            w.write_proto(&topics::cam_image("cam0"), t, &img).unwrap();
        }
        let data = [1.0f32, 2.0, 3.0, 4.0, -1.0, 0.5, 0.25, 9.0]
            .iter()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        let pc = PointCloud {
            timestamp: None,
            frame_id: "lidar0".into(),
            pose: None,
            point_stride: 16,
            fields: vec![
                field("x", 0),
                field("y", 4),
                field("z", 8),
                field("intensity", 12),
            ],
            data,
        };
        w.write_proto(&topics::lidar_points("lidar0"), t + 1, &pc)
            .unwrap();
    }
    let dir = w.dir().to_path_buf();
    w.finish().unwrap();
    dir
}

#[test]
fn lidar_camera_with_the_stand_in_writes_a_ros2_bag_and_keeps_the_seed() {
    let dir = tempfile::tempdir().unwrap();
    let session = lidar_session(&dir.path().join("sessions"));
    let work = dir.path().join("work");
    let out = run_with(
        &req(CalibTarget::LidarCamera),
        &session,
        &work,
        &StandIn,
        &mut |_| {},
        &image,
    )
    .unwrap();
    let rig_in = ados_capture_contract::SessionReader::open(&session)
        .unwrap()
        .rig()
        .clone();
    // The stand-in returns the seeded guess, so the LiDAR stays put.
    crate::calib::test_util::assert_close(
        &out.rig.sensors[2].t_body_sensor.0,
        &rig_in.sensors[2].t_body_sensor.0,
        1e-12,
    );
    assert_eq!(out.report["tool"], "dvlc");
    assert_eq!(out.report["images"], 2);
    assert_eq!(out.report["clouds"], 3);
    assert!(out.rig.calib_version.starts_with("1+lidar_camera-"));

    let bag = work.join("dvlc/bags/session");
    let meta = std::fs::read_to_string(bag.join("metadata.yaml")).unwrap();
    assert!(meta.contains("storage_identifier: mcap\n"), "{meta}");
    assert!(meta.contains("  message_count: 5\n"), "{meta}");
    let bytes = std::fs::read(bag.join("session_0.mcap")).unwrap();
    let mut seen = Vec::new();
    for m in mcap::MessageStream::new(&bytes).unwrap() {
        let m = m.unwrap();
        let schema = m.channel.schema.as_ref().unwrap();
        assert_eq!(schema.encoding, "ros2msg");
        assert_eq!(m.channel.message_encoding, "cdr");
        seen.push((m.channel.topic.clone(), schema.name.clone(), m.data.len()));
    }
    let images = seen
        .iter()
        .filter(|s| s.1 == "sensor_msgs/msg/Image")
        .count();
    let clouds: Vec<_> = seen
        .iter()
        .filter(|s| s.1 == "sensor_msgs/msg/PointCloud2")
        .collect();
    assert_eq!((images, clouds.len()), (2, 3));
    assert_eq!(clouds[0].0, "/points");
    let calib: serde_json::Value =
        serde_json::from_slice(&std::fs::read(work.join("dvlc/preprocessed/calib.json")).unwrap())
            .unwrap();
    assert_eq!(calib["camera"]["camera_model"], "plumb_bob");
    assert_eq!(calib["meta"]["bag_names"][0], "session");
}
