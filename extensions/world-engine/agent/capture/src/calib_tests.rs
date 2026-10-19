use super::*;
use ados_capture_contract::{CameraModelModel, RigSensorKind, RigSensorRole};
use serde_json::json;

/// A stereo pinhole-radtan + one equidistant camera, Kalibr output format.
const CAMCHAIN: &str = r#"
cam0:
  T_cam_imu:
  - [0.0, -1.0, 0.0, 0.1]
  - [0.0, 0.0, -1.0, 0.02]
  - [1.0, 0.0, 0.0, -0.03]
  - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: [1]
  camera_model: pinhole
  distortion_coeffs: [-0.28, 0.07, 0.0002, -0.0001]
  distortion_model: radtan
  intrinsics: [458.6, 457.3, 367.2, 248.4]
  resolution: [752, 480]
  rostopic: /cam0/image_raw
  timeshift_cam_imu: 0.0021
cam1:
  T_cam_imu:
  - [0.0, -1.0, 0.0, -0.01]
  - [0.0, 0.0, -1.0, 0.02]
  - [1.0, 0.0, 0.0, -0.03]
  - [0.0, 0.0, 0.0, 1.0]
  T_cn_cnm1:
  - [1.0, 0.0, 0.0, -0.11]
  - [0.0, 1.0, 0.0, 0.0]
  - [0.0, 0.0, 1.0, 0.0]
  - [0.0, 0.0, 0.0, 1.0]
  camera_model: pinhole
  distortion_coeffs: [-0.28, 0.07, 0.0002, -0.0001]
  distortion_model: radtan
  intrinsics: [457.6, 456.1, 379.9, 255.2]
  resolution: [752, 480]
  timeshift_cam_imu: 0.0021
cam2:
  camera_model: pinhole
  distortion_coeffs: [0.01, -0.002, 0.0003, -0.00004]
  distortion_model: equidistant
  intrinsics: [280.0, 280.0, 320.0, 240.0]
  resolution: [640, 480]
"#;

/// Kalibr's result `imu.yaml` (nested under `imu0`).
const IMU: &str = r#"
imu0:
  accelerometer_noise_density: 0.0028
  accelerometer_random_walk: 0.00086
  gyroscope_noise_density: 0.00016
  gyroscope_random_walk: 0.000022
  model: calibrated
  rostopic: /imu0
  time_offset: 0.0
  update_rate: 200.0
"#;

fn close(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-12)
}

#[test]
fn kalibr_cameras_become_contract_sensors_in_the_imu_body_frame() {
    let rig = import_kalibr("visor-01", "2026-10-19a", CAMCHAIN, Some(IMU)).unwrap();
    assert_eq!(rig.rig_id, "visor-01");
    assert_eq!(rig.calib_version, "2026-10-19a");
    let ids: Vec<&str> = rig.sensors.iter().map(|s| s.id.as_str()).collect();
    assert_eq!(ids, ["cam0", "cam1", "cam2", "imu0"]);

    let cam0 = &rig.sensors[0];
    assert_eq!(cam0.kind, RigSensorKind::Camera);
    assert_eq!(cam0.role, RigSensorRole::FrontLeft);
    assert_eq!(rig.sensors[1].role, RigSensorRole::FrontRight);
    assert_eq!(rig.sensors[2].role, RigSensorRole::Nav);
    // T_body_cam = T_cam_imu⁻¹ with body = IMU.
    let want = [
        0.0, 0.0, 1.0, 0.03, -1.0, 0.0, 0.0, 0.1, 0.0, -1.0, 0.0, 0.02, 0.0, 0.0, 0.0, 1.0,
    ];
    assert!(
        close(&cam0.t_body_sensor.0, &want),
        "{:?}",
        cam0.t_body_sensor.0
    );
    assert_eq!(cam0.time_offset_ns, 2_100_000);
    let k = cam0.camera.as_ref().unwrap();
    assert_eq!(k.model, CameraModelModel::Opencv);
    assert!(close(&k.dist, &[-0.28, 0.07, 0.0002, -0.0001, 0.0]));
    assert_eq!((k.width, k.height), (752, 480));
    assert_eq!((k.fx, k.fy, k.cx, k.cy), (458.6, 457.3, 367.2, 248.4));
    assert!(k.calibrated);

    let fisheye = rig.sensors[2].camera.as_ref().unwrap();
    assert_eq!(fisheye.model, CameraModelModel::OpencvFisheye);
    assert_eq!(fisheye.dist.len(), 4);
    // No extrinsics: at the body origin.
    assert_eq!(rig.sensors[2].t_body_sensor.0[3], 0.0);

    let imu = &rig.sensors[3];
    assert_eq!(imu.kind, RigSensorKind::Imu);
    let n = imu.imu_noise.as_ref().unwrap();
    assert_eq!((n.gyro_nd, n.accel_nd), (0.00016, 0.0028));
    assert_eq!((n.gyro_rw, n.accel_rw), (0.000022, 0.00086));
    assert_eq!(imu.t_body_sensor.0[0], 1.0);
    // The rig serializes as a contract rig.json.
    let doc = serde_json::to_value(&rig).unwrap();
    assert_eq!(doc["contract"], "ados.capture/1");
    assert_eq!(
        doc["sensors"][0]["T_body_sensor"].as_array().unwrap().len(),
        16
    );
}

#[test]
fn a_given_body_frame_composes_with_the_camera_extrinsic() {
    // Body is 0.5 m behind the IMU along x: T_i_b translates by +0.5.
    let imu = r#"
accelerometer_noise_density: 0.002
accelerometer_random_walk: 0.0003
gyroscope_noise_density: 0.0002
gyroscope_random_walk: 0.00002
T_i_b:
- [1.0, 0.0, 0.0, 0.5]
- [0.0, 1.0, 0.0, 0.0]
- [0.0, 0.0, 1.0, 0.0]
- [0.0, 0.0, 0.0, 1.0]
"#;
    let rig = import_kalibr("r", "v1", CAMCHAIN, Some(imu)).unwrap();
    // T_body_cam0 = T_i_b⁻¹ · T_cam_imu⁻¹: x shifts by -0.5.
    assert!((rig.sensors[0].t_body_sensor.0[3] - (0.03 - 0.5)).abs() < 1e-12);
    assert!((rig.sensors[3].t_body_sensor.0[3] + 0.5).abs() < 1e-12);
}

#[test]
fn unsupported_kalibr_models_are_refused() {
    let omni = CAMCHAIN.replacen("camera_model: pinhole", "camera_model: omni", 1);
    assert!(matches!(
        import_kalibr("r", "v", &omni, None),
        Err(KalibrError::Unsupported(_))
    ));
    let fov = CAMCHAIN.replacen("distortion_model: radtan", "distortion_model: fov", 1);
    assert!(matches!(
        import_kalibr("r", "v", &fov, None),
        Err(KalibrError::Unsupported(_))
    ));
    assert!(import_kalibr("r", "v", "imu0: {}", None).is_err());
}

fn record(version: &str, created: &str) -> CalibRecord {
    CalibRecord {
        calib_version: version.into(),
        created_utc: created.into(),
        source: "kalibr".into(),
        report: json!({"reprojection_rms_px": 0.21}),
        rig: import_kalibr("visor-01", version, CAMCHAIN, Some(IMU)).unwrap(),
    }
}

#[test]
fn the_store_keeps_versions_and_an_active_pointer() {
    let dir = tempfile::tempdir().unwrap();
    let store = CalibStore::open(dir.path().join("calib")).unwrap();
    assert!(store.list("visor-01").unwrap().is_empty());
    assert_eq!(store.active("visor-01"), None);

    store
        .put(&record("2026-10-20a", "2026-10-20T08:00:00Z"))
        .unwrap();
    store
        .put(&record("2026-10-19a", "2026-10-19T08:00:00Z"))
        .unwrap();
    // Versions are immutable.
    assert!(matches!(
        store.put(&record("2026-10-19a", "2026-10-19T09:00:00Z")),
        Err(CalibError::Exists { .. })
    ));
    let listed: Vec<String> = store
        .list("visor-01")
        .unwrap()
        .into_iter()
        .map(|s| s.calib_version)
        .collect();
    assert_eq!(listed, ["2026-10-19a", "2026-10-20a"]);
    assert_eq!(store.rigs(), ["visor-01"]);

    assert!(matches!(
        store.set_active("visor-01", "nope"),
        Err(CalibError::Unknown { .. })
    ));
    store.set_active("visor-01", "2026-10-20a").unwrap();
    assert_eq!(
        store.active_version("visor-01").as_deref(),
        Some("2026-10-20a")
    );
    assert_eq!(
        store.active("visor-01").unwrap().calib_version,
        "2026-10-20a"
    );
    let got = store.get("visor-01", "2026-10-19a").unwrap().unwrap();
    assert_eq!(got.report["reprojection_rms_px"], 0.21);

    // Path-escaping ids are refused.
    assert!(matches!(store.list("../x"), Err(CalibError::InvalidId(_))));
    let mut bad = record("v", "t");
    bad.calib_version = "other".into();
    assert!(matches!(
        store.put(&bad),
        Err(CalibError::VersionMismatch { .. })
    ));
}

#[test]
fn utc_formatting() {
    assert_eq!(format_utc(0), "1970-01-01T00:00:00Z");
    assert_eq!(format_utc(1_760_868_900), "2025-10-19T10:15:00Z");
    assert_eq!(format_utc(951_782_400), "2000-02-29T00:00:00Z");
}
