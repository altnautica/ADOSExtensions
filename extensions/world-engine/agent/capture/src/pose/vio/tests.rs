use ados_capture_contract::v1::{ImuBatch, ImuSample};
use ados_capture_contract::{
    CameraModel, CameraModelModel, ContractId, ImuNoise, Rig, RigSensor, RigSensorKind,
    RigSensorRole, Transform,
};
use world_engine_protocol::atlas::VioHealth;

use super::calib::{basalt_calib, basalt_config, scaled_rig, CalibError, DEFAULT_IMU_NOISE};
use super::pairing::VioScale;
use super::supervise::{apply_line, nominal_cov, to_estimate};
use super::wire::*;
use super::*;
use crate::pose::{se3, PoseTrack};

/// Body FLU -> camera OpenCV: camera z = body x, camera x = -body y, camera y = -body z.
const R_BODY_CAM: [f64; 9] = [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0];
/// 90 degrees about z.
pub(super) const R_Z90: [f64; 9] = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];

pub(super) fn mat4_mul(a: &[f64; 16], b: &[f64; 16]) -> [f64; 16] {
    let mut m = [0.0; 16];
    for i in 0..4 {
        for j in 0..4 {
            m[i * 4 + j] = (0..4).map(|k| a[i * 4 + k] * b[k * 4 + j]).sum();
        }
    }
    m
}

pub(super) fn mat4_inv(m: &[f64; 16]) -> [f64; 16] {
    let (r, t) = se3::from_mat4(m);
    let rt = se3::transpose(&r);
    let ti = se3::mat_vec(&rt, t);
    se3::to_mat4(&rt, [-ti[0], -ti[1], -ti[2]])
}

pub(super) fn assert_close(a: &[f64], b: &[f64], tol: f64) {
    assert_eq!(a.len(), b.len());
    for (x, y) in a.iter().zip(b) {
        assert!((x - y).abs() < tol, "{a:?} != {b:?}");
    }
}

fn camera(
    id: &str,
    role: RigSensorRole,
    y: f64,
    model: CameraModelModel,
    dist: Vec<f64>,
) -> RigSensor {
    RigSensor {
        camera: Some(CameraModel {
            calibrated: true,
            cx: 320.0,
            cy: 240.0,
            dist,
            fx: 400.0,
            fy: 401.0,
            height: 480,
            model,
            readout_ns: 0,
            width: 640,
        }),
        id: id.into(),
        imu_noise: None,
        kind: RigSensorKind::Camera,
        lever_arm_m: None,
        role,
        t_body_sensor: Transform(se3::to_mat4(&R_BODY_CAM, [0.2, y, 0.0])),
        time_offset_ns: 0,
    }
}

pub(super) fn rig(model: CameraModelModel, dist: Vec<f64>, imu_noise: Option<ImuNoise>) -> Rig {
    Rig {
        calib_version: "test".into(),
        contract: ContractId::AdosCapture1,
        rig_id: "visor".into(),
        sensors: vec![
            camera("cam_l", RigSensorRole::FrontLeft, 0.05, model, dist.clone()),
            camera("cam_r", RigSensorRole::FrontRight, -0.05, model, dist),
            RigSensor {
                camera: None,
                id: "imu0".into(),
                imu_noise,
                kind: RigSensorKind::Imu,
                lever_arm_m: None,
                role: RigSensorRole::Nav,
                t_body_sensor: Transform(se3::to_mat4(&R_Z90, [0.1, 0.0, 0.02])),
                time_offset_ns: 0,
            },
        ],
    }
}

fn t_imu_cam_of(v: &serde_json::Value) -> [f64; 16] {
    let f = |k: &str| v[k].as_f64().unwrap();
    let r = se3::matrix_from_quat([f("qw"), f("qx"), f("qy"), f("qz")]);
    se3::to_mat4(&r, [f("px"), f("py"), f("pz")])
}

#[test]
fn calib_extrinsics_are_imu_from_camera() {
    let rig = rig(
        CameraModelModel::Kb4,
        vec![0.01, -0.02, 0.003, -0.0004],
        None,
    );
    let c = basalt_calib(&rig, "cam_l", "cam_r", "imu0").unwrap();
    let v = &c["value0"];
    let t_body_imu = *rig.sensors[2].t_body_sensor;
    for (i, cam) in rig.sensors[..2].iter().enumerate() {
        let want = mat4_mul(&mat4_inv(&t_body_imu), &cam.t_body_sensor);
        assert_close(&t_imu_cam_of(&v["T_imu_cam"][i]), &want, 1e-12);
    }
    assert_eq!(v["resolution"], serde_json::json!([[640, 480], [640, 480]]));
    assert_eq!(v["cam_time_offset_ns"], 0);
    assert_eq!(v["calib_accel_bias"].as_array().unwrap().len(), 9);
    assert_eq!(v["calib_gyro_bias"].as_array().unwrap().len(), 12);
    assert_eq!(v["vignette"], serde_json::json!([]));
}

/// At half scale a scene point lands, under the scaled intrinsics, on the
/// half-resolution pixel the 2×2 average puts it at: `u' = (u + ½)/2 − ½`.
#[test]
fn half_scale_calibration_projects_onto_the_halved_image() {
    let full = rig(
        CameraModelModel::Opencv,
        vec![0.1, -0.05, 0.001, 0.002, 0.01],
        None,
    );
    assert_eq!(scaled_rig(&full, ["cam_l", "cam_r"], VioScale::Full), full);
    let half = scaled_rig(&full, ["cam_l", "cam_r"], VioScale::Half);
    let c = basalt_calib(&half, "cam_l", "cam_r", "imu0").unwrap();
    assert_eq!(
        c["value0"]["resolution"],
        serde_json::json!([[320, 240], [320, 240]])
    );
    for i in 0..2 {
        let (f, h) = (
            full.sensors[i].camera.as_ref().unwrap(),
            half.sensors[i].camera.as_ref().unwrap(),
        );
        assert_eq!(h.dist, f.dist, "distortion is resolution-free");
        // A distorted normalised point (any value; projection is linear in it).
        let (x, y) = (0.31, -0.22);
        let (u, v) = (f.fx * x + f.cx, f.fy * y + f.cy);
        let (uh, vh) = (h.fx * x + h.cx, h.fy * y + h.cy);
        assert!((uh - ((u + 0.5) / 2.0 - 0.5)).abs() < 1e-12);
        assert!((vh - ((v + 0.5) / 2.0 - 0.5)).abs() < 1e-12);
    }
    // Sensors VIO does not read keep their models.
    let other = scaled_rig(&full, ["cam_x", "cam_y"], VioScale::Half);
    assert_eq!(other, full);
}

#[test]
fn calib_maps_camera_models() {
    let kb = [0.01, -0.02, 0.003, -0.0004];
    for model in [CameraModelModel::Kb4, CameraModelModel::OpencvFisheye] {
        let c = basalt_calib(&rig(model, kb.to_vec(), None), "cam_l", "cam_r", "imu0").unwrap();
        let i = &c["value0"]["intrinsics"][0];
        assert_eq!(i["camera_type"], "kb4");
        assert_eq!(i["intrinsics"]["k4"], -0.0004);
        assert_eq!(i["intrinsics"]["fy"], 401.0);
    }
    let c = basalt_calib(
        &rig(CameraModelModel::Pinhole, vec![], None),
        "cam_l",
        "cam_r",
        "imu0",
    )
    .unwrap();
    assert_eq!(c["value0"]["intrinsics"][1]["camera_type"], "pinhole");
    let c = basalt_calib(
        &rig(
            CameraModelModel::Opencv,
            vec![0.1, -0.05, 0.001, 0.002, 0.01],
            None,
        ),
        "cam_l",
        "cam_r",
        "imu0",
    )
    .unwrap();
    let i = &c["value0"]["intrinsics"][0];
    assert_eq!(i["camera_type"], "pinhole-radtan8");
    assert_eq!(i["intrinsics"]["p2"], 0.002);
    assert_eq!(i["intrinsics"]["k3"], 0.01);
    assert_eq!(i["intrinsics"]["k6"], 0.0);
    assert_eq!(i["intrinsics"]["rpmax"], -1.0);
}

#[test]
fn calib_refuses_unsupported_distortion_and_bad_ids() {
    let bad = basalt_calib(
        &rig(CameraModelModel::Opencv, vec![0.1; 4], None),
        "cam_l",
        "cam_r",
        "imu0",
    );
    assert!(matches!(
        bad,
        Err(CalibError::UnsupportedDistortion { len: 4, .. })
    ));
    let bad = basalt_calib(
        &rig(CameraModelModel::Pinhole, vec![0.1], None),
        "cam_l",
        "cam_r",
        "imu0",
    );
    assert!(matches!(bad, Err(CalibError::UnsupportedDistortion { .. })));
    let r = rig(CameraModelModel::Pinhole, vec![], None);
    assert_eq!(
        basalt_calib(&r, "cam_l", "cam_x", "imu0"),
        Err(CalibError::MissingSensor("cam_x".into()))
    );
    assert!(matches!(
        basalt_calib(&r, "cam_l", "cam_r", "cam_l"),
        Err(CalibError::WrongKind { .. })
    ));
    assert!(matches!(
        basalt_calib(&r, "imu0", "cam_r", "imu0"),
        Err(CalibError::WrongKind { .. })
    ));
}

#[test]
fn calib_imu_noise_from_rig_or_default() {
    let c = basalt_calib(
        &rig(CameraModelModel::Pinhole, vec![], None),
        "cam_l",
        "cam_r",
        "imu0",
    )
    .unwrap();
    assert_eq!(c["value0"]["gyro_noise_std"][0], DEFAULT_IMU_NOISE.gyro_nd);
    assert_eq!(c["value0"]["accel_bias_std"][2], DEFAULT_IMU_NOISE.accel_rw);
    let noise = ImuNoise {
        gyro_nd: 1e-3,
        accel_nd: 2e-2,
        gyro_rw: 3e-5,
        accel_rw: 4e-4,
    };
    let c = basalt_calib(
        &rig(CameraModelModel::Pinhole, vec![], Some(noise)),
        "cam_l",
        "cam_r",
        "imu0",
    )
    .unwrap();
    let v = &c["value0"];
    assert_eq!(v["gyro_noise_std"], serde_json::json!([1e-3, 1e-3, 1e-3]));
    assert_eq!(v["accel_noise_std"][1], 2e-2);
    assert_eq!(v["gyro_bias_std"][0], 3e-5);
    assert_eq!(v["accel_bias_std"][0], 4e-4);
    assert_eq!(v["imu_update_rate"], calib::IMU_RATE_HZ);
}

#[test]
fn config_is_realtime_with_every_key() {
    let c = basalt_config();
    let v = c["value0"].as_object().unwrap();
    assert_eq!(v["config.vio_enforce_realtime"], true);
    assert_eq!(v["config.vio_linearization_type"], "ABS_QR");
    // Every key upstream's VioConfig serialiser reads (cereal needs them all).
    assert_eq!(v.len(), 48);
}

#[test]
fn hello_and_imu_framing() {
    let mut b = Vec::new();
    encode_hello(&mut b);
    assert_eq!(b, [0, 4, 0, 0, 0, 1, 0, 0, 0]);
    b.clear();
    encode_imu(
        &mut b,
        0x0102_0304_0506_0708,
        [1.0, 2.0, 3.0],
        [4.0, 5.0, -9.81],
    );
    assert_eq!(b.len(), HEADER_LEN + IMU_LEN);
    assert_eq!(&b[..5], &[MSG_IMU, 56, 0, 0, 0]);
    assert_eq!(&b[5..13], &[8, 7, 6, 5, 4, 3, 2, 1]);
    assert_eq!(&b[13..21], &1.0f64.to_le_bytes());
    assert_eq!(&b[53..61], &(-9.81f64).to_le_bytes());
}

#[test]
fn stereo_framing() {
    let mut b = Vec::new();
    encode_stereo(
        &mut b,
        42,
        3,
        2,
        &[1, 2, 3, 4, 5, 6],
        &[7, 8, 9, 10, 11, 12],
    );
    assert_eq!(b.len(), stereo_message_len(3, 2));
    assert_eq!(b[0], MSG_STEREO);
    assert_eq!(u32::from_le_bytes(b[1..5].try_into().unwrap()), 28);
    assert_eq!(u64::from_le_bytes(b[5..13].try_into().unwrap()), 42);
    assert_eq!(u32::from_le_bytes(b[13..17].try_into().unwrap()), 3);
    assert_eq!(u32::from_le_bytes(b[17..21].try_into().unwrap()), 2);
    assert_eq!(&b[21..], &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);
}

#[test]
fn imu_batch_is_rotated_into_imu_frame() {
    let batch = ImuBatch {
        samples: vec![
            ImuSample {
                t_ns: 10,
                gyro: vec![0.1, 0.0, 0.0],
                accel: vec![1.0, 0.0, 9.81],
                ..Default::default()
            },
            ImuSample {
                t_ns: 11,
                gyro: vec![0.1, 0.0],
                accel: vec![1.0, 0.0, 9.81],
                ..Default::default()
            },
        ],
    };
    // IMU mounted rotated 90 degrees about z: R_imu_body = R_z(-90).
    let mut b = Vec::new();
    assert_eq!(encode_imu_batch(&se3::transpose(&R_Z90), &batch, &mut b), 1);
    let f = |o: usize| f64::from_le_bytes(b[o..o + 8].try_into().unwrap());
    assert_close(&[f(13), f(21), f(29)], &[0.0, -0.1, 0.0], 1e-15);
    assert_close(&[f(37), f(45), f(53)], &[0.0, -1.0, 9.81], 1e-15);
}

pub(super) const POSE: &str =
    r#"{"t_ns":1000,"T_w_i":[1,0,0,1, 0,1,0,2, 0,0,1,3, 0,0,0,1],"cov":null,"features":42}"#;

#[test]
fn parses_status_and_state_lines() {
    assert_eq!(
        parse_line(r#"{"status":"tracking"}"#),
        Some(ChildLine::Status(ChildStatus::Tracking))
    );
    assert_eq!(
        parse_line(r#"{"status":"lost"}"#),
        Some(ChildLine::Status(ChildStatus::Lost))
    );
    let Some(ChildLine::State(s)) = parse_line(POSE) else {
        panic!()
    };
    assert_eq!(
        (s.t_ns, s.t_w_i[3], s.cov, s.features),
        (1000, 1.0, None, Some(42))
    );
    let cov: Vec<String> = (0..36).map(|i| i.to_string()).collect();
    let line = format!(
        r#"{{"t_ns":5,"T_w_i":{:?},"cov":[{}],"features":null}}"#,
        se3::to_mat4(&se3::IDENTITY, [0.0; 3]),
        cov.join(",")
    );
    let Some(ChildLine::State(s)) = parse_line(&line) else {
        panic!("{line}")
    };
    assert_eq!((s.cov.unwrap()[35], s.features), (35.0, None));
    assert_eq!(parse_line(r#"{"t_ns":5,"T_w_i":[1,0,0],"cov":null}"#), None);
    assert_eq!(parse_line(r#"{"status":"sleeping"}"#), None);
    assert_eq!(parse_line("basalt says hi"), None);
}

#[test]
fn state_becomes_body_pose_in_world() {
    let t_body_imu = se3::to_mat4(&R_Z90, [0.1, 0.0, 0.02]);
    let t_imu_body = mat4_inv(&t_body_imu);
    let t_w_i = se3::to_mat4(&R_BODY_CAM, [1.0, 2.0, 3.0]);
    let st = ChildState {
        t_ns: 7,
        t_w_i,
        cov: None,
        features: Some(3),
    };
    let e = to_estimate(&st, &t_imu_body, VioHealth::Good);
    let want = mat4_mul(&t_w_i, &t_imu_body);
    assert_close(&se3::to_mat4(&e.r, e.t), &want, 1e-12);
    assert_eq!(e.cov, nominal_cov(VioHealth::Good));
    assert_eq!(e.source, PoseSource::Vio);
}

#[test]
fn status_lines_drive_health() {
    let track = PoseTrack::new();
    let id = se3::to_mat4(&se3::IDENTITY, [0.0; 3]);
    let h = apply_line(&track, &id, VioHealth::Lost, r#"{"status":"initializing"}"#);
    assert_eq!(
        (h, track.health()),
        (VioHealth::Degraded, VioHealth::Degraded)
    );
    let h = apply_line(&track, &id, h, r#"{"status":"tracking"}"#);
    apply_line(&track, &id, h, POSE);
    assert_eq!(track.health(), VioHealth::Good);
    assert_eq!(track.quality().tracked_features, Some(42));
    assert_eq!(track.latest().unwrap().t, [1.0, 2.0, 3.0]);
}
