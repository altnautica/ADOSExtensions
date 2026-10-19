use ados_capture_contract::{CameraModelModel, ImuNoise};

use super::*;
use crate::calib::test_util::{assert_close, fixture, two_camera_rig};

#[test]
fn camchain_and_imu_fixture_fold_into_the_rig() {
    let base = two_camera_rig();
    let rig = rig_from_kalibr(
        &base,
        &fixture("camchain-imucam.yaml"),
        Some(&fixture("imu.yaml")),
    )
    .unwrap();

    // cam0 -> first rig camera (cam_l), cam1 -> second (cam_r).
    let l = &rig.sensors[0];
    let r = &rig.sensors[2];
    assert_eq!((l.id.as_str(), r.id.as_str()), ("cam_l", "cam_r"));
    // T_body_cam = T_body_kimu · inv(T_cam_imu), with the IMU at (0.01, 0, 0.03).
    let want_l = [
        0.0, 0.0, 1.0, 0.11, -1.0, 0.0, 0.0, 0.05, 0.0, -1.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0,
    ];
    let want_r = [
        0.0, 0.0, 1.0, 0.11, -1.0, 0.0, 0.0, -0.05, 0.0, -1.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0,
    ];
    assert_close(&l.t_body_sensor.0, &want_l, 1e-9);
    assert_close(&r.t_body_sensor.0, &want_r, 1e-9);

    let cl = l.camera.as_ref().unwrap();
    assert_eq!(cl.model, CameraModelModel::Opencv);
    assert!(cl.calibrated);
    assert_eq!((cl.width, cl.height), (752, 480));
    assert_close(
        &[cl.fx, cl.fy, cl.cx, cl.cy],
        &[460.5, 459.25, 368.0, 247.5],
        1e-12,
    );
    assert_close(&cl.dist, &[-0.28, 0.074, 0.0002, 1.8e-5, 0.0], 1e-15);
    let cr = r.camera.as_ref().unwrap();
    assert_close(
        &[cr.fx, cr.fy, cr.cx, cr.cy],
        &[457.75, 456.5, 379.25, 255.0],
        1e-12,
    );

    // timeshift_cam_imu: t_imu = t_cam + shift, so a positive shift adds.
    assert_eq!(l.time_offset_ns, 2_500_000);
    assert_eq!(r.time_offset_ns, -1_000_000);

    assert_eq!(
        rig.sensors[1].imu_noise,
        Some(ImuNoise {
            gyro_nd: 0.00016,
            accel_nd: 0.002,
            gyro_rw: 2.2e-5,
            accel_rw: 0.003,
        })
    );
    // The LiDAR and the IMU pose are untouched.
    assert_eq!(rig.sensors[3], base.sensors[3]);
    assert_eq!(rig.sensors[1].t_body_sensor, base.sensors[1].t_body_sensor);
}

#[test]
fn camera_only_chain_hangs_off_the_first_camera() {
    let base = two_camera_rig();
    let chain = "cam0:\n  camera_model: pinhole\n  intrinsics: [400.0, 401.0, 370.0, 240.0]\n  distortion_model: equidistant\n  distortion_coeffs: [0.01, -0.002, 0.0003, -0.00004]\n  resolution: [752, 480]\n  rostopic: /cam0/image_raw\ncam1:\n  T_cn_cnm1:\n  - [1.0, 0.0, 0.0, -0.12]\n  - [0.0, 1.0, 0.0, 0.0]\n  - [0.0, 0.0, 1.0, 0.0]\n  - [0.0, 0.0, 0.0, 1.0]\n  camera_model: pinhole\n  intrinsics: [402.0, 403.0, 371.0, 241.0]\n  distortion_model: equidistant\n  distortion_coeffs: [0.02, -0.003, 0.0004, -0.00005]\n  resolution: [752, 480]\n  rostopic: /cam1/image_raw\n";
    let rig = rig_from_kalibr(&base, chain, None).unwrap();
    assert_eq!(rig.sensors[0].t_body_sensor, base.sensors[0].t_body_sensor);
    let c = rig.sensors[0].camera.as_ref().unwrap();
    assert_eq!(c.model, CameraModelModel::Kb4);
    assert_eq!(c.dist.len(), 4);
    // Camera 1 sits 12 cm to the right (camera +x) of camera 0: body -y.
    let t = &rig.sensors[2].t_body_sensor.0;
    assert_close(&[t[3], t[7], t[11]], &[0.1, 0.05 - 0.12, 0.0], 1e-12);
    assert_eq!(rig.sensors[2].time_offset_ns, 0);
}

#[test]
fn a_chain_with_more_cameras_than_the_rig_is_refused() {
    let mut base = two_camera_rig();
    base.sensors.remove(2);
    let err = rig_from_kalibr(&base, &fixture("camchain-imucam.yaml"), None).unwrap_err();
    assert_eq!(err.code, "E_INPUT");
}

#[test]
fn results_text_gives_reprojection_rms() {
    let imucam = rms_from_imucam_results(&fixture("results-imucam.txt"));
    assert_eq!(imucam.len(), 2);
    assert_close(&[imucam[0].1, imucam[1].1], &[0.5, 1.0], 1e-12);
    let cam = rms_from_cam_results(&fixture("results-cam.txt"));
    assert_eq!(cam.iter().map(|c| c.0).collect::<Vec<_>>(), vec![0, 1]);
    assert_close(&[cam[0].1, cam[1].1], &[0.5, 1.0], 1e-12);
    assert_close(&[combined_rms(&cam).unwrap()], &[0.625f64.sqrt()], 1e-12);
}

#[test]
fn yaml_writers_round_trip_through_the_importer() {
    let mut rig = two_camera_rig();
    let out = rig_from_kalibr(&rig, &fixture("camchain-imucam.yaml"), None).unwrap();
    rig.sensors[0].camera = out.sensors[0].camera.clone();
    rig.sensors[2].camera = out.sensors[2].camera.clone();
    let chain = camchain_yaml(&rig).unwrap();
    assert!(chain.contains("  distortion_model: radtan\n"), "{chain}");
    assert!(chain.contains("  rostopic: /cam1/image_raw\n"));
    // Re-importing the exported chain reproduces the rig's cameras.
    let again = rig_from_kalibr(&rig, &chain, None).unwrap();
    for i in [0, 2] {
        assert_close(
            &again.sensors[i].t_body_sensor.0,
            &rig.sensors[i].t_body_sensor.0,
            1e-12,
        );
        assert_eq!(again.sensors[i].camera, rig.sensors[i].camera);
    }

    let noise = ImuNoise {
        gyro_nd: 1.6e-4,
        accel_nd: 2.0e-3,
        gyro_rw: 2.2e-5,
        accel_rw: 3.0e-3,
    };
    let y = imu_yaml(&noise, 200.0);
    assert!(
        y.starts_with("rostopic: /imu0\nupdate_rate: 200.0\n"),
        "{y}"
    );
    assert_eq!(imu_noise_from_kalibr(&y).unwrap(), noise);
    assert_eq!(
        target_yaml(&AprilGrid::default()),
        "target_type: 'aprilgrid'\ntagCols: 6\ntagRows: 6\ntagSize: 0.088\ntagSpacing: 0.3\n"
    );
}
