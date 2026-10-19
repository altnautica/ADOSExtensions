//! Shared helpers for the calibration tests.

use std::fs;
use std::path::{Path, PathBuf};

use ados_capture_contract::Rig;
use serde_json::json;

/// `node/tests/fixtures/calib/<name>` as text.
pub fn fixture(name: &str) -> String {
    let p = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/calib")
        .join(name);
    fs::read_to_string(&p).unwrap_or_else(|e| panic!("{}: {e}", p.display()))
}

/// The committed tiny capture session (2 frames of `cam0`, 20 `imu0` samples).
pub fn tiny_session() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../contract/fixtures/tiny-session/sim-1-20261019T000000Z-0001")
}

/// A copy of the tiny session under `root` with `edit` applied to its rig.
pub fn tiny_session_with(root: &Path, edit: impl FnOnce(&mut Rig)) -> PathBuf {
    let src = tiny_session();
    let dst = root.join("session");
    fs::create_dir_all(dst.join("segments")).unwrap();
    for f in ["manifest.json", "segments/000000.mcap"] {
        fs::copy(src.join(f), dst.join(f)).unwrap();
    }
    let mut rig: Rig = serde_json::from_slice(&fs::read(src.join("rig.json")).unwrap()).unwrap();
    edit(&mut rig);
    fs::write(
        dst.join("rig.json"),
        serde_json::to_vec_pretty(&rig).unwrap(),
    )
    .unwrap();
    dst
}

const CAM_R: [f64; 16] = [
    0.0, 0.0, 1.0, 0.1, -1.0, 0.0, 0.0, 0.05, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

/// Two forward cameras (nominal, uncalibrated), an IMU 1 cm forward and 3 cm
/// up, and a LiDAR.
pub fn two_camera_rig() -> Rig {
    let cam = |id: &str, role: &str, y: f64| {
        let mut t = CAM_R;
        t[7] = y;
        json!({
            "id": id, "kind": "camera", "role": role, "T_body_sensor": t,
            "time_offset_ns": 0,
            "camera": {"model": "pinhole", "width": 752, "height": 480, "fx": 450.0,
                "fy": 450.0, "cx": 376.0, "cy": 240.0, "dist": [], "calibrated": false,
                "readout_ns": 0}
        })
    };
    serde_json::from_value(json!({
        "contract": "ados.capture/1",
        "rig_id": "test-rig",
        "calib_version": "3",
        "sensors": [
            cam("cam_l", "front_left", 0.05),
            {"id": "imu0", "kind": "imu", "role": "nav", "time_offset_ns": 0,
             "T_body_sensor": [1.0, 0.0, 0.0, 0.01, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.03,
                               0.0, 0.0, 0.0, 1.0]},
            cam("cam_r", "front_right", -0.05),
            {"id": "lidar0", "kind": "lidar", "role": "up", "time_offset_ns": 0,
             "T_body_sensor": [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.12,
                               0.0, 0.0, 0.0, 1.0]}
        ]
    }))
    .unwrap()
}

/// Element-wise comparison within `tol`.
pub fn assert_close(got: &[f64], want: &[f64], tol: f64) {
    assert_eq!(got.len(), want.len());
    for (i, (g, w)) in got.iter().zip(want).enumerate() {
        assert!((g - w).abs() <= tol, "[{i}] got {g}, want {w}\n{got:?}");
    }
}
