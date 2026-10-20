//! Shared helpers for the calibration tests.

use std::fs;
use std::path::{Path, PathBuf};

use ados_capture_contract::Rig;
use serde_json::json;

/// A Mid-360-class solid-state LiDAR model block for test rigs.
pub fn mid360() -> serde_json::Value {
    json!({
        "model": "livox-mid360", "pattern": "solid", "h_fov_deg": 360.0,
        "v_min_deg": -7.0, "v_max_deg": 52.0, "rate_hz": 10.0, "points_per_s": 200000.0,
        "min_range_m": 0.1, "max_range_m": 40.0, "range_sigma_m": 0.02
    })
}

/// An RPLIDAR C1-class planar LiDAR model block for test rigs.
pub fn rplidar_c1() -> serde_json::Value {
    json!({
        "model": "rplidar-c1", "pattern": "planar", "h_fov_deg": 360.0,
        "v_min_deg": 0.0, "v_max_deg": 0.0, "rate_hz": 10.0, "points_per_s": 5000.0,
        "min_range_m": 0.05, "max_range_m": 12.0, "range_sigma_m": 0.03
    })
}

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
                               0.0, 0.0, 0.0, 1.0],
             "lidar": mid360()}
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

/// The AprilGrid `grid` as `cam` (pinhole) sees it with the board at
/// `p_cam = r · p_board + t`: white paper one tag wide around the tags on a
/// mid-grey background, 3x3 supersampled grey pixels. Tags are drawn upright
/// (data bits row-major from the top left, most significant first) with a
/// two-cell black border, and a black square fills every gap crossing.
pub fn render_board(
    grid: &super::AprilGrid,
    cam: &ados_capture_contract::CameraModel,
    r: &nalgebra::Matrix3<f64>,
    t: &nalgebra::Vector3<f64>,
) -> crate::recon::session_data::DecodedImage {
    use nalgebra::Vector3;
    let s = grid.tag_size_m;
    let g = s * grid.tag_spacing;
    let pitch = s + g;
    let (cols, rows) = (grid.tag_cols as i64, grid.tag_rows as i64);
    let (w, h) = (cols as f64 * pitch - g, rows as f64 * pitch - g);
    let codes = &aprilgrid::tag_families::T36H11;
    let shade = |bx: f64, by: f64| -> f64 {
        if bx < -g - s || by < -g - s || bx > w + g + s || by > h + g + s {
            return 128.0;
        }
        // Gap crossings, including the outer ones.
        let (ci, cj) = (((bx + g) / pitch).floor(), ((by + g) / pitch).floor());
        let (ox, oy) = (bx - (ci * pitch - g), by - (cj * pitch - g));
        if (0.0..=cols as f64).contains(&ci)
            && (0.0..=rows as f64).contains(&cj)
            && ox < g
            && oy < g
        {
            return 20.0;
        }
        let (c, r) = ((bx / pitch).floor() as i64, (by / pitch).floor() as i64);
        let (u, v) = (bx - c as f64 * pitch, by - r as f64 * pitch);
        if c < 0 || r < 0 || c >= cols || r >= rows || u >= s || v >= s {
            return 235.0;
        }
        let cell = s / 10.0;
        let (cu, cv) = ((u / cell) as i64, (v / cell) as i64);
        if !(2..8).contains(&cu) || !(2..8).contains(&cv) {
            return 20.0;
        }
        let code = codes[(r * cols + c) as usize];
        match (code >> (35 - ((cv - 2) * 6 + (cu - 2)))) & 1 {
            1 => 235.0,
            _ => 20.0,
        }
    };
    let n = r.column(2).into_owned();
    let rt = r.transpose();
    let mut data = Vec::with_capacity((cam.width * cam.height) as usize);
    for py in 0..cam.height {
        for px in 0..cam.width {
            let mut acc = 0.0;
            for sy in 0..3 {
                for sx in 0..3 {
                    let u = px as f64 + (sx as f64 + 0.5) / 3.0 - 0.5;
                    let v = py as f64 + (sy as f64 + 0.5) / 3.0 - 0.5;
                    let d = Vector3::new((u - cam.cx) / cam.fx, (v - cam.cy) / cam.fy, 1.0);
                    let depth = n.dot(t) / n.dot(&d);
                    acc += match depth.is_finite() && depth > 0.0 {
                        true => {
                            let b = rt * (d * depth - t);
                            shade(b.x, b.y)
                        }
                        false => 128.0,
                    };
                }
            }
            data.push((acc / 9.0).round() as u8);
        }
    }
    crate::recon::session_data::DecodedImage {
        width: cam.width,
        height: cam.height,
        channels: 1,
        data,
    }
}
