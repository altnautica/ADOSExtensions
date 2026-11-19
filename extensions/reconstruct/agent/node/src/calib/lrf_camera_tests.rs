use std::path::{Path, PathBuf};

use ados_capture_contract::foxglove::{PackedElementField, PointCloud, RawImage};
use ados_capture_contract::{topics, CameraModel, CameraModelModel, Manifest, SessionWriter};
use nalgebra::{Matrix3, Vector3};

use super::*;
use crate::calib::board::exp_so3;
use crate::calib::test_util::{render_board, rplidar_c1, tiny_session};
use crate::calib::{run_with, CalibRequest, CalibTarget};

const SESSION: &str = "sim-1-20261019T000000Z-0001";

fn camera() -> CameraModel {
    CameraModel {
        model: CameraModelModel::Pinhole,
        width: 800,
        height: 600,
        fx: 620.0,
        fy: 620.0,
        cx: 400.0,
        cy: 300.0,
        dist: vec![],
        calibrated: true,
        readout_ns: 0,
    }
}

/// The true mount: 3 cm ahead of and 10 cm below the camera, turned ~1.3°.
fn true_body_lidar() -> [f64; 16] {
    let r = exp_so3(&Vector3::new(0.01, -0.015, 0.02));
    from_na(&r, &Vector3::new(0.03, 0.0, -0.10))
}

/// A rig of the tiny session's camera (here 800 x 600) and a C1-class LiDAR
/// whose recorded mount is 2 cm and ~1.5° off the true one.
fn rig() -> Rig {
    let mut rig = SessionReader::open(&tiny_session()).unwrap().rig().clone();
    rig.sensors[0].camera = Some(camera());
    let nominal = mul(
        &true_body_lidar(),
        &from_na(
            &exp_so3(&Vector3::new(0.0, 0.02, -0.015)),
            &Vector3::new(0.02, 0.0, 0.0),
        ),
    );
    rig.sensors.push(
        serde_json::from_value(json!({
            "id": "lidar0", "kind": "lidar", "role": "front_left", "time_offset_ns": 0,
            "T_body_sensor": nominal, "lidar": rplidar_c1()
        }))
        .unwrap(),
    );
    rig
}

/// Board views `(tilt about camera x, about y, degrees)`, each centred on the
/// scan plane 1.0-1.5 m ahead.
fn tilts() -> Vec<(f64, f64)> {
    vec![
        (-25.0, 0.0),
        (25.0, 0.0),
        (0.0, -25.0),
        (0.0, 25.0),
        (-20.0, -20.0),
        (20.0, 20.0),
        (-20.0, 20.0),
        (20.0, -20.0),
        (10.0, -5.0),
        (-10.0, 5.0),
        (5.0, 15.0),
        (-15.0, -10.0),
    ]
}

/// One 500-beam revolution in the LiDAR frame: the board (its paper, one tag
/// wide around the tags) where a beam meets it, else a wall 4 m out; ranges
/// carry a ±1 cm deterministic ripple.
fn scan(
    t_cam_lidar: &Matrix4<f64>,
    r: &Matrix3<f64>,
    t: &Vector3<f64>,
    extent: [f64; 2],
) -> Vec<u8> {
    let (rl, tl) = (
        t_cam_lidar.fixed_view::<3, 3>(0, 0).into_owned(),
        t_cam_lidar.fixed_view::<3, 1>(0, 3).into_owned(),
    );
    let n = r.column(2).into_owned();
    let paper = 0.088;
    let mut data = Vec::new();
    for k in 0..500 {
        let az = k as f64 * std::f64::consts::TAU / 500.0;
        let dir = Vector3::new(az.cos(), az.sin(), 0.0);
        // Beam in the camera frame: origin tl, direction rl·dir.
        let dc = rl * dir;
        let s = n.dot(&(t - tl)) / n.dot(&dc);
        let b = r.transpose() * (tl + dc * s - t);
        let on_board = s > 0.0
            && (-paper..=extent[0] + paper).contains(&b.x)
            && (-paper..=extent[1] + paper).contains(&b.y);
        let range = if on_board { s } else { 4.0 } + 0.01 * ((k * 7919) % 13) as f64 / 6.0 - 0.01;
        for v in [range * az.cos(), range * az.sin(), 0.0] {
            data.extend_from_slice(&(v as f32).to_le_bytes());
        }
    }
    data
}

/// The calibration session: per view, one frame and one scan 1 ms later.
fn session(root: &Path, views: &[(f64, f64)]) -> PathBuf {
    let src = tiny_session();
    let manifest: Manifest =
        serde_json::from_slice(&std::fs::read(src.join("manifest.json")).unwrap()).unwrap();
    let rig = rig();
    let mut w = SessionWriter::create(root, manifest, &rig).unwrap();
    let grid = AprilGrid::default();
    let extent = Board::new(&grid).extent();
    let t_cam_lidar = to_na(&mul(
        &inv_rigid(&rig.sensors[0].t_body_sensor.0),
        &true_body_lidar(),
    ));
    let field = |name: &str, offset: u32| PackedElementField {
        name: name.into(),
        offset,
        r#type: 7,
    };
    for (k, &(ax, ay)) in views.iter().enumerate() {
        let dist = 1.0 + 0.1 * (k % 6) as f64;
        let lateral = 0.08 * ((k % 5) as f64 - 2.0);
        let c = (t_cam_lidar * nalgebra::Vector4::new(dist, lateral, 0.0, 1.0)).xyz();
        // Facing the camera, then tilted; board x right, y down.
        let face = nalgebra::Rotation3::rotation_between(&Vector3::z(), &c).unwrap();
        let r = exp_so3(&Vector3::new(ax.to_radians(), ay.to_radians(), 0.0)) * face.matrix();
        let t = c - r * Vector3::new(extent[0] / 2.0, extent[1] / 2.0, 0.0);
        let img = render_board(&grid, &camera(), &r, &t);
        let at = 2_000_000_000 + k as u64 * 500_000_000;
        let raw = RawImage {
            timestamp: None,
            frame_id: "cam0".into(),
            width: img.width,
            height: img.height,
            encoding: "mono8".into(),
            step: img.width,
            data: img.data,
        };
        w.write_proto(&topics::cam_image("cam0"), at, &raw).unwrap();
        let pc = PointCloud {
            timestamp: None,
            frame_id: "lidar0".into(),
            pose: None,
            point_stride: 12,
            fields: vec![field("x", 0), field("y", 4), field("z", 8)],
            data: scan(&t_cam_lidar, &r, &t, extent),
        };
        w.write_proto(&topics::lidar_points("lidar0"), at + 1_000_000, &pc)
            .unwrap();
    }
    let dir = w.dir().to_path_buf();
    w.finish().unwrap();
    dir
}

fn calibrate(session: &Path, work: &Path) -> Result<super::super::CalibOutcome, StageError> {
    let req = CalibRequest {
        session_id: SESSION.into(),
        target: CalibTarget::LidarCamera,
        board: AprilGrid::default(),
    };
    let no_tools = |tool: &str| -> Result<String, StageError> {
        Err(StageError::tool_missing(
            tool,
            "a planar LiDAR needs no container",
        ))
    };
    run_with(
        &req,
        session,
        work,
        &crate::recon::StandInRunner::default(),
        &mut |_| {},
        &no_tools,
    )
}

#[test]
fn board_views_place_a_planar_lidar() {
    let dir = tempfile::tempdir().unwrap();
    let session = session(&dir.path().join("sessions"), &tilts());
    let out = calibrate(&session, &dir.path().join("work")).unwrap();
    assert_eq!(out.report["tool"], "board_plane");
    assert_eq!(out.report["views"], 12, "{}", out.report);
    assert_eq!(out.report["frames_with_board"], 12);
    assert!(out.rig.calib_version.starts_with("1+lidar_camera-"));
    let got = to_na(&out.rig.sensors[2].t_body_sensor.0);
    let want = to_na(&true_body_lidar());
    let dt = (got.fixed_view::<3, 1>(0, 3) - want.fixed_view::<3, 1>(0, 3)).norm();
    let rel = got.fixed_view::<3, 3>(0, 0).transpose() * want.fixed_view::<3, 3>(0, 0);
    let dr = ((rel.trace() - 1.0) / 2.0)
        .clamp(-1.0, 1.0)
        .acos()
        .to_degrees();
    assert!(dt < 0.01, "translation off by {dt} m: {}", out.report);
    assert!(dr < 0.5, "rotation off by {dr} deg: {}", out.report);
    assert!(out.report["rms_m"].as_f64().unwrap() < MAX_RMS_M);
}

#[test]
fn too_few_board_views_are_refused() {
    let dir = tempfile::tempdir().unwrap();
    let session = session(&dir.path().join("sessions"), &tilts()[..4]);
    let err = calibrate(&session, &dir.path().join("work")).unwrap_err();
    assert_eq!(
        err.message,
        "need at least 10 board views at varied tilt; have 4"
    );
}

#[test]
fn ransac_keeps_the_board_line_and_drops_clutter() {
    let mut pts: Vec<[f64; 2]> = (0..30)
        .map(|i| [1.0 + 0.01 * i as f64, 0.5 - 0.02 * i as f64])
        .collect();
    pts.extend([[3.0, 3.0], [-1.0, 2.0], [0.2, 0.9], [2.5, -1.0]]);
    let line = line_inliers(&pts).unwrap();
    assert_eq!(line.len(), 30);
    assert!(line_inliers(&pts[..10]).is_none());
}
