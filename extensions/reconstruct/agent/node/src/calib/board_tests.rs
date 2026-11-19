use super::*;
use crate::calib::test_util::render_board;
use ados_capture_contract::CameraModelModel;

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

/// The board centred at `center` (camera frame), tilted by `tilt` (axis-angle,
/// radians) from facing the camera squarely.
pub(crate) fn board_pose(
    grid: &AprilGrid,
    tilt: Vector3<f64>,
    center: Vector3<f64>,
) -> (Matrix3<f64>, Vector3<f64>) {
    let r = exp_so3(&tilt);
    let e = Board::new(grid).extent();
    (r, center - r * Vector3::new(e[0] / 2.0, e[1] / 2.0, 0.0))
}

#[test]
fn tag_centres_give_the_board_plane() {
    let grid = AprilGrid::default();
    let board = Board::new(&grid);
    let cam = camera();
    for (tilt, center) in [
        (Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0)),
        (
            Vector3::new(0.35, -0.25, 0.1),
            Vector3::new(0.05, -0.03, 1.1),
        ),
        (
            Vector3::new(-0.3, 0.4, -0.2),
            Vector3::new(-0.04, 0.05, 1.2),
        ),
    ] {
        let (r, t) = board_pose(&grid, tilt, center);
        let img = render_board(&grid, &cam, &r, &t);
        let pose = board.detect(&img, &cam).expect("board found");
        assert!(pose.tags >= 30, "{} tags", pose.tags);
        let (n, d) = pose.plane();
        let n_true = r.column(2).into_owned();
        let (n_true, d_true) = match n_true.dot(&t) < 0.0 {
            true => (-n_true, -n_true.dot(&t)),
            false => (n_true, n_true.dot(&t)),
        };
        let angle = n.angle(&n_true).to_degrees();
        assert!(angle < 0.3, "normal off by {angle} deg");
        assert!((d - d_true).abs() < 0.003, "d {d} vs {d_true}");
    }
}

#[test]
fn a_blank_image_has_no_board() {
    let cam = camera();
    let img = DecodedImage {
        width: cam.width,
        height: cam.height,
        channels: 1,
        data: vec![128; (cam.width * cam.height) as usize],
    };
    assert!(Board::new(&AprilGrid::default())
        .detect(&img, &cam)
        .is_none());
}

#[test]
fn pose_from_exact_points_is_exact() {
    let r = exp_so3(&Vector3::new(0.2, -0.1, 0.3));
    let t = Vector3::new(-0.2, 0.1, 1.5);
    let board: Vec<[f64; 2]> = (0..12)
        .map(|i| [(i % 4) as f64 * 0.1, (i / 4) as f64 * 0.1])
        .collect();
    let img: Vec<[f64; 2]> = board
        .iter()
        .map(|b| {
            let p = r * Vector3::new(b[0], b[1], 0.0) + t;
            [p.x / p.z, p.y / p.z]
        })
        .collect();
    let pose = pose_from_points(&board, &img).unwrap();
    assert!((pose.r - r).norm() < 1e-9 && (pose.t - t).norm() < 1e-9);
    assert!(pose.rms < 1e-12);
}
