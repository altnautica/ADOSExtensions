use super::*;
use serde_json::json;

fn pose(t_ns: u64, x: f64, y: f64, yaw: f64, roll: f64, pitch: f64) -> PoseOut {
    PoseOut {
        t_ns,
        x,
        y,
        yaw,
        roll,
        pitch,
        score: 0.8,
        degeneracy: 0.5,
    }
}

#[test]
fn lidar_poses_become_level_body_poses_at_the_first_height() {
    // LiDAR 0.30 m ahead of and 0.25 m above the body origin, level.
    let mut mount = [0.0; 16];
    for i in 0..4 {
        mount[i * 5] = 1.0;
    }
    (mount[3], mount[11]) = (0.30, 0.25);
    let poses = [
        pose(1, 0.0, 0.0, 0.0, 0.02, -0.01),
        pose(2, 1.0, 0.5, 0.3, 0.0, 0.0),
    ];
    let body = body_trajectory(&poses, &mount);
    assert_eq!(body.len(), 2);
    assert_eq!(body[0].1[11], body[1].1[11]);
    assert!(body[0].1[11].abs() < 0.01);
    // A yaw of the LiDAR about its own centre swings the body around it.
    let (s, c) = 0.3f64.sin_cos();
    assert!((body[1].1[3] - (1.0 + 0.30 - 0.30 * c)).abs() < 1e-12);
    assert!((body[1].1[7] - (0.5 - 0.30 * s)).abs() < 1e-12);
    assert!((body[1].1[0] - c).abs() < 1e-12 && (body[1].1[4] - s).abs() < 1e-12);
}

#[test]
fn only_a_planar_lidar_is_matched_in_process() {
    let rig = |lidar: serde_json::Value| -> Rig {
        serde_json::from_value(json!({
            "contract": "ados.capture/1", "rig_id": "r", "calib_version": "1",
            "sensors": [{"id": "lidar0", "kind": "lidar", "role": "front_left",
                "time_offset_ns": 0,
                "T_body_sensor": [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                  0.0, 0.0, 0.0, 1.0],
                "lidar": lidar}]
        }))
        .unwrap()
    };
    let planar = rig(crate::calib::test_util::rplidar_c1());
    assert_eq!(
        rig_planar_lidar(&planar).map(|s| s.id.as_str()),
        Some("lidar0")
    );
    assert!(rig_planar_lidar(&rig(crate::calib::test_util::mid360())).is_none());
}
