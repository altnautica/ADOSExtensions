//! The planar provider fed synthetic scans: a ray caster over line-segment
//! walls, a level IMU and a known body motion.

use std::f64::consts::PI;
use std::time::{Duration, Instant};

use ados_capture_contract::v1::{ImuBatch, ImuSample as BusImu};
use serde_json::json;

use super::*;

const PERIOD_NS: u64 = 100_000_000;
const BEAMS: usize = 360;
const T0: u64 = 1_000_000_000;
/// LiDAR 0.30 m ahead of the body origin and 0.25 m up, level.
const LIDAR_X: f64 = 0.30;

type Seg = ([f64; 2], [f64; 2]);

fn rig() -> Rig {
    serde_json::from_value(json!({
        "contract": "ados.capture/1",
        "rig_id": "planar",
        "calib_version": "1",
        "sensors": [
            { "id": "lidar0", "kind": "lidar", "role": "front_left", "time_offset_ns": 0,
              "T_body_sensor": [1,0,0,LIDAR_X, 0,1,0,0, 0,0,1,0.25, 0,0,0,1],
              "lidar": { "model": "rplidar-c1", "pattern": "planar", "h_fov_deg": 360.0,
                         "v_min_deg": 0.0, "v_max_deg": 0.0, "rate_hz": 10.0,
                         "points_per_s": 5000.0, "min_range_m": 0.05, "max_range_m": 12.0,
                         "range_sigma_m": 0.03, "blocked_azimuth_deg": [[130.0, 230.0]] } },
            { "id": "imu0", "kind": "imu", "role": "nav", "time_offset_ns": 0,
              "T_body_sensor": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1] }
        ]
    }))
    .expect("rig")
}

fn spawn() -> PlanarLidarPose {
    PlanarLidarPose::spawn(PlanarLidarConfig {
        rig: rig(),
        lidar_id: "lidar0".into(),
        imu_id: "imu0".into(),
    })
    .expect("spawn")
}

fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<Seg> {
    vec![
        ([x0, y0], [x1, y0]),
        ([x1, y0], [x1, y1]),
        ([x1, y1], [x0, y1]),
        ([x0, y1], [x0, y0]),
    ]
}

/// Nearest wall hit along `angle` from `o`.
fn cast(o: [f64; 2], angle: f64, walls: &[Seg]) -> Option<f64> {
    let d = [angle.cos(), angle.sin()];
    walls
        .iter()
        .filter_map(|(a, b)| {
            let e = [b[0] - a[0], b[1] - a[1]];
            let den = d[0] * e[1] - d[1] * e[0];
            if den.abs() < 1e-12 {
                return None;
            }
            let w = [a[0] - o[0], a[1] - o[1]];
            let t = (w[0] * e[1] - w[1] * e[0]) / den;
            let u = (w[0] * d[1] - w[1] * d[0]) / den;
            (t > 0.0 && (0.0..=1.0).contains(&u)).then_some(t)
        })
        .min_by(f64::total_cmp)
}

/// One revolution starting at `t0`, every beam cast from the body pose at its
/// own time. Beams in the operator's sector see the operator at 0.3 m.
fn scan(t0: u64, walls: &[Seg], body: &dyn Fn(u64) -> [f64; 3]) -> LidarScan {
    let mut s = LidarScan {
        sensor_id: "lidar0".into(),
        t_ns: t0,
        ..LidarScan::default()
    };
    for k in 0..BEAMS {
        let off = (k as u64 * PERIOD_NS / BEAMS as u64) as u32;
        let [x, y, yaw] = body(t0 + off as u64);
        let o = [x + LIDAR_X * yaw.cos(), y + LIDAR_X * yaw.sin()];
        let az = k as f64 * 2.0 * PI / BEAMS as f64;
        let deg = az.to_degrees();
        let r = if (130.0..=230.0).contains(&deg) {
            Some(0.3)
        } else {
            cast(o, yaw + az, walls).filter(|r| *r <= 12.0)
        };
        if let Some(r) = r {
            s.points
                .push([(r * az.cos()) as f32, (r * az.sin()) as f32, 0.0]);
            s.t_offset_ns.push(off);
            s.intensity.push(47);
        }
    }
    s
}

/// A level IMU turning at `yaw_rate` over `[t0, t1)`, 200 Hz.
fn imu(t0: u64, t1: u64, yaw_rate: f64) -> ImuBatch {
    ImuBatch {
        samples: (t0..t1)
            .step_by(5_000_000)
            .map(|t| BusImu {
                t_ns: t,
                gyro: vec![0.0, 0.0, yaw_rate],
                accel: vec![0.0, 0.0, 9.81],
                ..Default::default()
            })
            .collect(),
    }
}

/// Feed `n` revolutions and their IMU, waiting for each to be matched.
fn drive(
    p: &PlanarLidarPose,
    walls: &[Seg],
    body: &dyn Fn(u64) -> [f64; 3],
    yaw_rate: f64,
    n: usize,
) -> Vec<PoseEstimate> {
    let mut out = Vec::new();
    for i in 0..n {
        let t = T0 + i as u64 * PERIOD_NS;
        p.feed_imu("imu0", &imu(t, t + PERIOD_NS, yaw_rate));
        p.feed_lidar(&scan(t, walls, body));
        let deadline = Instant::now() + Duration::from_secs(60);
        while p.stats().scans_processed < i as u64 + 1 {
            assert!(Instant::now() < deadline, "scan {i} not matched in 60 s");
            std::thread::sleep(Duration::from_millis(1));
        }
        p.drain_new(&mut out);
    }
    out
}

#[test]
fn a_box_room_walk_is_tracked_within_ten_centimetres_and_good() {
    let walls = rect(0.0, 0.0, 6.0, 4.0);
    let start = [1.5, 2.0];
    let v = 0.5;
    let body = |t: u64| [start[0] + v * (t - T0) as f64 * 1e-9, start[1], 0.0];
    let p = spawn();
    let est = drive(&p, &walls, &body, 0.0, 50);
    assert_eq!(est.len(), 50);
    assert_eq!(
        est[0].health,
        VioHealth::Lost,
        "lost until the submap fills"
    );
    assert_eq!(p.stats().scans_dropped, 0);
    // World = body frame at the first scan: truth is (v t, 0, 0).
    let mut trusted = 0;
    for e in est.iter().filter(|e| e.health != VioHealth::Lost) {
        let truth = v * (e.t_ns - T0) as f64 * 1e-9;
        let err = ((e.t[0] - truth).powi(2) + e.t[1].powi(2)).sqrt();
        assert!(err < 0.10, "t {} err {err:.3} at {:?}", e.t_ns, e.t);
        assert!(e.t[2].abs() < 0.02, "level walk stays at z 0: {:?}", e.t);
        trusted += 1;
    }
    assert!(trusted >= 20, "{trusted} trusted poses");
    let last = p.latest().expect("pose");
    assert_eq!(last.health, VioHealth::Good);
    assert_eq!(last.source, PoseSource::Lidar);
    let truth = v * (last.t_ns - T0) as f64 * 1e-9;
    assert!(
        (last.t[0] - truth).abs() < 0.10 && last.t[1].abs() < 0.10,
        "{:?}",
        last.t
    );
    assert!(p.quality().degeneracy_ratio.unwrap() > 0.05);
    assert!(p.single_floor());
}

#[test]
fn a_featureless_corridor_is_degraded() {
    // 2 m wide, its ends far beyond the 12 m range.
    let walls = vec![([-100.0, 0.0], [100.0, 0.0]), ([-100.0, 2.0], [100.0, 2.0])];
    let rate = 25f64.to_radians();
    let body = |t: u64| [0.0, 1.0, rate * (t - T0) as f64 * 1e-9];
    let p = spawn();
    let est = drive(&p, &walls, &body, rate, 20);
    let trusted: Vec<_> = est.iter().filter(|e| e.health != VioHealth::Lost).collect();
    assert!(
        trusted.len() >= 5,
        "{} poses past the first submap",
        trusted.len()
    );
    assert!(trusted.iter().all(|e| e.health == VioHealth::Degraded));
    assert!(p.quality().degeneracy_ratio.unwrap() < 0.05);
}

#[test]
fn only_the_configured_sensors_are_read_and_a_full_queue_drops() {
    let walls = rect(0.0, 0.0, 6.0, 4.0);
    let body = |_: u64| [2.0, 2.0, 0.0];
    let p = spawn();
    let s = scan(T0, &walls, &body);
    p.feed_lidar(&LidarScan {
        sensor_id: "lidar1".into(),
        ..s.clone()
    });
    p.feed_imu("imu1", &imu(T0, T0 + PERIOD_NS, 0.0));
    let mut bad = s.clone();
    bad.t_offset_ns.pop();
    p.feed_lidar(&bad);
    assert_eq!(p.stats().scans_rejected, 1);
    for i in 0..10u64 {
        p.feed_lidar(&LidarScan {
            t_ns: T0 + i * PERIOD_NS,
            ..s.clone()
        });
    }
    let deadline = Instant::now() + Duration::from_secs(60);
    loop {
        let st = p.stats();
        if st.scans_processed + st.scans_dropped == 10 {
            assert!(st.scans_dropped >= 1, "{st:?}");
            assert_eq!(st.imu_dropped, 0);
            break;
        }
        assert!(Instant::now() < deadline, "{st:?}");
        std::thread::sleep(Duration::from_millis(5));
    }
}

#[test]
fn health_and_frames_follow_the_matcher() {
    let p = PoseOut {
        t_ns: 1,
        x: 1.0,
        y: 2.0,
        yaw: PI / 2.0,
        roll: 0.0,
        pitch: 0.0,
        score: 0.7,
        degeneracy: 0.3,
    };
    assert_eq!(health_of(&p, MIN_SUBMAP_SCANS - 1, 0.05), VioHealth::Lost);
    assert_eq!(health_of(&p, MIN_SUBMAP_SCANS, 0.05), VioHealth::Good);
    let corridor = PoseOut {
        degeneracy: 0.01,
        ..p
    };
    assert_eq!(health_of(&corridor, 30, 0.05), VioHealth::Degraded);
    let unmatched = PoseOut { score: 0.0, ..p };
    assert_eq!(health_of(&unmatched, 30, 0.05), VioHealth::Degraded);
    // T_w_l: the planar pose at z 0, yawed +90 degrees.
    let (r, t) = se3::from_mat4(&t_w_l(&p));
    assert_eq!(t, [1.0, 2.0, 0.0]);
    let x = se3::mat_vec(&r, [1.0, 0.0, 0.0]);
    assert!(x[0].abs() < 1e-12 && (x[1] - 1.0).abs() < 1e-12);
    // A pitched pose tips +x down.
    let pitched = PoseOut {
        yaw: 0.0,
        pitch: 0.1,
        ..p
    };
    let (r, _) = se3::from_mat4(&t_w_l(&pitched));
    assert!((se3::mat_vec(&r, [1.0, 0.0, 0.0])[2] + 0.1f64.sin()).abs() < 1e-12);
}

#[test]
fn a_rig_without_the_planar_lidar_is_refused() {
    let err = PlanarLidarPose::spawn(PlanarLidarConfig {
        rig: rig(),
        lidar_id: "imu0".into(),
        imu_id: "imu0".into(),
    })
    .err()
    .expect("refused");
    assert!(matches!(&err, PlanarPoseError::NoLidar(id) if id == "imu0"));
}
