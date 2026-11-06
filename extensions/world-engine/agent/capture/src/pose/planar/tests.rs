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

/// Live order: each scan reaches the matcher before the IMU batch covering
/// it, while the body yaw zigzags (+90°/s, then −90°/s, each revolution).
/// Once an IMU stream exists, a scan waits until the IMU reaches its last
/// point, so the matcher never extrapolates the previous revolution's rate
/// across it; the batch releases it, and the heading stays on the truth.
#[test]
fn a_scan_waits_for_the_imu_that_covers_it() {
    let walls = rect(0.0, 0.0, 6.0, 4.0);
    let w = 90f64.to_radians();
    let rate = |i: u64| if i.is_multiple_of(2) { w } else { -w };
    // A triangle wave: up over even revolutions, back down over odd ones.
    let yaw = move |t: u64| {
        let dt = t - T0;
        let (i, within) = (dt / PERIOD_NS, (dt % PERIOD_NS) as f64 * 1e-9);
        let peak = w * PERIOD_NS as f64 * 1e-9;
        if i.is_multiple_of(2) {
            w * within
        } else {
            peak - w * within
        }
    };
    let body = move |t: u64| [3.0, 2.0, yaw(t)];
    let p = spawn();
    let mut est = Vec::new();
    let wait_for = |n: u64| {
        let deadline = Instant::now() + Duration::from_secs(60);
        while p.stats().scans_processed < n {
            assert!(Instant::now() < deadline, "scan {n} not matched in 60 s");
            std::thread::sleep(Duration::from_millis(1));
        }
    };
    for i in 0..40u64 {
        let t = T0 + i * PERIOD_NS;
        p.feed_lidar(&scan(t, &walls, &body));
        if i > 0 {
            // The IMU ends a revolution before this scan's last point.
            std::thread::sleep(Duration::from_millis(50));
            assert_eq!(
                p.stats().scans_processed,
                i,
                "scan {i} matched before its IMU arrived"
            );
        }
        p.feed_imu("imu0", &imu(t, t + PERIOD_NS, rate(i)));
        wait_for(i + 1);
        p.drain_new(&mut est);
    }
    let mut trusted = 0;
    for e in est.iter().filter(|e| e.health != VioHealth::Lost) {
        let got = e.r[3].atan2(e.r[0]);
        let err = (got - yaw(e.t_ns)).sin().asin().to_degrees().abs();
        assert!(err < 1.5, "t {} yaw err {err:.2}°", e.t_ns);
        trusted += 1;
    }
    assert!(trusted >= 10, "{trusted} trusted poses");
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

/// Down the same corridor at 1.5 m/s with the FC's ground velocity (body
/// frame, the IMU's) arriving beside the IMU: the pose follows it along the
/// corridor, which the scans cannot see, and stays `Degraded` while it does.
/// A velocity named for another IMU is ignored, and the pose falls behind.
#[test]
fn a_corridor_walk_follows_the_measured_velocity_and_stays_degraded() {
    let walls = vec![([-100.0, 0.0], [100.0, 0.0]), ([-100.0, 2.0], [100.0, 2.0])];
    let v = 1.5;
    let body = |t: u64| [v * (t - T0) as f64 * 1e-9, 1.0, 0.0];
    let run = |src: &str| {
        let p = spawn();
        let mut out = Vec::new();
        for i in 0..40u64 {
            let t = T0 + i * PERIOD_NS;
            p.feed_imu("imu0", &imu(t, t + PERIOD_NS, 0.0));
            let vel: Vec<VelocitySample> = (t..t + PERIOD_NS)
                .step_by(20_000_000)
                .map(|t_ns| VelocitySample { t_ns, v: [v, 0.0] })
                .collect();
            p.feed_velocity(src, &vel);
            p.feed_lidar(&scan(t, &walls, &body));
            let deadline = Instant::now() + Duration::from_secs(60);
            while p.stats().scans_processed < i + 1 {
                assert!(Instant::now() < deadline, "scan {i} not matched in 60 s");
                std::thread::sleep(Duration::from_millis(1));
            }
            p.drain_new(&mut out);
        }
        out
    };
    let est = run("imu0");
    let trusted: Vec<_> = est.iter().filter(|e| e.health != VioHealth::Lost).collect();
    assert!(trusted.len() >= 20, "{} trusted poses", trusted.len());
    assert!(trusted.iter().all(|e| e.health == VioHealth::Degraded));
    let last = est.last().unwrap();
    let truth = v * (last.t_ns - T0) as f64 * 1e-9;
    assert!(
        (last.t[0] - truth).abs() < 0.10 && last.t[1].abs() < 0.05,
        "at {:?}, walked {truth:.2} m",
        last.t
    );
    let other = run("imu1");
    let lag = truth - other.last().unwrap().t[0];
    assert!(lag > 1.0, "without the velocity the pose lags only {lag:.2} m");
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

/// The FC as the matcher's IMU (`atlas.lidar.imu = "fc"`): its `RAW_IMU`
/// frames go from the MAVLink delivery straight to the matcher through the
/// FC feed's sink, so a scan the IMU has not covered yet is matched by the
/// frame that covers it, with nothing else fed: no pose tick, no next scan.
#[test]
fn the_raw_imu_frame_that_covers_a_scan_releases_it() {
    use crate::mavlink_msg::{frame_v2, MSG_RAW_IMU};
    use crate::nav::fc_imu::{self, FeedState};
    use crate::pose::fc_ekf::FcEkfPose;

    /// `RAW_IMU` at 100 Hz.
    const STEP_NS: u64 = 10_000_000;
    /// The FC's clock runs this far ahead of the host's.
    const FC_AHEAD_NS: u64 = 4_000_000_000;
    let walls = rect(0.0, 0.0, 6.0, 4.0);
    let body = |_t: u64| [3.0, 2.0, 0.0];
    let p = Arc::new(
        PlanarLidarPose::spawn(PlanarLidarConfig {
            rig: rig(),
            lidar_id: "lidar0".into(),
            imu_id: fc_imu::FC_IMU_ID.into(),
        })
        .expect("spawn"),
    );
    let fc = FcEkfPose::new();
    let state = Arc::new(FeedState::default());
    fc.set_nav_sink(Some(fc_imu::sink(p.clone(), true, state.clone())));
    // A level, still FC; each frame is read the instant it is sampled.
    let raw_imu = |t: u64| {
        let mut b = ((t + FC_AHEAD_NS) / 1_000).to_le_bytes().to_vec();
        for v in [0i16, 0, -1000, 0, 0, 0, 0, 0, 0] {
            b.extend_from_slice(&v.to_le_bytes());
        }
        fc.on_frame(&frame_v2(MSG_RAW_IMU, &b), t);
    };
    let mut t_imu = T0 - 20 * STEP_NS;
    while t_imu <= T0 {
        raw_imu(t_imu);
        t_imu += STEP_NS;
    }
    for i in 0..5u64 {
        let s = scan(T0 + i * PERIOD_NS, &walls, &body);
        let end = s.t_ns + u64::from(*s.t_offset_ns.iter().max().expect("points"));
        p.feed_lidar(&s);
        while t_imu + IMU_COVER_SLACK_NS < end {
            raw_imu(t_imu);
            t_imu += STEP_NS;
            std::thread::sleep(Duration::from_millis(2));
            assert_eq!(
                p.stats().scans_processed,
                i,
                "scan {i} matched before the IMU covered it"
            );
        }
        raw_imu(t_imu);
        let deadline = Instant::now() + Duration::from_secs(5);
        while p.stats().scans_processed < i + 1 {
            assert!(Instant::now() < deadline, "the covering frame left scan {i} waiting");
            std::thread::sleep(Duration::from_millis(1));
        }
        t_imu += STEP_NS;
    }
    assert!(state.raw_imu());
    assert_eq!(state.newest_imu_ns(), Some(t_imu - STEP_NS));
    // 21 up to the first scan, then one every 10 ms to T0 + 490 ms.
    assert_eq!(state.take_imu_samples(), 21 + 49);
    assert_eq!(p.stats().imu_dropped, 0);
}
