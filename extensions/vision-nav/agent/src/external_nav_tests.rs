//! Tests for `external_nav.rs`.

use super::*;
use crate::fc_nav::{Attitude, LocalPosition};
use crate::mavlink_emit::x25_crc;
use ados_protocol::mavlink::{parse_v2, MavMessage};

const IDENTITY: [f64; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];

fn yaw_matrix(yaw: f64) -> [f64; 9] {
    let (s, c) = yaw.sin_cos();
    [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]
}

fn floats(v: &[f64]) -> Value {
    Value::Array(v.iter().map(|x| Value::F64(*x)).collect())
}

fn payload(frame: &str, health: &str, r: [f64; 9], t: [f64; 3]) -> Value {
    Value::Map(vec![
        (
            Value::from("pose"),
            Value::Map(vec![
                (Value::from("R"), floats(&r)),
                (Value::from("t"), floats(&t)),
                (Value::from("cov"), Value::Nil),
            ]),
        ),
        (Value::from("anchor"), Value::Nil),
        (Value::from("ts_ms"), Value::from(1234i64)),
        (Value::from("health"), Value::from(health)),
        (Value::from("frame"), Value::from(frame)),
        (Value::from("source"), Value::from("lidar_planar")),
    ])
}

fn pose(x: f64, y: f64, yaw: f64, health: ExtnavHealth) -> PoseSample {
    PoseSample {
        ts_ms: 0,
        r: yaw_matrix(yaw),
        t: [x, y, 0.0],
        health,
    }
}

fn fc() -> FcNavSnapshot {
    FcNavSnapshot {
        local: Some(LocalPosition { z: -3.0, vz: 0.0 }),
        attitude: Some(Attitude {
            roll: 0.0,
            pitch: 0.0,
            rollspeed: 0.0,
            pitchspeed: 0.0,
            yawspeed: 0.1,
        }),
        ..FcNavSnapshot::default()
    }
}

#[test]
fn map_ned_payload_passes_through() {
    let args = Value::Map(vec![
        (
            Value::from("topic"),
            Value::from("plugin.world-engine.pose"),
        ),
        (
            Value::from("payload"),
            payload("map_ned", "good", yaw_matrix(0.5), [1.0, 2.0, 0.0]),
        ),
    ]);
    let p = parse_pose_event(&args).unwrap();
    assert_eq!(p.t, [1.0, 2.0, 0.0]);
    assert_eq!(p.health, ExtnavHealth::Good);
    assert_eq!(p.ts_ms, 1234);
    assert!((p.yaw() - 0.5).abs() < 1e-12);
}

#[test]
fn map_flu_converts_to_frd() {
    // FLU: 2 m forward, 1 m left, yaw +30 deg (counter-clockwise).
    let yaw = 30f64.to_radians();
    let p = parse_pose_payload(&payload(
        "map_flu",
        "good",
        yaw_matrix(yaw),
        [2.0, 1.0, 0.5],
    ))
    .unwrap();
    assert_eq!(p.t, [2.0, -1.0, -0.5]);
    assert!((p.yaw() + yaw).abs() < 1e-12, "yaw {}", p.yaw());
}

#[test]
fn unknown_frames_and_health_are_refused() {
    let e = parse_pose_payload(&payload("enu", "good", IDENTITY, [0.0; 3])).unwrap_err();
    assert_eq!(e.reason(), "pose_frame_unsupported:enu");
    let e = parse_pose_payload(&payload("map_ned", "great", IDENTITY, [0.0; 3])).unwrap_err();
    assert_eq!(e, PoseError::UnknownHealth("great".into()));
    let mut short = payload("map_ned", "good", IDENTITY, [0.0; 3]);
    if let Value::Map(e) = &mut short {
        e[0].1 = Value::Map(vec![(Value::from("t"), floats(&[0.0; 3]))]);
    }
    assert_eq!(
        parse_pose_payload(&short).unwrap_err(),
        PoseError::Malformed("R")
    );
}

#[test]
fn quaternion_matches_yaw() {
    let q = quat_from_euler(0.0, 0.0, std::f64::consts::FRAC_PI_2);
    let h = std::f64::consts::FRAC_1_SQRT_2;
    assert!((q[0] - h).abs() < 1e-12 && (q[3] - h).abs() < 1e-12);
}

#[test]
fn world_velocity_rotates_into_body() {
    // Heading east (yaw 90 deg), moving east: body forward.
    let b = world_to_body([0.0, 2.0, 0.0], 0.0, 0.0, std::f64::consts::FRAC_PI_2);
    assert!((b[0] - 2.0).abs() < 1e-9 && b[1].abs() < 1e-9 && b[2].abs() < 1e-9);
}

#[test]
fn builder_needs_two_samples_then_reports_velocity() {
    let mut b = OdometryBuilder::default();
    let fc = fc();
    assert!(b
        .build(0, 0, &pose(0.0, 0.0, 0.0, ExtnavHealth::Good), &fc)
        .is_none());
    let o = b
        .build(
            1,
            50_000_000,
            &pose(0.05, 0.0, 0.0, ExtnavHealth::Good),
            &fc,
        )
        .unwrap();
    assert!((o.vx - 1.0).abs() < 1e-4, "vx {}", o.vx);
    assert!(o.vy.abs() < 1e-6);
    assert_eq!(o.z, -3.0);
    assert_eq!(o.quality, 100);
    assert_eq!(o.reset_counter, 0);
    assert!((o.pose_covariance[0] - 0.0025).abs() < 1e-7);
    assert!((o.pose_covariance[11] - 0.0025).abs() < 1e-7);
    assert!((o.pose_covariance[20] - 0.0004).abs() < 1e-7);
    assert_eq!(o.pose_covariance[1], 0.0);
    assert_eq!(o.yawspeed, 0.1);
}

#[test]
fn degraded_inflates_noise_and_halves_quality() {
    let mut b = OdometryBuilder::default();
    b.build(0, 0, &pose(0.0, 0.0, 0.0, ExtnavHealth::Degraded), &fc());
    let o = b
        .build(
            0,
            100_000_000,
            &pose(0.1, 0.0, 0.0, ExtnavHealth::Degraded),
            &fc(),
        )
        .unwrap();
    assert_eq!(o.quality, 50);
    assert!((o.pose_covariance[0] - 0.09).abs() < 1e-6);
}

#[test]
fn lost_emits_nothing_and_a_jump_bumps_reset_counter() {
    let mut b = OdometryBuilder::default();
    let fc = fc();
    b.build(0, 0, &pose(0.0, 0.0, 0.0, ExtnavHealth::Good), &fc);
    assert!(b
        .build(0, 33_000_000, &pose(0.0, 0.0, 0.0, ExtnavHealth::Lost), &fc)
        .is_none());
    // Relocalised 2 m away: counter bumps, velocity window restarts.
    assert!(b
        .build(0, 66_000_000, &pose(2.0, 0.0, 0.0, ExtnavHealth::Good), &fc)
        .is_none());
    assert_eq!(b.reset_counter(), 1);
    let o = b
        .build(
            0,
            100_000_000,
            &pose(2.01, 0.0, 0.0, ExtnavHealth::Good),
            &fc,
        )
        .unwrap();
    assert_eq!(o.reset_counter, 1);
    // A small step is motion, not a reset.
    b.build(
        0,
        133_000_000,
        &pose(2.3, 0.0, 0.0, ExtnavHealth::Good),
        &fc,
    );
    assert_eq!(b.reset_counter(), 1);
}

#[test]
fn velocity_window_keeps_about_100_ms() {
    let mut b = OdometryBuilder::default();
    let fc = fc();
    for i in 0..10 {
        let t = i * 33_000_000;
        b.build(
            0,
            t,
            &pose(i as f64 * 0.033, 0.0, 0.0, ExtnavHealth::Good),
            &fc,
        );
    }
    // 1 m/s steady: the windowed difference reads 1 m/s.
    let o = b
        .build(
            0,
            330_000_000,
            &pose(0.33, 0.0, 0.0, ExtnavHealth::Good),
            &fc,
        )
        .unwrap();
    assert!((o.vx - 1.0).abs() < 1e-3, "vx {}", o.vx);
    assert!(b.history.len() <= 5);
}

#[test]
fn odometry_frame_packs_base_and_extension_fields() {
    let mut b = OdometryBuilder::default();
    let fc = fc();
    b.build(0, 0, &pose(1.0, 2.0, 0.3, ExtnavHealth::Good), &fc);
    let o = b
        .build(
            42,
            50_000_000,
            &pose(1.0, 2.0, 0.3, ExtnavHealth::Good),
            &fc,
        )
        .unwrap();
    let frame = o.encode(9);
    assert_eq!(frame[0], 0xFD);
    assert_eq!(frame[1] as usize, ODOMETRY_LEN);
    assert_eq!(frame[4], 9);
    assert_eq!(frame[6], COMPONENT_EXTNAV);
    assert_eq!(u32::from_le_bytes([frame[7], frame[8], frame[9], 0]), 331);
    let p = &frame[10..10 + ODOMETRY_LEN];
    assert_eq!(p[228], MAV_FRAME_LOCAL_FRD);
    assert_eq!(p[229], MAV_FRAME_BODY_FRD);
    assert_eq!(p[230], 0); // reset_counter
    assert_eq!(p[231], MAV_ESTIMATOR_TYPE_LIDAR);
    assert_eq!(p[232] as i8, 100); // quality
    let crc_at = 10 + ODOMETRY_LEN;
    let crc = u16::from_le_bytes([frame[crc_at], frame[crc_at + 1]]);
    assert_eq!(crc, x25_crc(&frame[1..crc_at], ODOMETRY_CRC_EXTRA));
    // The plugin host refuses any `mavlink.send` buffer that is not one exact
    // frame of its dialect, extension bytes included.
    assert!(
        ados_protocol::mavlink::decode_exact_frame(&frame).is_ok(),
        "host mavlink gate refuses the frame"
    );
    let (_, msg) = parse_v2(&frame).expect("dialect parses ODOMETRY");
    let MavMessage::ODOMETRY(d) = msg else {
        panic!("expected ODOMETRY");
    };
    assert_eq!(d.time_usec, 42);
    assert_eq!((d.x, d.y, d.z), (1.0, 2.0, -3.0));
    let yaw = 2.0 * f64::from(d.q[3]).atan2(f64::from(d.q[0]));
    assert!((yaw - 0.3).abs() < 1e-6);
}

#[test]
fn ten_hz_poses_give_one_message_each() {
    let mut b = OdometryBuilder::default();
    let fc = fc();
    // Poses at the scan rate, 100 ms apart, moving 2 m/s north.
    let mut sent = 0;
    for i in 0..10i64 {
        let x = i as f64 * 0.2;
        if let Some(o) = b.build(
            0,
            i * 100_000_000,
            &pose(x, 0.0, 0.0, ExtnavHealth::Good),
            &fc,
        ) {
            sent += 1;
            assert!(
                (o.x as f64 - x).abs() < 1e-6,
                "a message carries the measured pose"
            );
            assert!((o.vx - 2.0).abs() < 1e-4, "vx {}", o.vx);
        }
    }
    // The first pose has no velocity yet; every later pose is one message.
    assert_eq!(sent, 9);
}

#[test]
fn the_retired_slam_heading_frame_is_refused() {
    // `map_frd` was the SLAM frame at the first scan's heading; with the EKF
    // yaw on the compass it would fly rotated, so it is not accepted.
    let e = parse_pose_payload(&payload("map_frd", "good", IDENTITY, [0.0; 3])).unwrap_err();
    assert!(matches!(e, PoseError::UnknownFrame(f) if f == "map_frd"));
}
