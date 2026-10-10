//! Tests for the MAVLink builders in `mavlink_emit.rs`.

use super::*;
use crate::estimator::EstimatorState;
use ados_protocol::mavlink::parse_v2;

fn sample(rate: (f32, f32), distance: Option<f32>) -> EstimatorOutput {
    EstimatorOutput {
        timestamp_us: 0,
        output_mode: OutputMode::OpticalFlow,
        state: EstimatorState::Converged,
        flow_rate_x: Some(rate.0),
        flow_rate_y: Some(rate.1),
        integrated_x: Some(rate.0 * 0.033),
        integrated_y: Some(rate.1 * 0.033),
        integrated_xgyro: Some(0.001),
        integrated_ygyro: Some(0.002),
        integrated_zgyro: Some(0.003),
        flow_quality: Some(200),
        flow_distance_m: distance,
        flow_scale_source: None,
        integration_time_us: Some(33_000),
    }
}

fn router(firmware: Firmware) -> ComponentRouter {
    ComponentRouter::new(
        0,
        firmware,
        Arc::new(ClockAlign::new()),
        Arc::new(AtomicU64::new(0)),
    )
}

fn f32_at(b: &[u8], at: usize) -> f32 {
    f32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}

#[test]
fn optical_flow_100_fields_ardupilot() {
    let frame = router(Firmware::Ardupilot)
        .flow_frame(&sample((0.25, -0.5), Some(1.5)), 123_456)
        .unwrap();
    assert_eq!(frame[0], 0xFD);
    assert_eq!(frame[1] as usize, OPTICAL_FLOW_LEN);
    assert_eq!(frame[6], COMPONENT_OF);
    assert_eq!(u32::from_le_bytes([frame[7], frame[8], frame[9], 0]), 100);
    let p = &frame[10..10 + OPTICAL_FLOW_LEN];
    assert_eq!(u64::from_le_bytes(p[0..8].try_into().unwrap()), 123_456);
    assert!((f32_at(p, 8) - 0.375).abs() < 1e-6); // comp_m_x = rate * d
    assert!((f32_at(p, 16) - 1.5).abs() < 1e-6); // ground_distance
    assert_eq!(p[25], 200); // quality
    assert!((f32_at(p, 26) - 0.25).abs() < 1e-6); // flow_rate_x, raw
    assert!((f32_at(p, 30) + 0.5).abs() < 1e-6); // flow_rate_y, raw
    let crc_at = 10 + OPTICAL_FLOW_LEN;
    let crc = u16::from_le_bytes([frame[crc_at], frame[crc_at + 1]]);
    assert_eq!(crc, x25_crc(&frame[1..crc_at], OPTICAL_FLOW_CRC_EXTRA));

    // iNav takes the same message.
    let inav = router(Firmware::Inav)
        .flow_frame(&sample((0.25, -0.5), Some(1.5)), 1)
        .unwrap();
    assert_eq!(u32::from_le_bytes([inav[7], inav[8], inav[9], 0]), 100);
}

#[test]
fn optical_flow_base_fields_parse_with_the_dialect() {
    // With zero rates the extension bytes truncate away, so the frame is
    // a base OPTICAL_FLOW the dialect parser checks (CRC extra included).
    let msg = OpticalFlow {
        time_usec: 42,
        flow_comp_m_x: 0.5,
        flow_comp_m_y: -0.25,
        ground_distance: -1.0,
        flow_x: 7,
        flow_y: -3,
        sensor_id: 0,
        quality: 99,
        flow_rate_x: 0.0,
        flow_rate_y: 0.0,
    };
    let frame = encode_optical_flow(COMPONENT_OF, 5, &msg);
    let (header, parsed) = parse_v2(&frame).unwrap();
    assert_eq!(header.sequence, 5);
    match parsed {
        MavMessage::OPTICAL_FLOW(d) => {
            assert_eq!(d.time_usec, 42);
            assert_eq!(d.flow_x, 7);
            assert_eq!(d.flow_y, -3);
            assert_eq!(d.quality, 99);
            assert_eq!(d.ground_distance, -1.0);
        }
        other => panic!("expected OPTICAL_FLOW, got {other:?}"),
    }
}

#[test]
fn optical_flow_rad_for_px4() {
    let frame = router(Firmware::Px4)
        .flow_frame(&sample((0.3, 0.0), Some(2.0)), 9)
        .unwrap();
    let (_h, parsed) = parse_v2(&frame).unwrap();
    match parsed {
        MavMessage::OPTICAL_FLOW_RAD(d) => {
            assert_eq!(d.integration_time_us, 33_000);
            assert!((d.integrated_x - 0.3 * 0.033).abs() < 1e-6);
            assert!((d.integrated_xgyro - 0.001).abs() < 1e-7);
            assert!((d.integrated_ygyro - 0.002).abs() < 1e-7);
            assert!((d.integrated_zgyro - 0.003).abs() < 1e-7);
            assert!((d.distance - 2.0).abs() < 1e-6);
        }
        other => panic!("expected OPTICAL_FLOW_RAD, got {other:?}"),
    }
}

#[test]
fn distance_unknown_is_minus_one() {
    let msg = build_optical_flow_rad(0, 0, 1, IntegratedFlow::default(), 0, None);
    match msg {
        MavMessage::OPTICAL_FLOW_RAD(d) => assert_eq!(d.distance, -1.0),
        other => panic!("{other:?}"),
    }
    let of = build_optical_flow(0, 0, &sample((0.1, 0.1), None));
    assert_eq!(of.ground_distance, -1.0);
}

#[test]
fn sequence_increments() {
    // A component no other test sends on, so parallel tests cannot race it.
    let msg = build_timesync(1);
    let a = frame_for(77, &msg).unwrap();
    let b = frame_for(77, &msg).unwrap();
    assert_eq!(b[4], a[4].wrapping_add(1));
}

#[test]
fn set_message_interval_sent_at_start() {
    let [imu, range] = stream_interval_requests(1, 1);
    for (msg, id, us) in [
        (imu, RAW_IMU_ID, RAW_IMU_INTERVAL_US),
        (range, DISTANCE_SENSOR_ID, DISTANCE_SENSOR_INTERVAL_US),
    ] {
        let frame = of_frame(&msg).unwrap();
        let (_h, parsed) = parse_v2(&frame).unwrap();
        match parsed {
            MavMessage::COMMAND_LONG(d) => {
                assert_eq!(d.command, MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL);
                assert_eq!(d.param1, id as f32);
                assert_eq!(d.param2, us);
                assert_eq!((d.target_system, d.target_component), (1, 1));
            }
            other => panic!("expected COMMAND_LONG, got {other:?}"),
        }
    }
    assert_eq!(RAW_IMU_INTERVAL_US, 10_000.0);
    assert_eq!(DISTANCE_SENSOR_INTERVAL_US, 50_000.0);
}

#[test]
fn distance_sensor_round_trips() {
    let msg = build_distance_sensor(1000, 20, 800, 150, 0, 10);
    let frame = of_frame(&msg).unwrap();
    let (_h, parsed) = parse_v2(&frame).unwrap();
    match parsed {
        MavMessage::DISTANCE_SENSOR(d) => {
            assert_eq!(d.current_distance, 150);
            assert_eq!(d.mavtype, MavDistanceSensor::MAV_DISTANCE_SENSOR_LASER);
            assert_eq!(
                d.orientation,
                MavSensorOrientation::MAV_SENSOR_ROTATION_PITCH_270
            );
        }
        other => panic!("expected DISTANCE_SENSOR, got {other:?}"),
    }
}

#[test]
fn timesync_query_carries_ts1() {
    let frame = of_frame(&build_timesync(987_654_321)).unwrap();
    let (_h, parsed) = parse_v2(&frame).unwrap();
    match parsed {
        MavMessage::TIMESYNC(d) => {
            assert_eq!(d.tc1, 0);
            assert_eq!(d.ts1, 987_654_321);
        }
        other => panic!("expected TIMESYNC, got {other:?}"),
    }
}
