//! Decoder tests against hand-authored packets.
//!
//! Each fixture is laid out byte by byte from the Mid-360 protocol (Livox
//! wiki, section 2 "Point Cloud & IMU Data Protocol") and Livox-SDK2
//! `livox_lidar_def.h`; its `crc32` was computed independently with
//! `zlib.crc32` over `timestamp` + `data` (bytes 28..length).

use ados_livox::crc::crc32;
use ados_livox::{
    ClockConfig, CrcStatus, DataType, DecodeError, Decoder, DecoderConfig, Event, Packet,
    PacketError, TimeType,
};

#[rustfmt::skip]
const CARTESIAN_HIGH: [u8; 64] = [
    0x00,                   // version 0
    0x40, 0x00,             // length 64 = 36 + 2 * 14
    0x64, 0x00,             // time_interval 100 x 0.1 us = 10 us
    0x02, 0x00,             // dot_num 2
    0x05, 0x00,             // udp_cnt 5
    0x07,                   // frame_cnt 7
    0x01,                   // data_type 1: cartesian, int32 mm
    0x00,                   // time_type 0: since power-on
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // reserved[12]
    0x63, 0x9a, 0x40, 0x7e, // crc32 0x7e409a63
    0x00, 0xca, 0x9a, 0x3b, 0x00, 0x00, 0x00, 0x00, // timestamp 1_000_000_000 ns
    // point 0: x 1000, y -2000, z 500 mm, reflectivity 100, tag 0
    0xe8, 0x03, 0x00, 0x00, 0x30, 0xf8, 0xff, 0xff, 0xf4, 0x01, 0x00, 0x00, 0x64, 0x00,
    // point 1: x 12345, y 0, z -1 mm, reflectivity 255, tag 0x10
    0x39, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0x10,
];

#[rustfmt::skip]
const CARTESIAN_LOW: [u8; 60] = [
    0x00,                   // version
    0x3c, 0x00,             // length 60 = 36 + 3 * 8
    0xc8, 0x00,             // time_interval 200 x 0.1 us = 20 us
    0x03, 0x00,             // dot_num 3
    0x00, 0x00,             // udp_cnt 0
    0x00,                   // frame_cnt
    0x02,                   // data_type 2: cartesian, int16 cm
    0x01,                   // time_type 1: PTP
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0x5a, 0xb5, 0xe5, 0xdd, // crc32 0xdde5b55a
    0x15, 0xcd, 0x0b, 0xdc, 0xac, 0xc6, 0x6c, 0x18, // timestamp 1_760_000_000_123_456_789 ns
    0x64, 0x00, 0xce, 0xff, 0x19, 0x00, 0x07, 0x00, // 100, -50, 25 cm, refl 7, tag 0
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // no return
    0xff, 0xff, 0x02, 0x00, 0xfd, 0xff, 0x09, 0x04, // -1, 2, -3 cm, refl 9, tag 0x04
];

#[rustfmt::skip]
const IMU: [u8; 60] = [
    0x00,
    0x3c, 0x00,             // length 60 = 36 + 24
    0x00, 0x00,             // time_interval 0
    0x01, 0x00,             // dot_num 1
    0x00, 0x00,
    0x00,
    0x00,                   // data_type 0: IMU
    0x00,                   // time_type 0
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0x05, 0xd2, 0x0a, 0xa3, // crc32 0xa30ad205
    0x00, 0x94, 0x35, 0x77, 0x00, 0x00, 0x00, 0x00, // timestamp 2_000_000_000 ns
    0xcd, 0xcc, 0xcc, 0x3d, // gyro_x 0.1 rad/s
    0xcd, 0xcc, 0x4c, 0xbe, // gyro_y -0.2
    0x9a, 0x99, 0x99, 0x3e, // gyro_z 0.3
    0x00, 0x00, 0x00, 0x00, // acc_x 0 g
    0x00, 0x00, 0x00, 0x00, // acc_y 0 g
    0x00, 0x00, 0x80, 0x3f, // acc_z 1 g
];

#[rustfmt::skip]
const SPHERICAL: [u8; 56] = [
    0x00,
    0x38, 0x00,             // length 56 = 36 + 2 * 10
    0x0a, 0x00,             // time_interval 1 us
    0x02, 0x00,             // dot_num 2
    0x01, 0x00,
    0x00,
    0x03,                   // data_type 3: spherical
    0x02,                   // time_type 2: GPS
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0x94, 0xe7, 0xf1, 0x84, // crc32 0x84f1e794
    0x00, 0x5e, 0xd0, 0xb2, 0x00, 0x00, 0x00, 0x00, // timestamp 3_000_000_000 ns
    0xd0, 0x07, 0x00, 0x00, 0x28, 0x23, 0x28, 0x23, 0x32, 0x00, // 2000 mm, theta 90, phi 90 deg
    0xe8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3c, 0x00, // 1000 mm, theta 0
];

fn close(a: [f32; 3], b: [f32; 3]) -> bool {
    a.iter().zip(b.iter()).all(|(x, y)| (x - y).abs() < 1e-5)
}

#[test]
fn cartesian_high_decodes() {
    let p = Packet::parse(&CARTESIAN_HIGH).unwrap();
    let h = p.header;
    assert_eq!(
        (h.version, h.length, h.dot_num, h.udp_cnt, h.frame_cnt),
        (0, 64, 2, 5, 7)
    );
    assert_eq!(h.data_type, DataType::CartesianHigh);
    assert_eq!(h.time_type, TimeType::PowerOn);
    assert_eq!(h.timestamp_ns, 1_000_000_000);
    assert_eq!(h.interval_ns(), 10_000);
    assert_eq!(p.crc_status(), CrcStatus::Valid);
    let pts: Vec<_> = p.points().collect();
    assert!(close(pts[0].xyz, [1.0, -2.0, 0.5]));
    assert_eq!(
        (pts[0].reflectivity, pts[0].tag, pts[0].offset_ns),
        (100, 0, 0)
    );
    assert!(close(pts[1].xyz, [12.345, 0.0, -0.001]));
    assert_eq!(
        (pts[1].reflectivity, pts[1].tag, pts[1].offset_ns),
        (255, 0x10, 10_000)
    );
    assert!(pts.iter().all(|p| p.valid));
    assert_eq!(p.imu().count(), 0);
}

#[test]
fn cartesian_low_decodes_and_flags_empty_returns() {
    let p = Packet::parse(&CARTESIAN_LOW).unwrap();
    assert_eq!(p.header.data_type, DataType::CartesianLow);
    assert_eq!(p.header.time_type, TimeType::Ptp);
    assert_eq!(p.header.timestamp_ns, 1_760_000_000_123_456_789);
    assert_eq!(p.crc_status(), CrcStatus::Valid);
    let pts: Vec<_> = p.points().collect();
    assert_eq!(pts.len(), 3);
    assert!(close(pts[0].xyz, [1.0, -0.5, 0.25]));
    assert!(!pts[1].valid);
    assert!(close(pts[2].xyz, [-0.01, 0.02, -0.03]));
    assert_eq!(pts[2].tag, 0x04);
    // Three samples equally spaced over 20 us.
    let offs: Vec<_> = pts.iter().map(|p| p.offset_ns).collect();
    assert_eq!(offs, [0, 10_000, 20_000]);
}

#[test]
fn spherical_decodes() {
    let p = Packet::parse(&SPHERICAL).unwrap();
    assert_eq!(p.header.time_type, TimeType::Gps);
    assert_eq!(p.crc_status(), CrcStatus::Valid);
    let pts: Vec<_> = p.points().collect();
    assert!(close(pts[0].xyz, [0.0, 2.0, 0.0]), "{:?}", pts[0].xyz);
    assert!(close(pts[1].xyz, [0.0, 0.0, 1.0]), "{:?}", pts[1].xyz);
    assert_eq!(pts[1].offset_ns, 1_000);
}

#[test]
fn imu_decodes_in_si_units() {
    let p = Packet::parse(&IMU).unwrap();
    assert_eq!(p.header.data_type, DataType::Imu);
    assert_eq!(p.crc_status(), CrcStatus::Valid);
    assert_eq!(p.points().count(), 0);
    let s: Vec<_> = p.imu().collect();
    assert_eq!(s.len(), 1);
    assert!(close(s[0].gyro, [0.1, -0.2, 0.3]));
    assert!(close(s[0].accel, [0.0, 0.0, 9.80665]));
}

#[test]
fn crc_mismatch_is_rejected_and_zero_crc_is_accepted() {
    let mut bad = CARTESIAN_HIGH;
    bad[40] ^= 0x01; // flip one bit of point 0's y
    let p = Packet::parse(&bad).unwrap();
    assert!(matches!(
        p.crc_status(),
        CrcStatus::Mismatch {
            field: 0x7e40_9a63,
            ..
        }
    ));
    let mut d = Decoder::new(DecoderConfig::default());
    let mut out = Vec::new();
    assert!(matches!(
        d.push(&bad, 5, &mut out),
        Err(DecodeError::Crc { .. })
    ));
    assert_eq!(d.stats().crc_errors, 1);

    let mut unfilled = CARTESIAN_HIGH;
    unfilled[24..28].fill(0);
    assert_eq!(
        Packet::parse(&unfilled).unwrap().crc_status(),
        CrcStatus::Absent
    );
    d.push(&unfilled, 5, &mut out).unwrap();
    assert_eq!(d.stats().crc_absent, 1);
}

#[test]
fn malformed_packets_are_rejected() {
    assert_eq!(
        Packet::parse(&CARTESIAN_HIGH[..20]).unwrap_err(),
        PacketError::Short(20)
    );
    assert!(matches!(
        Packet::parse(&CARTESIAN_HIGH[..50]).unwrap_err(),
        PacketError::Truncated {
            declared: 64,
            actual: 50
        }
    ));
    let mut t = CARTESIAN_HIGH;
    t[10] = 0x11; // dual-return type: not a Mid-360 single-return layout
    assert_eq!(
        Packet::parse(&t).unwrap_err(),
        PacketError::UnknownDataType(0x11)
    );
    let mut n = CARTESIAN_HIGH;
    n[5] = 3; // dot_num 3 does not fit 28 data bytes
    assert!(matches!(
        Packet::parse(&n).unwrap_err(),
        PacketError::DataTooShort { .. }
    ));
    // A receive buffer longer than `length` decodes the declared bytes only.
    let mut long = CARTESIAN_HIGH.to_vec();
    long.extend_from_slice(&[0xAA; 32]);
    assert_eq!(Packet::parse(&long).unwrap().crc_status(), CrcStatus::Valid);
}

/// A cartesian-high packet with `points` (mm) spaced over `interval_ns`.
fn high_packet(ts: u64, interval_ns: u64, points: &[(i32, i32, i32)]) -> Vec<u8> {
    let mut b = vec![0u8; 36];
    let len = (36 + points.len() * 14) as u16;
    b[1..3].copy_from_slice(&len.to_le_bytes());
    let interval = u16::try_from(interval_ns / 100).expect("time_interval fits 16 bits");
    b[3..5].copy_from_slice(&interval.to_le_bytes());
    b[5..7].copy_from_slice(&(points.len() as u16).to_le_bytes());
    b[10] = 1;
    b[28..36].copy_from_slice(&ts.to_le_bytes());
    for (i, &(x, y, z)) in points.iter().enumerate() {
        b.extend_from_slice(&x.to_le_bytes());
        b.extend_from_slice(&y.to_le_bytes());
        b.extend_from_slice(&z.to_le_bytes());
        b.push(i as u8);
        b.push(0);
    }
    let crc = crc32(&b[28..]);
    b[24..28].copy_from_slice(&crc.to_le_bytes());
    b
}

fn scans(out: &[Event]) -> Vec<&ados_livox::Scan> {
    out.iter()
        .filter_map(|e| match e {
            Event::Scan(s) => Some(s),
            Event::Imu(_) => None,
        })
        .collect()
}

#[test]
fn scans_split_at_period_boundaries_across_packets() {
    const MS: u64 = 1_000_000;
    let mut d = Decoder::new(DecoderConfig::default());
    let mut out = Vec::new();
    // Device clock 5 s behind the host; a packet every 5 ms carrying 5 points
    // 1 ms apart (time_interval 4 ms); host receive 1 ms after each
    // packet's last point. The packet at 5097 ms straddles the 5100 ms
    // boundary.
    let mut dev = 5_022 * MS;
    while dev < 5_300 * MS {
        let pkt = high_packet(dev, 4 * MS, &[(1000, 0, 0); 5]);
        d.push(&pkt, dev + 4 * MS + 5_000 * MS + MS, &mut out)
            .unwrap();
        dev += 5 * MS;
    }
    d.flush(&mut out);
    let s = scans(&out);
    // Windows [5000, 5100), [5100, 5200), [5200, 5300), [5300, 5400).
    assert_eq!(s.len(), 4);
    assert_eq!(s[0].device_t_ns, 5_022 * MS);
    assert_eq!(s[0].len(), 78);
    assert_eq!(s[0].t_offset_ns.last().copied(), Some((77 * MS) as u32));
    assert_eq!(s[1].device_t_ns, 5_100 * MS);
    assert_eq!((s[1].len(), s[2].len(), s[3].len()), (100, 100, 2));
    for w in s.windows(2) {
        assert_eq!(
            w[1].device_t_ns / (100 * MS),
            w[0].device_t_ns / (100 * MS) + 1
        );
    }
    // Host time = device + 5 s + 1 ms minimum latency.
    for scan in &s {
        assert_eq!(scan.t_ns, scan.device_t_ns + 5_001 * MS);
        assert_eq!(scan.points.len(), scan.t_offset_ns.len());
        assert_eq!(scan.points.len(), scan.reflectivity.len());
        assert!(scan.t_offset_ns.windows(2).all(|w| w[0] <= w[1]));
    }
    assert_eq!(s.iter().map(|s| s.len()).sum::<usize>(), 56 * 5);
}

#[test]
fn host_time_follows_minimum_latency() {
    const MS: u64 = 1_000_000;
    let mut d = Decoder::new(DecoderConfig {
        clock: ClockConfig::default(),
        ..DecoderConfig::default()
    });
    let mut out = Vec::new();
    // Latencies 9, 2, 6 ms: every later scan maps with the 2 ms floor.
    for (i, lat) in [9u64, 2, 6].into_iter().enumerate() {
        let dev = 10_000 * MS + i as u64 * 100 * MS;
        let pkt = high_packet(dev, 0, &[(500, 500, 500)]);
        d.push(&pkt, dev + 100 * MS + lat * MS, &mut out).unwrap();
    }
    d.flush(&mut out);
    let s = scans(&out);
    assert_eq!(s.len(), 3);
    // The first scan closes when the second packet arrives (offset 100+2 ms).
    assert_eq!(s[0].t_ns, s[0].device_t_ns + 102 * MS);
    assert_eq!(s[2].t_ns, s[2].device_t_ns + 102 * MS);
}

#[test]
fn clock_source_change_closes_the_open_scan_on_its_old_offset() {
    const MS: u64 = 1_000_000;
    let mut d = Decoder::new(DecoderConfig::default());
    let mut out = Vec::new();
    d.push(
        &high_packet(1_000 * MS, 0, &[(1, 1, 1)]),
        1_000 * MS + 7 * MS,
        &mut out,
    )
    .unwrap();
    // Same device, now PTP-locked to an epoch clock.
    let mut ptp = high_packet(1_760_000_000_000 * MS, 0, &[(2, 2, 2)]);
    ptp[11] = 1;
    let crc = crc32(&ptp[28..]);
    ptp[24..28].copy_from_slice(&crc.to_le_bytes());
    d.push(&ptp, 1_010 * MS, &mut out).unwrap();
    let s = scans(&out);
    assert_eq!(s.len(), 1);
    assert_eq!(s[0].t_ns, 1_007 * MS);
    assert_eq!(s[0].time_type, Some(TimeType::PowerOn));
    assert_eq!(d.stats().clock_resets, 1);
}

#[test]
fn imu_packets_become_host_timed_samples() {
    let mut d = Decoder::new(DecoderConfig::default());
    let mut out = Vec::new();
    d.push(&IMU, 2_000_500_000, &mut out).unwrap();
    match &out[..] {
        [Event::Imu(s)] => {
            assert_eq!(s.device_t_ns, 2_000_000_000);
            assert_eq!(s.t_ns, 2_000_500_000);
            assert!(close(s.gyro, [0.1, -0.2, 0.3]));
        }
        other => panic!("{other:?}"),
    }
}
