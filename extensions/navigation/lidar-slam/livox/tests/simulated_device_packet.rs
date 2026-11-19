//! A point packet as the simulator's served Mid-360 sends it, byte for byte
//! (the same bytes its encoder test asserts it produces), read by this
//! decoder: three cartesian points at device time 1 s, one of them a zero
//! (no-return) sample, with the packet CRC filled in.

use ados_livox::{CrcStatus, DataType, Packet, TimeType};

const PACKET_HEX: &str = "004e000000030007000001000000000000000000000000008901483f00ca9a3b00000000\
e80300003cf6fffffa00000064000000000000000000000000000000393000000100000048f4ffffff00";

fn hex(s: &str) -> Vec<u8> {
    (0..s.len())
        .step_by(2)
        .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
        .collect()
}

#[test]
fn the_simulators_point_packet_decodes() {
    let bytes = hex(PACKET_HEX);
    let pkt = Packet::parse(&bytes).unwrap();
    let h = pkt.header;
    assert_eq!(h.data_type, DataType::CartesianHigh);
    assert_eq!(h.time_type, TimeType::PowerOn);
    assert_eq!((h.dot_num, h.udp_cnt, h.timestamp_ns), (3, 7, 1_000_000_000));
    assert_eq!(pkt.crc_status(), CrcStatus::Valid);
    let pts: Vec<_> = pkt.points().collect();
    assert_eq!(pts.len(), 3);
    let near = |a: [f32; 3], b: [f32; 3]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-5);
    assert!(near(pts[0].xyz, [1.0, -2.5, 0.25]), "{:?}", pts[0].xyz);
    assert_eq!(pts[0].reflectivity, 100);
    assert!(pts[0].valid);
    assert!(!pts[1].valid);
    assert!(near(pts[2].xyz, [12.345, 0.001, -3.0]), "{:?}", pts[2].xyz);
    assert_eq!(pts[2].reflectivity, 255);
    assert!(pts.iter().all(|p| p.offset_ns == 0));
}
