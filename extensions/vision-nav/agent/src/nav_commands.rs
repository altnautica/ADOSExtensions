//! Commands the `external_nav` mode sends the flight controller: the EKF
//! source-set switch, the EKF origin and the origin request. The inputs it
//! reads (`LOCAL_POSITION_NED`, `ATTITUDE`, `GPS_RAW_INT`) are streamed by
//! the agent's MAVLink router on the drone profile; the plugin does not
//! request their rates, so no two schedules fight.

use ados_protocol::mavlink::ardupilotmega::{MavCmd, COMMAND_LONG_DATA};
use ados_protocol::mavlink::MavMessage;

use crate::mavlink_emit::encode_v2;

/// `GPS_GLOBAL_ORIGIN` message id (for `MAV_CMD_REQUEST_MESSAGE`).
pub const GPS_GLOBAL_ORIGIN_ID: u32 = 49;

fn command_long(target: (u8, u8), command: MavCmd, param1: f32) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command,
        target_system: target.0,
        target_component: target.1,
        confirmation: 0,
    })
}

/// `MAV_CMD_SET_EKF_SOURCE_SET` (42007), `param1` = set 1-3.
pub fn build_set_ekf_source_set(target: (u8, u8), set: u8) -> MavMessage {
    command_long(target, MavCmd::MAV_CMD_SET_EKF_SOURCE_SET, f32::from(set))
}

/// `MAV_CMD_REQUEST_MESSAGE` (512) for `GPS_GLOBAL_ORIGIN`, so a flight
/// controller that already has an origin reports it.
pub fn build_request_origin(target: (u8, u8)) -> MavMessage {
    command_long(
        target,
        MavCmd::MAV_CMD_REQUEST_MESSAGE,
        GPS_GLOBAL_ORIGIN_ID as f32,
    )
}

/// `COMMAND_INT` message id and CRC extra.
const COMMAND_INT_ID: u32 = 75;
const COMMAND_INT_CRC_EXTRA: u8 = 158;
/// `COMMAND_INT` payload length.
pub const COMMAND_INT_LEN: usize = 35;
/// `MAV_CMD_DO_SET_GLOBAL_ORIGIN`, which supersedes the
/// `SET_GPS_GLOBAL_ORIGIN` message. It lives in the development dialect,
/// so the shared codec has no variant for it and the frame is packed here.
pub const MAV_CMD_DO_SET_GLOBAL_ORIGIN: u16 = 611;
/// `MAV_FRAME_GLOBAL`: altitude above mean sea level.
const MAV_FRAME_GLOBAL: u8 = 0;

/// `MAV_CMD_DO_SET_GLOBAL_ORIGIN` as a `COMMAND_INT` (#75) from
/// `[lat_deg, lon_deg, alt_m]`: `x`/`y` in degE7, `z` in metres AMSL,
/// `MAV_FRAME_GLOBAL`. ArduPilot handles it as a COMMAND_INT only
/// (`GCS_MAVLINK::handle_command_do_set_global_origin`).
pub fn build_set_global_origin(
    comp_id: u8,
    sequence: u8,
    target: (u8, u8),
    lla: [f64; 3],
) -> Vec<u8> {
    let mut p = Vec::with_capacity(COMMAND_INT_LEN);
    for param in [0f32; 4] {
        p.extend_from_slice(&param.to_le_bytes());
    }
    p.extend_from_slice(&((lla[0] * 1e7).round() as i32).to_le_bytes());
    p.extend_from_slice(&((lla[1] * 1e7).round() as i32).to_le_bytes());
    p.extend_from_slice(&(lla[2] as f32).to_le_bytes());
    p.extend_from_slice(&MAV_CMD_DO_SET_GLOBAL_ORIGIN.to_le_bytes());
    p.push(target.0);
    p.push(target.1);
    p.push(MAV_FRAME_GLOBAL);
    p.push(0); // current
    p.push(0); // autocontinue
    encode_v2(comp_id, sequence, COMMAND_INT_ID, COMMAND_INT_CRC_EXTRA, p)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::external_nav::COMPONENT_EXTNAV;
    use crate::mavlink_emit::{frame_for, x25_crc};
    use ados_protocol::mavlink::parse_v2;

    fn roundtrip(msg: &MavMessage) -> MavMessage {
        let frame = frame_for(COMPONENT_EXTNAV, msg).unwrap();
        assert_eq!(frame[6], COMPONENT_EXTNAV);
        parse_v2(&frame).unwrap().1
    }

    #[test]
    fn source_set_command_carries_the_set() {
        let MavMessage::COMMAND_LONG(d) = roundtrip(&build_set_ekf_source_set((1, 1), 2)) else {
            panic!("expected COMMAND_LONG");
        };
        assert_eq!(d.command as u16, 42007);
        assert_eq!(d.param1, 2.0);
        assert_eq!((d.target_system, d.target_component), (1, 1));
    }

    #[test]
    fn global_origin_is_a_command_int_in_deg_e7_and_metres() {
        let frame = build_set_global_origin(
            COMPONENT_EXTNAV,
            4,
            (1, 1),
            [52.0731234, -0.6276543, 110.25],
        );
        assert_eq!(frame[6], COMPONENT_EXTNAV);
        assert_eq!(u32::from_le_bytes([frame[7], frame[8], frame[9], 0]), 75);
        // autocontinue/current/frame are zero, so MAVLink 2 truncates them.
        let len = frame[1] as usize;
        assert_eq!(len, 32);
        let p = &frame[10..10 + len];
        let i32_at = |at: usize| i32::from_le_bytes(p[at..at + 4].try_into().unwrap());
        assert_eq!(i32_at(16), 520_731_234);
        assert_eq!(i32_at(20), -6_276_543);
        assert_eq!(f32::from_le_bytes(p[24..28].try_into().unwrap()), 110.25);
        assert_eq!(u16::from_le_bytes([p[28], p[29]]), 611);
        assert_eq!((p[30], p[31]), (1, 1));
        let crc = u16::from_le_bytes([frame[10 + len], frame[11 + len]]);
        assert_eq!(crc, x25_crc(&frame[1..10 + len], 158));
    }

    #[test]
    fn origin_request_asks_for_message_49() {
        let MavMessage::COMMAND_LONG(d) = roundtrip(&build_request_origin((1, 1))) else {
            panic!("expected COMMAND_LONG");
        };
        assert_eq!(d.command as u16, 512);
        assert_eq!(d.param1, 49.0);
    }
}
