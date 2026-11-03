//! Commands the `external_nav` mode sends the flight controller: the EKF
//! source-set switch, the EKF origin and the origin request. The inputs it
//! reads (`LOCAL_POSITION_NED`, `ATTITUDE`, `GPS_RAW_INT`) are streamed by
//! the agent's MAVLink router on the drone profile; the plugin does not
//! request their rates, so no two schedules fight.

// `SET_GPS_GLOBAL_ORIGIN` is marked superseded (2025-04) by a command the
// agent's dialect lacks; see `build_set_global_origin`.
#[allow(deprecated)]
use ados_protocol::mavlink::ardupilotmega::SET_GPS_GLOBAL_ORIGIN_DATA;
use ados_protocol::mavlink::ardupilotmega::{MavCmd, COMMAND_LONG_DATA};
use ados_protocol::mavlink::MavMessage;

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

/// `SET_GPS_GLOBAL_ORIGIN` (#48) from `[lat_deg, lon_deg, alt_m]`:
/// latitude/longitude in degE7, altitude in mm AMSL.
///
/// Not `MAV_CMD_DO_SET_GLOBAL_ORIGIN` (611), which the message definitions
/// mark as its successor: that command lives only in the development
/// dialect, so the plugin host's MAVLink gate, which forwards only frames
/// its (ardupilotmega) dialect decodes whole, refused the `COMMAND_INT`
/// carrying it and the origin never reached the flight controller. ArduPilot
/// still handles the message (`AP_MAVLINK_SET_GPS_GLOBAL_ORIGIN_MESSAGE_ENABLED`,
/// on by default).
#[allow(deprecated)]
pub fn build_set_global_origin(target_system: u8, lla: [f64; 3]) -> MavMessage {
    MavMessage::SET_GPS_GLOBAL_ORIGIN(SET_GPS_GLOBAL_ORIGIN_DATA {
        latitude: (lla[0] * 1e7).round() as i32,
        longitude: (lla[1] * 1e7).round() as i32,
        altitude: (lla[2] * 1000.0).round() as i32,
        target_system,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::external_nav::COMPONENT_EXTNAV;
    use crate::mavlink_emit::frame_for;
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
    #[allow(deprecated)]
    fn global_origin_is_set_gps_global_origin_in_deg_e7_and_mm() {
        let MavMessage::SET_GPS_GLOBAL_ORIGIN(d) = roundtrip(&build_set_global_origin(
            1,
            [52.0731234, -0.6276543, 110.25],
        )) else {
            panic!("expected SET_GPS_GLOBAL_ORIGIN");
        };
        assert_eq!(d.latitude, 520_731_234);
        assert_eq!(d.longitude, -6_276_543);
        assert_eq!(d.altitude, 110_250);
        assert_eq!(d.target_system, 1);
        // The plugin host forwards only frames its dialect decodes whole
        // (`mavlink_gate::scan_outbound`); a refused origin never reaches the FC.
        let frame = frame_for(
            COMPONENT_EXTNAV,
            &build_set_global_origin(1, [52.0, -0.6, 110.0]),
        )
        .unwrap();
        assert!(
            ados_protocol::mavlink::decode_exact_frame(&frame).is_ok(),
            "the host MAVLink gate must accept the origin frame"
        );
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
