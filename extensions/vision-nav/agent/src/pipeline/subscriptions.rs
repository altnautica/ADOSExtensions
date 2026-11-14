//! MAVLink subscriptions: the FC streams the modes consume.
//!
//! The host delivers each subscribed message as `{msg_name, frame,
//! timestamp_ms}` with `frame` the raw MAVLink bytes; [`decode_delivery`]
//! parses it with the shared dialect. A subscription the host refuses is
//! returned as an error so the caller can record it in health
//! `degradedReasons` instead of running silently without that input.

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};

use ados_protocol::mavlink::ardupilotmega::{MavAutopilot, MavType};
use ados_protocol::mavlink::{parse_v1, parse_v2, MavMessage};
use ados_sdk::context::PluginContext;
use rmpv::Value;

use crate::clock_align::ClockAlign;
use crate::fc_nav::{Attitude, FcNavSink, MODE_FLAG_ARMED};
use crate::imu::ImuBuffer;
use crate::mavlink_emit::monotonic_ns;
use crate::nav_commands::is_extnav_fc_param;
use crate::rangefinder::RelayDistanceSensor;
use crate::scale::ScaleLadder;

/// FC parameters the GCS parameter cards show only when the FC reports
/// them.
pub const WATCHED_FC_PARAMS: &[&str] = &["EKF2_OF_CTRL", "EKF2_AID_MASK"];

/// The flight controller's system and component id, learned from the
/// header of its HEARTBEAT or RAW_IMU. [`Self::get`] reads 1/1 until the
/// first one arrives; [`Self::learned`] reads nothing. The FC is not always
/// system 1 (a SITL on another instance takes the next id), and RAW_IMU
/// streams only once the FC is asked at its own address, so the heartbeat
/// is what finds it.
#[derive(Default)]
pub struct FcTarget(AtomicU32);

/// The bit of [`FcTarget`]'s word that says the pair was learned.
const LEARNED: u32 = 1 << 16;

impl FcTarget {
    pub fn set(&self, system: u8, component: u8) {
        let pair = u32::from(u16::from_be_bytes([system, component]));
        self.0.store(LEARNED | pair, Ordering::Relaxed);
    }

    /// `(system, component)`, 1/1 until learned.
    pub fn get(&self) -> (u8, u8) {
        self.learned().unwrap_or((1, 1))
    }

    /// `(system, component)` once a HEARTBEAT or RAW_IMU named it.
    pub fn learned(&self) -> Option<(u8, u8)> {
        let w = self.0.load(Ordering::Relaxed);
        let [s, c] = (w as u16).to_be_bytes();
        (w & LEARNED != 0).then_some((s, c))
    }
}

/// Shared sinks the subscription callbacks write into.
#[derive(Clone)]
pub struct Sinks {
    pub imu: Arc<ImuBuffer>,
    pub relay: Arc<RelayDistanceSensor>,
    pub clock: Arc<ClockAlign>,
    pub ladder: Arc<ScaleLadder>,
    pub fc_target: Arc<FcTarget>,
    pub fc_params: Arc<Mutex<BTreeSet<String>>>,
    /// Values of the flight-controller parameters the external-nav pre-arm
    /// checks read ([`crate::nav_commands::extnav_fc_params`]), by id.
    pub fc_param_values: Arc<Mutex<BTreeMap<String, f32>>>,
    pub fc_nav: Arc<FcNavSink>,
}

/// The raw frame of a `mavlink.deliver` args map.
fn frame_of(args: &Value) -> Option<&[u8]> {
    let Value::Map(entries) = args else {
        return None;
    };
    entries.iter().find_map(|(k, v)| match (k.as_str(), v) {
        (Some("frame"), Value::Binary(b)) => Some(b.as_slice()),
        _ => None,
    })
}

/// Parse a `mavlink.deliver` args map into `(system, component,
/// message)`. `None` when it carries no parseable frame.
pub fn decode_delivery(args: &Value) -> Option<(u8, u8, MavMessage)> {
    decode_frame(frame_of(args)?)
}

/// Decode one `mavlink.deliver` args map and route it into the sinks.
pub fn deliver(sinks: &Sinks, args: &Value, now_ns: i64) {
    let Some(frame) = frame_of(args) else {
        return;
    };
    if let Some((system, component, msg)) = decode_frame(frame) {
        apply(sinks, system, component, &msg, frame, now_ns);
    }
}

/// `GPS_RAW_INT.h_acc` (mm) from the raw frame: an extension field
/// (payload bytes 34..38) the shared dialect does not decode. MAVLink 2
/// drops trailing zero bytes, so a short payload reads as 0 (unknown); a
/// MAVLink 1 frame never carries it.
pub fn gps_raw_h_acc_mm(frame: &[u8]) -> u32 {
    const H_ACC_AT: usize = 34;
    if frame.first() != Some(&0xFD) || frame.len() < 10 {
        return 0;
    }
    let len = usize::from(frame[1]);
    let payload = &frame[10..frame.len().min(10 + len)];
    let mut b = [0u8; 4];
    for (i, slot) in b.iter_mut().enumerate() {
        *slot = payload.get(H_ACC_AT + i).copied().unwrap_or(0);
    }
    u32::from_le_bytes(b)
}

/// Parse one raw MAVLink 1 or 2 frame.
pub fn decode_frame(frame: &[u8]) -> Option<(u8, u8, MavMessage)> {
    match *frame.first()? {
        0xFD => parse_v2(frame)
            .ok()
            .map(|(h, m)| (h.system_id, h.component_id, m)),
        0xFE => parse_v1(frame)
            .ok()
            .map(|(h, m)| (h.system_id, h.component_id, m)),
        _ => None,
    }
}

/// Route one decoded message (and its raw `frame`, for extension fields the
/// dialect drops) into the sinks. Pure apart from the sinks, so tests drive
/// it without a host.
pub fn apply(
    sinks: &Sinks,
    system: u8,
    component: u8,
    msg: &MavMessage,
    frame: &[u8],
    now_ns: i64,
) {
    match msg {
        // An autopilot's heartbeat names the FC. Our own and the agent's
        // heartbeats carry MAV_AUTOPILOT_INVALID; a GCS has MAV_TYPE_GCS.
        MavMessage::HEARTBEAT(h)
            if h.autopilot != MavAutopilot::MAV_AUTOPILOT_INVALID
                && h.mavtype != MavType::MAV_TYPE_GCS =>
        {
            sinks.fc_target.set(system, component);
            let armed = h.base_mode.bits() & MODE_FLAG_ARMED != 0;
            sinks.fc_nav.on_heartbeat(armed, now_ns);
        }
        MavMessage::RAW_IMU(d) => {
            sinks.fc_target.set(system, component);
            sinks.imu.record_raw_imu(
                now_ns,
                d.xgyro as f32,
                d.ygyro as f32,
                d.zgyro as f32,
                d.xacc as f32,
                d.yacc as f32,
                d.zacc as f32,
            );
        }
        MavMessage::DISTANCE_SENSOR(d) => sinks.relay.on_distance(
            i64::from(d.current_distance),
            Some(i64::from(d.min_distance)),
            Some(i64::from(d.max_distance)),
            i64::from(d.covariance),
            now_ns,
        ),
        MavMessage::TIMESYNC(d) => sinks.clock.handle_response(d.tc1, d.ts1),
        MavMessage::GLOBAL_POSITION_INT(d) => sinks
            .ladder
            .on_global_position(i64::from(d.relative_alt), now_ns),
        MavMessage::VFR_HUD(d) => sinks.ladder.on_vfr_hud(d.alt, now_ns),
        MavMessage::GPS_RAW_INT(d) => {
            sinks.ladder.on_gps_raw(
                i64::from(d.alt),
                d.fix_type as i32,
                i32::from(d.eph),
                now_ns,
            );
            if is_fc(sinks, system, component) {
                sinks
                    .fc_nav
                    .on_gps_fix(d.fix_type as u8, gps_raw_h_acc_mm(frame), now_ns);
            }
        }
        // The external-nav inputs come from the flight controller only, so
        // a gimbal or camera reporting its own ATTITUDE is never mixed in.
        MavMessage::LOCAL_POSITION_NED(d) if is_fc(sinks, system, component) => {
            sinks.fc_nav.on_local_position(d.z, d.vz, now_ns)
        }
        MavMessage::ATTITUDE(d) if is_fc(sinks, system, component) => sinks.fc_nav.on_attitude(
            Attitude {
                roll: d.roll,
                pitch: d.pitch,
                rollspeed: d.rollspeed,
                pitchspeed: d.pitchspeed,
                yawspeed: d.yawspeed,
            },
            now_ns,
        ),
        MavMessage::GPS_GLOBAL_ORIGIN(_) if is_fc(sinks, system, component) => {
            sinks.fc_nav.on_origin()
        }
        MavMessage::COMMAND_ACK(d) if is_fc(sinks, system, component) => sinks
            .fc_nav
            .on_command_ack(d.command as u16, d.result as u8),
        MavMessage::PARAM_VALUE(d) => {
            if let Ok(id) = d.param_id.to_str() {
                if WATCHED_FC_PARAMS.contains(&id) {
                    sinks
                        .fc_params
                        .lock()
                        .expect("fc params lock")
                        .insert(id.to_string());
                }
                if is_extnav_fc_param(id) && is_fc(sinks, system, component) {
                    sinks
                        .fc_param_values
                        .lock()
                        .expect("fc param values lock")
                        .insert(id.to_string(), d.param_value);
                }
            }
        }
        _ => {}
    }
}

/// Whether a message comes from the flight controller (learned from its
/// HEARTBEAT or RAW_IMU).
fn is_fc(sinks: &Sinks, system: u8, component: u8) -> bool {
    sinks.fc_target.get() == (system, component)
}

/// The streams every mode consumes.
pub const SUBSCRIBED: &[&str] = &[
    "HEARTBEAT",
    "RAW_IMU",
    "DISTANCE_SENSOR",
    "TIMESYNC",
    "GLOBAL_POSITION_INT",
    "VFR_HUD",
    "GPS_RAW_INT",
    "PARAM_VALUE",
];

/// The extra streams `external_nav` mode consumes.
pub const EXTERNAL_NAV_SUBSCRIBED: &[&str] = &[
    "LOCAL_POSITION_NED",
    "ATTITUDE",
    "GPS_GLOBAL_ORIGIN",
    "COMMAND_ACK",
];

/// Subscribe to every [`SUBSCRIBED`] stream plus `extra`. Returns one
/// `mavlink_subscribe_denied:<MSG>` entry per refused subscription.
pub async fn subscribe_all(ctx: &PluginContext, sinks: &Sinks, extra: &[&str]) -> Vec<String> {
    let mut denied = Vec::new();
    for name in SUBSCRIBED.iter().chain(extra) {
        let s = sinks.clone();
        let cb = Arc::new(move |args: Value| deliver(&s, &args, monotonic_ns()));
        if let Err(e) = ctx.mavlink.subscribe(name, cb).await {
            eprintln!("vision-nav: subscribe {name} failed: {e}");
            denied.push(format!("mavlink_subscribe_denied:{name}"));
        }
    }
    denied
}

#[cfg(test)]
mod tests {
    use super::*;
    use ados_protocol::mavlink::ardupilotmega::{
        MavDistanceSensor, MavSensorOrientation, DISTANCE_SENSOR_DATA, RAW_IMU_DATA,
    };
    use ados_protocol::mavlink::{serialize_v2, MavHeader};

    fn sinks() -> Sinks {
        Sinks {
            imu: Arc::new(ImuBuffer::default()),
            relay: Arc::new(RelayDistanceSensor::new()),
            clock: Arc::new(ClockAlign::new()),
            ladder: Arc::new(ScaleLadder::new(false)),
            fc_target: Arc::new(FcTarget::default()),
            fc_params: Arc::new(Mutex::new(BTreeSet::new())),
            fc_param_values: Arc::new(Mutex::new(BTreeMap::new())),
            fc_nav: Arc::new(FcNavSink::default()),
        }
    }

    fn delivery(system: u8, component: u8, msg: &MavMessage) -> Value {
        let header = MavHeader {
            system_id: system,
            component_id: component,
            sequence: 0,
        };
        let frame = serialize_v2(header, msg).unwrap();
        frame_delivery(frame)
    }

    fn frame_delivery(frame: Vec<u8>) -> Value {
        Value::Map(vec![
            (Value::from("msg_name"), Value::from("X")),
            (Value::from("frame"), Value::Binary(frame)),
            (Value::from("timestamp_ms"), Value::from(0i64)),
        ])
    }

    #[test]
    fn raw_imu_delivery_fills_the_buffer_and_target() {
        let s = sinks();
        let msg = MavMessage::RAW_IMU(RAW_IMU_DATA {
            xgyro: 1000,
            ..RAW_IMU_DATA::DEFAULT
        });
        deliver(&s, &delivery(7, 1, &msg), 0);
        assert!((s.imu.latest().unwrap().xgyro - 1.0).abs() < 1e-6);
        assert_eq!(s.fc_target.get(), (7, 1));
    }

    #[test]
    fn distance_sensor_delivery_feeds_the_relay() {
        let s = sinks();
        let msg = MavMessage::DISTANCE_SENSOR(DISTANCE_SENSOR_DATA {
            time_boot_ms: 0,
            min_distance: 20,
            max_distance: 800,
            current_distance: 150,
            mavtype: MavDistanceSensor::MAV_DISTANCE_SENSOR_LASER,
            id: 0,
            orientation: MavSensorOrientation::MAV_SENSOR_ROTATION_PITCH_270,
            covariance: 0,
        });
        deliver(&s, &delivery(1, 1, &msg), monotonic_ns());
        let r = s.relay.read_shared().unwrap();
        assert!((r.distance_m - 1.5).abs() < 1e-4);
    }

    #[test]
    fn non_frame_delivery_is_ignored() {
        let args = Value::Map(vec![(Value::from("frame"), Value::from("nope"))]);
        assert!(decode_delivery(&args).is_none());
    }

    #[test]
    fn fc_target_defaults_to_one_one() {
        assert_eq!(FcTarget::default().get(), (1, 1));
        assert_eq!(FcTarget::default().learned(), None);
    }

    /// An FC on system 2 (a SITL on its second instance): the RAW_IMU,
    /// DISTANCE_SENSOR and OPTICAL_FLOW interval requests wait for its
    /// heartbeat and go to it at once, never to a guessed 1/1; while RAW_IMU
    /// does not stream they repeat every 2 s, while it streams every 30 s,
    /// and once it stops they repeat every 2 s again.
    #[test]
    fn stream_requests_reach_an_fc_on_system_two_at_once() {
        use super::super::ticks::StreamRequests;
        use crate::mavlink_emit::{of_frame, stream_interval_requests};
        const S: i64 = 1_000_000_000;
        let s = sinks();
        let mut req = StreamRequests::default();
        let mut send = |now: i64| {
            let imu = s.imu.latest().map(|x| x.ts_ns);
            req.due(s.fc_target.learned(), now, imu, s.imu.rate_hz())
                .map(|(sys, comp)| stream_interval_requests(sys, comp))
        };
        assert!(send(0).is_none(), "requested before the FC is known");
        let hb = heartbeat(
            MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
            MavType::MAV_TYPE_QUADROTOR,
        );
        deliver(&s, &delivery(2, 1, &hb), S / 2);
        let sent = send(S).expect("requested once the FC is known");
        for msg in sent {
            let frame = of_frame(&msg).expect("frame");
            let Some((_, _, MavMessage::COMMAND_LONG(d))) = decode_frame(&frame) else {
                panic!("not a COMMAND_LONG");
            };
            assert_eq!((d.target_system, d.target_component), (2, 1));
        }
        // No RAW_IMU yet: again 2 s later, not before.
        assert!(send(2 * S).is_none());
        assert!(send(3 * S).is_some());
        // RAW_IMU at 100 Hz from 3 s to 33.5 s: the next request is 30 s on.
        let raw = MavMessage::RAW_IMU(RAW_IMU_DATA::DEFAULT);
        for k in 0..3050 {
            let t = 3 * S + k * S / 100;
            deliver(&s, &delivery(2, 1, &raw), t);
            if t % S == 0 && t > 3 * S {
                assert_eq!(send(t).is_some(), t == 33 * S, "at {} s", t / S);
            }
        }
        // Stopped at 33.5 s: stale after 34.5 s, so asked again at 35 s.
        assert!(send(34 * S).is_none());
        assert!(send(35 * S).is_some());
    }

    #[test]
    fn fc_attitude_and_position_reach_the_nav_sink_other_components_do_not() {
        use ados_protocol::mavlink::ardupilotmega::{ATTITUDE_DATA, LOCAL_POSITION_NED_DATA};
        let s = sinks();
        let att = MavMessage::ATTITUDE(ATTITUDE_DATA {
            roll: 0.1,
            pitch: -0.2,
            ..ATTITUDE_DATA::DEFAULT
        });
        // A gimbal's ATTITUDE (component 154) is ignored.
        deliver(&s, &delivery(1, 154, &att), 0);
        assert!(s.fc_nav.snapshot(0).attitude.is_none());
        deliver(&s, &delivery(1, 1, &att), 0);
        assert_eq!(s.fc_nav.snapshot(0).attitude.unwrap().pitch, -0.2);
        let pos = MavMessage::LOCAL_POSITION_NED(LOCAL_POSITION_NED_DATA {
            z: -4.0,
            vz: 0.5,
            ..LOCAL_POSITION_NED_DATA::DEFAULT
        });
        deliver(&s, &delivery(1, 1, &pos), 0);
        assert_eq!(s.fc_nav.snapshot(0).local.unwrap().z, -4.0);
    }

    fn heartbeat(autopilot: MavAutopilot, mavtype: MavType) -> MavMessage {
        use ados_protocol::mavlink::ardupilotmega::HEARTBEAT_DATA;
        MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            autopilot,
            mavtype,
            ..HEARTBEAT_DATA::DEFAULT
        })
    }

    /// A SITL on its second instance is system 2: only its heartbeat names
    /// it before any RAW_IMU streams, and its source-set ack must count.
    #[test]
    fn the_fc_heartbeat_sets_the_target_and_its_ack_is_accepted() {
        use ados_protocol::mavlink::ardupilotmega::{MavCmd, MavResult, COMMAND_ACK_DATA};
        let s = sinks();
        for (sys, comp, hb) in [
            (
                2,
                1,
                heartbeat(
                    MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
                    MavType::MAV_TYPE_QUADROTOR,
                ),
            ),
            // A GCS and a companion component (ours, the agent's) never move it.
            (
                255,
                190,
                heartbeat(MavAutopilot::MAV_AUTOPILOT_INVALID, MavType::MAV_TYPE_GCS),
            ),
            (
                1,
                197,
                heartbeat(
                    MavAutopilot::MAV_AUTOPILOT_INVALID,
                    MavType::MAV_TYPE_GENERIC,
                ),
            ),
            (
                1,
                191,
                heartbeat(
                    MavAutopilot::MAV_AUTOPILOT_INVALID,
                    MavType::MAV_TYPE_ONBOARD_CONTROLLER,
                ),
            ),
        ] {
            deliver(&s, &delivery(sys, comp, &hb), 0);
        }
        assert_eq!(s.fc_target.get(), (2, 1));

        // COMMAND_ACK here is the base message (no target extension fields).
        let ack = MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
            command: MavCmd::MAV_CMD_SET_EKF_SOURCE_SET,
            result: MavResult::MAV_RESULT_ACCEPTED,
        });
        // From the old default address the ack is someone else's.
        deliver(&s, &delivery(1, 1, &ack), 0);
        assert_eq!(s.fc_nav.snapshot(0).source_set_ack, None);
        deliver(&s, &delivery(2, 1, &ack), 0);
        assert_eq!(s.fc_nav.snapshot(0).source_set_ack, Some(0));
    }

    /// The FC heartbeat carries the armed bit; a 3 s gap is a reboot that
    /// forgets the origin.
    #[test]
    fn the_fc_heartbeat_reports_arming_and_reboots() {
        use ados_protocol::mavlink::ardupilotmega::{MavModeFlag, HEARTBEAT_DATA};
        const S: i64 = 1_000_000_000;
        let hb = |base_mode: MavModeFlag| {
            MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
                mavtype: MavType::MAV_TYPE_QUADROTOR,
                base_mode,
                ..HEARTBEAT_DATA::DEFAULT
            })
        };
        let s = sinks();
        deliver(&s, &delivery(1, 1, &hb(MavModeFlag::empty())), S);
        let origin = MavMessage::GPS_GLOBAL_ORIGIN(
            ados_protocol::mavlink::ardupilotmega::GPS_GLOBAL_ORIGIN_DATA::DEFAULT,
        );
        deliver(&s, &delivery(1, 1, &origin), S);
        let a = s.fc_nav.snapshot(S);
        assert!(!a.armed && a.origin_seen);
        assert_eq!((a.boot_ns, a.boot_epoch), (Some(S), 1));
        let armed = MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
        deliver(&s, &delivery(1, 1, &hb(armed)), 2 * S);
        assert!(s.fc_nav.snapshot(2 * S).armed);
        // The GCS's heartbeat never counts.
        let gcs = MavMessage::HEARTBEAT(HEARTBEAT_DATA {
            autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
            mavtype: MavType::MAV_TYPE_GCS,
            ..HEARTBEAT_DATA::DEFAULT
        });
        deliver(&s, &delivery(255, 190, &gcs), 6 * S);
        assert_eq!(s.fc_nav.snapshot(6 * S).boot_epoch, 1);
        deliver(&s, &delivery(1, 1, &hb(MavModeFlag::empty())), 6 * S);
        let b = s.fc_nav.snapshot(6 * S);
        assert!(!b.armed && !b.origin_seen);
        assert_eq!((b.boot_ns, b.boot_epoch), (Some(6 * S), 2));
    }

    /// `GPS_RAW_INT.h_acc` is an extension field: read from the raw frame.
    #[test]
    fn gps_h_acc_comes_from_the_extension_bytes() {
        use crate::mavlink_emit::encode_v2;
        use ados_protocol::mavlink::ardupilotmega::{GpsFixType, GPS_RAW_INT_DATA};
        let base = MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
            fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
            satellites_visible: 10,
            ..GPS_RAW_INT_DATA::DEFAULT
        });
        let header = MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 0,
        };
        let plain = serialize_v2(header, &base).unwrap();
        // No extension bytes: unknown accuracy.
        assert_eq!(gps_raw_h_acc_mm(&plain), 0);
        let mut payload = plain[10..10 + usize::from(plain[1])].to_vec();
        payload.resize(30, 0);
        payload.extend_from_slice(&0i32.to_le_bytes()); // alt_ellipsoid
        payload.extend_from_slice(&8000u32.to_le_bytes()); // h_acc
        payload.extend_from_slice(&3000u32.to_le_bytes()); // v_acc
        let frame = encode_v2(1, 0, 24, 24, payload);
        assert_eq!(gps_raw_h_acc_mm(&frame), 8000);
        let s = sinks();
        deliver(&s, &frame_delivery(frame), 0);
        let snap = s.fc_nav.snapshot(0);
        assert_eq!(snap.gps_fix, 3);
        assert_eq!(snap.gps_h_acc_mm, Some(8000));
    }

    /// The external-nav parameter values are recorded from the flight
    /// controller only.
    #[test]
    fn extnav_param_values_are_recorded_from_the_fc() {
        use ados_protocol::mavlink::ardupilotmega::{MavParamType, PARAM_VALUE_DATA};
        let pv = |id: &str, v: f32| {
            MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
                param_value: v,
                param_count: 1,
                param_index: 0,
                param_id: id.into(),
                param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
            })
        };
        let s = sinks();
        deliver(&s, &delivery(1, 1, &pv("VISO_TYPE", 1.0)), 0);
        deliver(&s, &delivery(1, 1, &pv("SERIAL1_BAUD", 57.0)), 0);
        deliver(&s, &delivery(1, 154, &pv("COMPASS_AUTODEC", 1.0)), 0);
        let values = s.fc_param_values.lock().unwrap().clone();
        assert_eq!(
            values.into_iter().collect::<Vec<_>>(),
            vec![("VISO_TYPE".to_string(), 1.0)]
        );
    }
}
