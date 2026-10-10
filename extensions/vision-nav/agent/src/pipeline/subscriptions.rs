//! MAVLink subscriptions: the FC streams the modes consume.
//!
//! The host delivers each subscribed message as `{msg_name, frame,
//! timestamp_ms}` with `frame` the raw MAVLink bytes; [`decode_delivery`]
//! parses it with the shared dialect. A subscription the host refuses is
//! returned as an error so the caller can record it in health
//! `degradedReasons` instead of running silently without that input.

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicU16, Ordering};
use std::sync::{Arc, Mutex};

use ados_protocol::mavlink::{parse_v1, parse_v2, MavMessage};
use ados_sdk::context::PluginContext;
use rmpv::Value;

use crate::clock_align::ClockAlign;
use crate::imu::ImuBuffer;
use crate::mavlink_emit::monotonic_ns;
use crate::rangefinder::RelayDistanceSensor;
use crate::scale::ScaleLadder;

/// FC parameters the GCS parameter cards show only when the FC reports
/// them.
pub const WATCHED_FC_PARAMS: &[&str] = &["EKF2_OF_CTRL", "EKF2_AID_MASK"];

/// The flight controller's system and component id, learned from the
/// header of its RAW_IMU stream. 1/1 until the first frame arrives.
pub struct FcTarget(AtomicU16);

impl Default for FcTarget {
    fn default() -> Self {
        Self(AtomicU16::new(u16::from_be_bytes([1, 1])))
    }
}

impl FcTarget {
    pub fn set(&self, system: u8, component: u8) {
        self.0
            .store(u16::from_be_bytes([system, component]), Ordering::Relaxed);
    }

    /// `(system, component)`.
    pub fn get(&self) -> (u8, u8) {
        let [s, c] = self.0.load(Ordering::Relaxed).to_be_bytes();
        (s, c)
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
}

/// Parse a `mavlink.deliver` args map into `(system, component,
/// message)`. `None` when it carries no parseable frame.
pub fn decode_delivery(args: &Value) -> Option<(u8, u8, MavMessage)> {
    let Value::Map(entries) = args else {
        return None;
    };
    let frame = entries.iter().find_map(|(k, v)| match (k.as_str(), v) {
        (Some("frame"), Value::Binary(b)) => Some(b.as_slice()),
        _ => None,
    })?;
    decode_frame(frame)
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

/// Route one decoded message into the sinks. Pure apart from the sinks,
/// so tests drive it without a host.
pub fn apply(sinks: &Sinks, system: u8, component: u8, msg: &MavMessage, now_ns: i64) {
    match msg {
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
        MavMessage::GPS_RAW_INT(d) => sinks.ladder.on_gps_raw(
            i64::from(d.alt),
            d.fix_type as i32,
            i32::from(d.eph),
            now_ns,
        ),
        MavMessage::PARAM_VALUE(d) => {
            if let Ok(id) = d.param_id.to_str() {
                if WATCHED_FC_PARAMS.contains(&id) {
                    sinks
                        .fc_params
                        .lock()
                        .expect("fc params lock")
                        .insert(id.to_string());
                }
            }
        }
        _ => {}
    }
}

/// The streams every mode consumes.
pub const SUBSCRIBED: &[&str] = &[
    "RAW_IMU",
    "DISTANCE_SENSOR",
    "TIMESYNC",
    "GLOBAL_POSITION_INT",
    "VFR_HUD",
    "GPS_RAW_INT",
    "PARAM_VALUE",
];

/// Subscribe to every [`SUBSCRIBED`] stream. Returns one
/// `mavlink_subscribe_denied:<MSG>` entry per refused subscription.
pub async fn subscribe_all(ctx: &PluginContext, sinks: &Sinks) -> Vec<String> {
    let mut denied = Vec::new();
    for name in SUBSCRIBED {
        let s = sinks.clone();
        let cb = Arc::new(move |args: Value| {
            if let Some((system, component, msg)) = decode_delivery(&args) {
                apply(&s, system, component, &msg, monotonic_ns());
            }
        });
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
        }
    }

    fn delivery(system: u8, component: u8, msg: &MavMessage) -> Value {
        let header = MavHeader {
            system_id: system,
            component_id: component,
            sequence: 0,
        };
        let frame = serialize_v2(header, msg).unwrap();
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
        let (sys, comp, m) = decode_delivery(&delivery(7, 1, &msg)).unwrap();
        apply(&s, sys, comp, &m, 0);
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
        let (sys, comp, m) = decode_delivery(&delivery(1, 1, &msg)).unwrap();
        apply(&s, sys, comp, &m, monotonic_ns());
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
    }
}
