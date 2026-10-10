//! MAVLink frame builders + the component router.
//!
//! The flow message depends on the configured firmware:
//!
//! * ArduPilot and iNav: `OPTICAL_FLOW` (#100) with the MAVLink 2
//!   extension fields `flow_rate_x/y` = the raw (not derotated) angular
//!   rate in rad/s. ArduPilot switches to these fields once it sees a
//!   non-zero value and subtracts its own AHRS gyro.
//! * PX4: `OPTICAL_FLOW_RAD` (#106): the angle swept over the interval
//!   (not derotated) and the gyro rotation over the same interval.
//!
//! Unknown distance is sent as -1 (the MAVLink "unknown" value). Also
//! built here: `DISTANCE_SENSOR` (#132), `TIMESYNC` (#111), `HEARTBEAT`
//! (#0) and the `MAV_CMD_SET_MESSAGE_INTERVAL` requests, all on comp 198
//! with a per-component sequence number.

use std::sync::atomic::{AtomicU64, AtomicU8, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use ados_protocol::mavlink::{
    ardupilotmega::{
        MavCmd, MavDistanceSensor, MavSensorOrientation, COMMAND_LONG_DATA, DISTANCE_SENSOR_DATA,
        OPTICAL_FLOW_RAD_DATA, TIMESYNC_DATA,
    },
    serialize_v2, MavHeader, MavMessage,
};
use ados_sdk::context::PluginContext;

use crate::clock_align::ClockAlign;
use crate::config::Firmware;
use crate::estimator::{EstimatorOutput, OutputMode};
use crate::flow::IntegratedFlow;

/// OF peripheral component id (`MAV_COMP_ID_PERIPHERAL`-class).
pub const COMPONENT_OF: u8 = 198;

const DEFAULT_SYS_ID: u8 = 1;

const OPTICAL_FLOW_ID: u32 = 100;
const OPTICAL_FLOW_CRC_EXTRA: u8 = 175;
/// Base fields (26 bytes) plus the `flow_rate_x/y` extension (8 bytes).
const OPTICAL_FLOW_LEN: usize = 34;

/// RAW_IMU and DISTANCE_SENSOR message ids and the intervals the plugin
/// requests for them (100 Hz and 20 Hz).
pub const RAW_IMU_ID: u32 = 27;
pub const DISTANCE_SENSOR_ID: u32 = 132;
pub const RAW_IMU_INTERVAL_US: f32 = 10_000.0;
pub const DISTANCE_SENSOR_INTERVAL_US: f32 = 50_000.0;

const EMIT_ERROR_LOG_EVERY: Duration = Duration::from_secs(5);

static SEQUENCE: [AtomicU8; 256] = [const { AtomicU8::new(0) }; 256];

/// The next outbound sequence number for `comp_id`, wrapping at 255.
pub fn next_sequence(comp_id: u8) -> u8 {
    SEQUENCE[comp_id as usize].fetch_add(1, Ordering::Relaxed)
}

/// Serialize a message under the given component id.
fn frame_for(comp_id: u8, msg: &MavMessage) -> Option<Vec<u8>> {
    let header = MavHeader {
        system_id: DEFAULT_SYS_ID,
        component_id: comp_id,
        sequence: next_sequence(comp_id),
    };
    serialize_v2(header, msg).ok()
}

/// Serialize an arbitrary message under the OF component id.
pub fn of_frame(msg: &MavMessage) -> Option<Vec<u8>> {
    frame_for(COMPONENT_OF, msg)
}

/// `OPTICAL_FLOW` (#100) fields, including the MAVLink 2 extensions.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpticalFlow {
    pub time_usec: u64,
    pub flow_comp_m_x: f32,
    pub flow_comp_m_y: f32,
    pub ground_distance: f32,
    pub flow_x: i16,
    pub flow_y: i16,
    pub sensor_id: u8,
    pub quality: u8,
    pub flow_rate_x: f32,
    pub flow_rate_y: f32,
}

/// Build the ArduPilot / iNav flow message from an estimator sample.
pub fn build_optical_flow(time_usec: u64, sensor_id: u8, sample: &EstimatorOutput) -> OpticalFlow {
    let rate_x = sample.flow_rate_x.unwrap_or(0.0);
    let rate_y = sample.flow_rate_y.unwrap_or(0.0);
    let distance = sample.flow_distance_m;
    let (comp_x, comp_y) = match distance {
        Some(d) if d > 0.0 => (rate_x * d, rate_y * d),
        _ => (0.0, 0.0),
    };
    let dt_s = sample.integration_time_us.unwrap_or(0) as f32 / 1e6;
    // flow_x/y: the legacy integer field, the swept angle in milliradians.
    let legacy = |rate: f32| (rate * dt_s * 1000.0).round().clamp(-32768.0, 32767.0) as i16;
    OpticalFlow {
        time_usec,
        flow_comp_m_x: comp_x,
        flow_comp_m_y: comp_y,
        ground_distance: distance.unwrap_or(-1.0),
        flow_x: legacy(rate_x),
        flow_y: legacy(rate_y),
        sensor_id,
        quality: quality_u8(sample),
        flow_rate_x: rate_x,
        flow_rate_y: rate_y,
    }
}

/// Encode an `OPTICAL_FLOW` as a MAVLink 2 frame carrying the extension
/// fields (trailing zero bytes truncated, as MAVLink 2 requires).
pub fn encode_optical_flow(comp_id: u8, sequence: u8, m: &OpticalFlow) -> Vec<u8> {
    let mut payload = Vec::with_capacity(OPTICAL_FLOW_LEN);
    payload.extend_from_slice(&m.time_usec.to_le_bytes());
    payload.extend_from_slice(&m.flow_comp_m_x.to_le_bytes());
    payload.extend_from_slice(&m.flow_comp_m_y.to_le_bytes());
    payload.extend_from_slice(&m.ground_distance.to_le_bytes());
    payload.extend_from_slice(&m.flow_x.to_le_bytes());
    payload.extend_from_slice(&m.flow_y.to_le_bytes());
    payload.push(m.sensor_id);
    payload.push(m.quality);
    payload.extend_from_slice(&m.flow_rate_x.to_le_bytes());
    payload.extend_from_slice(&m.flow_rate_y.to_le_bytes());
    while payload.len() > 1 && payload.last() == Some(&0) {
        payload.pop();
    }
    let id = OPTICAL_FLOW_ID.to_le_bytes();
    let mut frame = Vec::with_capacity(12 + payload.len());
    frame.extend_from_slice(&[
        0xFD,
        payload.len() as u8,
        0,
        0,
        sequence,
        DEFAULT_SYS_ID,
        comp_id,
        id[0],
        id[1],
        id[2],
    ]);
    frame.extend_from_slice(&payload);
    let crc = x25_crc(&frame[1..], OPTICAL_FLOW_CRC_EXTRA);
    frame.extend_from_slice(&crc.to_le_bytes());
    frame
}

/// MAVLink X.25 checksum over `bytes` followed by the message's CRC extra.
fn x25_crc(bytes: &[u8], crc_extra: u8) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &b in bytes.iter().chain(std::iter::once(&crc_extra)) {
        let mut tmp = b ^ (crc & 0xFF) as u8;
        tmp ^= tmp << 4;
        let t = tmp as u16;
        crc = (crc >> 8) ^ (t << 8) ^ (t << 3) ^ (t >> 4);
    }
    crc
}

/// Build the PX4 `OPTICAL_FLOW_RAD` (#106): integrated angle (not
/// derotated) and integrated gyro over `integration_time_us`.
pub fn build_optical_flow_rad(
    time_usec: u64,
    sensor_id: u8,
    integration_time_us: u32,
    integrated: IntegratedFlow,
    quality: u8,
    distance_m: Option<f32>,
) -> MavMessage {
    MavMessage::OPTICAL_FLOW_RAD(OPTICAL_FLOW_RAD_DATA {
        time_usec,
        integration_time_us,
        integrated_x: integrated.x,
        integrated_y: integrated.y,
        integrated_xgyro: integrated.xgyro,
        integrated_ygyro: integrated.ygyro,
        integrated_zgyro: integrated.zgyro,
        time_delta_distance_us: 0,
        distance: distance_m.unwrap_or(-1.0),
        temperature: 0,
        sensor_id,
        quality,
    })
}

/// Build a DISTANCE_SENSOR message (downward laser).
pub fn build_distance_sensor(
    time_boot_ms: u32,
    min_distance_cm: u16,
    max_distance_cm: u16,
    current_distance_cm: u16,
    sensor_id: u8,
    covariance: u8,
) -> MavMessage {
    MavMessage::DISTANCE_SENSOR(DISTANCE_SENSOR_DATA {
        time_boot_ms,
        min_distance: min_distance_cm,
        max_distance: max_distance_cm,
        current_distance: current_distance_cm,
        mavtype: MavDistanceSensor::MAV_DISTANCE_SENSOR_LASER,
        id: sensor_id,
        orientation: MavSensorOrientation::MAV_SENSOR_ROTATION_PITCH_270,
        covariance,
    })
}

/// Build a TIMESYNC query (`tc1=0`, `ts1=monotonic_ns`).
pub fn build_timesync(ts1: i64) -> MavMessage {
    MavMessage::TIMESYNC(TIMESYNC_DATA { tc1: 0, ts1 })
}

/// Build `MAV_CMD_SET_MESSAGE_INTERVAL` for `msg_id` at `interval_us`.
pub fn build_set_message_interval(
    target_system: u8,
    target_component: u8,
    msg_id: u32,
    interval_us: f32,
) -> MavMessage {
    MavMessage::COMMAND_LONG(COMMAND_LONG_DATA {
        param1: msg_id as f32,
        param2: interval_us,
        param3: 0.0,
        param4: 0.0,
        param5: 0.0,
        param6: 0.0,
        param7: 0.0,
        command: MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL,
        target_system,
        target_component,
        confirmation: 0,
    })
}

/// The stream-rate requests the plugin sends on start and every 30 s:
/// RAW_IMU at 100 Hz and DISTANCE_SENSOR at 20 Hz.
pub fn stream_interval_requests(target_system: u8, target_component: u8) -> [MavMessage; 2] {
    [
        build_set_message_interval(
            target_system,
            target_component,
            RAW_IMU_ID,
            RAW_IMU_INTERVAL_US,
        ),
        build_set_message_interval(
            target_system,
            target_component,
            DISTANCE_SENSOR_ID,
            DISTANCE_SENSOR_INTERVAL_US,
        ),
    ]
}

fn quality_u8(sample: &EstimatorOutput) -> u8 {
    sample.flow_quality.unwrap_or(0).clamp(0, 255) as u8
}

/// Routes estimator output to MAVLink. The flow message follows the
/// configured firmware; send failures are counted (health `emitErrors`)
/// and logged at most once per 5 s.
pub struct ComponentRouter {
    sensor_id: u8,
    firmware: Firmware,
    clock: Arc<ClockAlign>,
    emit_errors: Arc<AtomicU64>,
    last_error_log: Mutex<Option<Instant>>,
}

impl ComponentRouter {
    pub fn new(
        sensor_id: u8,
        firmware: Firmware,
        clock: Arc<ClockAlign>,
        emit_errors: Arc<AtomicU64>,
    ) -> Self {
        Self {
            sensor_id,
            firmware,
            clock,
            emit_errors,
            last_error_log: Mutex::new(None),
        }
    }

    /// Encode the flow frame for one sample, or `None` when the sample
    /// carries no flow.
    pub fn flow_frame(&self, sample: &EstimatorOutput, time_usec: u64) -> Option<Vec<u8>> {
        if sample.output_mode != OutputMode::OpticalFlow {
            return None;
        }
        sample.flow_rate_x?;
        sample.flow_rate_y?;
        match self.firmware {
            Firmware::Ardupilot | Firmware::Inav => {
                let msg = build_optical_flow(time_usec, self.sensor_id, sample);
                Some(encode_optical_flow(
                    COMPONENT_OF,
                    next_sequence(COMPONENT_OF),
                    &msg,
                ))
            }
            Firmware::Px4 => {
                let integrated = IntegratedFlow {
                    x: sample.integrated_x.unwrap_or(0.0),
                    y: sample.integrated_y.unwrap_or(0.0),
                    xgyro: sample.integrated_xgyro.unwrap_or(0.0),
                    ygyro: sample.integrated_ygyro.unwrap_or(0.0),
                    zgyro: sample.integrated_zgyro.unwrap_or(0.0),
                };
                let msg = build_optical_flow_rad(
                    time_usec,
                    self.sensor_id,
                    sample.integration_time_us.unwrap_or(0).max(0) as u32,
                    integrated,
                    quality_u8(sample),
                    sample.flow_distance_m,
                );
                of_frame(&msg)
            }
        }
    }

    /// Route one sample. A `none` sample emits nothing.
    pub async fn emit(&self, ctx: &PluginContext, sample: &EstimatorOutput) {
        let time_usec = self.clock.fc_time_us(monotonic_ns());
        if let Some(frame) = self.flow_frame(sample, time_usec) {
            self.send(ctx, &frame).await;
        }
    }

    /// Send a frame on comp 198, counting and rate-limit logging failures.
    pub async fn send(&self, ctx: &PluginContext, frame: &[u8]) {
        if let Err(e) = ctx.mavlink.send(frame, Some(COMPONENT_OF as i64)).await {
            self.emit_errors.fetch_add(1, Ordering::Relaxed);
            let mut last = self.last_error_log.lock().expect("emit log lock");
            if last.is_none_or(|t| t.elapsed() >= EMIT_ERROR_LOG_EVERY) {
                *last = Some(Instant::now());
                eprintln!(
                    "vision-nav: mavlink send failed ({} total): {e}",
                    self.emit_errors.load(Ordering::Relaxed)
                );
            }
        }
    }

    /// Build + send a DISTANCE_SENSOR co-emission on comp 198.
    pub async fn emit_distance_sensor(
        &self,
        ctx: &PluginContext,
        distance_m: f32,
        min_m: f32,
        max_m: f32,
        quality_0_100: i32,
    ) {
        let time_boot_ms = (self.clock.fc_time_us(monotonic_ns()) / 1000) as u32;
        let cm = |m: f32| (m * 100.0).round().clamp(0.0, 65535.0) as u16;
        let covariance = (100 - quality_0_100).clamp(0, 255) as u8;
        let msg = build_distance_sensor(
            time_boot_ms,
            cm(min_m),
            cm(max_m),
            cm(distance_m),
            self.sensor_id,
            covariance,
        );
        if let Some(frame) = of_frame(&msg) {
            self.send(ctx, &frame).await;
        }
    }
}

/// Monotonic clock in nanoseconds.
pub fn monotonic_ns() -> i64 {
    static EPOCH: std::sync::LazyLock<Instant> = std::sync::LazyLock::new(Instant::now);
    EPOCH.elapsed().as_nanos() as i64
}

#[cfg(test)]
#[path = "mavlink_emit_tests.rs"]
mod tests;
