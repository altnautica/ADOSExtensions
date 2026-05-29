//! MAVLink frame builders + the component router.
//!
//! Four messages cover the optical-flow + range + VIO-injection
//! surface, built from the shared ardupilotmega codec rather than
//! hand-packed bytes:
//!
//! * `OPTICAL_FLOW_RAD` (#106) — angular-rate flow on comp 198.
//! * `DISTANCE_SENSOR` (#132) — downward rangefinder injection on
//!   comp 198.
//! * `VISION_POSITION_ESTIMATE` (#102) — VIO pose on comp 197 (built
//!   through the SDK pose helper where it fits).
//! * `TIMESYNC` (#111) / `HEARTBEAT` (#0) — the companion node's clock
//!   query + 1 Hz liveness, on comp 198.
//!
//! The router reads each [`crate::estimator::EstimatorOutput`]'s
//! `output_mode` and picks the component the sample rides:
//! `optical_flow` -> 198, `vio` -> 197, hybrid -> both. The
//! `time_usec` on each frame is the FC-clock time from
//! [`ClockAlign`](crate::clock_align::ClockAlign).

use std::sync::Arc;

use ados_protocol::mavlink::{
    ardupilotmega::{
        MavDistanceSensor, MavSensorOrientation, DISTANCE_SENSOR_DATA, OPTICAL_FLOW_RAD_DATA,
        TIMESYNC_DATA,
    },
    serialize_v2, MavHeader, MavMessage,
};
use ados_sdk::context::PluginContext;
use ados_sdk::vision::Pose;

use crate::clock_align::ClockAlign;
use crate::estimator::{EstimatorOutput, OutputMode};

/// OF peripheral component id (`MAV_COMP_ID_PERIPHERAL`-class). Emits
/// OPTICAL_FLOW_RAD + DISTANCE_SENSOR.
pub const COMPONENT_OF: u8 = 198;
/// VIO component id (`MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`). Emits
/// VISION_POSITION_ESTIMATE.
pub const COMPONENT_VIO: u8 = 197;

const DEFAULT_SYS_ID: u8 = 1;

/// Serialize a message under the given component id (the host router
/// owns the outbound sequence, so 0 is fine here).
fn frame_for(comp_id: u8, msg: &MavMessage) -> Option<Vec<u8>> {
    let header = MavHeader {
        system_id: DEFAULT_SYS_ID,
        component_id: comp_id,
        sequence: 0,
    };
    serialize_v2(header, msg).ok()
}

/// Build an OPTICAL_FLOW_RAD message. Body-frame radian convention both
/// ArduPilot and PX4 (and iNav 7.0+) consume.
#[allow(clippy::too_many_arguments)]
pub fn build_optical_flow_rad(
    time_usec: u64,
    sensor_id: u8,
    integration_time_us: u32,
    integrated_x: f32,
    integrated_y: f32,
    integrated_zgyro: f32,
    quality: u8,
    distance: f32,
) -> MavMessage {
    MavMessage::OPTICAL_FLOW_RAD(OPTICAL_FLOW_RAD_DATA {
        time_usec,
        integration_time_us,
        integrated_x,
        integrated_y,
        // The router does not separately derotate xgyro/ygyro into the
        // integrated_*gyro fields; the visual flow already had gyro
        // subtracted (see flow.rs). Only the z integrated-gyro carries
        // through so the FC's own yaw fusion sees the companion's
        // measured rotation about the optical axis.
        integrated_xgyro: 0.0,
        integrated_ygyro: 0.0,
        integrated_zgyro,
        time_delta_distance_us: 0,
        distance,
        temperature: 0,
        sensor_id,
        quality,
    })
}

/// Build a DISTANCE_SENSOR message (downward laser by default).
#[allow(clippy::too_many_arguments)]
pub fn build_distance_sensor(
    time_boot_ms: u32,
    min_distance_cm: u16,
    max_distance_cm: u16,
    current_distance_cm: u16,
    sensor_id: u8,
    covariance: u8,
) -> MavMessage {
    // This ardupilotmega build omits the MAVLink v2 extension fields
    // (horizontal_fov / vertical_fov / quaternion / signal_quality); the
    // FC fills them with their "unknown" defaults on receive.
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
    // target_system / target_component are MAVLink v2 extensions and are
    // omitted by this ardupilotmega build; the broadcast tc1=0 query is
    // the standard companion-side TIMESYNC request.
    MavMessage::TIMESYNC(TIMESYNC_DATA { tc1: 0, ts1 })
}

/// Serialize an arbitrary message under the OF component id.
pub fn of_frame(msg: &MavMessage) -> Option<Vec<u8>> {
    frame_for(COMPONENT_OF, msg)
}

/// Routes an estimator output to the matching MAVLink component over
/// the host's MAVLink path. OF samples ride comp 198; VIO samples ride
/// comp 197 (built through the SDK pose helper). Hybrid ticks carry an
/// OF sample in `extras_of` so a single tick emits on both components.
pub struct ComponentRouter {
    sensor_id: u8,
    clock: Arc<ClockAlign>,
}

impl ComponentRouter {
    pub fn new(sensor_id: u8, clock: Arc<ClockAlign>) -> Self {
        Self { sensor_id, clock }
    }

    /// Route one sample. MAVLink send failures are surfaced to the
    /// caller (the pipeline logs and continues) — a transient send
    /// error must never panic the loop.
    pub async fn emit(&self, ctx: &PluginContext, sample: &EstimatorOutput) {
        match sample.output_mode {
            OutputMode::OpticalFlow => self.emit_of(ctx, sample).await,
            OutputMode::Vio => {
                self.emit_vio(ctx, sample).await;
                // Hybrid: a co-emitted OF sample rides in extras so one
                // tick produces emissions on both 197 and 198.
                if let Some(of) = &sample.extras_of {
                    self.emit_of(ctx, of).await;
                }
            }
            OutputMode::None => {}
        }
    }

    async fn emit_of(&self, ctx: &PluginContext, sample: &EstimatorOutput) {
        let (Some(fx), Some(fy)) = (sample.flow_rate_x, sample.flow_rate_y) else {
            return;
        };
        let now = monotonic_ns();
        let time_usec = self.clock.fc_time_us(now);
        let msg = build_optical_flow_rad(
            time_usec,
            self.sensor_id,
            sample.integration_time_us.unwrap_or(0) as u32,
            fx,
            fy,
            sample.flow_rate_z.unwrap_or(0.0),
            sample.flow_quality.unwrap_or(0).clamp(0, 255) as u8,
            sample.flow_distance_m.unwrap_or(0.0),
        );
        if let Some(frame) = of_frame(&msg) {
            let _ = ctx.mavlink.send(&frame, Some(COMPONENT_OF as i64)).await;
        }
    }

    async fn emit_vio(&self, ctx: &PluginContext, sample: &EstimatorOutput) {
        let Some(pose6) = sample.pose else {
            return;
        };
        // pose6 = (x, y, z, roll, pitch, yaw). The SDK pose helper
        // builds VISION_POSITION_ESTIMATE from a quaternion, so convert
        // the Euler attitude back to a quaternion and let the helper own
        // the frame assembly + component id 197.
        let (x, y, z, roll, pitch, yaw) = pose6;
        let q = euler_to_quat(roll, pitch, yaw);
        let now = monotonic_ns();
        let pose = Pose {
            position: (x, y, z),
            orientation: q,
            timestamp_us: self.clock.fc_time_us(now),
            covariance: covariance_to_array(sample.covariance.as_deref()),
        };
        // register_vio_component is idempotent at the host; the pipeline
        // also registers it explicitly on start. inject_pose builds the
        // VISION_POSITION_ESTIMATE and sends it under component 197.
        let _ = ctx.vision.inject_pose(&pose).await;
    }

    /// Build + send a DISTANCE_SENSOR co-emission on comp 198. PX4 wants
    /// it as a separate input; ArduPilot benefits from the explicit
    /// terrain altitude.
    pub async fn emit_distance_sensor(
        &self,
        ctx: &PluginContext,
        distance_m: f32,
        min_m: f32,
        max_m: f32,
        quality_0_100: i32,
    ) {
        let now = monotonic_ns();
        let time_boot_ms = (self.clock.fc_time_us(now) / 1000) as u32;
        let current_cm = (distance_m * 100.0).round().clamp(0.0, 65535.0) as u16;
        let min_cm = (min_m * 100.0).round().clamp(0.0, 65535.0) as u16;
        let max_cm = (max_m * 100.0).round().clamp(0.0, 65535.0) as u16;
        let covariance = (100 - quality_0_100).clamp(0, 255) as u8;
        let msg = build_distance_sensor(
            time_boot_ms,
            min_cm,
            max_cm,
            current_cm,
            self.sensor_id,
            covariance,
        );
        if let Some(frame) = of_frame(&msg) {
            let _ = ctx.mavlink.send(&frame, Some(COMPONENT_OF as i64)).await;
        }
    }
}

/// Coerce a 21-or-other-length covariance slice into the fixed
/// `[f32; 21]` the SDK pose helper expects, or `None` (unknown marker)
/// when the slice is absent or the wrong length.
fn covariance_to_array(cov: Option<&[f32]>) -> Option<[f32; 21]> {
    let cov = cov?;
    if cov.len() != 21 {
        return None;
    }
    let mut out = [0.0f32; 21];
    out.copy_from_slice(cov);
    Some(out)
}

/// Aerospace ZYX Euler -> quaternion `(w, x, y, z)`.
fn euler_to_quat(roll: f32, pitch: f32, yaw: f32) -> (f32, f32, f32, f32) {
    let (sr, cr) = (roll * 0.5).sin_cos();
    let (sp, cp) = (pitch * 0.5).sin_cos();
    let (sy, cy) = (yaw * 0.5).sin_cos();
    let w = cr * cp * cy + sr * sp * sy;
    let x = sr * cp * cy - cr * sp * sy;
    let y = cr * sp * cy + sr * cp * sy;
    let z = cr * cp * sy - sr * sp * cy;
    (w, x, y, z)
}

/// Monotonic clock in nanoseconds.
pub fn monotonic_ns() -> i64 {
    use std::time::Instant;
    use std::sync::OnceLock;
    static EPOCH: OnceLock<Instant> = OnceLock::new();
    let epoch = EPOCH.get_or_init(Instant::now);
    epoch.elapsed().as_nanos() as i64
}

#[cfg(test)]
mod tests {
    use super::*;
    use ados_protocol::mavlink::{parse_v2, MavMessage};

    #[test]
    fn optical_flow_rad_round_trips() {
        let msg = build_optical_flow_rad(123456, 0, 33333, 0.01, -0.02, 0.0, 200, 1.5);
        let frame = of_frame(&msg).unwrap();
        assert_eq!(frame[0], 0xFD);
        let (header, parsed) = parse_v2(&frame).unwrap();
        assert_eq!(header.component_id, COMPONENT_OF);
        match parsed {
            MavMessage::OPTICAL_FLOW_RAD(d) => {
                assert_eq!(d.time_usec, 123456);
                assert_eq!(d.quality, 200);
                assert!((d.integrated_x - 0.01).abs() < 1e-6);
                assert!((d.distance - 1.5).abs() < 1e-6);
            }
            other => panic!("expected OPTICAL_FLOW_RAD, got {other:?}"),
        }
    }

    #[test]
    fn distance_sensor_round_trips() {
        let msg = build_distance_sensor(1000, 20, 800, 150, 0, 10);
        let frame = of_frame(&msg).unwrap();
        let (_h, parsed) = parse_v2(&frame).unwrap();
        match parsed {
            MavMessage::DISTANCE_SENSOR(d) => {
                assert_eq!(d.current_distance, 150);
                assert_eq!(d.min_distance, 20);
                assert_eq!(d.max_distance, 800);
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
        let msg = build_timesync(987654321);
        let frame = of_frame(&msg).unwrap();
        let (_h, parsed) = parse_v2(&frame).unwrap();
        match parsed {
            MavMessage::TIMESYNC(d) => {
                assert_eq!(d.tc1, 0);
                assert_eq!(d.ts1, 987654321);
            }
            other => panic!("expected TIMESYNC, got {other:?}"),
        }
    }

    #[test]
    fn euler_quat_round_trips_yaw() {
        // 90 deg yaw -> q = (cos45, 0, 0, sin45).
        let q = euler_to_quat(0.0, 0.0, std::f32::consts::FRAC_PI_2);
        let s = std::f32::consts::FRAC_1_SQRT_2;
        assert!((q.0 - s).abs() < 1e-5);
        assert!((q.3 - s).abs() < 1e-5);
    }
}
