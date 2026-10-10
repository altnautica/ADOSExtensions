//! Periodic tasks: TIMESYNC, the companion HEARTBEAT, the health
//! publish, and the FC stream-rate requests.

use std::collections::BTreeSet;
use std::sync::atomic::{AtomicI64, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use ados_protocol::mavlink::ardupilotmega::{
    MavAutopilot, MavModeFlag, MavState, MavType, HEARTBEAT_DATA,
};
use ados_protocol::mavlink::MavMessage;
use ados_sdk::context::PluginContext;
use rmpv::Value;
use tokio::task::JoinHandle;

use super::subscriptions::FcTarget;
use crate::clock_align::ClockAlign;
use crate::health::{companion_system_status, CompanionState, HealthSnapshot};
use crate::mavlink_emit::{
    build_timesync, monotonic_ns, of_frame, stream_interval_requests, ComponentRouter,
};

/// How often the RAW_IMU / DISTANCE_SENSOR interval requests repeat (an
/// FC reboot drops them).
pub const STREAM_INTERVAL_PERIOD: Duration = Duration::from_secs(30);

/// TIMESYNC query at 1 Hz.
pub fn spawn_timesync(
    ctx: &PluginContext,
    clock: Arc<ClockAlign>,
    router: Arc<ComponentRouter>,
) -> JoinHandle<()> {
    let ctx = ctx.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs(1));
        loop {
            interval.tick().await;
            let ts1 = monotonic_ns();
            clock.mark_outgoing(ts1);
            if let Some(frame) = of_frame(&build_timesync(ts1)) {
                router.send(&ctx, &frame).await;
            }
        }
    })
}

/// Companion HEARTBEAT at 1 Hz, carrying the companion state.
pub fn spawn_heartbeat(
    ctx: &PluginContext,
    companion: Arc<AtomicI64>,
    router: Arc<ComponentRouter>,
) -> JoinHandle<()> {
    let ctx = ctx.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs(1));
        loop {
            interval.tick().await;
            let state = companion_state_from_i64(companion.load(Ordering::Relaxed));
            let msg = build_companion_heartbeat(companion_system_status(state));
            if let Some(frame) = of_frame(&msg) {
                router.send(&ctx, &frame).await;
            }
        }
    })
}

/// `MAV_CMD_SET_MESSAGE_INTERVAL` for RAW_IMU and DISTANCE_SENSOR, at
/// start and every [`STREAM_INTERVAL_PERIOD`], addressed to the FC.
pub fn spawn_stream_intervals(
    ctx: &PluginContext,
    fc_target: Arc<FcTarget>,
    router: Arc<ComponentRouter>,
) -> JoinHandle<()> {
    let ctx = ctx.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(STREAM_INTERVAL_PERIOD);
        loop {
            interval.tick().await;
            let (system, component) = fc_target.get();
            for msg in stream_interval_requests(system, component) {
                if let Some(frame) = of_frame(&msg) {
                    router.send(&ctx, &frame).await;
                }
            }
        }
    })
}

/// Shared counters the health tick folds into the snapshot.
pub struct HealthInputs {
    pub health: Arc<Mutex<HealthSnapshot>>,
    pub emit_errors: Arc<AtomicU64>,
    pub fc_params: Arc<Mutex<BTreeSet<String>>>,
    pub engaged: bool,
}

/// Publish the `navigation` block and the engage state at 1 Hz.
pub fn spawn_health(ctx: &PluginContext, inputs: HealthInputs) -> JoinHandle<()> {
    let ctx = ctx.clone();
    tokio::spawn(async move {
        let engage_state = if inputs.engaged { "active" } else { "idle" };
        let mut interval = tokio::time::interval(Duration::from_secs(1));
        loop {
            interval.tick().await;
            let fc_params: Vec<String> = inputs
                .fc_params
                .lock()
                .expect("fc params lock")
                .iter()
                .cloned()
                .collect();
            let snapshot = {
                let mut h = inputs.health.lock().expect("health lock");
                h.emit_errors = inputs.emit_errors.load(Ordering::Relaxed);
                h.fc_params = fc_params;
                h.to_value()
            };
            let _ = ctx.telemetry.extend("navigation", snapshot).await;
            // The engage Skill reads this event to reflect whether vision
            // navigation is actually engaged.
            let _ = ctx
                .events
                .publish(
                    "navigation.engage",
                    Value::Map(vec![(Value::from("state"), Value::from(engage_state))]),
                )
                .await;
        }
    })
}

pub fn companion_state_from_i64(v: i64) -> CompanionState {
    match v {
        x if x == CompanionState::Active as i64 => CompanionState::Active,
        x if x == CompanionState::Critical as i64 => CompanionState::Critical,
        x if x == CompanionState::Terminating as i64 => CompanionState::Terminating,
        _ => CompanionState::Inactive,
    }
}

/// Build the companion HEARTBEAT (#0) with the given system_status.
fn build_companion_heartbeat(system_status: u8) -> MavMessage {
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GENERIC,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: match system_status {
            4 => MavState::MAV_STATE_ACTIVE,
            6 => MavState::MAV_STATE_CRITICAL,
            8 => MavState::MAV_STATE_FLIGHT_TERMINATION,
            _ => MavState::MAV_STATE_STANDBY,
        },
        mavlink_version: 3,
    })
}
