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
use tokio::sync::Notify;
use tokio::task::JoinHandle;
use tokio::time::Interval;

use super::subscriptions::FcTarget;
use crate::clock_align::ClockAlign;
use crate::health::{companion_system_status, CompanionState, HealthSnapshot};
use crate::imu::ImuBuffer;
use crate::mavlink_emit::{
    build_timesync, monotonic_ns, of_frame, stream_interval_requests, ComponentRouter,
};

/// How often the RAW_IMU / DISTANCE_SENSOR / OPTICAL_FLOW interval requests
/// repeat while RAW_IMU streams (an FC reboot drops them).
pub const STREAM_INTERVAL_PERIOD: Duration = Duration::from_secs(30);
/// How soon they repeat while RAW_IMU does not stream as asked.
pub const STREAM_RETRY: Duration = Duration::from_secs(2);
/// RAW_IMU streams as asked while its newest sample is no older than this
/// and it arrives at [`RAW_IMU_MIN_HZ`] or more (it is asked for 100 Hz).
const RAW_IMU_STALE_NS: i64 = 1_000_000_000;
const RAW_IMU_MIN_HZ: f32 = 40.0;

/// When to send the stream interval requests, and to whom: only to the
/// flight controller learned from its own HEARTBEAT, never a guessed 1/1
/// (a SITL on another instance, or an FC set to another id, ignores a
/// request addressed to system 1); at once when it is learned or changes;
/// every [`STREAM_RETRY`] while RAW_IMU does not stream as asked; otherwise
/// every [`STREAM_INTERVAL_PERIOD`].
#[derive(Debug, Default)]
pub struct StreamRequests {
    /// The target and monotonic ns of the last request.
    sent: Option<((u8, u8), i64)>,
}

impl StreamRequests {
    /// The target to request the streams from at `now_ns`, if due, given the
    /// learned FC and the newest RAW_IMU sample time and its rate.
    pub fn due(
        &mut self,
        target: Option<(u8, u8)>,
        now_ns: i64,
        imu_newest_ns: Option<i64>,
        imu_hz: Option<f32>,
    ) -> Option<(u8, u8)> {
        let target = target?;
        let streaming = imu_newest_ns.is_some_and(|t| now_ns - t <= RAW_IMU_STALE_NS)
            && imu_hz.is_some_and(|hz| hz >= RAW_IMU_MIN_HZ);
        let wait = if streaming {
            STREAM_INTERVAL_PERIOD
        } else {
            STREAM_RETRY
        };
        let due = match self.sent {
            Some((to, at)) if to == target => now_ns - at >= wait.as_nanos() as i64,
            _ => true,
        };
        if due {
            self.sent = Some((target, now_ns));
        }
        due.then_some(target)
    }
}

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

/// `MAV_CMD_SET_MESSAGE_INTERVAL` for RAW_IMU, DISTANCE_SENSOR and
/// OPTICAL_FLOW, addressed to the learned FC when [`StreamRequests`] says
/// so, checked every second.
pub fn spawn_stream_intervals(
    ctx: &PluginContext,
    fc_target: Arc<FcTarget>,
    imu: Arc<ImuBuffer>,
    router: Arc<ComponentRouter>,
) -> JoinHandle<()> {
    let ctx = ctx.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs(1));
        let mut requests = StreamRequests::default();
        loop {
            interval.tick().await;
            let newest = imu.latest().map(|s| s.ts_ns);
            let due = requests.due(fc_target.learned(), monotonic_ns(), newest, imu.rate_hz());
            let Some((system, component)) = due else {
                continue;
            };
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
    /// Signalled on a companion transition (frames lost, frames back) so the
    /// GCS sees it at once instead of up to a tick later.
    pub publish_now: Arc<Notify>,
}

/// Wait for the next health publish: the 1 Hz tick or an early request.
pub async fn next_publish(interval: &mut Interval, publish_now: &Notify) {
    tokio::select! {
        _ = interval.tick() => {}
        _ = publish_now.notified() => {}
    }
}

/// Publish the `navigation` block and the engage state at 1 Hz, and at once
/// on a companion transition.
pub fn spawn_health(ctx: &PluginContext, inputs: HealthInputs) -> JoinHandle<()> {
    let ctx = ctx.clone();
    tokio::spawn(async move {
        let engage_state = if inputs.engaged { "active" } else { "idle" };
        let mut interval = tokio::time::interval(Duration::from_secs(1));
        loop {
            next_publish(&mut interval, &inputs.publish_now).await;
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
