//! The `external_nav` and obstacle ticks.
//!
//! * The external-nav tick (at `external_nav.rate_hz`) sends `ODOMETRY` for
//!   each new pose, runs the source-set machine and resends its command
//!   until the flight controller accepts it (three tries, 1 s apart),
//!   requests the EKF origin and sets it from `origin_lla` under the rule in
//!   [`crate::origin`], reads the flight-controller parameters the pre-arm
//!   checks need (every 10 s while any is missing, again after a reboot),
//!   and keeps the `navigation` health block and pre-arm report current.
//! * The obstacle tick (at `obstacle.rate_hz`) sends `OBSTACLE_DISTANCE`
//!   for each new scan.
//! * The `nav.state` publisher reports both.

use std::sync::atomic::{AtomicI64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use ados_protocol::mavlink::MavMessage;
use ados_sdk::context::PluginContext;
use tokio::task::JoinHandle;
use tokio::time::MissedTickBehavior;

use super::nav_shared::{companion_for, estimator_state_for, NavShared};
use super::subscriptions::Sinks;
use super::ticks::companion_state_from_i64;
use crate::config::Mode;
use crate::external_nav::{measured_at_ns, ExtnavHealth, OdometryBuilder, COMPONENT_EXTNAV};
use crate::health::HealthSnapshot;
use crate::mavlink_emit::{frame_for, monotonic_ns, next_sequence, ComponentRouter};
use crate::nav_commands::{
    build_param_request_read, build_request_origin, build_set_ekf_source_set,
    build_set_global_origin, extnav_fc_params,
};
use crate::nav_config::{ExternalNavConfig, ObstacleConfig};
use crate::nav_state::{AgeWindow, RateMeter, NAV_STATE_TOPIC};
use crate::obstacle::sectorise;
use crate::origin::OriginRule;
use crate::pre_arm::{ExtnavPreArmInputs, PreArmGate, PreArmInputs};
use crate::source_set::{gps_usable, SourceSwitch};

const SECOND: i64 = 1_000_000_000;
const ACK_WAIT_NS: i64 = SECOND;
const MAX_COMMAND_TRIES: u8 = 3;
const RATE_WINDOW_NS: i64 = SECOND;
/// Missing flight-controller parameters are requested again this often.
const PARAM_RETRY_NS: i64 = 10 * SECOND;
/// `MAV_RESULT_ACCEPTED`.
const ACCEPTED: u8 = 0;

/// A source-set command waiting for its `COMMAND_ACK`.
struct PendingSet {
    set: u8,
    sent_at: Option<i64>,
    tries: u8,
}

/// Unix milliseconds now.
fn unix_ms() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |d| d.as_millis() as i64)
}

/// Everything the external-nav tick needs.
pub struct ExtnavTask {
    pub ctx: PluginContext,
    pub cfg: ExternalNavConfig,
    pub obstacle_enabled: bool,
    pub sinks: Sinks,
    pub router: Arc<ComponentRouter>,
    pub shared: Arc<NavShared>,
    pub health: Arc<Mutex<HealthSnapshot>>,
    pub companion: Arc<AtomicI64>,
}

impl ExtnavTask {
    async fn send_msg(&self, msg: &MavMessage) -> bool {
        match frame_for(COMPONENT_EXTNAV, msg) {
            Some(frame) => {
                self.router
                    .send_as(&self.ctx, COMPONENT_EXTNAV, &frame)
                    .await
            }
            None => false,
        }
    }
}

pub fn spawn_external_nav(task: ExtnavTask) -> JoinHandle<()> {
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs_f32(1.0 / task.cfg.rate_hz));
        interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
        let gate = PreArmGate::default();
        let mut builder = OdometryBuilder::default();
        let mut switch = SourceSwitch::new(&task.cfg);
        let mut odom_rate = RateMeter::new(RATE_WINDOW_NS);
        let mut pose_age = AgeWindow::default();
        let mut last_seq = 0u64;
        let mut pending: Option<PendingSet> = None;
        let mut origin = OriginRule::default();
        let param_ids = extnav_fc_params(&task.cfg);
        let mut param_epoch = 0u64;
        let mut last_param_request: Option<i64> = None;
        let mut last_tracked: Option<i64> = None;
        loop {
            interval.tick().await;
            let now = monotonic_ns();
            let target = task.sinks.fc_target.get();
            let fc = task.sinks.fc_nav.snapshot(now);

            // One ODOMETRY per new pose (the tracker's scan rate, ~10 Hz);
            // nothing between poses, nothing while lost or stale. It carries
            // the time the pose describes, not the time it arrived.
            let view = task.shared.pose_view(now);
            let health = view.fresh.map(|(p, _)| p.health);
            let mut odom = None;
            let mut odom_ts_ms = 0;
            if let Some((pose, recv)) = view.fresh.filter(|_| view.seq != last_seq) {
                last_seq = view.seq;
                odom_ts_ms = pose.ts_ms;
                let t = measured_at_ns(pose.ts_ms, unix_ms(), now, recv);
                odom = builder.build(task.sinks.clock.fc_time_us(t), t, &pose, &fc);
            }
            if let Some(odom) = odom {
                let frame = odom.encode(next_sequence(COMPONENT_EXTNAV));
                if task
                    .router
                    .send_as(&task.ctx, COMPONENT_EXTNAV, &frame)
                    .await
                {
                    odom_rate.mark(now);
                    if odom_ts_ms > 0 {
                        pose_age.record((unix_ms() - odom_ts_ms) as f32);
                    }
                }
            }

            let gps_ok = gps_usable(fc.gps_fix, fc.gps_h_acc_mm);
            if let Some(set) = switch.update(now, health, gps_ok) {
                eprintln!("vision-nav: EKF source set -> {set}");
                task.sinks.fc_nav.clear_source_set_ack();
                pending = Some(PendingSet {
                    set,
                    sent_at: None,
                    tries: 0,
                });
            }
            if let Some(p) = pending.as_mut() {
                let due = p.sent_at.is_none_or(|t| now - t >= ACK_WAIT_NS);
                if p.sent_at.is_some() && fc.source_set_ack == Some(ACCEPTED) {
                    pending = None;
                } else if due && p.tries >= MAX_COMMAND_TRIES {
                    eprintln!(
                        "vision-nav: EKF source set {} not accepted (ack {:?})",
                        p.set, fc.source_set_ack
                    );
                    pending = None;
                } else if due {
                    p.tries += 1;
                    p.sent_at = Some(now);
                    task.send_msg(&build_set_ekf_source_set(target, p.set))
                        .await;
                }
            }

            let origin_act = origin.tick(now, &fc, &task.cfg);
            if origin_act.request {
                task.send_msg(&build_request_origin(target)).await;
            }
            if let Some(lla) = origin_act.set {
                eprintln!(
                    "vision-nav: no EKF origin from the flight controller; sending origin_lla"
                );
                task.send_msg(&build_set_global_origin(target.0, lla)).await;
            }

            if fc.boot_epoch != param_epoch {
                param_epoch = fc.boot_epoch;
                task.sinks
                    .fc_param_values
                    .lock()
                    .expect("fc param values lock")
                    .clear();
                last_param_request = None;
            }
            let params = task
                .sinks
                .fc_param_values
                .lock()
                .expect("fc param values lock")
                .clone();
            if last_param_request.is_none_or(|t| now - t >= PARAM_RETRY_NS) {
                let missing: Vec<&String> = param_ids
                    .iter()
                    .filter(|id| !params.contains_key(*id))
                    .collect();
                if !missing.is_empty() {
                    last_param_request = Some(now);
                    for id in missing {
                        task.send_msg(&build_param_request_read(target, id)).await;
                    }
                }
            }

            if matches!(health, Some(ExtnavHealth::Good | ExtnavHealth::Degraded)) {
                last_tracked = Some(now);
            }
            if let Some(state) = companion_for(health, last_tracked, now) {
                task.companion.store(state as i64, Ordering::Relaxed);
            }
            let companion = companion_state_from_i64(task.companion.load(Ordering::Relaxed));
            let report = gate.evaluate(&PreArmInputs {
                mode: Mode::ExternalNav,
                companion_state: companion,
                flow_quality: None,
                flow_scale_source: None,
                rangefinder_topology: None,
                rangefinder_reading_m: None,
                rangefinder_min_m: 0.0,
                rangefinder_max_m: 0.0,
                extnav: Some(ExtnavPreArmInputs {
                    source_denied: task.shared.pose_denied.load(Ordering::Relaxed),
                    pose_error: view.error.clone(),
                    health,
                    north: view.fresh.and_then(|(p, _)| p.north),
                    imu_feed: view.fresh.and_then(|(p, _)| p.imu_feed),
                    origin_seen: fc.origin_seen,
                    origin_by_plugin: origin.origin_by_plugin(),
                    gps_primary: task.cfg.gps_primary,
                    extnav_set: task.cfg.extnav_set,
                    fallback_set: task.cfg.fallback_set,
                    origin_lla: task.cfg.origin_lla,
                    params,
                    pose_age_ms: pose_age.median_ms(),
                    obstacle: task
                        .obstacle_enabled
                        .then(|| task.shared.obstacle_feed(now)),
                    fc_flow: fc.flow,
                    fc_heard_ns: fc.boot_ns.map(|b| now - b),
                }),
            });
            {
                let mut h = task.health.lock().expect("health lock");
                h.companion_state = Some(companion);
                h.estimator_state = Some(estimator_state_for(health).to_string());
                h.pre_arm_report = Some(report.to_value());
                h.pose_age_ms = pose_age.median_ms();
            }
            let rate = odom_rate.rate_hz(now);
            task.shared.update_state(|s| {
                s.active_set = switch.active();
                s.extnav_health = health;
                s.odom_rate_hz = Some(rate);
                s.last_reset_counter = Some(builder.reset_counter());
            });
        }
    })
}

/// The obstacle tick.
pub fn spawn_obstacle(
    ctx: &PluginContext,
    cfg: ObstacleConfig,
    sinks: Sinks,
    router: Arc<ComponentRouter>,
    shared: Arc<NavShared>,
) -> JoinHandle<()> {
    let ctx = ctx.clone();
    tokio::spawn(async move {
        let mut interval = tokio::time::interval(Duration::from_secs_f32(1.0 / cfg.rate_hz));
        interval.set_missed_tick_behavior(MissedTickBehavior::Skip);
        let mut rate = RateMeter::new(RATE_WINDOW_NS);
        let mut last_seq = 0u64;
        let mut min_obstacle = None;
        loop {
            interval.tick().await;
            let now = monotonic_ns();
            match shared.fresh_scan(now) {
                Some((scan, recv, seq)) if seq != last_seq => {
                    last_seq = seq;
                    let msg = sectorise(&scan, cfg.sectors, sinks.clock.fc_time_us(recv));
                    min_obstacle = msg.min_obstacle_m();
                    let frame = msg.encode(next_sequence(COMPONENT_EXTNAV));
                    if router.send_as(&ctx, COMPONENT_EXTNAV, &frame).await {
                        rate.mark(now);
                    }
                }
                // The same scan again: keep the last reading.
                Some(_) => {}
                None => min_obstacle = None,
            }
            let hz = rate.rate_hz(now);
            shared.update_state(|s| {
                s.obstacle_rate_hz = Some(hz);
                s.min_obstacle_m = min_obstacle;
            });
        }
    })
}

/// Publish `nav.state`: 5 Hz in `external_nav`, 1 Hz otherwise.
pub fn spawn_nav_state(
    ctx: &PluginContext,
    mode: Mode,
    sinks: Sinks,
    shared: Arc<NavShared>,
) -> JoinHandle<()> {
    let ctx = ctx.clone();
    tokio::spawn(async move {
        let period = if mode == Mode::ExternalNav {
            Duration::from_millis(200)
        } else {
            Duration::from_secs(1)
        };
        let mut interval = tokio::time::interval(period);
        loop {
            interval.tick().await;
            let fix = sinks.fc_nav.snapshot(monotonic_ns()).gps_fix;
            shared.update_state(|s| s.gps_fix = fix);
            let _ = ctx
                .events
                .publish(NAV_STATE_TOPIC, shared.state().to_value())
                .await;
        }
    })
}
