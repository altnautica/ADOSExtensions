//! State the external-nav and obstacle runtime shares: the latest pose
//! and scan from the topic callbacks, the subscription outcome, and the
//! `nav.state` being published. Plus the pure decisions the ticks make
//! from it.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use ados_sdk::context::PluginContext;
use rmpv::Value;

use crate::config::{Mode, VisionNavConfig};
use crate::external_nav::{parse_pose_event, ExtnavHealth, PoseSample};
use crate::health::CompanionState;
use crate::mavlink_emit::monotonic_ns;
use crate::nav_state::NavState;
use crate::obstacle::{parse_scan_event, Scan};
use crate::pre_arm::ObstacleFeed;

/// A pose older than this is no pose.
pub const POSE_STALE_NS: i64 = 500_000_000;
/// A scan older than this is no scan.
pub const SCAN_STALE_NS: i64 = 500_000_000;
const SECOND: i64 = 1_000_000_000;
/// Lost for this long after tracking flips the companion critical.
const COMPANION_LOST_NS: i64 = 2 * SECOND;

#[derive(Default)]
struct PoseSlot {
    latest: Option<(PoseSample, i64)>,
    seq: u64,
    error: Option<String>,
}

#[derive(Default)]
struct ScanSlot {
    latest: Option<(Scan, i64)>,
    seq: u64,
}

/// A fresh pose plus its receive time, the pose sequence, and why the
/// last payload was rejected (if it was).
pub struct PoseView {
    pub fresh: Option<(PoseSample, i64)>,
    pub seq: u64,
    pub error: Option<String>,
}

/// State shared by the topic callbacks, the ticks and the publisher.
pub struct NavShared {
    pose: Mutex<PoseSlot>,
    scan: Mutex<ScanSlot>,
    pub(super) pose_denied: AtomicBool,
    pub(super) scan_denied: AtomicBool,
    state: Mutex<NavState>,
}

impl NavShared {
    pub fn new(mode: Mode) -> Self {
        Self {
            pose: Mutex::default(),
            scan: Mutex::default(),
            pose_denied: AtomicBool::new(false),
            scan_denied: AtomicBool::new(false),
            state: Mutex::new(NavState {
                mode: mode.as_str().to_string(),
                ..NavState::default()
            }),
        }
    }

    /// Store a pose delivery; a payload that does not parse keeps the
    /// previous pose (which then goes stale) and records why.
    pub fn on_pose_event(&self, args: &Value, now_ns: i64) {
        let mut slot = self.pose.lock().expect("pose lock");
        match parse_pose_event(args) {
            Ok(p) => {
                slot.latest = Some((p, now_ns));
                slot.seq += 1;
                slot.error = None;
            }
            Err(e) => slot.error = Some(e.reason()),
        }
    }

    pub fn on_scan_event(&self, args: &Value, now_ns: i64) {
        if let Some(scan) = parse_scan_event(args) {
            let mut slot = self.scan.lock().expect("scan lock");
            slot.latest = Some((scan, now_ns));
            slot.seq += 1;
        }
    }

    pub fn pose_view(&self, now_ns: i64) -> PoseView {
        let slot = self.pose.lock().expect("pose lock");
        PoseView {
            fresh: slot.latest.filter(|(_, t)| now_ns - t <= POSE_STALE_NS),
            seq: slot.seq,
            error: slot.error.clone(),
        }
    }

    /// The fresh scan, its receive time and sequence.
    pub fn fresh_scan(&self, now_ns: i64) -> Option<(Scan, i64, u64)> {
        let slot = self.scan.lock().expect("scan lock");
        slot.latest
            .as_ref()
            .filter(|(_, t)| now_ns - t <= SCAN_STALE_NS)
            .map(|(s, t)| (s.clone(), *t, slot.seq))
    }

    pub fn obstacle_feed(&self, now_ns: i64) -> ObstacleFeed {
        if self.scan_denied.load(Ordering::Relaxed) {
            ObstacleFeed::Denied
        } else if self.fresh_scan(now_ns).is_some() {
            ObstacleFeed::Live
        } else {
            ObstacleFeed::Waiting
        }
    }

    pub fn state(&self) -> NavState {
        self.state.lock().expect("nav state lock").clone()
    }

    pub fn update_state(&self, f: impl FnOnce(&mut NavState)) {
        f(&mut self.state.lock().expect("nav state lock"));
    }
}

/// Whether the obstacle feed runs for this config.
pub fn obstacle_active(cfg: &VisionNavConfig) -> bool {
    cfg.obstacle.enabled && cfg.effective_mode() != Mode::Off
}

/// Whether this config sends from component 197.
pub fn uses_extnav_component(cfg: &VisionNavConfig) -> bool {
    cfg.effective_mode() == Mode::ExternalNav || obstacle_active(cfg)
}

/// Subscribe to the pose and scan topics the config uses. Returns one
/// `event_subscribe_denied:<topic>` entry per refused subscription.
pub async fn subscribe_topics(
    ctx: &PluginContext,
    shared: &Arc<NavShared>,
    cfg: &VisionNavConfig,
) -> Vec<String> {
    let mut denied = Vec::new();
    if cfg.effective_mode() == Mode::ExternalNav {
        let topic = cfg.external_nav.pose_topic.as_str();
        let s = shared.clone();
        let cb = Arc::new(move |args: Value| s.on_pose_event(&args, monotonic_ns()));
        if let Err(e) = ctx.events.subscribe(topic, cb).await {
            eprintln!("vision-nav: subscribe {topic} failed: {e}");
            shared.pose_denied.store(true, Ordering::Relaxed);
            denied.push(format!("event_subscribe_denied:{topic}"));
        }
    }
    if obstacle_active(cfg) {
        let topic = cfg.obstacle.scan_topic.as_str();
        let s = shared.clone();
        let cb = Arc::new(move |args: Value| s.on_scan_event(&args, monotonic_ns()));
        if let Err(e) = ctx.events.subscribe(topic, cb).await {
            eprintln!("vision-nav: subscribe {topic} failed: {e}");
            shared.scan_denied.store(true, Ordering::Relaxed);
            denied.push(format!("event_subscribe_denied:{topic}"));
        }
    }
    denied
}

/// Companion state from the pose: tracking (good or degraded) is active;
/// no tracking for 2 s after the last tracked pose is critical; no pose
/// ever leaves it where it is.
pub fn companion_for(
    health: Option<ExtnavHealth>,
    last_tracked_ns: Option<i64>,
    now_ns: i64,
) -> Option<CompanionState> {
    match health {
        Some(ExtnavHealth::Good | ExtnavHealth::Degraded) => Some(CompanionState::Active),
        _ => last_tracked_ns
            .filter(|t| now_ns - t >= COMPANION_LOST_NS)
            .map(|_| CompanionState::Critical),
    }
}

/// The GCS estimator-state word for a pose health.
pub fn estimator_state_for(health: Option<ExtnavHealth>) -> &'static str {
    match health {
        Some(ExtnavHealth::Good) => "converged",
        Some(ExtnavHealth::Degraded) => "degraded",
        Some(ExtnavHealth::Lost) => "failed",
        None => "init",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pose_event(health: &str) -> Value {
        let floats = |v: &[f64]| Value::Array(v.iter().map(|x| Value::F64(*x)).collect());
        Value::Map(vec![(
            Value::from("payload"),
            Value::Map(vec![
                (
                    Value::from("pose"),
                    Value::Map(vec![
                        (
                            Value::from("R"),
                            floats(&[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]),
                        ),
                        (Value::from("t"), floats(&[1.0, 2.0, 0.0])),
                    ]),
                ),
                (Value::from("health"), Value::from(health)),
                (Value::from("frame"), Value::from("map_ned")),
            ]),
        )])
    }

    #[test]
    fn a_pose_goes_stale_and_a_bad_payload_is_recorded() {
        let s = NavShared::new(Mode::ExternalNav);
        s.on_pose_event(&pose_event("good"), 0);
        let v = s.pose_view(100_000_000);
        assert_eq!(v.seq, 1);
        assert_eq!(v.fresh.unwrap().0.t, [1.0, 2.0, 0.0]);
        assert!(s.pose_view(POSE_STALE_NS + 1).fresh.is_none());
        s.on_pose_event(&pose_event("bogus"), 0);
        let v = s.pose_view(0);
        assert_eq!(v.seq, 1);
        assert_eq!(v.error.as_deref(), Some("pose_health_unknown:bogus"));
    }

    #[test]
    fn obstacle_feed_reports_denied_waiting_live() {
        let s = NavShared::new(Mode::ExternalNav);
        assert_eq!(s.obstacle_feed(0), ObstacleFeed::Waiting);
        let scan = Value::Map(vec![(
            Value::from("payload"),
            Value::Map(vec![
                (Value::from("frame"), Value::from("body_frd")),
                (Value::from("angle_increment_rad"), Value::F64(0.1)),
                (Value::from("ranges_m"), Value::Array(vec![Value::F64(1.0)])),
            ]),
        )]);
        s.on_scan_event(&scan, 0);
        assert_eq!(s.obstacle_feed(0), ObstacleFeed::Live);
        assert_eq!(s.obstacle_feed(SCAN_STALE_NS + 1), ObstacleFeed::Waiting);
        s.scan_denied.store(true, Ordering::Relaxed);
        assert_eq!(s.obstacle_feed(0), ObstacleFeed::Denied);
    }

    #[test]
    fn companion_follows_tracking() {
        assert_eq!(
            companion_for(Some(ExtnavHealth::Degraded), None, 0),
            Some(CompanionState::Active)
        );
        // Never tracked: unchanged.
        assert_eq!(companion_for(None, None, 10 * SECOND), None);
        // Lost under 2 s after tracking: unchanged; then critical.
        assert_eq!(
            companion_for(Some(ExtnavHealth::Lost), Some(0), SECOND),
            None
        );
        assert_eq!(
            companion_for(None, Some(0), 2 * SECOND),
            Some(CompanionState::Critical)
        );
        assert_eq!(estimator_state_for(Some(ExtnavHealth::Lost)), "failed");
        assert_eq!(estimator_state_for(None), "init");
    }

    #[test]
    fn the_extnav_component_is_used_only_when_needed() {
        let mut cfg = VisionNavConfig::default();
        assert!(!uses_extnav_component(&cfg));
        cfg.mode = Mode::OpticalFlow;
        cfg.obstacle.enabled = true;
        assert!(uses_extnav_component(&cfg));
        cfg.active = false;
        assert!(!uses_extnav_component(&cfg));
        cfg.active = true;
        cfg.obstacle.enabled = false;
        cfg.mode = Mode::ExternalNav;
        assert!(uses_extnav_component(&cfg));
    }
}
