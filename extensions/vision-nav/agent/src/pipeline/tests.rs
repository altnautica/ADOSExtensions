//! Tests for the per-frame pipeline helpers.

use super::status::pre_arm_inputs;
use super::ticks::companion_state_from_i64;
use super::*;
use crate::estimator::EstimatorState;
use crate::pre_arm::Severity;
use std::sync::atomic::Ordering;

fn field<'a>(v: &'a Value, key: &str) -> Option<&'a Value> {
    match v {
        Value::Map(e) => e
            .iter()
            .find(|(k, _)| k.as_str() == Some(key))
            .map(|(_, val)| val),
        _ => None,
    }
}

#[test]
fn companion_state_machine_promotes_and_demotes() {
    let companion = Arc::new(AtomicI64::new(CompanionState::Inactive as i64));
    let mut streak = None;
    update_companion(&companion, EstimatorState::Converged, 0, &mut streak);
    assert_eq!(
        companion_state_from_i64(companion.load(Ordering::Relaxed)),
        CompanionState::Active
    );
    update_companion(
        &companion,
        EstimatorState::Degraded,
        1_000_000_000,
        &mut streak,
    );
    assert_eq!(
        companion_state_from_i64(companion.load(Ordering::Relaxed)),
        CompanionState::Active
    );
    update_companion(
        &companion,
        EstimatorState::Degraded,
        1_000_000_000 + DEGRADED_GRACE_NS,
        &mut streak,
    );
    assert_eq!(
        companion_state_from_i64(companion.load(Ordering::Relaxed)),
        CompanionState::Critical
    );
}

#[test]
fn stale_frames_mark_failed_and_critical() {
    let health = Arc::new(Mutex::new(HealthSnapshot {
        flow_quality: Some(180),
        flow_distance_m: Some(1.2),
        estimator_state: Some("converged".into()),
        ..Default::default()
    }));
    let companion = Arc::new(AtomicI64::new(CompanionState::Active as i64));
    let cfg = VisionNavConfig {
        mode: Mode::OpticalFlow,
        ..VisionNavConfig::default()
    };
    let gate = PreArmGate::with_flow_quality_gate(cfg.flow_quality_min);

    mark_frames_stale(&health, &gate, &cfg, &companion);

    assert_eq!(
        companion_state_from_i64(companion.load(Ordering::Relaxed)),
        CompanionState::Critical
    );
    let h = health.lock().unwrap();
    assert_eq!(h.estimator_state.as_deref(), Some("failed"));
    assert_eq!(h.flow_rate_hz, Some(0.0));
    assert_eq!(h.flow_quality, None);
    assert_eq!(h.flow_distance_m, None);
    let report = h.pre_arm_report.clone().unwrap();
    assert_eq!(field(&report, "armable").unwrap().as_bool(), Some(false));

    let inputs = pre_arm_inputs(&cfg, CompanionState::Critical, None, None);
    let checks = gate.evaluate(&inputs).checks;
    assert!(checks
        .iter()
        .any(|c| c.id == "companion_active" && c.severity == Severity::Blocking));
}

#[test]
fn frame_stale_threshold_matches_degraded_grace() {
    assert_eq!(FRAME_STALE_NS, DEGRADED_GRACE_NS);
    assert_eq!(FRAME_STALE_NS, 2_000_000_000);
}

#[test]
fn default_config_builds_the_null_estimator() {
    let ladder = Arc::new(ScaleLadder::new(false));
    let e = build_estimator(&VisionNavConfig::default(), ladder);
    assert_eq!(e.estimator_id(), "off");
}

#[test]
fn health_flow_quality_clears_on_a_none_sample() {
    let health = Arc::new(Mutex::new(HealthSnapshot {
        flow_quality: Some(200),
        ..Default::default()
    }));
    let companion = Arc::new(AtomicI64::new(CompanionState::Active as i64));
    let cfg = VisionNavConfig::default();
    let gate = PreArmGate::default();
    publish_health(
        &health,
        &gate,
        &cfg,
        None,
        &companion,
        TickInputs {
            sync_offset_ms: None,
            imu_rate_hz: None,
            intrinsics_loaded: false,
            dt: 0.033,
            rangefinder: &NoRangefinder,
            reading_m: None,
        },
    );
    assert_eq!(health.lock().unwrap().flow_quality, None);
    let v = health.lock().unwrap().to_value();
    assert!(matches!(field(&v, "flowQuality"), Some(Value::Nil)));
}
