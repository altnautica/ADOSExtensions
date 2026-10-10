//! Companion state, health snapshot and pre-arm evaluation for the
//! per-frame worker.

use std::sync::atomic::{AtomicI64, Ordering};
use std::sync::{Arc, Mutex};

use super::ticks::companion_state_from_i64;
use super::DEGRADED_GRACE_NS;
use crate::config::{Topology, VisionNavConfig};
use crate::estimator::{EstimatorOutput, EstimatorState};
use crate::health::{CompanionState, HealthSnapshot};
use crate::pre_arm::{PreArmGate, PreArmInputs};
use crate::rangefinder::Rangefinder;

pub(super) fn update_companion(
    companion: &Arc<AtomicI64>,
    state: EstimatorState,
    ts_ns: i64,
    degraded_streak_start: &mut Option<i64>,
) {
    match state {
        EstimatorState::Converged => {
            *degraded_streak_start = None;
            companion.store(CompanionState::Active as i64, Ordering::Relaxed);
        }
        EstimatorState::Degraded | EstimatorState::Failed => match *degraded_streak_start {
            None => *degraded_streak_start = Some(ts_ns),
            Some(start) if ts_ns - start >= DEGRADED_GRACE_NS => {
                companion.store(CompanionState::Critical as i64, Ordering::Relaxed);
            }
            _ => {}
        },
        // init / converging / off: leave the companion where it was.
        _ => *degraded_streak_start = None,
    }
}

/// Per-tick measurements for [`publish_health`].
pub(super) struct TickInputs<'a> {
    pub(super) sync_offset_ms: Option<f32>,
    pub(super) imu_rate_hz: Option<f32>,
    pub(super) intrinsics_loaded: bool,
    pub(super) dt: f32,
    pub(super) rangefinder: &'a dyn Rangefinder,
    pub(super) reading_m: Option<f32>,
}

pub(super) fn pre_arm_inputs(
    cfg: &VisionNavConfig,
    companion_state: CompanionState,
    output: Option<&EstimatorOutput>,
    rangefinder: Option<(&dyn Rangefinder, Option<f32>)>,
) -> PreArmInputs {
    let (reading, min, max) = match rangefinder {
        Some((rf, reading)) => (reading, rf.min_range_m(), rf.max_range_m()),
        None => (None, 0.0, 0.0),
    };
    PreArmInputs {
        mode: cfg.effective_mode(),
        companion_state,
        flow_quality: output.and_then(|o| o.flow_quality),
        flow_scale_source: output
            .and_then(|o| o.flow_scale_source)
            .map(|s| s.as_str().to_string()),
        rangefinder_topology: match cfg.rangefinder.topology {
            Topology::None => None,
            t => Some(t.as_str().to_string()),
        },
        rangefinder_reading_m: reading,
        rangefinder_min_m: min,
        rangefinder_max_m: max,
    }
}

pub(super) fn publish_health(
    health: &Arc<Mutex<HealthSnapshot>>,
    gate: &PreArmGate,
    cfg: &VisionNavConfig,
    output: Option<&EstimatorOutput>,
    companion: &Arc<AtomicI64>,
    tick: TickInputs<'_>,
) {
    let companion_state = companion_state_from_i64(companion.load(Ordering::Relaxed));
    let inputs = pre_arm_inputs(
        cfg,
        companion_state,
        output,
        Some((tick.rangefinder, tick.reading_m)),
    );
    let report = gate.evaluate(&inputs);

    let mut h = health.lock().expect("health lock");
    h.companion_state = Some(companion_state);
    h.estimator_state = Some(
        output
            .map(|o| o.state)
            .unwrap_or(EstimatorState::Off)
            .as_str()
            .to_string(),
    );
    h.camera_intrinsics_loaded = tick.intrinsics_loaded;
    h.camera_imu_sync_offset_ms = tick.sync_offset_ms;
    h.imu_rate_hz = tick.imu_rate_hz;
    h.flow_scale_source = inputs.flow_scale_source.clone();
    h.flow_distance_m = tick.reading_m;
    h.rangefinder_error = tick.rangefinder.error();
    h.pre_arm_report = Some(report.to_value());
    if tick.dt > 0.0 {
        h.flow_rate_hz = Some(1.0 / tick.dt);
    }
    h.flow_quality = output.and_then(|o| o.flow_quality);
}

/// No frame for [`FRAME_STALE_NS`]: the companion goes critical, the
/// estimator reports failed with zero flow rate and no quality or
/// distance, and pre-arm is re-evaluated (companion_active blocks).
pub(super) fn mark_frames_stale(
    health: &Arc<Mutex<HealthSnapshot>>,
    gate: &PreArmGate,
    cfg: &VisionNavConfig,
    companion: &Arc<AtomicI64>,
) {
    companion.store(CompanionState::Critical as i64, Ordering::Relaxed);
    let report = gate.evaluate(&pre_arm_inputs(cfg, CompanionState::Critical, None, None));
    let mut h = health.lock().expect("health lock");
    h.companion_state = Some(CompanionState::Critical);
    h.estimator_state = Some(EstimatorState::Failed.as_str().to_string());
    h.flow_rate_hz = Some(0.0);
    h.flow_quality = None;
    h.flow_distance_m = None;
    h.pre_arm_report = Some(report.to_value());
}
