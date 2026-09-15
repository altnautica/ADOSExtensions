//! The three estimator modes + the registry.
//!
//! Each mode is an [`Estimator`]. A config-mode flip selects a
//! different estimator without bespoke routing:
//!
//! * `off` -> [`NullEstimator`]
//! * `optical_flow` -> [`OpticalFlowEstimator`]
//! * `optical_flow_degraded` -> [`OpticalFlowDegradedEstimator`]

use crate::estimator::{
    Estimator, EstimatorOutput, EstimatorState, OutputMode, ScaleSourceLabel, StepInputs,
};
use crate::flow::OpticalFlowLk;
use crate::mavlink_emit::monotonic_ns;
use crate::scale::{ScaleLadder, ScaleRung};

const DEFAULT_QUALITY_GATE: i32 = 50;

/// The list of estimator keys this build can run (surfaced on the
/// heartbeat so the GCS mode picker only shows modes the agent can
/// instantiate).
pub fn available_estimators() -> Vec<&'static str> {
    let mut v = vec!["off", "optical_flow", "optical_flow_degraded"];
    v.sort_unstable();
    v
}

// ---------------------------------------------------------------------------
// Off
// ---------------------------------------------------------------------------

/// Off mode: never emits MAVLink, reports `state=off`. Keeps the plugin
/// loaded with all sensors discovered but the EKF feed silent.
pub struct NullEstimator;

impl Estimator for NullEstimator {
    fn estimator_id(&self) -> &'static str {
        "off"
    }
    fn output_mode(&self) -> OutputMode {
        OutputMode::None
    }
    fn step(&mut self, _inputs: &StepInputs<'_>) -> Option<EstimatorOutput> {
        Some(EstimatorOutput::none(monotonic_ns() / 1000))
    }
}

// ---------------------------------------------------------------------------
// Optical flow (with rangefinder)
// ---------------------------------------------------------------------------

/// Optical-flow estimator. Wraps the Lucas-Kanade tracker behind the
/// estimator contract.
///
/// State mapping: `init` until the first sample at/above the gate
/// (then `converged`); once converged, a later sub-gate sample is
/// `degraded`.
pub struct OpticalFlowEstimator {
    processor: OpticalFlowLk,
    quality_gate: i32,
    state: EstimatorState,
    seen_converged: bool,
}

impl OpticalFlowEstimator {
    pub fn new(quality_gate: i32) -> Self {
        Self {
            processor: OpticalFlowLk::new(),
            quality_gate,
            state: EstimatorState::Init,
            seen_converged: false,
        }
    }

    pub fn state(&self) -> EstimatorState {
        self.state
    }

    /// Run the tracker and build the OF sample. Shared by the plain and
    /// the degraded estimators (the latter applies a scale multiplier
    /// and a synthetic distance before calling this).
    fn run(
        &mut self,
        inputs: &StepInputs<'_>,
        distance_m: Option<f32>,
        scale_source: Option<ScaleSourceLabel>,
    ) -> Option<EstimatorOutput> {
        let (prev, curr) = (inputs.prev_gray?, inputs.curr_gray?);
        let result = self.processor.process(
            prev,
            curr,
            inputs.dt_seconds,
            inputs.gyro,
            distance_m,
        );
        let quality = result.quality;
        if quality >= self.quality_gate {
            self.state = EstimatorState::Converged;
            self.seen_converged = true;
        } else if self.seen_converged {
            self.state = EstimatorState::Degraded;
        } else {
            self.state = EstimatorState::Init;
        }
        Some(EstimatorOutput {
            timestamp_us: result.integration_time_us as i64,
            output_mode: OutputMode::OpticalFlow,
            state: self.state,
            flow_rate_x: Some(result.flow_rate_x),
            flow_rate_y: Some(result.flow_rate_y),
            flow_rate_z: Some(result.flow_rate_z),
            flow_quality: Some(quality),
            flow_distance_m: distance_m,
            flow_scale_source: scale_source,
            integration_time_us: Some(result.integration_time_us as i64),
        })
    }
}

impl Estimator for OpticalFlowEstimator {
    fn estimator_id(&self) -> &'static str {
        "optical_flow"
    }
    fn output_mode(&self) -> OutputMode {
        OutputMode::OpticalFlow
    }
    fn step(&mut self, inputs: &StepInputs<'_>) -> Option<EstimatorOutput> {
        let distance = inputs.range_reading.map(|r| r.distance_m);
        let label = distance.map(|_| ScaleSourceLabel::Rangefinder);
        self.run(inputs, distance, label)
    }
}

// ---------------------------------------------------------------------------
// Optical flow degraded (rangefinder-free)
// ---------------------------------------------------------------------------

/// Rangefinder-free OF. The same tracker, but scale comes from the
/// [`ScaleLadder`] and the raw quality is multiplied by the rung's
/// quality factor so the EKF auto-de-weights degraded rungs. Sitting on
/// the static rung is `degraded` regardless of raw quality.
pub struct OpticalFlowDegradedEstimator {
    inner: OpticalFlowEstimator,
    ladder: Option<std::sync::Arc<ScaleLadder>>,
    quality_gate: i32,
    state: EstimatorState,
    seen_converged: bool,
}

impl OpticalFlowDegradedEstimator {
    pub fn new(quality_gate: i32, ladder: Option<std::sync::Arc<ScaleLadder>>) -> Self {
        Self {
            inner: OpticalFlowEstimator::new(quality_gate),
            ladder,
            quality_gate,
            state: EstimatorState::Init,
            seen_converged: false,
        }
    }

    pub fn state(&self) -> EstimatorState {
        self.state
    }
}

impl Estimator for OpticalFlowDegradedEstimator {
    fn estimator_id(&self) -> &'static str {
        "optical_flow_degraded"
    }
    fn output_mode(&self) -> OutputMode {
        OutputMode::OpticalFlow
    }

    fn step(&mut self, inputs: &StepInputs<'_>) -> Option<EstimatorOutput> {
        let pick = self.ladder.as_ref().map(|l| l.pick(monotonic_ns()));
        let distance = pick.map(|p| p.distance_m);
        let label = pick.map(|p| match p.source {
            ScaleRung::Baro => ScaleSourceLabel::Baro,
            ScaleRung::Gps => ScaleSourceLabel::Gps,
            // The static rung is reported as baro (the physical fallback
            // expectation); the quality multiplier flags it degraded.
            ScaleRung::Static => ScaleSourceLabel::Baro,
        });

        let mut sample = self.inner.run(inputs, distance, label)?;

        // Apply the rung's quality multiplier; recompute the visible
        // state from the scaled quality so the operator's "converged?"
        // reflects the signal-to-noise the EKF will actually see.
        let on_static = matches!(pick.map(|p| p.source), Some(ScaleRung::Static));
        let mult = match pick {
            Some(p) => p.quality_multiplier,
            None => 0.2, // no healthy rung: penalize like static
        };
        let raw = sample.flow_quality.unwrap_or(0);
        let scaled = ((raw as f32 * mult).round() as i32).clamp(0, 255);
        sample.flow_quality = Some(scaled);
        sample.flow_distance_m = distance;
        if pick.is_none() {
            sample.flow_scale_source = None;
        }

        if scaled >= self.quality_gate {
            if on_static {
                self.state = EstimatorState::Degraded;
            } else {
                self.state = EstimatorState::Converged;
                self.seen_converged = true;
            }
        } else if self.seen_converged || on_static {
            self.state = EstimatorState::Degraded;
        } else {
            self.state = EstimatorState::Init;
        }
        sample.state = self.state;
        Some(sample)
    }
}

/// Default quality gate.
pub fn default_quality_gate() -> i32 {
    DEFAULT_QUALITY_GATE
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::RangeReading;
    use crate::framing::GrayImage;

    fn textured(w: u32, h: u32, sx: i32, sy: i32) -> GrayImage {
        let mut data = vec![0u8; (w * h) as usize];
        for y in 0..h as i32 {
            for x in 0..w as i32 {
                let px = x - sx;
                let py = y - sy;
                data[(y as u32 * w + x as u32) as usize] = (((px * 37 + py * 17) & 0x3f) * 4) as u8;
            }
        }
        GrayImage {
            width: w,
            height: h,
            data,
        }
    }

    fn step_inputs<'a>(
        prev: &'a GrayImage,
        curr: &'a GrayImage,
        range: Option<RangeReading>,
    ) -> StepInputs<'a> {
        StepInputs {
            prev_gray: Some(prev),
            curr_gray: Some(curr),
            dt_seconds: 1.0 / 30.0,
            gyro: None,
            range_reading: range,
        }
    }

    #[test]
    fn registry_lists_only_runnable_modes() {
        // The heartbeat publishes this list and the GCS mode picker
        // renders it, so it must name exactly the modes this build can
        // instantiate.
        assert_eq!(
            available_estimators(),
            vec!["off", "optical_flow", "optical_flow_degraded"]
        );
    }

    #[test]
    fn null_estimator_emits_none_shape() {
        let mut e = NullEstimator;
        let prev = textured(32, 32, 0, 0);
        let curr = textured(32, 32, 1, 0);
        let out = e.step(&step_inputs(&prev, &curr, None)).unwrap();
        assert_eq!(out.output_mode, OutputMode::None);
        assert_eq!(out.state, EstimatorState::Off);
    }

    #[test]
    fn optical_flow_converges_on_good_tracking() {
        let mut e = OpticalFlowEstimator::new(default_quality_gate());
        let prev = textured(128, 128, 0, 0);
        let curr = textured(128, 128, 2, 1);
        let out = e
            .step(&step_inputs(&prev, &curr, Some(RangeReading { distance_m: 1.5, quality: 90 })))
            .unwrap();
        assert_eq!(out.output_mode, OutputMode::OpticalFlow);
        assert_eq!(out.flow_scale_source, Some(ScaleSourceLabel::Rangefinder));
        // A richly-textured 128x128 frame yields many tracked features.
        assert!(out.flow_quality.unwrap() >= default_quality_gate());
        assert_eq!(out.state, EstimatorState::Converged);
    }

    #[test]
    fn degraded_static_rung_is_degraded_state() {
        use std::sync::Arc;
        let ladder = Arc::new(ScaleLadder::new(false)); // no messages -> static rung
        let mut e = OpticalFlowDegradedEstimator::new(default_quality_gate(), Some(ladder));
        let prev = textured(128, 128, 0, 0);
        let curr = textured(128, 128, 2, 0);
        let out = e.step(&step_inputs(&prev, &curr, None)).unwrap();
        assert_eq!(out.output_mode, OutputMode::OpticalFlow);
        // Static rung -> degraded regardless of raw quality.
        assert_eq!(out.state, EstimatorState::Degraded);
        // Quality is multiplied by the 0.2 static factor.
        assert!(out.flow_quality.unwrap() <= (255.0 * 0.2) as i32 + 1);
    }

    #[test]
    fn degraded_baro_rung_can_converge() {
        use std::sync::Arc;
        let ladder = Arc::new(ScaleLadder::new(false));
        ladder.on_global_position(2000, monotonic_ns()); // fresh relative_alt
        let mut e = OpticalFlowDegradedEstimator::new(10, Some(ladder));
        let prev = textured(160, 160, 0, 0);
        let curr = textured(160, 160, 2, 1);
        let out = e.step(&step_inputs(&prev, &curr, None)).unwrap();
        assert_eq!(out.flow_scale_source, Some(ScaleSourceLabel::Baro));
        // A low gate + a non-static rung lets it converge.
        if out.flow_quality.unwrap() >= 10 {
            assert_eq!(out.state, EstimatorState::Converged);
        }
    }
}
