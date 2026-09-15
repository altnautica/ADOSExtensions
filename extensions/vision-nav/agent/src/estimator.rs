//! Estimator contract + output shape.
//!
//! Every estimator answers `step(...)` with an [`EstimatorOutput`]
//! carrying the `flow_*` fields. The pipeline reads `output_mode` to
//! decide whether the sample is emitted and on which MAVLink component.
//!
//! `EstimatorState` is shared across kinds: `off`, `init`,
//! `converging`, `converged`, `degraded`, `failed`. The companion
//! heartbeat + the degradation ladder are driven by it.

use crate::flow::GyroReading;

/// Which MAVLink path a sample rides.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputMode {
    OpticalFlow,
    None,
}

/// Estimator readiness, shared across modes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EstimatorState {
    Off,
    Init,
    Converging,
    Converged,
    Degraded,
    Failed,
}

impl EstimatorState {
    pub fn as_str(self) -> &'static str {
        match self {
            EstimatorState::Off => "off",
            EstimatorState::Init => "init",
            EstimatorState::Converging => "converging",
            EstimatorState::Converged => "converged",
            EstimatorState::Degraded => "degraded",
            EstimatorState::Failed => "failed",
        }
    }
}

/// Which scale source produced an OF sample's distance, for the GCS.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ScaleSourceLabel {
    Rangefinder,
    Baro,
    Gps,
    Vision,
}

impl ScaleSourceLabel {
    pub fn as_str(self) -> &'static str {
        match self {
            ScaleSourceLabel::Rangefinder => "rangefinder",
            ScaleSourceLabel::Baro => "baro",
            ScaleSourceLabel::Gps => "gps",
            ScaleSourceLabel::Vision => "vision",
        }
    }
}

/// One sample produced by an estimator: the tracked angular flow, its
/// quality, and the range the flow was scaled with.
#[derive(Debug, Clone)]
pub struct EstimatorOutput {
    pub timestamp_us: i64,
    pub output_mode: OutputMode,
    pub state: EstimatorState,
    // OF path
    pub flow_rate_x: Option<f32>,
    pub flow_rate_y: Option<f32>,
    pub flow_rate_z: Option<f32>,
    pub flow_quality: Option<i32>,
    pub flow_distance_m: Option<f32>,
    pub flow_scale_source: Option<ScaleSourceLabel>,
    pub integration_time_us: Option<i64>,
}

impl EstimatorOutput {
    /// An explicit empty-shape `none` sample (off mode).
    pub fn none(timestamp_us: i64) -> Self {
        Self {
            timestamp_us,
            output_mode: OutputMode::None,
            state: EstimatorState::Off,
            flow_rate_x: None,
            flow_rate_y: None,
            flow_rate_z: None,
            flow_quality: None,
            flow_distance_m: None,
            flow_scale_source: None,
            integration_time_us: None,
        }
    }
}

/// A range reading handed to an estimator's `step`.
#[derive(Debug, Clone, Copy)]
pub struct RangeReading {
    pub distance_m: f32,
    pub quality: i32,
}

/// Inputs to one estimator step. The frames are already grayscale; the
/// gyro reading is the time-aligned pick for the current frame.
pub struct StepInputs<'a> {
    pub prev_gray: Option<&'a crate::framing::GrayImage>,
    pub curr_gray: Option<&'a crate::framing::GrayImage>,
    pub dt_seconds: f32,
    pub gyro: Option<GyroReading>,
    pub range_reading: Option<RangeReading>,
}

/// The estimator contract consumed by the runtime pipeline.
pub trait Estimator: Send {
    /// Stable id (matches the registry key + the config mode).
    fn estimator_id(&self) -> &'static str;

    /// Which MAVLink path this estimator's samples ride.
    fn output_mode(&self) -> OutputMode;

    /// Process one frame pair (+ optional IMU + range) into a sample.
    /// `None` means insufficient input this tick (warm-up, off, or no
    /// trackable features) and the pipeline skips emission.
    fn step(&mut self, inputs: &StepInputs<'_>) -> Option<EstimatorOutput>;
}
