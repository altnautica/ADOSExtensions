//! Estimator contract + output shape.
//!
//! Every estimator answers `step(...)` with an [`EstimatorOutput`].
//! Optical-flow estimators fill the `flow_*` fields and leave the VIO
//! fields `None`; VIO estimators fill `pose` / `velocity` /
//! `covariance` and leave the flow fields `None`. The pipeline reads
//! `output_mode` to decide which MAVLink component + message family to
//! emit on.
//!
//! `EstimatorState` is shared across kinds: `off`, `init`,
//! `converging`, `converged`, `degraded`, `failed`. The companion
//! heartbeat + the degradation ladder are driven by it.

use crate::flow::GyroReading;
use crate::imu::ImuSample;

/// Which MAVLink path a sample rides.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputMode {
    OpticalFlow,
    Vio,
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

    /// Map a vendor-side state string to the estimator vocabulary; an
    /// unknown string is treated as degraded so the GCS surfaces a
    /// warning rather than silently treating it as healthy.
    pub fn from_vendor(s: &str) -> EstimatorState {
        match s {
            "off" => EstimatorState::Off,
            "init" => EstimatorState::Init,
            "converging" => EstimatorState::Converging,
            "converged" => EstimatorState::Converged,
            "failed" => EstimatorState::Failed,
            "degraded" => EstimatorState::Degraded,
            _ => EstimatorState::Degraded,
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

/// One sample produced by an estimator. The two halves never carry data
/// at once: an OF sample sets `flow_*`; a VIO sample sets
/// `pose`/`velocity`/`covariance`. A hybrid VIO sample may carry a
/// co-emitted OF sample in `extras_of` so one tick emits on both
/// MAVLink components.
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
    // VIO path
    pub pose: Option<(f32, f32, f32, f32, f32, f32)>, // (x,y,z,roll,pitch,yaw)
    pub velocity: Option<(f32, f32, f32)>,
    pub covariance: Option<Vec<f32>>, // 21-element upper-triangular
    pub feature_count: Option<i32>,
    pub reset_counter: Option<u32>,
    // Hybrid co-emit: an OF sample to also fire on comp 198.
    pub extras_of: Option<Box<EstimatorOutput>>,
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
            pose: None,
            velocity: None,
            covariance: None,
            feature_count: None,
            reset_counter: None,
            extras_of: None,
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
/// IMU sample is the time-aligned pick for the current frame.
pub struct StepInputs<'a> {
    pub prev_gray: Option<&'a crate::framing::GrayImage>,
    pub curr_gray: Option<&'a crate::framing::GrayImage>,
    /// VIO frame bytes for the SHM bridge (raw frame, not grayscale),
    /// carried alongside the grayscale so the OF tracker and the VIO
    /// engine each get what they need from one tick.
    pub curr_vio_frame: Option<&'a VioFrame>,
    pub dt_seconds: f32,
    pub gyro: Option<GyroReading>,
    pub imu_sample: Option<ImuSample>,
    pub range_reading: Option<RangeReading>,
}

/// A frame in the layout the VIO SHM ring needs: the raw pixel bytes
/// plus geometry + format + the capture timestamp in microseconds.
#[derive(Debug, Clone)]
pub struct VioFrame {
    pub ts_us: u64,
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    /// SHM ring fourcc-ish format tag (see `vio::FRAME_FORMAT_*`).
    pub pixel_format: u32,
    pub pixels: Vec<u8>,
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

    /// Start any backing engine. Default no-op; VIO estimators spawn
    /// their vendor binary here.
    fn configure(&mut self) {}

    /// Release held resources (subprocess teardown). Default no-op.
    fn shutdown(&mut self) {}
}
