//! The capture service's runtime configuration: the `capture.*` plugin config
//! keys, the pose-provider choice, and the camera-intrinsics resolution.
//!
//! The capture *core* ([`crate::CaptureConfig`]) declares the rig and the
//! selection thresholds. This runtime layer wraps it with the service-only
//! settings: the enable gate, the pose provider, any per-camera intrinsics
//! override, and the stated pose prior. A camera without an override gets a
//! pinhole from the field of view its frames carry (the agent's camera
//! roster).
//!
//! The plugin host stores the config as flat dotted keys (`capture.enabled`,
//! `capture.selection.max_interval_ms`, ...). [`CaptureRuntimeConfig::from_values`]
//! resolves a map of those keys (absent keys take today's defaults) so the
//! resolution is pure and testable without a host.

use std::collections::{BTreeMap, HashMap};

use ados_capture_contract::{CameraModel, CameraModelModel};
use serde::Deserialize;
use serde_json::Value;

pub use ados_mapping_pose::config::ConfigError;
use ados_mapping_pose::config::{integer, optional_unsigned, typed, unsigned};
use ados_mapping_protocol::capture::POSE_COV_LEN;
use ados_mapping_protocol::capture_ctl::{ExposureMode, ExposureView};

use crate::config::{CameraConfig, CaptureConfig, CaptureProfile, SelectionParams};
use crate::recorder::convert::ImageEncoding;

/// Segment roll (`capture.segment`); the contract default is 4 GiB or 10 min.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct SegmentSettings {
    pub max_bytes: u64,
    pub max_duration_s: u64,
}

impl Default for SegmentSettings {
    fn default() -> Self {
        Self {
            max_bytes: 4 << 30,
            max_duration_s: 600,
        }
    }
}

impl SegmentSettings {
    pub fn limits(&self) -> ados_capture_contract::SegmentLimits {
        ados_capture_contract::SegmentLimits {
            max_bytes: self.max_bytes.max(1 << 20),
            max_duration_ns: self.max_duration_s.max(1) * 1_000_000_000,
        }
    }
}

/// The horizontal field of view assumed for a camera whose roster entry
/// gives no `fov_deg`: the vision engine's own assumption for such a camera.
pub const DEFAULT_HFOV_DEG: f64 = 70.0;

/// A per-camera intrinsics override in the capture contract's camera-model
/// form (frame size comes from the frames themselves). When a camera has been
/// calibrated the operator supplies these so reconstruction is metric; absent,
/// the service derives an uncalibrated pinhole from the frame size and the
/// field of view.
///
/// Unknown keys are refused, so a pre-contract override
/// (`distortion_model`/`distortion_params`) fails with an error naming the
/// accepted keys instead of silently becoming a calibrated pinhole.
#[derive(Debug, Clone, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct IntrinsicsOverride {
    #[serde(default = "pinhole")]
    pub model: CameraModelModel,
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    /// Coefficients in the model's order: none for `pinhole`, `k1, k2, p1, p2,
    /// k3` for `opencv`, `k1..k4` for `opencv_fisheye` and `kb4`.
    #[serde(default)]
    pub dist: Vec<f64>,
    /// Rolling-shutter readout time; 0 is a global shutter.
    #[serde(default)]
    pub readout_ns: u64,
}

fn pinhole() -> CameraModelModel {
    CameraModelModel::Pinhole
}

impl IntrinsicsOverride {
    fn to_camera_model(&self, width: u32, height: u32) -> CameraModel {
        CameraModel {
            model: self.model,
            width,
            height,
            fx: self.fx,
            fy: self.fy,
            cx: self.cx,
            cy: self.cy,
            dist: self.dist.clone(),
            // An operator-supplied override IS the calibration.
            calibrated: true,
            readout_ns: self.readout_ns,
        }
    }
}

/// Derive an uncalibrated pinhole from the frame size and horizontal field of
/// view: `fx = fy = (width/2) / tan(hfov/2)`, principal point at the centre, no
/// distortion. The compute node treats these as an initial guess to refine.
///
/// The result is stamped `calibrated: false`. This matters: a rig captured on a
/// nominal 70-degree pinhole with zero distortion produces a metrically wrong
/// reconstruction — scaled, and bent at the frame edges — and nothing else in
/// the output says so. Labelling the guess is what lets the operator surface
/// badge it and the reconstructor treat it as an estimate.
pub fn default_intrinsics(width: u32, height: u32, hfov_deg: f64) -> CameraModel {
    let w = width.max(1) as f64;
    let h = height.max(1) as f64;
    let hfov = hfov_deg.clamp(1.0, 179.0).to_radians();
    let fx = (w / 2.0) / (hfov / 2.0).tan();
    CameraModel {
        model: CameraModelModel::Pinhole,
        width: width.max(1),
        height: height.max(1),
        fx,
        fy: fx,
        cx: w / 2.0,
        cy: h / 2.0,
        dist: Vec::new(),
        calibrated: false,
        readout_ns: 0,
    }
}

/// The DOP assumed when the flight controller reports none, so a pose still
/// carries a stated prior rather than silently carrying none. Chosen as a
/// mid-range open-sky figure; a rig that reports real DOP never uses it.
const ASSUMED_DOP: f64 = 2.0;

/// The pose uncertainty this rig states on every keyframe.
///
/// This is a DECLARED PRIOR, not a measurement, and that distinction is the
/// whole reason it is configuration. GPS_RAW_INT's `eph`/`epv` are
/// dilution-of-precision figures — UNITLESS — and the metric `h_acc`/`v_acc`
/// extension fields are not carried in the vehicle state. A metre sigma
/// is therefore `DOP x UERE`, and the UERE depends on the receiver: roughly
/// 0.02 m on an RTK-fixed module and 2-3 m on a consumer one. Only the operator
/// knows which module is bolted to the aircraft, so the default is the
/// conservative consumer figure and an operator with better hardware states it.
///
/// Erring large is the safe direction for a reconstruction prior: it biases the
/// bundle adjustment toward trusting its own imagery over the GNSS. Erring
/// small pins cameras to wrong positions and is exactly what makes
/// `pose_prior_mapper` unstable.
#[derive(Debug, Clone, Copy, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct PosePrior {
    /// Position sigma in metres, stated outright. When set it wins over the
    /// DOP derivation — the right knob for an RTK rig.
    pub position_sigma_m: Option<f64>,
    /// The receiver's user-equivalent range error in metres, multiplied by the
    /// reported DOP when `position_sigma_m` is unset.
    pub gnss_uere_m: f64,
    /// Orientation sigma in radians. The flight controller publishes no
    /// attitude uncertainty in the vehicle state, so this is a declared figure;
    /// the default is ~1.1 degrees.
    pub orientation_sigma_rad: f64,
}

impl Default for PosePrior {
    fn default() -> Self {
        Self {
            position_sigma_m: None,
            gnss_uere_m: 2.5,
            orientation_sigma_rad: 0.02,
        }
    }
}

impl PosePrior {
    /// The row-major 6x6 covariance stated with every FC EKF pose: the
    /// operator's position sigma, else an assumed DOP times the receiver UERE.
    pub fn fc_covariance(&self) -> [f64; POSE_COV_LEN] {
        let sigma = self
            .position_sigma_m
            .unwrap_or(ASSUMED_DOP * self.gnss_uere_m)
            .max(f64::MIN_POSITIVE);
        ados_mapping_pose::pose::diagonal_cov(sigma, self.orientation_sigma_rad)
    }
}

/// Every plugin config key the capture service reads. Each is read with a nil
/// default, and an absent (nil) key resolves to the default below.
pub const CONFIG_KEYS: &[&str] = &[
    KEY_ENABLED,
    KEY_CAPTURE_PROFILE,
    KEY_CAMERAS,
    KEY_MIN_TRANSLATION_M,
    KEY_MIN_ROTATION_RAD,
    KEY_MAX_INTERVAL_MS,
    KEY_MAX_KEYFRAMES,
    KEY_INTRINSICS,
    KEY_POSE_PRIOR,
    KEY_RECORD_ON_START,
    KEY_IMAGE_ENCODING,
    KEY_MAINS_HZ,
    KEY_EXPOSURE,
    KEY_SEGMENT,
    KEY_MEMORY_BUDGET_MB,
    KEY_RECONSTRUCT_STEPS,
    KEY_POSE_SOURCE,
];

pub const KEY_ENABLED: &str = "capture.enabled";
pub const KEY_CAPTURE_PROFILE: &str = "capture.capture_profile";
pub const KEY_CAMERAS: &str = "capture.cameras";
pub const KEY_MIN_TRANSLATION_M: &str = "capture.selection.min_translation_m";
pub const KEY_MIN_ROTATION_RAD: &str = "capture.selection.min_rotation_rad";
pub const KEY_MAX_INTERVAL_MS: &str = "capture.selection.max_interval_ms";
pub const KEY_MAX_KEYFRAMES: &str = "capture.selection.max_keyframes";
pub const KEY_INTRINSICS: &str = "capture.intrinsics";
pub const KEY_POSE_PRIOR: &str = "capture.pose_prior";
pub const KEY_RECORD_ON_START: &str = "capture.record_on_start";
pub const KEY_IMAGE_ENCODING: &str = "capture.image_encoding";
pub const KEY_MAINS_HZ: &str = "capture.mains_hz";
pub const KEY_EXPOSURE: &str = "capture.exposure";
pub const KEY_SEGMENT: &str = "capture.segment";
pub const KEY_MEMORY_BUDGET_MB: &str = "capture.memory_budget_mb";
pub const KEY_RECONSTRUCT_STEPS: &str = "capture.reconstruct_steps";
pub const KEY_POSE_SOURCE: &str = "capture.pose_source";

/// Where a session's pose comes from (`capture.pose_source`), chosen when
/// the session starts and kept until it closes.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PoseSourceChoice {
    /// Navigation's pose when it streamed one in the last 2 s, else the
    /// flight controller's EKF.
    #[default]
    Auto,
    FcEkf,
    Navigation,
}

/// The initial exposure lock: unlocked, walking caps.
fn default_exposure() -> ExposureView {
    ExposureView {
        locked: false,
        exposure_us: None,
        gain: None,
        mode: Some(ExposureMode::Walking),
        applied: false,
        reason: None,
    }
}

/// The capture service's full runtime configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct CaptureRuntimeConfig {
    pub enabled: bool,
    /// The device id (the plugin host's agent id); the session id embeds it.
    pub device_id: String,
    pub capture: CaptureConfig,
    pub intrinsics: HashMap<String, IntrinsicsOverride>,
    /// The uncertainty stated on every FC EKF pose (see [`PosePrior`]).
    pub pose_prior: PosePrior,
    /// Start a session as soon as the service runs (a drone mapping flight).
    pub record_on_start: bool,
    pub image_encoding: ImageEncoding,
    /// Mains frequency the exposure rule avoids flicker for (50 or 60).
    pub mains_hz: u32,
    pub exposure: ExposureView,
    pub segment: SegmentSettings,
    /// `capture.memory_budget_mb` (the keyframe and scan queues); `None`
    /// resolves from the unit's cap.
    pub memory_budget_mb: Option<u64>,
    /// `capture.reconstruct_steps`: the splat training length the compute node
    /// uses for this drone's sessions, carried on every capture status. `None`
    /// (absent or 0) leaves the node's default in charge. Not a capture
    /// setting: a change reaches the running service without a restart (see
    /// [`CaptureRuntimeConfig::same_run`]).
    pub reconstruct_steps: Option<u64>,
    pub pose_source: PoseSourceChoice,
}

/// The live `capture.reconstruct_steps`, shared between the supervisor (which
/// re-reads it on every config poll) and the running capture loop (which
/// stamps it on every capture status), so a new detail level reaches the
/// compute node without restarting a capture in flight.
#[derive(Debug, Clone, Default)]
pub struct SharedSteps(std::sync::Arc<std::sync::atomic::AtomicU64>);

impl SharedSteps {
    /// Store the resolved value; `None` is kept as 0 (unset).
    pub fn set(&self, steps: Option<u64>) {
        self.0
            .store(steps.unwrap_or(0), std::sync::atomic::Ordering::Relaxed);
    }

    /// The current value, `None` when unset.
    pub fn get(&self) -> Option<u64> {
        Some(self.0.load(std::sync::atomic::Ordering::Relaxed)).filter(|n| *n > 0)
    }
}

impl Default for CaptureRuntimeConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            device_id: String::new(),
            capture: CaptureConfig::default(),
            intrinsics: HashMap::new(),
            pose_prior: PosePrior::default(),
            record_on_start: false,
            image_encoding: ImageEncoding::Raw,
            mains_hz: 50,
            exposure: default_exposure(),
            segment: SegmentSettings::default(),
            memory_budget_mb: None,
            reconstruct_steps: None,
            pose_source: PoseSourceChoice::Auto,
        }
    }
}

/// The per-camera intrinsics overrides, refusing one whose `dist` length does
/// not match its model (a misread coefficient bends the reconstruction).
fn intrinsics(
    values: &BTreeMap<String, Value>,
) -> Result<HashMap<String, IntrinsicsOverride>, ConfigError> {
    let map: HashMap<String, IntrinsicsOverride> = typed(values, KEY_INTRINSICS, HashMap::new())?;
    for (id, o) in &map {
        let want = CameraModel::expected_dist_len(o.model);
        if o.dist.len() != want {
            return Err(ConfigError {
                key: KEY_INTRINSICS,
                reason: format!(
                    "camera `{id}`: model {} takes {want} distortion coefficients, got {}",
                    o.model,
                    o.dist.len()
                ),
            });
        }
    }
    Ok(map)
}

/// `capture.mains_hz`: 50 or 60.
fn mains_hz(values: &BTreeMap<String, Value>) -> Result<u32, ConfigError> {
    match integer(values, KEY_MAINS_HZ, 50)? {
        50 => Ok(50),
        60 => Ok(60),
        n => Err(ConfigError {
            key: KEY_MAINS_HZ,
            reason: format!("expected 50 or 60, got {n}"),
        }),
    }
}

impl CaptureRuntimeConfig {
    /// Resolve the config from the plugin's `capture.*` keys (see
    /// [`CONFIG_KEYS`]). An absent or null key takes its default, so an empty
    /// map is the disabled default. A present key that does not parse (an
    /// unknown camera role, a string where a number belongs) is an error naming
    /// the key, never a silent fallback: one bad value must not quietly leave
    /// capture reporting `enabled: false` with no reason.
    pub fn from_values(
        device_id: &str,
        values: &BTreeMap<String, Value>,
    ) -> Result<Self, ConfigError> {
        let defaults = SelectionParams::default();
        let selection = SelectionParams {
            min_translation_m: typed(values, KEY_MIN_TRANSLATION_M, defaults.min_translation_m)?,
            min_rotation_rad: typed(values, KEY_MIN_ROTATION_RAD, defaults.min_rotation_rad)?,
            max_interval_ms: integer(values, KEY_MAX_INTERVAL_MS, defaults.max_interval_ms)?,
            max_keyframes: unsigned(values, KEY_MAX_KEYFRAMES, defaults.max_keyframes)?,
        };
        let mains_hz = mains_hz(values)?;
        // The configured exposure goes through the same flicker/cap rule as
        // an operator request.
        let exposure = crate::quality::flicker::settle_exposure(
            &ExposureView::default(),
            typed(values, KEY_EXPOSURE, default_exposure())?,
            mains_hz,
        );
        let memory_budget_mb = optional_unsigned(values, KEY_MEMORY_BUDGET_MB)?;
        let reconstruct_steps =
            Some(unsigned(values, KEY_RECONSTRUCT_STEPS, 0)?).filter(|n| *n > 0);
        Ok(Self {
            enabled: typed(values, KEY_ENABLED, false)?,
            device_id: device_id.to_string(),
            capture: CaptureConfig {
                cameras: typed::<Vec<CameraConfig>>(values, KEY_CAMERAS, Vec::new())?,
                profile: typed(values, KEY_CAPTURE_PROFILE, CaptureProfile::default())?,
                selection,
            },
            intrinsics: intrinsics(values)?,
            pose_prior: typed(values, KEY_POSE_PRIOR, PosePrior::default())?,
            record_on_start: typed(values, KEY_RECORD_ON_START, false)?,
            image_encoding: typed(values, KEY_IMAGE_ENCODING, ImageEncoding::Raw)?,
            mains_hz,
            exposure,
            segment: typed(values, KEY_SEGMENT, SegmentSettings::default())?,
            memory_budget_mb,
            reconstruct_steps,
            pose_source: typed(values, KEY_POSE_SOURCE, PoseSourceChoice::Auto)?,
        })
    }

    /// Whether a run started with `self` can keep running under `desired`:
    /// everything but `reconstruct_steps` (which only rides the status and
    /// never changes how the service captures) is equal. A capture in flight is
    /// not torn down because the operator picked a different detail level.
    pub fn same_run(&self, desired: &Self) -> bool {
        let mut running = self.clone();
        running.reconstruct_steps = desired.reconstruct_steps;
        running == *desired
    }

    /// The camera model for a camera: the configured override if present, else
    /// an uncalibrated pinhole derived from the frame size and the field of
    /// view its frames carry (`hfov_deg`, from the agent's camera roster), or
    /// [`DEFAULT_HFOV_DEG`] when the roster gives none.
    pub fn intrinsics_for(
        &self,
        camera_id: &str,
        width: u32,
        height: u32,
        hfov_deg: Option<f64>,
    ) -> CameraModel {
        match self.intrinsics.get(camera_id) {
            Some(o) => o.to_camera_model(width, height),
            None => default_intrinsics(width, height, hfov_deg.unwrap_or(DEFAULT_HFOV_DEG)),
        }
    }
}

#[cfg(test)]
mod tests;
