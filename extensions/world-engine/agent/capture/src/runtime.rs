//! The capture service's runtime configuration: the `atlas.*` plugin config
//! keys, the pose-source tier selector, and the camera-intrinsics resolution.
//!
//! The capture *core* ([`crate::CaptureConfig`]) declares the rig and the
//! selection thresholds. This runtime layer wraps it with the service-only
//! settings: the enable gate, the pose tier, the field of view used to derive a
//! default pinhole when a camera is uncalibrated, any per-camera intrinsics
//! override, and the stated pose prior.
//!
//! The plugin host stores the config as flat dotted keys (`atlas.enabled`,
//! `atlas.selection.max_interval_ms`, ...). [`AtlasRuntimeConfig::from_values`]
//! resolves a map of those keys (absent keys take today's defaults) so the
//! resolution is pure and testable without a host.

use std::collections::{BTreeMap, HashMap};

use serde::de::DeserializeOwned;
use serde::Deserialize;
use serde_json::Value;
use world_engine_protocol::atlas::{CameraIntrinsics, Distortion};

use crate::config::{CameraConfig, CaptureConfig, CaptureProfile, SelectionParams};

fn default_hfov_deg() -> f64 {
    70.0
}

/// The configured pose-source preference. `Auto` lets the service pick (see
/// [`select_pose_tier`]); the explicit variants pin the choice.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "lowercase")]
pub enum PoseTierConfig {
    #[default]
    Auto,
    Local,
    Offload,
    Hybrid,
}

/// The resolved pose-source tier the daemon runs with.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PoseTier {
    /// On-board pose from the flight controller's fused state (the vehicle-state
    /// snapshot the plugin host delivers).
    Local,
    /// Pose returned by a compute node running SLAM on streamed frames.
    Offload,
    /// Local as the primary control-rate pose, corrected by the offloaded pose
    /// when one is fresher.
    Hybrid,
}

/// Resolve the configured preference into a concrete tier.
///
/// The flight controller's fused pose (the host's vehicle-state snapshot) is always
/// available, so `Local` is the floor. `Auto` prefers offloaded SLAM only when a
/// compute node is paired AND this board lacks a local accelerator to run good
/// perception itself; otherwise it stays local.
pub fn select_pose_tier(cfg: PoseTierConfig, npu_present: bool, compute_paired: bool) -> PoseTier {
    match cfg {
        PoseTierConfig::Local => PoseTier::Local,
        PoseTierConfig::Offload => PoseTier::Offload,
        PoseTierConfig::Hybrid => PoseTier::Hybrid,
        PoseTierConfig::Auto => {
            if compute_paired && !npu_present {
                PoseTier::Offload
            } else {
                PoseTier::Local
            }
        }
    }
}

/// A per-camera intrinsics override. When a camera has been calibrated the
/// operator supplies these so reconstruction is metric; absent, the service
/// derives an uncalibrated pinhole from the frame size and the field of view.
#[derive(Debug, Clone, Deserialize, PartialEq)]
pub struct IntrinsicsOverride {
    pub fx: f64,
    pub fy: f64,
    pub cx: f64,
    pub cy: f64,
    #[serde(default)]
    pub distortion_model: Option<String>,
    #[serde(default)]
    pub distortion_params: Vec<f64>,
}

impl IntrinsicsOverride {
    fn to_intrinsics(&self) -> CameraIntrinsics {
        CameraIntrinsics {
            k: [self.fx, 0.0, self.cx, 0.0, self.fy, self.cy, 0.0, 0.0, 1.0],
            distortion: Distortion {
                model: self
                    .distortion_model
                    .clone()
                    .unwrap_or_else(|| "radtan".to_string()),
                params: if self.distortion_params.is_empty() {
                    vec![0.0, 0.0, 0.0, 0.0]
                } else {
                    self.distortion_params.clone()
                },
            },
            // An operator-supplied override IS the calibration.
            calibrated: true,
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
pub fn default_intrinsics(width: u32, height: u32, hfov_deg: f64) -> CameraIntrinsics {
    let w = width.max(1) as f64;
    let h = height.max(1) as f64;
    let hfov = hfov_deg.clamp(1.0, 179.0).to_radians();
    let fx = (w / 2.0) / (hfov / 2.0).tan();
    CameraIntrinsics {
        k: [fx, 0.0, w / 2.0, 0.0, fx, h / 2.0, 0.0, 0.0, 1.0],
        distortion: Distortion {
            model: "radtan".to_string(),
            params: vec![0.0, 0.0, 0.0, 0.0],
        },
        calibrated: false,
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
#[serde(default)]
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
    /// Position sigma in metres for an offloaded SLAM pose.
    pub slam_position_sigma_m: f64,
    /// Orientation sigma in radians for an offloaded SLAM pose.
    pub slam_orientation_sigma_rad: f64,
    /// One-sigma uncertainty of this node's `CLOCK_REALTIME`, in nanoseconds,
    /// or negative for UNMEASURED (the default and the honest value on a node
    /// with no PPS-disciplined clock). A rig running chrony against a GPS PPS
    /// refclock states its real figure here, which is what lets the consumer
    /// apply the strict keyframe budget.
    pub clock_offset_sigma_ns: i64,
}

impl Default for PosePrior {
    fn default() -> Self {
        Self {
            position_sigma_m: None,
            gnss_uere_m: 2.5,
            orientation_sigma_rad: 0.02,
            slam_position_sigma_m: 0.25,
            slam_orientation_sigma_rad: 0.01,
            clock_offset_sigma_ns: -1,
        }
    }
}

impl PosePrior {
    /// The row-major 6x6 covariance for a flight-controller pose with the given
    /// reported `dop` (unitless, `None` when the FC reported none).
    ///
    /// Returns EMPTY without a 3D fix: with no fix the position is the local
    /// origin rather than a measurement, so there is no position to state a
    /// sigma for, and inventing one would hand the reconstructor a prior on a
    /// value that is not a position at all.
    pub fn covariance(&self, dop: Option<f64>, has_fix: bool) -> Vec<f64> {
        if !has_fix {
            return Vec::new();
        }
        let sigma = self
            .position_sigma_m
            .unwrap_or_else(|| dop.unwrap_or(ASSUMED_DOP) * self.gnss_uere_m)
            .max(f64::MIN_POSITIVE);
        crate::pose_source::diagonal_cov(sigma, self.orientation_sigma_rad)
    }

    /// The row-major 6x6 covariance for an offloaded SLAM pose.
    pub fn slam_covariance(&self) -> Vec<f64> {
        crate::pose_source::diagonal_cov(
            self.slam_position_sigma_m,
            self.slam_orientation_sigma_rad,
        )
    }
}

/// The default keyframe-to-pose age budget (ms). A pose older than this cannot
/// pose a frame: at 15 m/s a 500 ms-old position is 7.5 m away, which is larger
/// than the keyframe selector's own baseline threshold, so the frame would be
/// entered into the reconstruction at a place the aircraft was not.
fn default_pose_max_age_ms() -> i64 {
    500
}

/// The default interval (ms) between published pose descriptors — the ~10 Hz
/// the contract documents. Publishing per accepted frame per camera instead
/// multiplied pose traffic by the camera count and raised eviction pressure on
/// the same 16-deep bus that carries multi-MB keyframes.
fn default_pose_publish_interval_ms() -> i64 {
    100
}

/// Every plugin config key the capture service reads. Each is read with a nil
/// default, and an absent (nil) key resolves to the default below.
pub const CONFIG_KEYS: &[&str] = &[
    KEY_ENABLED,
    KEY_CAPTURE_PROFILE,
    KEY_POSE_TIER,
    KEY_HFOV_DEG,
    KEY_CAMERAS,
    KEY_MIN_TRANSLATION_M,
    KEY_MIN_ROTATION_RAD,
    KEY_MAX_INTERVAL_MS,
    KEY_MAX_KEYFRAMES,
    KEY_INTRINSICS,
    KEY_POSE_PRIOR,
    KEY_POSE_MAX_AGE_MS,
    KEY_POSE_PUBLISH_INTERVAL_MS,
];

pub const KEY_ENABLED: &str = "atlas.enabled";
pub const KEY_CAPTURE_PROFILE: &str = "atlas.capture_profile";
pub const KEY_POSE_TIER: &str = "atlas.pose_tier";
pub const KEY_HFOV_DEG: &str = "atlas.hfov_deg";
pub const KEY_CAMERAS: &str = "atlas.cameras";
pub const KEY_MIN_TRANSLATION_M: &str = "atlas.selection.min_translation_m";
pub const KEY_MIN_ROTATION_RAD: &str = "atlas.selection.min_rotation_rad";
pub const KEY_MAX_INTERVAL_MS: &str = "atlas.selection.max_interval_ms";
pub const KEY_MAX_KEYFRAMES: &str = "atlas.selection.max_keyframes";
pub const KEY_INTRINSICS: &str = "atlas.intrinsics";
pub const KEY_POSE_PRIOR: &str = "atlas.pose_prior";
pub const KEY_POSE_MAX_AGE_MS: &str = "atlas.pose_max_age_ms";
pub const KEY_POSE_PUBLISH_INTERVAL_MS: &str = "atlas.pose_publish_interval_ms";

/// The capture service's full runtime configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct AtlasRuntimeConfig {
    pub enabled: bool,
    /// The drone's device id (the plugin host's agent id), used to mint a
    /// globally-unique capture `session_id` so two drones on one shared compute
    /// node never collide (empty when absent — the session id then falls back
    /// to a nonce).
    pub device_id: String,
    pub capture: CaptureConfig,
    pub pose_tier: PoseTierConfig,
    pub hfov_deg: f64,
    pub intrinsics: HashMap<String, IntrinsicsOverride>,
    /// The uncertainty this rig states on every pose (see [`PosePrior`]).
    pub pose_prior: PosePrior,
    /// Age budget for the pose a frame is tagged with, in milliseconds.
    pub pose_max_age_ms: i64,
    /// Minimum interval between published pose descriptors, in milliseconds.
    pub pose_publish_interval_ms: i64,
}

impl Default for AtlasRuntimeConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            device_id: String::new(),
            capture: CaptureConfig::default(),
            pose_tier: PoseTierConfig::Auto,
            hfov_deg: default_hfov_deg(),
            intrinsics: HashMap::new(),
            pose_prior: PosePrior::default(),
            pose_max_age_ms: default_pose_max_age_ms(),
            pose_publish_interval_ms: default_pose_publish_interval_ms(),
        }
    }
}

/// A config key whose value does not parse. Carries the key and the reason so
/// the operator sees exactly why capture stays off.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("invalid `{key}`: {reason}")]
pub struct ConfigError {
    pub key: &'static str,
    pub reason: String,
}

/// The value at `key`, or `None` when the key is absent or null.
fn present<'a>(values: &'a BTreeMap<String, Value>, key: &str) -> Option<&'a Value> {
    values.get(key).filter(|v| !v.is_null())
}

/// Deserialize the value at `key` into `T`, or `default` when absent.
fn typed<T: DeserializeOwned>(
    values: &BTreeMap<String, Value>,
    key: &'static str,
    default: T,
) -> Result<T, ConfigError> {
    match present(values, key) {
        None => Ok(default),
        Some(v) => serde_json::from_value(v.clone()).map_err(|e| ConfigError {
            key,
            reason: e.to_string(),
        }),
    }
}

/// An integer at `key`, accepting an integral float (a JSON editor may store
/// `2000` as `2000.0`), or `default` when absent.
fn integer(
    values: &BTreeMap<String, Value>,
    key: &'static str,
    default: i64,
) -> Result<i64, ConfigError> {
    let Some(v) = present(values, key) else {
        return Ok(default);
    };
    v.as_i64()
        .or_else(|| {
            v.as_f64()
                .filter(|f| f.fract() == 0.0 && f.abs() < i64::MAX as f64)
                .map(|f| f as i64)
        })
        .ok_or_else(|| ConfigError {
            key,
            reason: format!("expected an integer, got {v}"),
        })
}

/// A non-negative integer at `key`, or `default` when absent.
fn unsigned(
    values: &BTreeMap<String, Value>,
    key: &'static str,
    default: u64,
) -> Result<u64, ConfigError> {
    let n = integer(values, key, default as i64)?;
    u64::try_from(n).map_err(|_| ConfigError {
        key,
        reason: format!("expected a non-negative integer, got {n}"),
    })
}

impl AtlasRuntimeConfig {
    /// Resolve the config from the plugin's `atlas.*` keys (see
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
        Ok(Self {
            enabled: typed(values, KEY_ENABLED, false)?,
            device_id: device_id.to_string(),
            capture: CaptureConfig {
                cameras: typed::<Vec<CameraConfig>>(values, KEY_CAMERAS, Vec::new())?,
                profile: typed(values, KEY_CAPTURE_PROFILE, CaptureProfile::default())?,
                selection,
            },
            pose_tier: typed(values, KEY_POSE_TIER, PoseTierConfig::Auto)?,
            hfov_deg: typed(values, KEY_HFOV_DEG, default_hfov_deg())?,
            intrinsics: typed(values, KEY_INTRINSICS, HashMap::new())?,
            pose_prior: typed(values, KEY_POSE_PRIOR, PosePrior::default())?,
            pose_max_age_ms: integer(values, KEY_POSE_MAX_AGE_MS, default_pose_max_age_ms())?,
            pose_publish_interval_ms: integer(
                values,
                KEY_POSE_PUBLISH_INTERVAL_MS,
                default_pose_publish_interval_ms(),
            )?,
        })
    }

    /// Intrinsics for a camera: the configured override if present, else an
    /// uncalibrated pinhole derived from the frame size and the field of view.
    pub fn intrinsics_for(&self, camera_id: &str, width: u32, height: u32) -> CameraIntrinsics {
        match self.intrinsics.get(camera_id) {
            Some(o) => o.to_intrinsics(),
            None => default_intrinsics(width, height, self.hfov_deg),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;
    use world_engine_protocol::atlas::CameraRole;

    fn values(pairs: &[(&str, Value)]) -> BTreeMap<String, Value> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), v.clone()))
            .collect()
    }

    #[test]
    fn no_keys_is_the_disabled_default() {
        let c = AtlasRuntimeConfig::from_values("drone-42", &BTreeMap::new()).unwrap();
        assert!(!c.enabled);
        assert_eq!(c.device_id, "drone-42");
        assert_eq!(c.pose_tier, PoseTierConfig::Auto);
        assert_eq!(c.capture.profile, CaptureProfile::Freeform);
        assert!(c.capture.cameras.is_empty());
        assert!((c.hfov_deg - 70.0).abs() < 1e-12);
        assert_eq!(c.capture.selection, SelectionParams::default());
        assert_eq!(c.pose_max_age_ms, 500);
        assert_eq!(c.pose_publish_interval_ms, 100);
        // A null value (the host's "unset") is the same as an absent key.
        let nulls = values(&[(KEY_ENABLED, Value::Null), (KEY_CAMERAS, Value::Null)]);
        assert_eq!(
            AtlasRuntimeConfig::from_values("drone-42", &nulls).unwrap(),
            c
        );
    }

    #[test]
    fn full_key_set_loads() {
        let v = values(&[
            (KEY_ENABLED, json!(true)),
            (KEY_CAPTURE_PROFILE, json!("orbit")),
            (KEY_POSE_TIER, json!("hybrid")),
            (KEY_HFOV_DEG, json!(90)),
            (
                KEY_CAMERAS,
                json!([
                    {"id": "front", "role": "primary", "enabled": true, "reconstruct": true},
                    {"id": "down", "role": "down", "enabled": false, "reconstruct": false},
                ]),
            ),
            (KEY_MIN_TRANSLATION_M, json!(1.0)),
            (KEY_MIN_ROTATION_RAD, json!(0.3)),
            // An integral float for an integer key is accepted.
            (KEY_MAX_INTERVAL_MS, json!(1500.0)),
            (KEY_MAX_KEYFRAMES, json!(400)),
            (
                KEY_INTRINSICS,
                json!({"front": {"fx": 900.0, "fy": 900.0, "cx": 640.0, "cy": 360.0}}),
            ),
            (KEY_POSE_PRIOR, json!({"position_sigma_m": 0.05})),
        ]);
        let c = AtlasRuntimeConfig::from_values("", &v).unwrap();
        assert!(c.enabled);
        assert_eq!(c.pose_tier, PoseTierConfig::Hybrid);
        assert_eq!(c.capture.profile, CaptureProfile::Orbit);
        assert_eq!(c.capture.enabled_camera_count(), 1);
        assert!((c.hfov_deg - 90.0).abs() < 1e-9);
        assert!((c.capture.selection.min_translation_m - 1.0).abs() < 1e-9);
        assert!((c.capture.selection.min_rotation_rad - 0.3).abs() < 1e-9);
        assert_eq!(c.capture.selection.max_interval_ms, 1500);
        assert_eq!(c.capture.selection.max_keyframes, 400);
        // A partial prior keeps the other figures at their defaults.
        assert_eq!(c.pose_prior.position_sigma_m, Some(0.05));
        assert!((c.pose_prior.gnss_uere_m - 2.5).abs() < 1e-12);
        // The configured intrinsics override is used for `front`.
        let k = c.intrinsics_for("front", 1280, 720);
        assert!((k.k[0] - 900.0).abs() < 1e-9);
        assert!((k.k[2] - 640.0).abs() < 1e-9);
        assert!(k.calibrated);
        // An uncalibrated camera falls back to the derived pinhole.
        assert!(!c.intrinsics_for("down", 1280, 720).calibrated);
    }

    #[test]
    fn a_minimal_camera_entry_fills_its_defaults() {
        let v = values(&[(KEY_CAMERAS, json!([{"id": "front"}]))]);
        let c = AtlasRuntimeConfig::from_values("", &v).unwrap();
        assert_eq!(c.capture.cameras[0].role, CameraRole::Primary);
        assert!(c.capture.cameras[0].enabled);
        assert!(c.capture.cameras[0].reconstruct);
    }

    #[test]
    fn an_unparseable_key_is_an_error_naming_it() {
        // An unknown enum is loud, never a silent default-to-disabled.
        let bad_role = values(&[
            (KEY_ENABLED, json!(true)),
            (KEY_CAMERAS, json!([{"id": "front", "role": "sideways"}])),
        ]);
        assert_eq!(
            AtlasRuntimeConfig::from_values("", &bad_role)
                .unwrap_err()
                .key,
            KEY_CAMERAS
        );
        let bad_tier = values(&[(KEY_POSE_TIER, json!("telepathic"))]);
        assert_eq!(
            AtlasRuntimeConfig::from_values("", &bad_tier)
                .unwrap_err()
                .key,
            KEY_POSE_TIER
        );
        // A fractional or negative count is not silently truncated.
        let frac = values(&[(KEY_MAX_INTERVAL_MS, json!(1500.5))]);
        assert_eq!(
            AtlasRuntimeConfig::from_values("", &frac).unwrap_err().key,
            KEY_MAX_INTERVAL_MS
        );
        let neg = values(&[(KEY_MAX_KEYFRAMES, json!(-1))]);
        assert_eq!(
            AtlasRuntimeConfig::from_values("", &neg).unwrap_err().key,
            KEY_MAX_KEYFRAMES
        );
    }

    #[test]
    fn pose_tier_auto_prefers_offload_only_when_npu_less_and_paired() {
        // The always-available floor is Local.
        assert_eq!(
            select_pose_tier(PoseTierConfig::Auto, false, false),
            PoseTier::Local
        );
        // A paired node + no local accelerator → offload.
        assert_eq!(
            select_pose_tier(PoseTierConfig::Auto, false, true),
            PoseTier::Offload
        );
        // A local accelerator keeps it local even when a node is paired.
        assert_eq!(
            select_pose_tier(PoseTierConfig::Auto, true, true),
            PoseTier::Local
        );
        // Explicit config always wins.
        assert_eq!(
            select_pose_tier(PoseTierConfig::Local, false, true),
            PoseTier::Local
        );
        assert_eq!(
            select_pose_tier(PoseTierConfig::Offload, true, false),
            PoseTier::Offload
        );
    }

    #[test]
    fn derived_intrinsics_centre_the_principal_point() {
        let k = default_intrinsics(1280, 720, 70.0);
        assert!((k.k[2] - 640.0).abs() < 1e-9, "cx at centre");
        assert!((k.k[5] - 360.0).abs() < 1e-9, "cy at centre");
        assert!(k.k[0] > 0.0, "positive focal length");
        // fx == fy (square pixels) and bottom row is [0,0,1].
        assert!((k.k[0] - k.k[4]).abs() < 1e-9);
        assert_eq!(k.k[8], 1.0);
    }
}
