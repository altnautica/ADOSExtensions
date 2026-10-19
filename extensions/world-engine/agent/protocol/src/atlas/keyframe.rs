//! The keyframe lane (drone or Visor to compute node) and the capture state.
//!
//! Camera model, geo anchor, timebase and IMU sample are the capture
//! contract's own types (`ados_capture_contract`), so a keyframe states its
//! optics, world origin and timing in exactly the vocabulary a recorded
//! session uses.

use ados_capture_contract::v1::ImuSample;
use ados_capture_contract::{CameraModel, GeoAnchor, Timebase};
use serde::{Deserialize, Serialize};
use thiserror::Error;

// --- Enums ----------------------------------------------------------------

/// Which camera on the rig produced a keyframe. Camera count is configurable
/// from one camera to an all-sides rig; the role tags each stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum CameraRole {
    Primary,
    Aux,
    Down,
    Left,
    Right,
    Back,
    Up,
}

/// Delivery tier of a keyframe. A `Light` descriptor fits an in-flight relay
/// link (a thumbnail or an occupancy slice); a `Full` keyframe carries the
/// full-resolution image and IMU window for a LAN-bulk or post-flight pull.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum KeyframeTier {
    Light,
    Full,
}

/// Where a keyframe's pose came from. Both produce the identical envelope, so
/// nothing downstream forks; only the producer changed.
///
/// `Default` is [`PoseSource::LocalVio`]: the flight controller's fused pose is
/// the floor every board has, so a decoder filling an absent field lands on the
/// tier that is always present rather than claiming an offload that may not
/// exist.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PoseSource {
    /// Computed on-board by the drone's own VIO (a VIO-capable board).
    #[default]
    LocalVio,
    /// Computed on the compute node from a streamed image and returned on
    /// [`super::ATLAS_POSE_OFFLOAD_TOPIC`] (an NPU-less board, first-class).
    OffloadedSlam,
}

/// Image encoding carried in a full keyframe.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ImageEncoding {
    Jpeg,
    /// HEVC keyframe (I-frame) bytes. Serializes as `hevc-keyframe`.
    HevcKeyframe,
}

// --- Structs --------------------------------------------------------------

/// The number of elements in a row-major 6x6 pose covariance.
pub const POSE_COV_LEN: usize = 36;

/// A 6-DoF pose. `r` is a row-major 3x3 rotation (world-from-camera), `t` the
/// translation, `cov` an optional 6x6 covariance (36 row-major elements).
///
/// `cov` stays OPTIONAL here deliberately. This struct is also the ~10 Hz live
/// pose descriptor ([`PoseDescriptor`]), and 36 `f64` is 288 bytes — a fifth of
/// the whole 1300-byte relay datagram budget, per message, for a value no live
/// consumer reads. The consumer that genuinely requires a sigma is the
/// reconstructor, and it reads [`KeyframeEnvelope::pose_cov`], which is
/// required. So the cost is paid on the keyframe lane that needs it and not on
/// the pose lane that does not.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Pose {
    /// Row-major 3x3 rotation (wire key `R`, the math convention).
    #[serde(rename = "R")]
    pub r: [f64; 9],
    pub t: [f64; 3],
    pub cov: Option<Vec<f64>>,
}

/// Image bytes carried in a keyframe. For a `Light` tier this may be a
/// thumbnail; for a `Full` tier it is the full-resolution frame.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct KeyframeImage {
    pub encoding: ImageEncoding,
    pub width: u32,
    pub height: u32,
    pub bytes: Vec<u8>,
}

/// Per-keyframe boolean flags.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct KeyframeFlags {
    pub is_loop_closure: bool,
    pub is_session_start: bool,
    pub is_session_end: bool,
}

/// The default ceiling on [`TimeAlignment::sigma_ns`] for a keyframe a
/// reconstructor will trust under [`KeyframeBudget::strict`]: 5 ms. A frame
/// timed no better than 5 ms is already worse than one frame period at 200 fps
/// and comparable to one at 30 fps, so above this it cannot be placed to
/// sub-frame accuracy and is not reconstruction input.
pub const KEYFRAME_MAX_SIGMA_NS: i64 = 5_000_000;

/// When a keyframe was exposed, on which timebase, and how well that is known,
/// so a consumer can DECIDE whether the frame is trustworthy instead of
/// assuming it.
///
/// `t_exposure_mid_ns` is on `timebase`, the same timebase a recorded session
/// of the capture contract uses (`host_monotonic` for a frame stamped by the
/// capture host, `sim` in simulation, `gnss`/`hub_freerun` once a disciplined
/// sensor clock drives the camera). Accuracy is bounded by the clock, not by the
/// protocol: an undisciplined host clock is honest only as `sigma_ns = -1`
/// (unmeasured), which every strict trust check refuses.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct TimeAlignment {
    /// Mid-exposure instant of the frame on `timebase`, nanoseconds.
    pub t_exposure_mid_ns: u64,
    pub timebase: Timebase,
    /// One-sigma uncertainty of `timebase` against its reference, in
    /// nanoseconds. Negative means UNMEASURED.
    pub sigma_ns: i64,
    /// The camera trigger sequence this frame was joined on, when a trigger
    /// source (flight controller or sensor hub) provides one. Present means the
    /// exposure time is the trigger's own record; absent means it is the
    /// capture host's stamp.
    #[serde(default)]
    pub trigger_seq: Option<u64>,
}

impl Default for TimeAlignment {
    /// Fails CLOSED: an unmeasured alignment, not a perfect one. A derived
    /// `Default` would zero `sigma_ns`, and a zero sigma reads as "this clock is
    /// exact" — the single most dangerous possible default for a field whose
    /// whole purpose is to admit uncertainty.
    fn default() -> Self {
        Self::unmeasured()
    }
}

impl TimeAlignment {
    /// An alignment whose uncertainty is unknown, so every strict trust check
    /// refuses it. The honest value for a producer with no disciplined clock.
    pub fn unmeasured() -> Self {
        Self {
            t_exposure_mid_ns: 0,
            timebase: Timebase::HostMonotonic,
            sigma_ns: -1,
            trigger_seq: None,
        }
    }

    /// Whether the timebase uncertainty is measured and within `max_sigma_ns`.
    pub fn is_trustworthy(&self, max_sigma_ns: i64) -> bool {
        self.sigma_ns >= 0 && self.sigma_ns <= max_sigma_ns
    }
}

/// A pose-tagged keyframe sent to the compute node. Carries the camera
/// identity so multi-camera rigs are unambiguous, and the tier and pose-source
/// so the same struct serves the LAN-bulk and the in-flight-relay paths.
///
/// Two fields exist so a consumer can refuse a frame instead of reconstructing
/// a wrong world from it: [`TimeAlignment`] says how well the frame is timed,
/// and `pose_cov` carries the pose sigma the reconstructor's position prior
/// needs. Check both with [`KeyframeEnvelope::validate`] before a frame enters
/// a dataset.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct KeyframeEnvelope {
    pub session_id: String,
    pub kf_id: u64,
    /// Wall-clock milliseconds the frame was captured, for display only.
    pub ts_unix_ms: i64,
    pub camera_id: String,
    pub camera_role: CameraRole,
    pub tier: KeyframeTier,
    pub image: KeyframeImage,
    /// The camera's projection model in the capture contract's form; its
    /// `calibrated` flag says whether the values were measured or are nominal.
    pub camera: CameraModel,
    pub pose: Pose,
    /// Row-major 6x6 covariance of `pose`, `POSE_COV_LEN` elements. REQUIRED.
    ///
    /// This is the reconstructor's position-prior sigma. COLMAP's
    /// `--prior_position_std_*` defaults to 1.0 m, which is wrong for both
    /// RTK-fixed (0.02-0.05 m) and consumer GNSS (1.5-3 m), and a prior with no
    /// sigma is the documented cause of `pose_prior_mapper` instability — so a
    /// missing covariance does not degrade the reconstruction quietly, it
    /// destabilises the bundle adjustment. Carried on the keyframe lane only,
    /// never on the ~10 Hz pose lane (see [`Pose::cov`]).
    ///
    /// `#[serde(default)]` decodes a frame without it to an EMPTY vec, which
    /// [`KeyframeEnvelope::validate`] rejects rather than inventing an identity
    /// covariance that asserts a precision nobody measured.
    #[serde(default)]
    pub pose_cov: Vec<f64>,
    pub pose_source: PoseSource,
    /// When the frame was exposed. `#[serde(default)]` reads a frame without it
    /// as [`TimeAlignment::unmeasured`].
    #[serde(default)]
    pub time: TimeAlignment,
    /// The session's world origin (the ENU frame `pose` is in), or `None`
    /// before it latched.
    pub anchor: Option<GeoAnchor>,
    /// Body-frame (FLU) IMU samples around the exposure, on `time.timebase`.
    pub imu_window: Vec<ImuSample>,
    pub flags: KeyframeFlags,
}

/// Why a keyframe is not usable as reconstruction input.
#[derive(Debug, Clone, Copy, PartialEq, Error)]
pub enum KeyframeReject {
    /// `pose_cov` was absent or not `POSE_COV_LEN` elements, so the
    /// reconstructor has no position-prior sigma for this frame.
    #[error("pose covariance must be {POSE_COV_LEN} elements, got {got}")]
    PoseCovLen { got: usize },
    /// The timebase uncertainty is unmeasured or over budget, so the frame
    /// cannot be placed in time to sub-frame accuracy.
    #[error("timebase sigma {sigma_ns} ns is unmeasured or over the {budget_ns} ns budget")]
    TimeSigma { sigma_ns: i64, budget_ns: i64 },
    /// The session's geo anchor was not latched when the frame was captured, so
    /// its translation is in a different frame from every post-anchor keyframe.
    #[error("keyframe carries no anchor, so its world frame is unresolved")]
    NoAnchor,
}

/// What a consumer requires of a keyframe before it enters a dataset.
///
/// The time budget is an `Option` on purpose. Measuring the timebase sigma
/// needs a disciplined clock (chrony with a PPS refclock, or a sensor hub), so
/// on a node without one the honest report is `unmeasured`, and a hard default
/// gate would silently reject every frame and produce ZERO reconstructions with
/// no operator-visible cause. The default carries the sigma through for the
/// quality gate to weigh; [`KeyframeBudget::strict`] is the opt-in for a rig
/// that does have a disciplined clock.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct KeyframeBudget {
    /// Ceiling on `time.sigma_ns`, or `None` to carry the sigma without gating
    /// on it.
    pub max_sigma_ns: Option<i64>,
    /// Whether a frame captured before the session's geo anchor latched is
    /// refused. Always `true` in practice: a pre-anchor frame sits at the local
    /// origin while every post-anchor frame sits in ENU metres from the anchor,
    /// so mixing them shows the reconstructor a teleport inside one session.
    pub require_anchor: bool,
}

impl Default for KeyframeBudget {
    fn default() -> Self {
        Self {
            max_sigma_ns: None,
            require_anchor: true,
        }
    }
}

impl KeyframeBudget {
    /// The budget for a rig with a disciplined clock: also gate on the timebase
    /// sigma, so a frame that cannot be placed to sub-frame accuracy never
    /// reaches the reconstructor.
    pub fn strict() -> Self {
        Self {
            max_sigma_ns: Some(KEYFRAME_MAX_SIGMA_NS),
            ..Self::default()
        }
    }
}

impl KeyframeEnvelope {
    /// Whether this frame is usable as reconstruction input under the default
    /// budget ([`KeyframeBudget::default`]).
    ///
    /// Every rejection here is a frame that would otherwise enter a dataset and
    /// bake a wrong pose into the reconstruction with no other symptom, which is
    /// unrecoverable after the fact.
    pub fn validate(&self) -> Result<(), KeyframeReject> {
        self.validate_with(&KeyframeBudget::default())
    }

    /// Whether this frame is usable as reconstruction input under `budget`.
    pub fn validate_with(&self, budget: &KeyframeBudget) -> Result<(), KeyframeReject> {
        if self.pose_cov.len() != POSE_COV_LEN {
            return Err(KeyframeReject::PoseCovLen {
                got: self.pose_cov.len(),
            });
        }
        if let Some(budget_ns) = budget.max_sigma_ns {
            if !self.time.is_trustworthy(budget_ns) {
                return Err(KeyframeReject::TimeSigma {
                    sigma_ns: self.time.sigma_ns,
                    budget_ns,
                });
            }
        }
        if budget.require_anchor && self.anchor.is_none() {
            return Err(KeyframeReject::NoAnchor);
        }
        Ok(())
    }

    /// Whether the timebase behind this frame was actually measured. A dataset
    /// built from frames where this is `false` is metrically fine but its
    /// timing is only as good as an undisciplined clock, which is a fact the
    /// reconstruction quality gate must see.
    pub fn clock_is_disciplined(&self) -> bool {
        self.time.sigma_ns >= 0
    }
}

/// The pose the compute node returns to the drone on
/// [`super::ATLAS_POSE_OFFLOAD_TOPIC`] after running SLAM on a streamed image.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OffloadedPose {
    pub session_id: String,
    pub kf_id: u64,
    pub camera_id: String,
    pub pose: Pose,
    /// Always [`PoseSource::OffloadedSlam`] on this lane; carried for symmetry.
    pub source: PoseSource,
    pub ts_ms: i64,
}

/// Capture-session lifecycle state published on
/// [`super::ATLAS_CAPTURE_STATE_TOPIC`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CaptureState {
    Idle,
    Capturing,
    Paused,
    Finalizing,
    Bagged,
}

/// SLAM / VIO health summary carried with the capture state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum VioHealth {
    Good,
    Degraded,
    Lost,
}

/// The descriptor on [`super::ATLAS_CAPTURE_STATE_TOPIC`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CaptureStatus {
    pub session_id: String,
    pub state: CaptureState,
    pub keyframes: u64,
    pub vio_health: VioHealth,
    /// Count of enabled cameras (1 to N); the fusion layer keys off this.
    pub camera_count: u32,
    pub ingest_rate_hz: f32,
    /// True once the session-wide keyframe cap is reached. Keyframe selection
    /// has stopped while the pose stream and `Capturing` continue, so without
    /// this flag the operator surface shows "capturing" forever against a frozen
    /// count and cannot tell that apart from a stalled camera.
    #[serde(default)]
    pub capped: bool,
    /// True once the session's geo anchor is latched (the first 3D fix). Before
    /// it, a pose translation is `[0, 0, alt_rel]` at the origin while every
    /// post-anchor pose is ENU metres from the anchor, so a session that mixes
    /// the two shows the reconstructor a teleport.
    #[serde(default)]
    pub anchored: bool,
    /// Which producer filled the pose the capture path is currently tagging
    /// with, so a silent switch to offloaded SLAM is visible to the operator
    /// rather than inferred.
    #[serde(default)]
    pub pose_tier: PoseSource,
    /// Keyframes the capture path produced but the bus could not deliver (a
    /// subscriber too slow for the 16-deep per-client queue was pruned). A
    /// non-zero count means the reconstruction is sparser than the keyframe
    /// count claims.
    #[serde(default)]
    pub dropped_keyframes: u64,
}

/// Shared-data descriptor on [`super::PLUGIN_ATLAS_POSE_TOPIC`].
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PoseDescriptor {
    pub pose: Pose,
    pub anchor: Option<GeoAnchor>,
    pub ts_ms: i64,
}
