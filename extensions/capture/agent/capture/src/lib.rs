//! Capture: the on-device half of reconstruction.
//!
//! The drone (or a Visor) selects pose-tagged keyframes from its camera
//! stream(s), records sessions, and emits the keyframe + pose contract a
//! separate compute node reconstructs a 3D model from (gaussian splat /
//! point cloud / mesh / occupancy). It does no reconstruction and no
//! training: that is the compute node's job.
//!
//! Three pieces:
//!
//! - [`CaptureConfig`] declares the rig: one camera up to an all-sides set, the
//!   flight [`CaptureProfile`], and the keyframe [`SelectionParams`]. Camera
//!   count is configurable from 1 to N and drives ONE flow at any count.
//! - [`KeyframeSelector`] is the per-camera gate that decides, from pose deltas
//!   and elapsed time, when a frame is worth a keyframe.
//! - [`CaptureSession`] is the state machine: it takes pose-tagged
//!   [`FrameInput`]s, always feeds the ~10 Hz pose stream, and produces a
//!   [`KeyframeEnvelope`] whenever a camera's selector fires.
//!
//! The service around them runs as the extension's `capture` plugin process
//! ([`supervisor`]): it reads the `capture.*` plugin config, frames from the
//! vision engine, the vehicle state and Navigation's pose and scans through
//! the plugin host, and publishes keyframes and the capture state on the
//! extension's private bus.
//!
//! The wire types ([`KeyframeEnvelope`], [`PoseDescriptor`], [`CaptureStatus`],
//! …) are the shared `ados_mapping_protocol::atlas` contract; this crate
//! consumes them, it does not redefine them. They are re-exported here for one
//! import surface.

/// The extension's plugin id, as installed by the plugin host.
pub const PLUGIN_ID: &str = "com.altnautica.capture";
/// The capture service's private bus to the link (one framed event per
/// broadcast: keyframes and the capture state).
pub const BUS_SOCKET: &str = "capture.sock";
/// The capture service's control socket (start / stop / pause / resume /
/// status).
pub const CONTROL_SOCKET: &str = "capture-control.sock";

/// Every pose estimate Navigation produces, batched (`PoseTrackBatch`).
pub const NAV_POSE_TRACK_TOPIC: &str = "plugin.navigation.pose_track";
/// Each LiDAR scan as measured, in parts (`ScanChunk`), while a session
/// records.
pub const NAV_SCAN_RAW_TOPIC: &str = "plugin.navigation.scan_raw";

mod config;
mod selector;
mod session;

pub mod calib;
mod calib_kalibr;
pub mod camera_control;
pub mod control;
pub mod encode;
pub mod hud;
pub mod lidar_record;
pub mod navigation_track;
pub mod publish;
pub mod quality;
pub mod recorder;
pub mod recording;
pub mod rig_source;
pub mod runtime;
pub mod scan_raw;
pub mod service;
pub mod storage;
pub mod supervisor;

pub use config::{CameraConfig, CaptureConfig, CaptureProfile, SelectionParams};
pub use selector::{rotation_angle, KeyframeSelector};
pub use session::{CaptureOutput, CaptureSession, FrameInput};

pub use control::{serve_control, ControlMsg};
pub use encode::encode_keyframe_jpeg;
pub use publish::CapturePublisher;
pub use runtime::{default_intrinsics, CaptureRuntimeConfig, ConfigError, PosePrior, CONFIG_KEYS};
pub use service::{run_capture_loop, LoopDeps, Paths};

// Re-export the shared wire contract so callers get one import surface.
pub use ados_mapping_protocol::atlas::{
    CameraRole, CaptureState, CaptureStatus, ImageEncoding, KeyframeBudget, KeyframeEnvelope,
    KeyframeFlags, KeyframeImage, KeyframeReject, KeyframeTier, Pose, PoseSource, TimeAlignment,
    VioHealth, CAPTURE_STATE_TOPIC,
};

/// Errors from the capture core. The frame-by-frame capture path is
/// infallible by design (a frame either selects a keyframe or it does not);
/// the only fallible surface is validating a capture configuration before a
/// session runs.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum CaptureError {
    /// A capture config has no enabled camera, so it could never produce a
    /// keyframe.
    #[error("capture config has no enabled cameras")]
    NoEnabledCameras,
    /// Two cameras share an id, which would collapse their per-camera selectors
    /// onto one entry.
    #[error("duplicate camera id `{0}` in capture config")]
    DuplicateCameraId(String),
}
