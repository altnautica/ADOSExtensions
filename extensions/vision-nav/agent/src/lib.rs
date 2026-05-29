//! Vision Navigation agent half (Rust).
//!
//! GPS-denied navigation: optical flow from a downward camera, or
//! monocular visual-inertial odometry from a forward/downward camera,
//! feeding a pose estimate into the autopilot EKF over MAVLink so
//! position-hold, loiter, and missions keep working when GPS is
//! unreliable.
//!
//! The plugin no longer opens a camera. It subscribes to the shared
//! vision frame bus (`ctx.vision.subscribe_frames`); the engine owns
//! frame capture. For VIO modes the shared frames are bridged into the
//! vendored C++ estimator's shared-memory ring.
//!
//! Module map:
//!
//! - [`config`] — per-drone config model + validation (mirrors
//!   `config-schema.json`).
//! - [`framing`] — luma extraction from the shared bus frame formats.
//! - [`flow`] — Lucas-Kanade pyramidal optical-flow tracker.
//! - [`estimator`] — the [`estimator::Estimator`] contract +
//!   [`estimator::EstimatorOutput`] shape shared across modes.
//! - [`estimators`] — the six modes (off, optical_flow,
//!   optical_flow_degraded, vio_openvins, vio_vins_fusion,
//!   hybrid_of_plus_vio) + the registry.
//! - [`scale`] — the rangefinder-free altitude ladder.
//! - [`rangefinder`] — UART/I2C rangefinder drivers + the FC relay.
//! - [`imu`] — IMU source (MAVLink RAW_IMU) + frame/IMU time aligner.
//! - [`clock_align`] — TIMESYNC clock alignment to FC time.
//! - [`mavlink_emit`] — OPTICAL_FLOW_RAD / DISTANCE_SENSOR /
//!   VISION_POSITION_ESTIMATE / HEARTBEAT builders + the component
//!   router.
//! - [`pre_arm`] — the mode-aware pre-arm gate.
//! - [`health`] — the `navigation` heartbeat snapshot.
//! - [`vio`] — the VIO vendor-binary bridge (UDS msgpack control
//!   channel + SHM frame ring + heartbeat watchdog).
//! - [`pipeline`] — the per-frame runtime loop that ties it together.

pub mod clock_align;
pub mod config;
pub mod estimator;
pub mod estimators;
pub mod flow;
pub mod framing;
pub mod health;
pub mod imu;
pub mod mavlink_emit;
pub mod pipeline;
pub mod pre_arm;
pub mod rangefinder;
pub mod scale;
pub mod vio;
