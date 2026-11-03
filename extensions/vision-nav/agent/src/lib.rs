//! Vision Navigation agent half (Rust).
//!
//! GPS-denied navigation: optical flow from a downward camera, feeding
//! angular flow and the matching range into the autopilot EKF over
//! MAVLink so position-hold, loiter, and missions keep working when GPS
//! is unreliable; or, in `external_nav` mode, a pose from another
//! plugin's shared pose topic sent as `ODOMETRY`, with EKF source-set
//! switching and a planar-scan obstacle feed.
//!
//! The plugin does not open a camera. It subscribes to the shared
//! vision frame bus (`ctx.vision.subscribe_frames`); the engine owns
//! frame capture.
//!
//! Module map:
//!
//! - [`config`] — per-drone config model + validation (mirrors
//!   `config-schema.json`); [`nav_config`] — its `external_nav` and
//!   `obstacle` blocks.
//! - [`live_config`] — the live per-drone keys and the restart-on-change
//!   watcher.
//! - [`framing`] — luma extraction from the shared bus frame formats.
//! - [`flow`] — Lucas-Kanade pyramidal optical-flow tracker.
//! - [`estimator`] — the [`estimator::Estimator`] contract +
//!   [`estimator::EstimatorOutput`] shape shared across modes.
//! - [`estimators`] — the optical-flow modes + the registry.
//! - [`scale`] — the rangefinder-free altitude ladder.
//! - [`rangefinder`] — the TF-Luna UART reader + the FC relay.
//! - [`imu`] — IMU source (MAVLink RAW_IMU) + frame/IMU time aligner.
//! - [`clock_align`] — TIMESYNC clock alignment to FC time.
//! - [`mavlink_emit`] — OPTICAL_FLOW / OPTICAL_FLOW_RAD /
//!   DISTANCE_SENSOR / TIMESYNC / HEARTBEAT / stream-interval builders +
//!   the firmware-aware component router.
//! - [`external_nav`] — pose topic parsing, FLU/FRD conversion and the
//!   `ODOMETRY` builder; [`fc_nav`] — the flight-controller state it
//!   reads; [`source_set`] — EKF source-set switching;
//!   [`nav_commands`] — the commands it sends.
//! - [`obstacle`] — planar scan to `OBSTACLE_DISTANCE`.
//! - [`nav_state`] — the `nav.state` event.
//! - [`pre_arm`] — the mode-aware pre-arm gate ([`pre_arm_extnav`] for
//!   `external_nav`).
//! - [`health`] — the `navigation` heartbeat snapshot.
//! - [`pipeline`] — the runtime that ties it together.

pub mod clock_align;
pub mod config;
pub mod estimator;
pub mod estimators;
pub mod external_nav;
pub mod fc_nav;
pub mod flow;
pub mod framing;
pub mod health;
pub mod imu;
pub mod live_config;
pub mod mavlink_emit;
pub mod nav_commands;
pub mod nav_config;
pub mod nav_state;
pub mod obstacle;
pub mod pipeline;
pub mod pre_arm;
pub mod pre_arm_extnav;
pub mod rangefinder;
pub mod scale;
pub mod source_set;
