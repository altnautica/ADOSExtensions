//! The pose and sensor inputs the mapping extensions share.
//!
//! Capture records with the flight controller's EKF pose outdoors
//! ([`fc_ekf::FcEkfPose`]) or with navigation's pose track; navigation runs its
//! own scan matcher and reads the same flight-controller feeds. Both need the
//! same pieces, which live here rather than in either extension:
//!
//! - [`pose`]: the [`pose::PoseProvider`] interface, the [`pose::PoseTrack`]
//!   buffer and the [`pose::PoseEstimate`] it holds;
//! - [`fc_ekf`], [`geodesy`], [`geo`]: the FC EKF pose placed in a session
//!   ENU frame with exact WGS-84 geodesy and the EGM2008 geoid;
//! - [`mavlink_msg`], [`fc_flow`], [`align`]: the MAVLink messages read, the
//!   FC optical-flow feed and the map-to-world alignment;
//! - [`se3`]: rotations, quaternions and the yaw helpers;
//! - [`frame_source`], [`lidar_scan`], [`sensor_bus`], [`vehicle`], [`clock`]:
//!   the frame, scan, sensor-bus, vehicle-state and host-clock inputs;
//! - [`replay`]: a fixed estimate list, for tests and SITL.

pub mod align;
pub mod clock;
pub mod fc_ekf;
pub mod fc_flow;
pub mod frame_source;
pub mod geo;
pub mod geodesy;
pub mod lidar_scan;
pub mod mavlink_msg;
pub mod pose;
pub mod replay;
pub mod se3;
pub mod sensor_bus;
pub mod vehicle;

pub use pose::{
    diagonal_cov, health_str, interpolate, source_str, PoseEstimate, PoseProvider, PoseQuality,
    PoseTrack, MAX_EXTRAPOLATION_NS, TRACK_HISTORY_NS,
};
