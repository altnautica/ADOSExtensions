//! The pose providers this process runs itself. The interface, the track,
//! the flight controller's EKF pose and the replay provider are shared
//! (`ados_mapping_pose`); these three run a sensor's own odometry:
//!
//! - [`vio::VioPose`]: Basalt stereo-inertial VIO as a supervised subprocess.
//!   Indoor Visor default on a rig without a LiDAR.
//! - [`lidar::LidarPose`]: KISS-ICP LiDAR odometry for a 3D LiDAR as a
//!   supervised subprocess, with the ICP Hessian eigenvalues for corridor
//!   degeneracy.
//! - [`planar::PlanarLidarPose`]: 2D scan matching for a planar LiDAR on a
//!   worker thread; single floor only.

pub mod lidar;
pub mod planar;
pub mod vio;
