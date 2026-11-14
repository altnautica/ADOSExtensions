//! Planar LiDAR scan matching and mapping.
//!
//! A 2D LiDAR (one level scan ring, such as an RPLIDAR) and an IMU give a
//! planar trajectory and an occupancy map of a single floor:
//!
//! - **Live**, [`LocalSlam`]: feed IMU samples ([`LocalSlam::feed_imu`]) and
//!   scans ([`LocalSlam::feed_scan`]) in time order, the IMU up to the scan's
//!   last point where it is available; each scan returns a [`PoseOut`] in the
//!   local frame (the LiDAR frame at the first scan, levelled). Scans are
//!   levelled with the IMU's roll and pitch, deskewed with the IMU's turn
//!   over the revolution and the recent velocity, matched by correlative
//!   search plus Gauss-Newton refinement (pulled towards the motion
//!   prediction by `translation_weight` / `rotation_weight`; after a gap in
//!   the IMU the heading follows the scans instead once three in a row
//!   clearly disagree with the gyro's) into the older of two overlapping
//!   probability-grid submaps, and inserted into both when the motion filter
//!   passes (a scan held to the gyro against such a disagreement is not).
//!   Along a featureless corridor the pose keeps its motion prior in the
//!   unconstrained direction, the prior's velocity there fading to rest, and
//!   [`PoseOut::degeneracy`] drops below [`Scan2dConfig::degenerate_ratio`].
//!   A measured ground velocity ([`LocalSlam::feed_velocity`]: a downward
//!   optical-flow sensor scaled by its range, levelled with the IMU's roll
//!   and pitch) replaces the recent poses' velocity in the prior while it is
//!   fresh, so along such a direction the pose follows the measured motion.
//!   Isolated posts and uprights count as constraints in both directions,
//!   walls only across themselves. A planar scan cuts the scene at its own
//!   height, so with the height fed ([`LocalSlam::feed_height`]) a scan
//!   taken while the height
//!   changes faster than [`Scan2dConfig::max_climb_rate_mps`], and faster
//!   than the rig moves sideways, rides the motion prior, unmatched and
//!   unmapped, for the first [`Scan2dConfig::max_unmatched_climb_m`] of the
//!   climb (a take-off), and without a measured ground velocity for at most
//!   [`Scan2dConfig::max_unmatched_climb_s`]; past either (a shaft, a
//!   descent drifting in the wind) it is matched. A submap only
//!   takes scans from within [`Scan2dConfig::submap_height_band_m`] of the
//!   height it started at, except along such a transit.
//!   [`LocalSlam::inserted_scans`] counts the scans in the map so far.
//! - **Offline**, [`offline::run`]: the same local SLAM over a whole session,
//!   then branch-and-bound loop search against finished submaps and a
//!   Levenberg-Marquardt pose graph over the submap poses (first fixed). It
//!   returns a [`Trajectory`]: one optimised [`PoseOut`] per matched scan, the
//!   submap count, accepted ([`LoopClosure`]) and rejected loop counts, and
//!   the map points. [`offline::local`] and [`backend::optimize`] are its two
//!   halves, for callers that adjust the submap poses in between.
//!
//! A loop match is accepted only when it scores at least `loop_min_score`,
//! its optimum lies inside the search window, its translation is
//! constrained in both directions, and it moves the scan from the graph's
//! prediction by no more than the drift the walked distance allows (at most
//! `loop_max_correction_m` / `loop_max_correction_deg`). Loop and odometry
//! edges weigh each direction by how well the scans constrain it, so a
//! corridor's long axis gives way first.
//!
//! Inputs are plain values: [`ScanInput`] (first-point time, LiDAR-frame
//! points, per-point time offsets), [`ImuSample`] (time, gyro rad/s, accel
//! m/s², IMU frame, FLU) and [`VelocitySample`] (time, velocity along the
//! IMU frame's x and y, m/s). [`Scan2dConfig::for_sensor`] builds the
//! defaults from the LiDAR model's range limits and occluded sectors;
//! `r_lidar_imu` aligns an IMU mounted at another orientation.
//!
//! The methods are the published ones: correlative scan matching (Olson
//! 2009), Gauss-Newton matching on an interpolated grid (Kohlbrecher et al.
//! 2011), branch-and-bound loop closure on submaps (Hess et al. 2016), the
//! Mahony attitude filter (Mahony et al. 2008) and degeneracy-aware updates
//! (Zhang et al. 2016).

pub mod attitude;
pub mod backend;
pub mod config;
pub mod frontend;
mod graph;
pub mod grid;
pub mod matcher;
pub mod offline;
pub mod pose;
pub mod scan;
pub mod submap;

pub use attitude::{ImuSample, VelocitySample};
pub use backend::{LoopClosure, Trajectory};
pub use config::Scan2dConfig;
pub use frontend::{HeightSample, LocalMap, LocalSlam, PoseOut};
pub use pose::Pose2;
pub use scan::{Scan2d, ScanInput};
