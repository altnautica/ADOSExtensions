//! Tuning for the matcher, the submaps and the pose graph.

/// Every tunable of the front end and the back end. Build it with
/// [`Scan2dConfig::for_sensor`] from the LiDAR model's numbers, then override
/// fields as needed.
#[derive(Debug, Clone, PartialEq)]
pub struct Scan2dConfig {
    /// Grid cell edge, metres.
    pub resolution_m: f64,
    /// Occupancy probability a hit pulls a cell towards (one update).
    pub hit_p: f64,
    /// Occupancy probability a pass-through pulls a cell towards.
    pub miss_p: f64,
    /// Cell probability clamp; unknown cells score `p_min` when matching.
    pub p_min: f64,
    pub p_max: f64,

    /// Scans one submap receives before it is finished. Two submaps are
    /// active at a time; the next starts when the newest holds half this.
    pub scans_per_submap: usize,

    /// A matched scan is inserted into the submaps only when it moved this
    /// far, turned this much, or this long passed since the last insertion.
    pub min_translation_m: f64,
    pub min_rotation_deg: f64,
    pub max_interval_s: f64,

    /// Points nearer than this are dropped (the rig and the operator).
    pub min_range_m: f64,
    /// Points farther than this are dropped.
    pub max_range_m: f64,
    /// Occluded sectors `[start, end]`, degrees, sensor frame,
    /// counter-clockwise from +x; `start > end` wraps through 0°.
    pub blocked_azimuth_deg: Vec<[f64; 2]>,

    /// Half-width of the scan-to-submap search around the motion prior.
    pub linear_window_m: f64,
    pub angular_window_deg: f64,
    /// Gauss-Newton refinement on the interpolated grid.
    pub gn_max_iters: usize,
    pub gn_eps: f64,
    /// Pull of the local refinement towards the motion prediction (IMU yaw,
    /// constant velocity), against the mean squared scan residual.
    pub translation_weight: f64,
    pub rotation_weight: f64,

    /// Every n-th inserted scan is matched against older finished submaps.
    pub loop_every_n_nodes: usize,
    /// Only submaps holding a scan taken this close to the scan are searched.
    pub loop_search_radius_m: f64,
    /// Half-width of the loop search around the pose-graph prior.
    pub loop_linear_window_m: f64,
    pub loop_angular_window_deg: f64,
    /// Matches scoring below this are not loop candidates.
    pub loop_min_score: f64,
    /// Candidates that move the scan farther than this from its pose-graph
    /// prior are rejected; submaps passed recently allow less, in proportion
    /// to the path walked since (see [`crate::backend`]).
    pub loop_max_correction_m: f64,
    pub loop_max_correction_deg: f64,
    /// A scan is only matched against submaps at least this many submap
    /// indices older than its own.
    pub loop_min_submap_gap: usize,

    /// Standard deviations of the submap-to-submap odometry edges.
    pub odom_sigma_m: f64,
    pub odom_sigma_deg: f64,
    /// Standard deviations of the loop edges.
    pub loop_sigma_m: f64,
    pub loop_sigma_deg: f64,
    /// Huber threshold on loop edges, metres of translation-equivalent
    /// residual (scaled by `loop_sigma_m` into whitened units).
    pub huber_delta: f64,

    /// Mahony filter gains.
    pub mahony_kp: f64,
    pub mahony_ki: f64,
    /// Rotation taking IMU-frame vectors into the LiDAR frame (row-major),
    /// `R_lidar_imu`. Identity when the IMU and the LiDAR share axes.
    pub r_lidar_imu: [[f64; 3]; 3],

    /// `eig_min / eig_max` of the match Hessian below this is degenerate:
    /// the pose keeps its motion prior along the weak directions.
    pub degenerate_ratio: f64,
}

impl Scan2dConfig {
    /// Defaults for a LiDAR with the given range limits and occluded sectors.
    /// The minimum range is raised to 0.35 m so returns off the rig and the
    /// operator's hands never enter the map.
    pub fn for_sensor(
        min_range_m: f64,
        max_range_m: f64,
        blocked_azimuth_deg: &[[f64; 2]],
    ) -> Self {
        Scan2dConfig {
            resolution_m: 0.05,
            hit_p: 0.55,
            miss_p: 0.49,
            p_min: 0.12,
            p_max: 0.97,
            scans_per_submap: 60,
            min_translation_m: 0.10,
            min_rotation_deg: 2.0,
            max_interval_s: 1.0,
            min_range_m: min_range_m.max(0.35),
            max_range_m,
            blocked_azimuth_deg: blocked_azimuth_deg.to_vec(),
            linear_window_m: 0.30,
            angular_window_deg: 15.0,
            gn_max_iters: 10,
            gn_eps: 1e-4,
            translation_weight: 1.0,
            rotation_weight: 40.0,
            loop_every_n_nodes: 5,
            loop_search_radius_m: 6.0,
            loop_linear_window_m: 2.0,
            loop_angular_window_deg: 20.0,
            loop_min_score: 0.60,
            loop_max_correction_m: 1.0,
            loop_max_correction_deg: 10.0,
            loop_min_submap_gap: 2,
            odom_sigma_m: 0.05,
            odom_sigma_deg: 1.0,
            loop_sigma_m: 0.05,
            loop_sigma_deg: 1.0,
            huber_delta: 0.10,
            mahony_kp: 2.0,
            mahony_ki: 0.005,
            r_lidar_imu: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            degenerate_ratio: 0.05,
        }
    }
}
