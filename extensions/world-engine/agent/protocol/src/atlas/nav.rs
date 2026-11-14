//! The live navigation and mapping payloads the drone publishes while the
//! LiDAR runs, independent of any capture session: the planar scan, the live
//! occupancy tiles, the live sweep-cloud delta and the map status.
//!
//! Frames:
//! - `map_ned` is north-aligned: x north, y east, z down, metres (z from the
//!   flight controller's height above the first scan). The planar SLAM world
//!   (body FLU at the first scan) is turned to FRD and rotated about down so
//!   x points north, origin at the first scan. Its north ([`MapNorth`]) is
//!   latched once from the configured take-off heading, or else taken from
//!   the flight controller's attitude yaw at take-off (followed while the
//!   vehicle sits where the first pose was taken). While the FC navigates on
//!   GNSS it is moved onto the FC's `LOCAL_POSITION_NED` (yaw and offset
//!   fitted between the SLAM track and the EKF's GNSS track, frozen when GNSS
//!   goes). Nothing in it is published before a heading is known.
//! - `body_frd` is the vehicle body, forward-right-down.

use serde::{Deserialize, Serialize};

/// Bins in one [`ScanDescriptor`] revolution (1° each).
pub const SCAN_BINS: usize = 360;
/// Cells along one side of a live occupancy tile.
pub const OCCUPANCY_TILE_CELLS: u32 = 64;
/// Live occupancy cell value: occupied (ROS `map_server` convention).
pub const OCCUPANCY_OCCUPIED: u8 = 0;
/// Live occupancy cell value: free.
pub const OCCUPANCY_FREE: u8 = 254;
/// Live occupancy cell value: unknown.
pub const OCCUPANCY_UNKNOWN: u8 = 205;
/// Most points one live sweep-cloud delta carries.
pub const POINTCLOUD_DELTA_MAX_POINTS: usize = 20_000;

/// The frame a published pose or map is expressed in.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NavFrame {
    /// North-aligned local frame (see the module docs).
    #[default]
    MapNed,
    /// The vehicle body, forward-right-down.
    BodyFrd,
}

/// Where the north of a `map_ned` pose comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MapNorth {
    /// The flight controller's attitude yaw, followed while the vehicle sits
    /// where the first pose was taken and fixed when it leaves. Before take-off
    /// this north echoes the FC's own heading, so the FC must not take its
    /// yaw from this map.
    FcHeading,
    /// Latched once at the first pose from the configured take-off heading
    /// and never taken from the flight controller, so the FC may take its yaw
    /// from the map.
    Declared,
    /// Fitted between the SLAM track and the EKF's GNSS-referenced track.
    Gnss,
}

/// Which estimator produced a published pose.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NavPoseSource {
    /// Planar LiDAR scan matching on the companion.
    LidarPlanar,
    /// The flight controller's EKF.
    #[default]
    FcEkf,
    /// Visual-inertial odometry on the companion.
    Vio,
    /// A recorded pose list played back.
    Replay,
}

/// Whether payloads on a topic that also carries node products are live
/// (built on the drone as it flies) or a finished product.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MapPayloadKind {
    Live,
    /// A reconstruction product from the compute node.
    #[default]
    Product,
}

/// One planar scan in the body frame, binned to [`SCAN_BINS`] 1° bins
/// (`plugin.world-engine.scan`, 10 Hz).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ScanDescriptor {
    /// Host monotonic ns of the scan's first point.
    pub t_ns: u64,
    /// Always [`NavFrame::BodyFrd`].
    pub frame: NavFrame,
    /// Angle of bin 0, radians, clockwise from body +x toward +y.
    pub angle_start_rad: f32,
    pub angle_increment_rad: f32,
    /// Nearest horizontal range per bin, metres; `0.0` = no return.
    pub ranges_m: Vec<f32>,
}

/// One changed live occupancy tile.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct OccupancyTile {
    pub ix: i32,
    pub iy: i32,
    /// 8-bit grey PNG, `tile_cells` square: row 0 is the tile's largest x
    /// (north up), column 0 its smallest y (west left).
    pub png_b64: String,
}

/// The live occupancy delta on `plugin.world-engine.occupancy` (1 Hz, only
/// tiles that changed since the previous one).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LiveOccupancy {
    /// Always [`MapPayloadKind::Live`].
    pub kind: MapPayloadKind,
    /// Increments per published delta.
    pub seq: u64,
    pub frame: NavFrame,
    pub res_m: f32,
    pub tile_cells: u32,
    pub tiles: Vec<OccupancyTile>,
}

/// The live sweep-cloud delta on `plugin.world-engine.pointcloud` (1 Hz, at
/// most [`POINTCLOUD_DELTA_MAX_POINTS`] voxel centroids new since the last).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LivePointCloud {
    /// Always [`MapPayloadKind::Live`].
    pub kind: MapPayloadKind,
    pub seq: u64,
    pub frame: NavFrame,
    /// The voxel edge the centroids were merged at. A change means the cloud
    /// was coarsened and every earlier delta is superseded.
    pub voxel_m: f32,
    pub count: u32,
    /// `count` little-endian `f32` `x, y, z` triples, base64.
    pub points_b64: String,
}

/// Where the heavy map products are built (`atlas.compute_placement`).
/// Navigation SLAM and the operator's live occupancy stay on board in every
/// mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ComputePlacement {
    /// The full-resolution sweep cloud is built on board; the session is saved.
    Onboard,
    /// Keyframes and poses stream to the compute node, which reconstructs
    /// during the flight; the drone keeps only a coarse live cloud.
    Stream,
    /// Record only (plus the coarse live cloud); upload after landing.
    PostFlight,
    /// A medium live cloud on board plus the full session uploaded after
    /// landing for the high-fidelity products.
    #[default]
    Hybrid,
}

impl ComputePlacement {
    /// The live sweep-cloud voxel edge the placement starts at, metres.
    pub fn live_voxel_m(self) -> f32 {
        match self {
            ComputePlacement::Onboard => 0.05,
            ComputePlacement::Hybrid => 0.10,
            ComputePlacement::Stream | ComputePlacement::PostFlight => 0.20,
        }
    }

    /// Whether the finished session uploads after landing.
    pub fn uploads_after_flight(self) -> bool {
        matches!(
            self,
            ComputePlacement::PostFlight | ComputePlacement::Hybrid
        )
    }

    /// Whether the compute node reconstructs live as data arrives.
    pub fn streams_live(self) -> bool {
        self == ComputePlacement::Stream
    }
}

/// When the navigation pose is published (`atlas.nav_pose`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum NavPoseMode {
    /// Whenever the pose provider runs.
    #[default]
    Always,
    /// Only while a capture session is open.
    Capture,
}

/// The post-flight upload's progress.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum UploadPhase {
    #[default]
    Idle,
    Queued,
    Uploading,
    Done,
    Failed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct UploadProgress {
    pub state: UploadPhase,
    pub bytes_done: u64,
    pub bytes_total: u64,
}

/// The in-flight stream to the compute node.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum StreamPhase {
    #[default]
    Off,
    Connecting,
    Live,
    Degraded,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
pub struct StreamProgress {
    pub state: StreamPhase,
    pub bytes_per_s: u64,
}

/// Whether the flight controller's IMU feed measures the heading a planar
/// LiDAR pose rides between scans.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ImuFeedState {
    Ok,
    /// The feed is live but starved: the heading is measured, coarsely. The
    /// pose is degraded.
    Degraded,
    /// No yaw rate reaches the matcher (the attitude stand-in, or a stalled
    /// `RAW_IMU`): the heading cannot follow a yaw at all, and the pose is
    /// lost.
    Lost,
}

/// Why the FC IMU feed is degraded or lost.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ImuFeedReason {
    /// No `RAW_IMU` yet: the FC attitude stand-in, which carries no yaw rate.
    AttitudeStandIn,
    /// The newest `RAW_IMU` sample is older than 250 ms.
    Stalled,
    /// Under 40 `RAW_IMU` samples over the last second.
    Starved,
}

impl ImuFeedReason {
    pub fn as_str(self) -> &'static str {
        match self {
            ImuFeedReason::AttitudeStandIn => "attitude_stand_in",
            ImuFeedReason::Stalled => "stalled",
            ImuFeedReason::Starved => "starved",
        }
    }

    /// The feed's state for this reason: lost when no yaw rate reaches the
    /// matcher, degraded while it is live but starved.
    pub fn state(self) -> ImuFeedState {
        match self {
            ImuFeedReason::AttitudeStandIn | ImuFeedReason::Stalled => ImuFeedState::Lost,
            ImuFeedReason::Starved => ImuFeedState::Degraded,
        }
    }
}

/// [`MapStatus::imu_feed`]: the FC IMU feed of a planar LiDAR pose that
/// takes the flight controller as its IMU (`atlas.lidar.imu = "fc"`). The
/// pose is degraded or lost while the feed is ([`ImuFeedReason::state`]).
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct ImuFeedStatus {
    pub state: ImuFeedState,
    /// Null while `state` is ok.
    pub reason: Option<ImuFeedReason>,
    /// `RAW_IMU` samples over the last second; null before `RAW_IMU`.
    pub rate_hz: Option<f32>,
}

/// The live map's state on `plugin.world-engine.map_status` (1 Hz).
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct MapStatus {
    /// The open capture session, if any.
    pub session_id: Option<String>,
    pub placement: ComputePlacement,
    /// The capture process's resident memory (`VmRSS`), MiB; null when it
    /// could not be read.
    pub memory_used_mb: Option<f32>,
    /// `atlas.memory_budget_mb` as resolved.
    pub memory_budget_mb: f32,
    /// The live sweep-cloud voxel edge, metres (grows when coarsened).
    pub voxel_m: f32,
    /// Keyframes dropped to stay inside the keyframe share of the budget.
    pub keyframes_dropped: u64,
    pub sweep_points: u64,
    /// Area of known (free or occupied) live occupancy cells, m².
    pub area_m2: f32,
    pub nav_pose: NavPoseMode,
    pub upload: UploadProgress,
    pub stream: StreamProgress,
    /// The FC IMU feed; null when the pose does not take the FC as its IMU.
    #[serde(default)]
    pub imu_feed: Option<ImuFeedStatus>,
}

/// Schema version of [`LidarStatus`].
pub const LIDAR_STATUS_VERSION: u16 = 1;

/// A LiDAR's own health report (the RPLIDAR `GET_HEALTH` status).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum LidarHealth {
    Good,
    Warning,
    Error,
}

/// The configured LiDAR's link on `plugin.world-engine.lidar`, 1 Hz while
/// the rig declares the sensor. A driver that is not running (the device
/// absent at start, or not configured) reports `connected: false`, null
/// device, health and rate, and zero counters.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct LidarStatus {
    /// [`LIDAR_STATUS_VERSION`].
    pub v: u16,
    /// The port the device is on (the resolved `/dev/serial/by-id` path
    /// under USB discovery); null when it was never opened.
    pub device: Option<String>,
    pub connected: bool,
    /// The device's health from its latest (re)connect; null when it never
    /// connected or the driver has no health report.
    pub health: Option<LidarHealth>,
    /// The device's error code beside `health`.
    pub health_code: Option<u16>,
    /// Revolutions per second over the last status period; null when not
    /// connected.
    pub rate_hz: Option<f32>,
    /// Scans handed to the capture service since the run started.
    pub scans: u64,
    /// Scans dropped because a consumer fell behind, since the run started.
    pub dropped_scans: u64,
    /// Link re-establishments after a loss, since the run started.
    pub reconnects: u64,
}

impl Default for LidarStatus {
    fn default() -> Self {
        Self {
            v: LIDAR_STATUS_VERSION,
            device: None,
            connected: false,
            health: None,
            health_code: None,
            rate_hz: None,
            scans: 0,
            dropped_scans: 0,
            reconnects: 0,
        }
    }
}
