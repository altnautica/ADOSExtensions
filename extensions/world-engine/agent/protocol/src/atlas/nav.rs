//! The live navigation and mapping payloads the drone publishes while the
//! LiDAR runs, independent of any capture session: the planar scan, the live
//! occupancy tiles, the live sweep-cloud delta and the map status.
//!
//! Frames:
//! - `map_ned` is north-aligned: x north, y east, z down, metres (z from the
//!   flight controller's height above the first scan). The planar SLAM world
//!   (body FLU at the first scan) is turned to FRD and rotated about down by
//!   the flight controller's attitude yaw (its compass heading) at take-off
//!   (followed while the vehicle sits where the first pose was taken), origin
//!   at the first scan. While the FC navigates on
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

/// The live map's state on `plugin.world-engine.map_status` (1 Hz).
#[derive(Debug, Clone, PartialEq, Default, Serialize, Deserialize)]
pub struct MapStatus {
    /// The open capture session, if any.
    pub session_id: Option<String>,
    pub placement: ComputePlacement,
    /// Bytes held by the bounded map stores, MiB.
    pub memory_used_mb: f32,
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
}
