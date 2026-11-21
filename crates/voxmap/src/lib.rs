//! A robot-centred 3D voxel map for obstacle avoidance and planning.
//!
//! The map is a window of `nx * ny * nz` voxels (default 120 x 120 x 50 at
//! 0.2 m: 24 x 24 x 10 m) in the navigation map frame (north, east, down,
//! metres) that follows the vehicle. It is stored as a ring buffer: absolute
//! voxel `c` lives at ring slot `c mod n` on each axis, so moving the window
//! only clears the slabs that scroll out ([`VoxMap::recentre`]).
//!
//! Each voxel holds a log-odds occupancy in fixed point (one `i8` unit is
//! [`LO_UNIT`]): a lidar or depth return adds [`LO_HIT`] at its voxel and every
//! voxel its ray crossed takes [`LO_MISS`] (except the voxels touching the
//! return's own, so a ray grazing a surface does not erode it), clamped to
//! [`LO_MIN`]..[`LO_MAX`]. A voxel at or above [`LO_OCCUPIED`] is occupied, at
//! or below [`LO_FREE`] free, otherwise unknown. Within one insertion a voxel
//! is updated once, and a hit wins over a miss.
//!
//! Two rules keep the map honest about change:
//!
//! * Decay ([`VoxMap::decay`]): a voxel with positive evidence not hit for
//!   [`DECAY_GRACE_S`] loses [`DECAY_PER_S`] log-odds per second until it is
//!   back at unknown.
//! * Dynamic clearing: a voxel with positive evidence that rays pass through
//!   on [`DYNAMIC_PASSES`] insertions in a row, without a hit in between, is
//!   set free at once (someone walked through it).
//!
//! Alongside occupancy the map keeps a distance field: for every voxel, the
//! Euclidean distance to the nearest obstacle voxel in the window, capped at
//! [`ESDF_CAP_M`]. A voxel joins the obstacle set when it becomes occupied
//! and leaves it only when its positive evidence is gone, so a sparsely seen
//! surface does not flicker in and out of the field. The field is updated
//! incrementally ([`VoxMap::update_esdf`]) from the voxels whose membership
//! changed, with raise and lower waves over a bucketed queue, so a frame's
//! cost follows what changed, not the window. [`VoxMap::tick_esdf`] runs it
//! every frame or, configured with `esdf_every`, every few frames (every 2nd
//! at 10 Hz gives a 5 Hz field when a slower board needs the time).
//!
//! [`VoxMap::to_seam`] publishes occupancy and distance (decimetres) through
//! the agent's voxel-map seam.
//!
//! A typical 10 Hz frame: [`VoxMap::recentre`] on the body position, one or
//! more insertions, [`VoxMap::decay`], [`VoxMap::tick_esdf`], then
//! [`VoxMap::to_seam`].

mod decay;
mod esdf;
mod insert;
mod logodds;
mod query;
mod seam;
mod window;

#[cfg(test)]
mod tests;

pub use ados_protocol::voxmap::{VoxTier, ESDF_CAP_DM, VOX_FREE, VOX_OCCUPIED, VOX_UNKNOWN};
pub use insert::{PlanarReturn, Pose};
pub use logodds::{
    DECAY_GRACE_S, DECAY_PER_S, DYNAMIC_PASSES, LO_FREE, LO_HIT, LO_MAX, LO_MIN, LO_MISS,
    LO_OCCUPIED, LO_UNIT,
};
pub use query::{RayHit, VoxelState};

/// Default voxel edge, metres.
pub const DEFAULT_RES_M: f32 = 0.2;
/// Default window: 24 x 24 x 10 m at the default resolution.
pub const DEFAULT_DIMS: [u16; 3] = [120, 120, 50];
/// Voxel edge of [`VoxMapConfig::fine_window`], metres.
pub const FINE_RES_M: f32 = 0.15;
/// Window of [`VoxMapConfig::fine_window`]: 30 x 30 x 12 m.
pub const FINE_DIMS: [u16; 3] = [200, 200, 80];
/// The largest window edge in voxels (window coordinates pack into 10 bits).
pub const MAX_AXIS: u16 = 1024;
/// Distances beyond this are reported as this, metres.
pub const ESDF_CAP_M: f32 = 4.0;

/// Window geometry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VoxMapConfig {
    /// Voxel edge, metres.
    pub res_m: f32,
    /// Window size in voxels (north, east, down).
    pub dims: [u16; 3],
    /// How far, metres per axis, the vehicle may drift from the window centre
    /// before [`VoxMap::recentre`] moves the window. A larger slack moves the
    /// window less often, in larger steps, for the same total work.
    pub recentre_slack_m: f32,
    /// [`VoxMap::tick_esdf`] runs the distance-field update on every this
    /// many calls (1: every frame; 2 at a 10 Hz frame rate is 5 Hz).
    pub esdf_every: u16,
}

impl Default for VoxMapConfig {
    fn default() -> Self {
        Self {
            res_m: DEFAULT_RES_M,
            dims: DEFAULT_DIMS,
            recentre_slack_m: 1.0,
            esdf_every: 1,
        }
    }
}

impl VoxMapConfig {
    /// The 0.15 m, 30 x 30 x 12 m window: finer and farther-reaching than the
    /// default, at about four times the frame cost (on the reference 3D
    /// lidar forest benchmark it measured over the 15 ms frame budget even
    /// on a desktop x86 core). Only for an RK3588-class board with 8 GB, and
    /// there with `esdf_every` 2 as the first fallback.
    pub fn fine_window() -> Self {
        Self {
            res_m: FINE_RES_M,
            dims: FINE_DIMS,
            ..Self::default()
        }
    }
}

/// The voxel map. Every buffer is allocated at construction; frames reuse
/// them.
pub struct VoxMap {
    pub(crate) res: f32,
    pub(crate) inv_res: f32,
    /// Window size per axis.
    pub(crate) n: [i32; 3],
    /// Ring index strides: 1, nx, nx * ny.
    pub(crate) stride: [i32; 3],
    /// Absolute voxel of window (0, 0, 0).
    pub(crate) origin: [i32; 3],
    /// `origin mod n`: the ring slot of window (0, 0, 0).
    pub(crate) om: [i32; 3],
    pub(crate) slack: i32,
    /// Occupancy record per ring slot.
    pub(crate) vox: Vec<Vox>,
    /// `VOX_*` class per ring slot, kept equal to the class of the log-odds
    /// (the published plane, ring order).
    pub(crate) state: Vec<u8>,
    /// The current scan.
    pub(crate) epoch_now: u8,
    pub(crate) esdf: esdf::Esdf,
    pub(crate) esdf_every: u16,
    pub(crate) esdf_ticks: u16,
    /// Map points of the planar scan being inserted.
    pub(crate) scratch: Vec<[f32; 3]>,
    /// Whole seconds (mod 256) of the newest insertion.
    pub(crate) now_s: u8,
    pub(crate) stamp_ns: u64,
    /// ADOS time up to which decay has been applied.
    pub(crate) decay_clock_ns: Option<u64>,
    pub(crate) stats: WorkStats,
}

/// One voxel's occupancy record, kept together so an update touches one
/// cache line.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
#[repr(C)]
pub(crate) struct Vox {
    /// Log-odds in units of [`LO_UNIT`].
    pub(crate) lo: i8,
    /// The dynamic pass streak and the marks of scan `epoch`.
    pub(crate) flags: u8,
    /// The scan whose marks `flags` holds.
    pub(crate) epoch: u8,
    /// Whole seconds of ADOS time (mod 256) of the last hit.
    pub(crate) last_hit: u8,
}

/// Work done since the last [`VoxMap::take_stats`], for logs and tuning.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct WorkStats {
    /// Rays cast (after merging rays that end in the same voxel).
    pub rays_cast: u64,
    /// Voxels those rays stepped through.
    pub ray_steps: u64,
    /// Voxels updated by insertions.
    pub voxels_updated: u64,
    /// Voxels that became occupied.
    pub obstacles_added: u64,
    /// Voxels that stopped being occupied.
    pub obstacles_removed: u64,
    /// Distance-field queue entries processed.
    pub esdf_visits: u64,
}

/// A configuration the map cannot hold.
#[derive(Debug, Clone, PartialEq)]
pub struct ConfigError(pub String);

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "voxmap config: {}", self.0)
    }
}

impl std::error::Error for ConfigError {}

impl VoxMap {
    /// An empty (all unknown) map with its window at the map origin.
    pub fn new(cfg: VoxMapConfig) -> Result<Self, ConfigError> {
        if !(cfg.res_m.is_finite() && cfg.res_m > 0.0) {
            return Err(ConfigError(format!("resolution {} m", cfg.res_m)));
        }
        if cfg.dims.iter().any(|d| !(4..=MAX_AXIS).contains(d)) {
            return Err(ConfigError(format!(
                "dims {:?}: each axis 4..={MAX_AXIS}",
                cfg.dims
            )));
        }
        let n = cfg.dims.map(i32::from);
        let count = (n[0] * n[1] * n[2]) as usize;
        let origin = [-n[0] / 2, -n[1] / 2, -n[2] / 2];
        Ok(Self {
            res: cfg.res_m,
            inv_res: 1.0 / cfg.res_m,
            n,
            stride: [1, n[0], n[0] * n[1]],
            origin,
            om: [0, 1, 2].map(|a| origin[a].rem_euclid(n[a])),
            slack: (cfg.recentre_slack_m.max(0.0) / cfg.res_m).round() as i32,
            vox: vec![Vox::default(); count],
            state: vec![VOX_UNKNOWN; count],
            esdf: esdf::Esdf::new(count, cfg.res_m),
            epoch_now: 0,
            esdf_every: cfg.esdf_every.max(1),
            esdf_ticks: 0,
            scratch: Vec::with_capacity(4096),
            now_s: 0,
            stamp_ns: 0,
            decay_clock_ns: None,
            stats: WorkStats::default(),
        })
    }

    /// Voxel edge, metres.
    pub fn res_m(&self) -> f32 {
        self.res
    }

    /// Window size in voxels.
    pub fn dims(&self) -> [u16; 3] {
        self.n.map(|v| v as u16)
    }

    /// Absolute voxel of window (0, 0, 0).
    pub fn origin_cell(&self) -> [i32; 3] {
        self.origin
    }

    /// ADOS time of the newest insertion, nanoseconds.
    pub fn stamp_ns(&self) -> u64 {
        self.stamp_ns
    }

    /// The work counters since the last call, which resets them.
    pub fn take_stats(&mut self) -> WorkStats {
        std::mem::take(&mut self.stats)
    }

    /// Run [`VoxMap::update_esdf`] when this call is due under the
    /// configured `esdf_every`; changes queue up in between. Returns whether
    /// it ran. Queries and the published distances reflect the last run.
    pub fn tick_esdf(&mut self) -> bool {
        self.esdf_ticks += 1;
        if self.esdf_ticks < self.esdf_every {
            return false;
        }
        self.esdf_ticks = 0;
        self.update_esdf();
        true
    }

    /// The absolute voxel holding map point `p`.
    pub fn cell_of(&self, p: [f32; 3]) -> [i32; 3] {
        p.map(|v| (v * self.inv_res).floor() as i32)
    }

    /// The centre of absolute voxel `c`, map metres.
    pub fn centre_of(&self, c: [i32; 3]) -> [f32; 3] {
        c.map(|v| (v as f32 + 0.5) * self.res)
    }
}
