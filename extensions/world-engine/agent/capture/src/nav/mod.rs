//! The live navigation and mapping lanes, independent of capture sessions:
//! while the pose provider runs (`atlas.nav_pose = always`, or only with a
//! session open for `capture`) the capture service publishes
//!
//! - `plugin.world-engine.pose` in the north-aligned `map_ned` at most every
//!   `atlas.pose_publish_interval_ms` (no faster than the provider estimates);
//! - `plugin.world-engine.scan`, each LiDAR scan binned in the body frame;
//! - `plugin.world-engine.occupancy` live tiles and
//!   `plugin.world-engine.pointcloud` sweep-cloud deltas, 1 Hz;
//! - `plugin.world-engine.map_status`, 1 Hz.
//!
//! Navigation SLAM and the live occupancy are built on board in every
//! placement; the placement sets the live sweep-cloud resolution and whether
//! the node reconstructs in flight or after landing.
//!
//! `map_ned` starts as the SLAM world rotated to the flight controller's
//! compass heading at the first pose, origin at the first scan. While the EKF
//! navigates on GNSS it is moved onto the EKF's local frame
//! (`LOCAL_POSITION_NED`): yaw and offset fitted between the two tracks
//! ([`align`]), frozen when GNSS goes, so a switch to external navigation
//! finds the two frames in agreement.

pub mod align;
pub mod budget;
pub mod fc_imu;
pub mod frames;

#[cfg(test)]
mod tests;

use std::path::{Path, PathBuf};

use world_engine_livemap::LiveMap;
use world_engine_protocol::atlas::{
    ComputePlacement, LiveOccupancy, LivePointCloud, MapStatus, NavFrame, NavPoseMode,
    NavPoseSource, PolarScan, Pose, PoseDescriptor, PoseSource, ScanDescriptor, StreamPhase,
    StreamProgress, VioHealth,
};

use crate::lidar_scan::LidarScan;
use crate::pose::{se3, PoseEstimate};
use align::MapAlign;
use budget::Budget;

/// A fitted yaw is logged again once it has moved this far, radians.
const YAW_LOG_STEP_RAD: f64 = 0.25 * std::f64::consts::PI / 180.0;

/// The session subdirectory the live map is saved under.
pub const SESSION_MAP_DIR: &str = "map";
/// The sweep cloud's file in [`SESSION_MAP_DIR`]: little-endian `f32` xyz.
pub const SWEEP_FILE: &str = "sweep_cloud.f32";
/// Occupancy tiles' directory in [`SESSION_MAP_DIR`].
pub const TILES_DIR: &str = "occupancy";

/// The resolved `atlas.nav_pose`, `atlas.compute_placement` and budget.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavSettings {
    pub nav_pose: NavPoseMode,
    pub placement: ComputePlacement,
    pub budget: Budget,
    /// `atlas.pose_publish_interval_ms`.
    pub pose_interval_ms: i64,
}

/// What one scan produced.
#[derive(Debug, Default)]
pub struct ScanOut {
    /// The binned body-frame scan, when the lanes are on.
    pub scan: Option<ScanDescriptor>,
    /// The polar record with its sensor pose, when a pose was known.
    pub polar: Option<PolarScan>,
}

/// What the 1 Hz tick produced.
#[derive(Debug)]
pub struct TickOut {
    pub occupancy: Option<LiveOccupancy>,
    pub pointcloud: Option<LivePointCloud>,
    pub status: MapStatus,
}

/// Facts the tick reports that the map does not own.
#[derive(Debug, Default, Clone)]
pub struct TickInputs {
    pub session_id: Option<String>,
    /// Where evicted occupancy tiles go (the open session's map dir, else a
    /// scratch dir).
    pub evict_dir: Option<PathBuf>,
    /// Bytes waiting in the recorder's keyframe and scan queues.
    pub queued_bytes: usize,
    pub keyframes_dropped: u64,
}

fn nav_source(s: PoseSource) -> NavPoseSource {
    match s {
        PoseSource::Lidar => NavPoseSource::LidarPlanar,
        PoseSource::FcEkf => NavPoseSource::FcEkf,
        PoseSource::Vio => NavPoseSource::Vio,
        PoseSource::Replay => NavPoseSource::Replay,
    }
}

/// The live map state the capture loop owns.
pub struct NavMap {
    settings: NavSettings,
    map: LiveMap,
    last_pub_ms: Option<i64>,
    /// The flight controller's up height at the first nav pose: `map_ned` z is
    /// measured from it.
    fc_up0: Option<f64>,
    /// The compass heading of the SLAM world's x axis, radians clockwise from
    /// north: `map_ned = Rz(yaw0) · map_frd` until the GNSS fit takes over.
    /// Latched from the flight controller's attitude yaw at the first pose
    /// that has one; until then nothing is placed in the map.
    yaw0: Option<f64>,
    /// The GNSS alignment onto the EKF's local frame.
    align: MapAlign,
    /// `map_ned`'s horizontal offset from the rotated SLAM world (north,
    /// east): zero until GNSS, then the EKF's, frozen when GNSS goes.
    offset: [f64; 2],
    /// The fitted yaw last logged.
    logged_yaw: Option<f64>,
    /// The latest pose in `map_ned`: `(R, t)`.
    latest: Option<([f64; 9], [f64; 3])>,
}

/// `Rz(yaw)` about the down axis, row-major.
fn rz(yaw: f64) -> [f64; 9] {
    let (s, c) = yaw.sin_cos();
    [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0]
}

/// The FRD yaw of a world-from-body FRD rotation (body x's heading).
pub fn frd_yaw(r: &[f64; 9]) -> f64 {
    r[3].atan2(r[0])
}

/// An angle wrapped to `(-π, π]`.
fn wrap_pi(a: f64) -> f64 {
    let w = a.rem_euclid(std::f64::consts::TAU);
    if w > std::f64::consts::PI {
        w - std::f64::consts::TAU
    } else {
        w
    }
}

impl NavMap {
    pub fn new(settings: NavSettings) -> Self {
        Self {
            map: LiveMap::new(settings.placement.live_voxel_m(), settings.budget.sweep()),
            settings,
            last_pub_ms: None,
            fc_up0: None,
            yaw0: None,
            align: MapAlign::default(),
            offset: [0.0; 2],
            logged_yaw: None,
            latest: None,
        }
    }

    pub fn settings(&self) -> &NavSettings {
        &self.settings
    }

    /// Whether the map is north-aligned yet (the FC heading has been seen).
    pub fn heading_known(&self) -> bool {
        self.map_yaw().is_some()
    }

    /// `map_ned = Rz(map_yaw) · map_frd + offset`: the GNSS fit's yaw when
    /// there is one, else the compass latch.
    pub fn map_yaw(&self) -> Option<f64> {
        self.align.fit().map(|f| f.yaw).or(self.yaw0)
    }

    /// Fold one provider estimate paired with the flight controller's EKF
    /// position at the same instant (`ekf_ne`: north, east, metres), taken
    /// while that position is GNSS-referenced. A tracking pose only.
    pub fn on_gnss_ref(&mut self, e: &PoseEstimate, ekf_ne: [f64; 2]) {
        if e.health != VioHealth::Good {
            return;
        }
        let t_frd = frames::flu_to_frd_vec(e.t);
        let fit = self.align.observe([t_frd[0], t_frd[1]], ekf_ne);
        let Some(yaw) = self.map_yaw() else { return };
        if let Some(c) = self.align.offset(yaw) {
            self.offset = c;
        }
        let Some(fit) = fit else { return };
        let moved = self
            .logged_yaw
            .is_none_or(|y| wrap_pi(fit.yaw - y).abs() >= YAW_LOG_STEP_RAD);
        if moved {
            self.logged_yaw = Some(fit.yaw);
            tracing::info!(
                yaw_deg = fit.yaw.to_degrees(),
                compass_offset_deg = self.yaw0.map(|y| wrap_pi(fit.yaw - y).to_degrees()),
                offset_n_m = self.offset[0],
                offset_e_m = self.offset[1],
                baseline_m = fit.baseline_m,
                residual_m = fit.residual_m,
                sigma_deg = fit.sigma_rad.to_degrees(),
                "nav_map_gnss_fitted"
            );
        }
    }

    /// GNSS is not usable now: the alignment freezes.
    pub fn on_gnss_lost(&mut self) {
        if self.align.freeze() {
            tracing::info!(
                yaw_deg = self.map_yaw().map(f64::to_degrees),
                yaw_fitted = self.align.fit().is_some(),
                offset_n_m = self.offset[0],
                offset_e_m = self.offset[1],
                "nav_map_gnss_frozen"
            );
        }
    }

    /// Whether the lanes publish now.
    pub fn active(&self, session_open: bool) -> bool {
        self.settings.nav_pose == NavPoseMode::Always || session_open
    }

    /// Whether the compute node reconstructs this flight live.
    pub fn streams(&self) -> bool {
        self.settings.placement.streams_live()
    }

    /// The provider's world-from-body (FLU) turned to FRD (`map_frd`), with z
    /// from the flight controller's height above the first pose when known.
    fn place_frd(&mut self, r: &[f64; 9], t: [f64; 3], fc_up: Option<f64>) -> ([f64; 9], [f64; 3]) {
        let mut t_frd = frames::flu_to_frd_vec(t);
        if let Some(up) = fc_up {
            let up0 = *self.fc_up0.get_or_insert(up);
            t_frd[2] = -(up - up0);
        }
        (frames::flu_to_frd_rot(r), t_frd)
    }

    /// `map_frd` to the north-aligned `map_ned`, or `None` before the heading
    /// is known.
    fn ned_of(&self, r: &[f64; 9], t: [f64; 3]) -> Option<([f64; 9], [f64; 3])> {
        let rot = rz(self.map_yaw()?);
        let t = se3::mat_vec(&rot, t);
        let t = [t[0] + self.offset[0], t[1] + self.offset[1], t[2]];
        Some((se3::mat_mul(&rot, r), t))
    }

    /// Fold one provider estimate; the descriptor to publish when the lanes
    /// are on and the interval is due. `fc_heading` is the flight
    /// controller's attitude yaw now (radians from north); the first one seen
    /// with a pose fixes the map's north until the GNSS fit replaces it.
    /// Nothing is published before the map has a north.
    pub fn on_pose(
        &mut self,
        e: &PoseEstimate,
        fc_up: Option<f64>,
        fc_heading: Option<f64>,
        now_ms: i64,
        session_open: bool,
    ) -> Option<PoseDescriptor> {
        let (r_frd, t_frd) = self.place_frd(&e.r, e.t, fc_up);
        if let (None, Some(heading)) = (self.yaw0, fc_heading) {
            let yaw0 = heading - frd_yaw(&r_frd);
            tracing::info!(
                heading_rad = heading,
                yaw0_rad = yaw0,
                "nav_map_north_latched"
            );
            self.yaw0 = Some(yaw0);
        }
        let (r, t) = self.ned_of(&r_frd, t_frd)?;
        self.latest = Some((r, t));
        if !self.active(session_open) {
            return None;
        }
        let due = self
            .last_pub_ms
            .is_none_or(|last| now_ms < last || now_ms - last >= self.settings.pose_interval_ms);
        if !due {
            return None;
        }
        self.last_pub_ms = Some(now_ms);
        Some(PoseDescriptor {
            pose: Pose { r, t, cov: None },
            anchor: None,
            ts_ms: now_ms,
            health: e.health,
            frame: NavFrame::MapNed,
            source: nav_source(e.source),
        })
    }

    /// Fold one LiDAR scan. `world` is the provider's body pose at the scan
    /// time (FLU world), when known; `t_body_sensor` the mount. The polar
    /// record and the live map wait for the map's north.
    pub fn on_scan(
        &mut self,
        scan: &LidarScan,
        t_body_sensor: &[f64; 16],
        world: Option<&PoseEstimate>,
        fc_up: Option<f64>,
        session_open: bool,
    ) -> ScanOut {
        let active = self.active(session_open);
        let mut out = ScanOut {
            scan: active.then(|| frames::body_scan(scan, t_body_sensor)),
            polar: None,
        };
        let Some(e) = world else { return out };
        let (r_bs, t_bs) = se3::from_mat4(t_body_sensor);
        let r_ws = se3::mat_mul(&e.r, &r_bs);
        let off = se3::mat_vec(&e.r, t_bs);
        let t_ws = [e.t[0] + off[0], e.t[1] + off[1], e.t[2] + off[2]];
        let (r_frd, mut t_frd) = self.place_frd(&r_ws, t_ws, None);
        if let (Some(up), Some(up0)) = (fc_up, self.fc_up0) {
            // The sensor sits `off` from the body origin; keep that offset
            // under the flight controller's height.
            t_frd[2] = -(up - up0) - off[2];
        }
        let Some((r_map, t_map)) = self.ned_of(&r_frd, t_frd) else {
            return out;
        };
        let polar = frames::polar_scan(scan, scan.t_ns, &r_map, t_map, e.health);
        if active {
            self.map.insert(&polar);
        }
        out.polar = Some(polar);
        out
    }

    /// The 1 Hz tick: changed occupancy tiles, the sweep delta, evictions and
    /// the map status.
    pub fn tick(&mut self, inputs: &TickInputs, session_open: bool) -> TickOut {
        let active = self.active(session_open);
        let (occupancy, pointcloud) = self.map.tick(
            self.settings.budget.occupancy(),
            inputs.evict_dir.as_deref(),
        );
        let (occupancy, pointcloud) = if active {
            (occupancy, pointcloud)
        } else {
            (None, None)
        };
        let used = self.map.bytes() + inputs.queued_bytes;
        // The link, which sees the bearer, fills in the state and rate.
        let stream = if self.streams() {
            StreamProgress {
                state: StreamPhase::Connecting,
                bytes_per_s: 0,
            }
        } else {
            StreamProgress::default()
        };
        TickOut {
            occupancy,
            pointcloud,
            status: MapStatus {
                session_id: inputs.session_id.clone(),
                placement: self.settings.placement,
                memory_used_mb: used as f32 / (1024.0 * 1024.0),
                memory_budget_mb: self.settings.budget.total_mb(),
                voxel_m: self.map.sweep.voxel_m(),
                keyframes_dropped: inputs.keyframes_dropped,
                sweep_points: self.map.sweep.len() as u64,
                area_m2: self.map.grid.area_m2(),
                nav_pose: self.settings.nav_pose,
                stream,
                ..MapStatus::default()
            },
        }
    }

    /// Save the live map into a closing session: the sweep cloud and every
    /// occupancy tile still in RAM (evicted ones are already there).
    pub fn save(&self, session_dir: &Path) {
        let dir = session_dir.join(SESSION_MAP_DIR);
        if let Err(e) = self.map.sweep.save(&dir.join(SWEEP_FILE)) {
            tracing::warn!(error = %e, "sweep_cloud_save_failed");
        }
        self.map.grid.save_all(&dir.join(TILES_DIR));
    }
}
