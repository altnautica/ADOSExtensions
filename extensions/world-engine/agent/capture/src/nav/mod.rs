//! The live navigation and mapping lanes, independent of capture sessions:
//! while the pose provider runs (`atlas.nav_pose = always`, or only with a
//! session open for `capture`) the capture service publishes
//!
//! - `plugin.world-engine.pose` in the north-aligned `map_ned`, one every
//!   `atlas.pose_publish_interval_ms` of the poses' own measurement time
//!   (no faster than the provider estimates; see [`NavMap::on_pose`]);
//! - `plugin.world-engine.scan`, each LiDAR scan binned in the body frame;
//! - `plugin.world-engine.occupancy` live tiles and
//!   `plugin.world-engine.pointcloud` sweep-cloud deltas, 1 Hz;
//! - `plugin.world-engine.map_status`, 1 Hz.
//!
//! Navigation SLAM and the live occupancy are built on board in every
//! placement; the placement sets the live sweep-cloud resolution and whether
//! the node reconstructs in flight or after landing.
//!
//! `map_ned` starts as the SLAM world rotated to north, origin at the first
//! scan. Its north ([`MapNorth`]) comes from one of two places:
//!
//! - **Declared** (`atlas.takeoff_heading_deg` set): latched once at the first
//!   pose from the configured heading the vehicle faces there, and never taken
//!   from the flight controller. This is the north a flight controller may
//!   take its yaw from (ArduPilot `EK3_SRCn_YAW` 6): with the FC's yaw coming
//!   from the map, a north that followed the FC heading would only echo the
//!   FC's own yaw back to it, and nothing would hold north while gyro drift
//!   walks it on the ground.
//! - **FC heading** (unset): the FC's attitude yaw, followed while the vehicle
//!   still sits where the first pose was taken and fixed once it leaves that
//!   spot: before ArduPilot has an EKF origin its heading is magnetic, and it
//!   turns onto true north over the following seconds once the origin brings
//!   the declination in (12.8° at the bench's location), so a heading latched
//!   at the first pose could leave the map that far off the EKF's own yaw.
//!
//! While the EKF navigates on GNSS the map is moved onto the EKF's local
//! frame (`LOCAL_POSITION_NED`): yaw and offset fitted between the two tracks
//! ([`align`]), frozen when GNSS goes, so a switch to external navigation
//! finds the two frames in agreement.

pub mod align;
pub mod budget;
pub mod fc_flow;
pub mod fc_imu;
pub mod frames;
pub mod scan_wait;

#[cfg(test)]
mod tests;

use std::path::{Path, PathBuf};

use world_engine_livemap::LiveMap;
use world_engine_protocol::atlas::{
    ComputePlacement, LiveOccupancy, LivePointCloud, MapNorth, MapStatus, NavFrame, NavPoseMode,
    NavPoseSource, PolarScan, Pose, PoseDescriptor, PoseSource, ScanDescriptor, StreamPhase,
    StreamProgress, VioHealth,
};

use crate::lidar_scan::LidarScan;
use crate::pose::{se3, PoseEstimate};
use align::MapAlign;
use budget::Budget;

/// A fitted yaw is logged again once it has moved this far, radians.
const YAW_LOG_STEP_RAD: f64 = 0.25 * std::f64::consts::PI / 180.0;
/// How far the vehicle may move from where the first pose was taken,
/// horizontally or in climb, and still count as on that spot, metres: the
/// map's north follows the FC heading until it leaves.
pub const ON_SPOT_M: f64 = 0.5;

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
    /// `atlas.takeoff_heading_deg` in radians: the declared north (see the
    /// module docs); `None` follows the FC heading.
    pub takeoff_heading_rad: Option<f64>,
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
    /// The capture process's resident memory (`VmRSS`), MiB; `None` when
    /// `/proc` could not be read.
    pub rss_mb: Option<f32>,
    pub keyframes_dropped: u64,
}

/// `VmRSS` of a `/proc/<pid>/status` text, MiB; `None` without that line.
pub fn vm_rss_mb(status: &str) -> Option<f32> {
    let line = status.lines().find(|l| l.starts_with("VmRSS:"))?;
    let mut words = line["VmRSS:".len()..].split_whitespace();
    let kib: f64 = words.next()?.parse().ok()?;
    (words.next()? == "kB").then_some((kib / 1024.0) as f32)
}

/// This process's resident memory, MiB; `None` when `/proc/self/status`
/// cannot be read.
pub fn read_vm_rss_mb() -> Option<f32> {
    vm_rss_mb(&std::fs::read_to_string("/proc/self/status").ok()?)
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
    /// The measurement time (`t_ns`) of the last published pose and the time
    /// the next one is due.
    last_pub: Option<(u64, u64)>,
    /// The flight controller's up height at the first nav pose: `map_ned` z is
    /// measured from it.
    fc_up0: Option<f64>,
    /// The heading of the SLAM world's x axis, radians clockwise from north:
    /// `map_ned = Rz(yaw0) · map_frd` until the GNSS fit takes over. Latched
    /// at the first pose from the declared take-off heading, or else taken
    /// from the flight controller's attitude yaw at each pose while the
    /// vehicle is on the spot of the first one, then fixed; until the first
    /// heading nothing is placed in the map.
    yaw0: Option<f64>,
    /// Where the first heading was taken (`map_frd`), while the vehicle is
    /// still there; `None` before it and once the vehicle has left.
    spot: Option<[f64; 3]>,
    /// Whether the vehicle has left the spot of the first heading.
    left_spot: bool,
    /// The GNSS alignment onto the EKF's local frame.
    align: MapAlign,
    /// `map_ned`'s horizontal offset from the rotated SLAM world (north,
    /// east): zero until GNSS, then the EKF's, frozen when GNSS goes.
    offset: [f64; 2],
    /// The fitted yaw last logged.
    logged_yaw: Option<f64>,
    /// The latest pose in `map_ned`: `(R, t)`.
    latest: Option<([f64; 9], [f64; 3])>,
    /// The host ns the matcher's map last started over at
    /// ([`Self::restart_heading_at`]).
    restarted_at: Option<u64>,
    /// The north was re-anchored to the FC heading in flight
    /// ([`Self::restart_in_flight`]), whatever the declared take-off heading.
    north_from_fc: bool,
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
            last_pub: None,
            fc_up0: None,
            yaw0: None,
            spot: None,
            left_spot: false,
            align: MapAlign::default(),
            offset: [0.0; 2],
            logged_yaw: None,
            latest: None,
            restarted_at: None,
            north_from_fc: false,
        }
    }

    /// The matcher started a new map at host `t_ns` (at the first `RAW_IMU`
    /// sample: the attitude stand-in before it held no heading through a
    /// turn, so the old map's heading against the world is unknown). The
    /// map, its north and its GNSS fit start over once per restart: the
    /// north is latched afresh as at the first pose.
    pub fn restart_heading_at(&mut self, t_ns: u64) {
        if self.restarted_at == Some(t_ns) {
            return;
        }
        tracing::info!(t_ns, "nav_map_restarted");
        *self = Self {
            restarted_at: Some(t_ns),
            ..Self::new(self.settings)
        };
    }

    /// The matcher started a new map in flight at pose `e` (a heading a gap
    /// in the IMU left unmeasured was not re-confirmed in time,
    /// [`crate::pose::PoseQuality::map_restarts`]): its heading against the
    /// old map is the gyro's guess across the gap. Its north is taken from
    /// the FC's attitude yaw now (`fc_heading`), even with a declared
    /// take-off heading, which holds only at the take-off spot; the pose is
    /// lost (external navigation not fused) until the new map holds enough
    /// scans, so this yaw is the FC's own. The offset keeps `e` where the
    /// old map placed it, so `map_ned` does not jump, and the GNSS fit
    /// starts over (it fitted the old map). Without an FC heading the old
    /// north stays.
    pub fn restart_in_flight(&mut self, e: &PoseEstimate, fc_up: Option<f64>, fc_heading: Option<f64>) {
        let (r_frd, t_frd) = self.place_frd(&e.r, e.t, fc_up);
        let Some(heading) = fc_heading else {
            tracing::warn!("nav_map_restart_without_fc_heading");
            return;
        };
        let old = self.ned_of(&r_frd, t_frd).map(|(_, t)| t);
        let yaw0 = wrap_pi(heading - frd_yaw(&r_frd));
        let placed = se3::mat_vec(&rz(yaw0), t_frd);
        self.offset = old.map_or([0.0; 2], |t| [t[0] - placed[0], t[1] - placed[1]]);
        self.yaw0 = Some(yaw0);
        self.spot = None;
        self.left_spot = true;
        self.north_from_fc = true;
        self.align = MapAlign::default();
        self.logged_yaw = None;
        tracing::info!(
            heading_rad = heading,
            yaw0_rad = yaw0,
            offset_n_m = self.offset[0],
            offset_e_m = self.offset[1],
            "nav_map_north_restarted"
        );
    }

    pub fn settings(&self) -> &NavSettings {
        &self.settings
    }

    /// Whether the map is north-aligned yet (the FC heading has been seen).
    pub fn heading_known(&self) -> bool {
        self.map_yaw().is_some()
    }

    /// `map_ned = Rz(map_yaw) · map_frd + offset`: the GNSS fit's yaw when
    /// there is one, else the latched heading.
    pub fn map_yaw(&self) -> Option<f64> {
        self.align.fit().map(|f| f.yaw).or(self.yaw0)
    }

    /// Where the map's north comes from now; `None` before it has one.
    pub fn north(&self) -> Option<MapNorth> {
        if self.align.fit().is_some() {
            Some(MapNorth::Gnss)
        } else if self.yaw0.is_none() {
            None
        } else if self.settings.takeoff_heading_rad.is_some() && !self.north_from_fc {
            Some(MapNorth::Declared)
        } else {
            Some(MapNorth::FcHeading)
        }
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

    /// Latch the map's north at the first pose from the declared take-off
    /// heading when there is one. Otherwise take it from the FC heading while
    /// the vehicle is on the spot of the first heading, and fix it once the
    /// vehicle has moved [`ON_SPOT_M`] from there horizontally or in climb.
    fn follow_heading(&mut self, r_frd: &[f64; 9], t_frd: [f64; 3], fc_heading: Option<f64>) {
        if self.left_spot {
            return;
        }
        if let Some(declared) = self.settings.takeoff_heading_rad {
            let yaw0 = wrap_pi(declared - frd_yaw(r_frd));
            self.yaw0 = Some(yaw0);
            self.left_spot = true;
            tracing::info!(
                heading_rad = declared,
                fc_heading_rad = fc_heading,
                yaw0_rad = yaw0,
                "nav_map_north_declared"
            );
            return;
        }
        if let Some(spot) = self.spot {
            let horizontal = (t_frd[0] - spot[0]).hypot(t_frd[1] - spot[1]);
            let climb = spot[2] - t_frd[2];
            if horizontal > ON_SPOT_M || climb > ON_SPOT_M {
                self.left_spot = true;
                self.spot = None;
                tracing::info!(
                    yaw0_rad = self.yaw0,
                    horizontal_m = horizontal,
                    climb_m = climb,
                    "nav_map_north_fixed"
                );
                return;
            }
        }
        let Some(heading) = fc_heading else { return };
        let yaw0 = heading - frd_yaw(r_frd);
        if self.yaw0.is_none() {
            tracing::info!(
                heading_rad = heading,
                yaw0_rad = yaw0,
                "nav_map_north_latched"
            );
            self.spot = Some(t_frd);
        }
        self.yaw0 = Some(yaw0);
    }

    /// Fold one provider estimate; the descriptor to publish when the lanes
    /// are on and the pose is due (`pose_due`). `fc_heading` is the flight
    /// controller's attitude yaw now (radians from north): without a declared
    /// take-off heading the map's north follows it while the vehicle is on
    /// the spot of the first heading (see the module docs) and is fixed once
    /// it leaves, until the GNSS fit replaces it. Nothing is published before
    /// the map has a north.
    /// `now_ms` (unix) and `now_mono_ns` are the same instant; the
    /// descriptor is stamped with the unix time of the estimate's own
    /// `t_ns` (a LiDAR pose describes its scan's first return, at least a
    /// revolution before it is published), so a consumer such as the
    /// flight controller's external navigation fuses it at the time it was
    /// measured.
    pub fn on_pose(
        &mut self,
        e: &PoseEstimate,
        fc_up: Option<f64>,
        fc_heading: Option<f64>,
        now_ms: i64,
        now_mono_ns: u64,
        session_open: bool,
    ) -> Option<PoseDescriptor> {
        let (r_frd, t_frd) = self.place_frd(&e.r, e.t, fc_up);
        self.follow_heading(&r_frd, t_frd, fc_heading);
        let (r, t) = self.ned_of(&r_frd, t_frd)?;
        self.latest = Some((r, t));
        if !self.active(session_open) {
            return None;
        }
        if !self.pose_due(e.t_ns) {
            return None;
        }
        Some(PoseDescriptor {
            pose: Pose { r, t, cov: None },
            anchor: None,
            ts_ms: now_ms - (now_mono_ns.saturating_sub(e.t_ns) / 1_000_000) as i64,
            health: e.health,
            frame: NavFrame::MapNed,
            source: nav_source(e.source),
            north: self.north(),
            sigma_xy_m: e.sigma_xy_m,
            // Set by the capture loop, which knows the FC IMU feed.
            imu_feed: None,
        })
    }

    /// Whether the pose measured at `t_ns` is published, on the provider's
    /// own clock rather than the time a poll hands it over. Poses are due one
    /// interval apart on average; one up to half an interval early is
    /// published, so a provider at exactly the interval, whose measurement
    /// times jitter a few ms either side of it, loses none, and no two are
    /// published within half an interval. A time that goes backwards (the
    /// provider reset) publishes and restarts the schedule.
    fn pose_due(&mut self, t_ns: u64) -> bool {
        let interval_ns = self.settings.pose_interval_ms.max(0) as u64 * 1_000_000;
        let next_ns = match self.last_pub {
            None => t_ns + interval_ns,
            Some((last_ns, _)) if t_ns < last_ns => t_ns + interval_ns,
            Some((_, next_ns)) if t_ns + interval_ns / 2 >= next_ns => {
                // Kept on the schedule, re-anchored after a gap so the poses
                // that follow it are not let through in a burst.
                (next_ns + interval_ns).max(t_ns + interval_ns)
            }
            Some(_) => return false,
        };
        self.last_pub = Some((t_ns, next_ns));
        true
    }

    /// Fold one LiDAR scan: the body-frame scan and the placed polar record
    /// ([`Self::body_scan`], [`Self::place_scan`]) at once.
    pub fn on_scan(
        &mut self,
        scan: &LidarScan,
        t_body_sensor: &[f64; 16],
        world: Option<&PoseEstimate>,
        fc_up: Option<f64>,
        session_open: bool,
    ) -> ScanOut {
        ScanOut {
            scan: self.body_scan(scan, t_body_sensor, session_open),
            polar: self.place_scan(scan, t_body_sensor, world, fc_up, session_open),
        }
    }

    /// The binned body-frame scan when the lanes are on; it needs no pose.
    pub fn body_scan(
        &self,
        scan: &LidarScan,
        t_body_sensor: &[f64; 16],
        session_open: bool,
    ) -> Option<ScanDescriptor> {
        self.active(session_open)
            .then(|| frames::body_scan(scan, t_body_sensor))
    }

    /// Place one LiDAR scan. `world` is the provider's body pose at the scan
    /// time (FLU world), when known; `t_body_sensor` the mount. The polar
    /// record and the live map wait for the map's north.
    pub fn place_scan(
        &mut self,
        scan: &LidarScan,
        t_body_sensor: &[f64; 16],
        world: Option<&PoseEstimate>,
        fc_up: Option<f64>,
        session_open: bool,
    ) -> Option<PolarScan> {
        let e = world?;
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
        let (r_map, t_map) = self.ned_of(&r_frd, t_frd)?;
        let polar = frames::polar_scan(scan, scan.t_ns, &r_map, t_map, e.health);
        if self.active(session_open) {
            self.map.insert(&polar);
        }
        Some(polar)
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
                memory_used_mb: inputs.rss_mb,
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
