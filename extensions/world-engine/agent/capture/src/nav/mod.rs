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
mod restart_replay;
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
use crate::pose::fc_ekf::FcEkfPose;
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
    /// Where the first heading was taken (`map_frd`), the declared one or
    /// the FC's, while the vehicle is still there; `None` before it and once
    /// the vehicle has left.
    spot: Option<[f64; 3]>,
    /// Whether the vehicle has left the spot of the first heading.
    left_spot: bool,
    /// Whether the vehicle has left the take-off spot (that of the first
    /// heading the capture took): a declared take-off heading holds only
    /// there. Kept when the map starts over ([`Self::start_over`]).
    left_takeoff: bool,
    /// The north was latched from the FC heading although a take-off heading
    /// is declared: the map started over off the take-off spot.
    north_from_fc: bool,
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
    /// The last good pose ([`Trusted`]), preferred for placing a restarted
    /// map while the FC's attitude history still reaches back to it
    /// ([`Self::trusted_at`]).
    good: Option<Trusted>,
    /// The last pose not lost whose heading was measured
    /// ([`PoseEstimate::heading_ok`]), the fallback once the last good pose
    /// is older than the FC's attitude history: a degraded one in a
    /// featureless corridor keeps its measured heading, but a scan the IMU
    /// did not reach (a stall before the gap is noticed) has the matcher's
    /// guess on a stale prior for a heading and is never taken.
    measured: Option<Trusted>,
    /// A map restarted in flight is placed from the FC on each lost pose
    /// until a pose is not lost ([`Self::seat_while_lost`]).
    seating: bool,
    /// A map restarted in flight without the FC's turn since a trusted pose
    /// ([`Self::restart_in_flight`]): its north is unknown, so every pose
    /// of it is lost ([`Self::north_unknown`]). No pose of it is trusted,
    /// so a later restart finds the FC's turn only from the same old
    /// trusted pose: from one older than the FC's attitude history the map
    /// stays blind until the matcher's first-`RAW_IMU` restart
    /// ([`Self::restart_heading_at`]).
    blind: bool,
}

/// A pose a restarted map is placed from: its host ns, `map_ned` north,
/// east and heading (radians clockwise from north).
type Trusted = (u64, [f64; 2], f64);

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
pub(crate) fn wrap_pi(a: f64) -> f64 {
    let w = a.rem_euclid(std::f64::consts::TAU);
    if w > std::f64::consts::PI {
        w - std::f64::consts::TAU
    } else {
        w
    }
}

/// How far `t_frd` is from `spot` (both `map_frd`), horizontally and in
/// climb, metres, and whether that is still on the spot ([`ON_SPOT_M`]).
fn from_spot(spot: [f64; 3], t_frd: [f64; 3]) -> (f64, f64, bool) {
    let horizontal = (t_frd[0] - spot[0]).hypot(t_frd[1] - spot[1]);
    let climb = spot[2] - t_frd[2];
    (horizontal, climb, horizontal <= ON_SPOT_M && climb <= ON_SPOT_M)
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
            good: None,
            measured: None,
            seating: false,
            blind: false,
            left_takeoff: false,
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
        self.restarted_at = Some(t_ns);
        self.start_over();
    }

    /// The map, its north and its GNSS fit start over, as at the first pose;
    /// the first-`RAW_IMU` restart already taken stays taken, and so does a
    /// take-off spot the vehicle has left.
    fn start_over(&mut self) {
        *self = Self {
            restarted_at: self.restarted_at,
            left_takeoff: self.left_takeoff,
            ..Self::new(self.settings)
        };
    }

    /// One matcher pose `e` for placing a map the matcher started over in
    /// flight: `restarted` when the matcher's restart count rose since the
    /// last pose ([`crate::pose::PoseQuality::map_restarts`]), so
    /// [`Self::restart_in_flight`], else [`Self::seat_while_lost`].
    pub fn on_matcher_pose(&mut self, e: &PoseEstimate, restarted: bool, fc_up: Option<f64>, fc: Option<&FcEkfPose>) {
        if restarted {
            self.restart_in_flight(e, fc_up, fc);
        } else {
            self.seat_while_lost(e, fc_up, fc);
        }
    }

    /// The capture's tick: the matcher's poses `out` drained since the last
    /// tick and its restart count `map_restarts`, against `seen`, the count
    /// at the last tick (a new provider counts from zero again). The newest
    /// pose goes to [`Self::on_matcher_pose`].
    pub fn on_matcher_poses(
        &mut self,
        out: &[PoseEstimate],
        map_restarts: u32,
        seen: &mut u32,
        fc_up: Option<f64>,
        fc: Option<&FcEkfPose>,
    ) {
        if let Some(e) = out.last() {
            self.on_matcher_pose(e, map_restarts > *seen, fc_up, fc);
            *seen = map_restarts;
        }
    }

    /// The matcher started a new map in flight at pose `e` (a heading a gap
    /// in the IMU left unmeasured was not re-confirmed in time): its heading
    /// and position against the old map are the matcher's guess across the
    /// gap, tracked without a measured heading. The pose has been lost
    /// (external navigation not fused) since the gap and stays lost until
    /// the new map holds enough scans, so the flight controller (`fc`) ran
    /// on its other sources meanwhile, and the new map is placed from it:
    /// - its heading at `e` is [`Self::outage_heading`], latched once: the
    ///   old map's north carries over, never the FC's own;
    /// - `e` is placed at [`Self::outage_seed`]; without it, where the old
    ///   map placed `e`. While the pose stays lost (the new map's warm-up)
    ///   each later lost pose is placed there again
    ///   ([`Self::seat_while_lost`]): the new map's first scans track the
    ///   drone poorly, and only a pose that is not lost ends the placing.
    ///
    /// The GNSS fit starts over (it fitted the old map). Without the FC's
    /// turn since the trusted pose (no FC attitude, or none that old) the
    /// new map's north is unknown: it stays lost, never fused on the
    /// matcher's guess through the outage. Without the FC's position at
    /// both ends the pose keeps the old map's blind position. Both warn, as
    /// does a turn bridged across a gap in the FC's attitudes.
    ///
    /// With no pose trusted since the map last started (on the ground before
    /// its first, or a LiDAR that never matched), the old map has no north
    /// of its own to carry over: the map starts over as at the first
    /// `RAW_IMU` ([`Self::restart_heading_at`]), and the next pose latches
    /// the north afresh: on the take-off spot as at the first pose, off it
    /// from the FC heading even with a declared take-off heading, which holds
    /// only on that spot.
    pub fn restart_in_flight(&mut self, e: &PoseEstimate, fc_up: Option<f64>, fc: Option<&FcEkfPose>) {
        let (r_frd, t_frd) = self.place_frd(&e.r, e.t, fc_up);
        if self.good.is_none() && self.measured.is_none() {
            // The restart pose may have left the spot since the last pose.
            if self.spot.is_some_and(|spot| !from_spot(spot, t_frd).2) {
                self.left_takeoff = true;
            }
            tracing::warn!(
                left_takeoff = self.left_takeoff,
                "nav_map_restarted_before_a_trusted_pose"
            );
            self.start_over();
            return;
        }
        let Some((heading, bridged)) = fc.and_then(|fc| self.outage_heading(e.t_ns, fc)) else {
            tracing::warn!("nav_map_restart_without_fc_heading");
            self.blind = true;
            self.seating = false;
            self.align = MapAlign::default();
            self.logged_yaw = None;
            return;
        };
        self.blind = false;
        if bridged {
            tracing::warn!("nav_map_restart_fc_turn_bridged");
        }
        let seed = fc.and_then(|fc| self.outage_seed(e.t_ns, fc));
        if seed.is_none() {
            tracing::warn!("nav_map_restart_without_fc_position");
        }
        let old = self.ned_of(&r_frd, t_frd).map(|(_, t)| [t[0], t[1]]);
        let yaw0 = wrap_pi(heading - frd_yaw(&r_frd));
        let placed = se3::mat_vec(&rz(yaw0), t_frd);
        self.offset = seed
            .or(old)
            .map_or([0.0; 2], |t| [t[0] - placed[0], t[1] - placed[1]]);
        self.yaw0 = Some(yaw0);
        self.spot = None;
        self.left_spot = true;
        self.align = MapAlign::default();
        self.logged_yaw = None;
        tracing::info!(
            heading_rad = heading,
            yaw0_rad = yaw0,
            offset_n_m = self.offset[0],
            offset_e_m = self.offset[1],
            seeded = seed.is_some(),
            "nav_map_north_restarted"
        );
        self.seating = seed.is_some();
    }

    /// After [`Self::restart_in_flight`], while the pose `e` is still lost,
    /// place it at [`Self::outage_seed`] again (the north stays latched).
    /// Nothing once a pose was not lost, or without the FC's position.
    pub fn seat_while_lost(&mut self, e: &PoseEstimate, fc_up: Option<f64>, fc: Option<&FcEkfPose>) {
        if !self.seating || e.health != VioHealth::Lost {
            return;
        }
        let (Some(yaw0), Some(seed)) = (self.yaw0, fc.and_then(|fc| self.outage_seed(e.t_ns, fc))) else {
            return;
        };
        let (_, t_frd) = self.place_frd(&e.r, e.t, fc_up);
        let placed = se3::mat_vec(&rz(yaw0), t_frd);
        self.offset = [seed[0] - placed[0], seed[1] - placed[1]];
    }

    /// Where a pose at host `t_ns` is in `map_ned` (north, east) after an
    /// outage of the pose, for placing a map started over for any reason:
    /// the trusted pose ([`Self::trusted_at`]), moved by the FC's own position change
    /// (`LOCAL_POSITION_NED`, `fc`) from that pose's time to `t_ns`, turned
    /// from the FC's frame into the map's by the FC's heading against the
    /// map's then ([`Self::outage_heading`] less [`FcEkfPose::heading_at`]:
    /// the FC's north is degrees off the map's, more after an EKF yaw
    /// reset). Only the change over the outage is taken, never the FC's
    /// position: while the pose was lost the FC fused no external navigation
    /// and ran on its other sources. `None` before a trusted pose,
    /// or without the FC's position at both ends.
    pub fn outage_seed(&self, t_ns: u64, fc: &FcEkfPose) -> Option<[f64; 2]> {
        let (t, ne, _) = self.trusted_at(t_ns, fc)?;
        let (then, now) = (fc.ned_at(t)?, fc.ned_at(t_ns)?);
        let (n, e) = (now[0] - then[0], now[1] - then[1]);
        let off = match (self.outage_heading(t_ns, fc), fc.heading_at(t_ns)) {
            (Some((map, _)), Some(own)) => map - own,
            _ => 0.0,
        };
        let (s, c) = off.sin_cos();
        Some([ne[0] + c * n - s * e, ne[1] + s * n + c * e])
    }

    /// The heading in `map_ned` (radians clockwise from north) of a pose at
    /// host `t_ns` after an outage of the pose, as [`Self::outage_seed`]:
    /// the trusted pose's ([`Self::trusted_at`]), turned by the FC's turn from that
    /// pose's time to `t_ns` ([`FcEkfPose::turn_between`]: its heading rate
    /// integrated, so to the pose's own time and through an EKF yaw reset),
    /// and whether a gap in the FC's attitudes was bridged by its yaw's own
    /// change. The FC's own heading may sit degrees off the map's north and
    /// is never taken. `None` before a trusted pose, or without
    /// the FC's attitude at both ends.
    pub fn outage_heading(&self, t_ns: u64, fc: &FcEkfPose) -> Option<(f64, bool)> {
        let (t, _, yaw) = self.trusted_at(t_ns, fc)?;
        let (turn, bridged) = fc.turn_between(t, t_ns)?;
        Some((wrap_pi(yaw + turn), bridged))
    }

    /// The pose a map restarted at host `t_ns` is placed from: the last
    /// good pose while the FC's attitude history reaches back to it, else
    /// the last pose not lost with a measured heading (a degenerate
    /// corridor can hold no good pose for longer than that history).
    fn trusted_at(&self, t_ns: u64, fc: &FcEkfPose) -> Option<Trusted> {
        self.good
            .filter(|g| fc.turn_between(g.0, t_ns).is_some())
            .or(self.measured)
    }

    /// The map was restarted in flight without the FC's turn since a
    /// trusted pose: its north is unknown and every pose of it is lost.
    pub fn north_unknown(&self) -> bool {
        self.blind
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
        if self.blind || e.health != VioHealth::Good {
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
    /// heading when there is one and the vehicle has not left the take-off
    /// spot. Otherwise take it from the FC heading while the vehicle is on
    /// the spot of the first heading. Either is fixed once the vehicle has
    /// moved [`ON_SPOT_M`] from there horizontally or in climb.
    fn follow_heading(&mut self, r_frd: &[f64; 9], t_frd: [f64; 3], fc_heading: Option<f64>) {
        if self.left_spot {
            return;
        }
        if let Some(spot) = self.spot {
            let (horizontal, climb, on_spot) = from_spot(spot, t_frd);
            if !on_spot {
                self.left_spot = true;
                self.left_takeoff = true;
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
        if let Some(declared) = self.settings.takeoff_heading_rad.filter(|_| !self.left_takeoff) {
            if self.yaw0.is_none() {
                let yaw0 = wrap_pi(declared - frd_yaw(r_frd));
                self.yaw0 = Some(yaw0);
                self.spot = Some(t_frd);
                tracing::info!(
                    heading_rad = declared,
                    fc_heading_rad = fc_heading,
                    yaw0_rad = yaw0,
                    "nav_map_north_declared"
                );
            }
            return;
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
            self.north_from_fc = self.settings.takeoff_heading_rad.is_some();
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
        let health = if self.blind { VioHealth::Lost } else { e.health };
        if health != VioHealth::Lost && e.heading_ok {
            let at = Some((e.t_ns, [t[0], t[1]], frd_yaw(&r)));
            self.measured = at;
            if health == VioHealth::Good {
                self.good = at;
            }
        }
        if health != VioHealth::Lost {
            self.seating = false;
        }
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
            health,
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

    /// Fold one LiDAR scan: the body-frame scan (the default
    /// [`frames::ScanFilter`]) and the placed polar record
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
            scan: self.body_scan(scan, t_body_sensor, &frames::ScanFilter::default(), session_open),
            polar: self.place_scan(scan, t_body_sensor, world, fc_up, session_open),
        }
    }

    /// The binned body-frame scan when the lanes are on; it needs no pose.
    pub fn body_scan(
        &self,
        scan: &LidarScan,
        t_body_sensor: &[f64; 16],
        filter: &frames::ScanFilter,
        session_open: bool,
    ) -> Option<ScanDescriptor> {
        self.active(session_open)
            .then(|| frames::body_scan(scan, t_body_sensor, filter))
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
        let health = if self.blind { VioHealth::Lost } else { e.health };
        let polar = frames::polar_scan(scan, scan.t_ns, &r_map, t_map, health);
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
