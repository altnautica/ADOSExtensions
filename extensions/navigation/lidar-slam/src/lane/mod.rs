//! One navigation run: the pose provider fed from the sensor bus, the
//! frames, the flight controller and the LiDAR; the navigation map built
//! from its poses; and every `plugin.navigation.*` topic.
//!
//! - every [`NAV_POLL`]: the FC attitude stand-in while there is no
//!   `RAW_IMU`, the newest pose when its interval is due, and the scans the
//!   new poses settled;
//! - every [`TRACK_EVERY`]: every estimate since the last batch
//!   (`pose_track`);
//! - every [`SLOW_EVERY`]: the changed occupancy tiles, the map status, the
//!   sweep history `GET /navigation/sweep` serves, and the status block;
//! - per LiDAR scan: the binned body-frame scan, the raw scan while a capture
//!   session records (`scan_raw`), and once its pose is settled the placed
//!   scan into the map (and `scan_polar` while the map is built on a node).

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use ados_capture_contract::Rig;
use ados_mapping_pose::fc_ekf::FcEkfPose;
use ados_mapping_pose::frame_source::{CapturedFrame, FrameFeed};
use ados_mapping_pose::lidar_scan::LidarScan;
use ados_mapping_pose::sensor_bus::BusEvent;
use ados_mapping_pose::{se3, PoseEstimate, PoseProvider};
use ados_mapping_protocol::capture::{
    chunk_scan, pack_points, ImuFeedReason, PoseDescriptor, PoseTrackBatch, PoseTrackEntry,
};
use ados_protocol::shutdown::Shutdown;
use tokio::sync::{mpsc, oneshot};

use crate::capture_state::CaptureWatch;
use crate::nav::{fc_imu, scan_wait::ScanWait, NavMap, NavSettings, TickInputs};
use crate::publish::Publisher;
use crate::settings::NavConfig;
use crate::status::Shared;

/// The pose lane's poll, independent of `navigation.pose_publish_interval_ms`
/// (which only thins the poses, on their own measurement time): a pose waits
/// at most one poll for publication, and the flight controller fuses it that
/// much older.
pub const NAV_POLL: Duration = Duration::from_millis(10);
/// How often the pose track is batched.
pub const TRACK_EVERY: Duration = Duration::from_millis(100);
/// The live-map tick.
pub const SLOW_EVERY: Duration = Duration::from_secs(1);

/// A request for the whole sweep cloud (packed triples, voxel edge, seq).
pub type WholeSweep = oneshot::Sender<(Vec<u8>, f32, u64)>;

/// Everything a run needs.
pub struct LaneDeps {
    pub cfg: NavConfig,
    pub settings: NavSettings,
    /// The pose provider; `None` runs the LiDAR lanes without a pose.
    pub pose: Option<Arc<dyn PoseProvider>>,
    /// The flight controller's EKF feed: the map's z is its height, its
    /// heading the map's north until the GNSS fit.
    pub fc: FcEkfPose,
    pub rig: Option<Rig>,
    /// Whether the configured LiDAR is planar (the FC can be its IMU).
    pub lidar_planar: bool,
    /// The frames VIO tracks on; `None` without VIO.
    pub frames: Option<FrameFeed>,
    pub bus_rx: mpsc::Receiver<BusEvent>,
    pub lidar_rx: Option<mpsc::Receiver<LidarScan>>,
    pub out: Publisher,
    pub capture: CaptureWatch,
    pub shared: Arc<Shared>,
    /// Where evicted occupancy tiles go.
    pub scratch_dir: Option<PathBuf>,
}

pub(crate) struct Lane {
    cfg: NavConfig,
    pose: Option<Arc<dyn PoseProvider>>,
    fc: FcEkfPose,
    rig: Option<Rig>,
    lidar_planar: bool,
    out: Publisher,
    capture: CaptureWatch,
    shared: Arc<Shared>,
    scratch_dir: Option<PathBuf>,
    pub(crate) nav: NavMap,
    /// LiDAR scans waiting for their pose before they are placed.
    scan_wait: ScanWait,
    /// The last FC attitude turned into an IMU sample (before `RAW_IMU`).
    fc_imu_prev: Option<(u64, [f64; 9])>,
    /// What the FC feed has handed the pose provider.
    fc_nav: Arc<fc_imu::FeedState>,
    /// Why the FC IMU feed last left the planar pose degraded.
    imu_feed_degraded: Option<ImuFeedReason>,
    /// The planar matcher's in-flight map restarts the map has re-anchored.
    nav_map_restarts: u32,
    nav_feed: NavFeedStats,
    /// Scans seen, numbering `scan_raw`.
    scan_seq: u64,
    /// Estimates since the last `pose_track` batch.
    track: Vec<PoseTrackEntry>,
}

impl Lane {
    pub(crate) fn new(deps: &mut LaneDeps) -> Self {
        Self {
            cfg: deps.cfg.clone(),
            pose: deps.pose.clone(),
            fc: deps.fc.clone(),
            rig: deps.rig.clone(),
            lidar_planar: deps.lidar_planar,
            out: deps.out.clone(),
            capture: deps.capture.clone(),
            shared: deps.shared.clone(),
            scratch_dir: deps.scratch_dir.clone(),
            nav: NavMap::new(deps.settings),
            scan_wait: ScanWait::default(),
            fc_imu_prev: None,
            fc_nav: Default::default(),
            imu_feed_degraded: None,
            nav_map_restarts: 0,
            nav_feed: NavFeedStats::default(),
            scan_seq: 0,
            track: Vec::new(),
        }
    }

    /// The flight controller's up height (its EKF world is ENU), metres.
    fn fc_up(&self) -> Option<f64> {
        self.fc.track().latest().map(|e| e.t[2])
    }

    /// The flight controller's attitude yaw, radians clockwise from north.
    fn fc_heading(&self) -> Option<f64> {
        self.fc.attitude().map(|(_, r)| se3::frd_yaw(&r))
    }

    /// Pair each new estimate with the FC's EKF position at the same instant
    /// for the map's GNSS alignment while that position was GNSS-referenced,
    /// and freeze the alignment at an estimate where it was not.
    fn fold_gnss_alignment(&mut self, poses: &[PoseEstimate]) {
        for e in poses {
            let Some(ekf) = self.fc.ned_at(e.t_ns) else {
                continue;
            };
            if self.fc.gnss_steady_at(e.t_ns) {
                self.nav.on_gnss_ref(e, ekf);
            } else {
                self.nav.on_gnss_lost();
            }
        }
    }

    /// The rig LiDAR's blocked sectors, which the shared scan lists as
    /// `blocked_bins` while keeping every range raw.
    fn blocked_azimuth_deg(&self, sensor: &str) -> Vec<[f64; 2]> {
        self.rig
            .as_ref()
            .and_then(|r| ados_mapping_pose::rig::lidar_sensor(r, sensor))
            .map(|(_, m)| m.blocked_azimuth_deg.clone())
            .unwrap_or_default()
    }

    fn lidar_mount(&self, sensor: &str) -> [f64; 16] {
        self.rig
            .as_ref()
            .and_then(|r| r.sensors.iter().find(|s| s.id == sensor))
            .map(|s| s.t_body_sensor.0)
            .unwrap_or_else(|| se3::to_mat4(&se3::IDENTITY, [0.0; 3]))
    }

    /// One LiDAR scan: the pose provider, the body-frame scan and the raw
    /// stream now; the map once its pose is known ([`Self::place_scans`]).
    pub(crate) async fn on_lidar(&mut self, scan: LidarScan) {
        self.feed_fc_attitude();
        if self.fc_feeds_lidar_pose() {
            let end = scan.t_ns + u64::from(scan.t_offset_ns.iter().copied().max().unwrap_or(0));
            self.nav_feed.scans += 1;
            if let Some(n) = self.fc_nav.newest_imu_ns() {
                self.nav_feed.gaps_ns.push(end as i64 - n as i64);
            }
        }
        if let Some(p) = &self.pose {
            p.feed_lidar(&scan);
        }
        let mount = self.lidar_mount(&scan.sensor_id);
        let blocked = self.blocked_azimuth_deg(&scan.sensor_id);
        let body = self.nav.body_scan(&scan, &mount, &blocked);
        self.out.scan(&body).await;
        self.scan_seq += 1;
        if self.capture.recording(std::time::Instant::now()) {
            let packed = pack_points(&scan.points, &scan.intensity, &scan.t_offset_ns);
            for c in chunk_scan(&scan.sensor_id, scan.t_ns, self.scan_seq, &packed) {
                self.out.scan_raw(&c).await;
            }
        }
        if self.pose.is_none() {
            return;
        }
        if let Some(oldest) = self.scan_wait.push(scan) {
            self.place_scan(oldest, None).await;
        }
        self.place_scans().await;
    }

    /// Place every waiting scan whose pose is settled, oldest first.
    async fn place_scans(&mut self) {
        let Some(pose) = self.pose.clone() else {
            return;
        };
        loop {
            let now = ados_mapping_pose::clock::mono_ns();
            let newest = pose.latest().map(|e| e.t_ns);
            let Some((scan, world)) = self.scan_wait.next(now, newest, |t| pose.pose_at(t)) else {
                return;
            };
            self.place_scan(scan, world).await;
        }
    }

    async fn place_scan(&mut self, scan: LidarScan, world: Option<PoseEstimate>) {
        let mount = self.lidar_mount(&scan.sensor_id);
        let fc_up = self.fc_up();
        let polar = self.nav.place_scan(&scan, &mount, world.as_ref(), fc_up);
        // While streaming, a node builds the full-resolution live map.
        if let (true, Some(polar)) = (self.nav.streams(), &polar) {
            self.out.scan_polar(polar).await;
        }
    }

    /// The pose lane's poll ([`NAV_POLL`]).
    pub(crate) async fn on_nav_tick(&mut self) {
        self.feed_fc_attitude();
        if let Some(pd) = self.drain_poses() {
            self.out.pose(&pd).await;
        }
        self.place_scans().await;
    }

    /// Fold every new estimate into the map and the pose track; the pose to
    /// publish when its interval is due.
    pub(crate) fn drain_poses(&mut self) -> Option<PoseDescriptor> {
        let pose = self.pose.clone()?;
        let mut out = Vec::new();
        pose.drain_new(&mut out);
        let q = pose.quality();
        let now_mono_ns = ados_mapping_pose::clock::mono_ns();
        let now_ms = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_millis() as i64);
        self.degrade_on_imu_feed(&mut out, now_mono_ns);
        let (fc_up, heading) = (self.fc_up(), self.fc_heading());
        let fc = self.fc.clone();
        self.nav.on_matcher_poses(
            &out,
            q.map_restarts,
            &mut self.nav_map_restarts,
            fc_up,
            Some(&fc),
        );
        self.fold_gnss_alignment(&out);
        let imu_feed = self
            .imu_feed_degraded
            .or(self.nav_imu_reason(q.heading_unconfirmed));
        let due = out
            .last()
            .and_then(|e| self.nav.on_pose(e, fc_up, heading, now_ms, now_mono_ns))
            .map(|pd| PoseDescriptor { imu_feed, ..pd });
        self.track.extend(out.iter().map(|e| PoseTrackEntry {
            t_ns: e.t_ns,
            r: e.r,
            t: e.t,
            cov: e.cov.to_vec(),
            health: e.health,
            sigma_xy_m: e.sigma_xy_m,
            heading_ok: e.heading_ok,
        }));
        due
    }

    /// Every estimate since the last batch, oldest first.
    pub(crate) async fn on_track_tick(&mut self) {
        let Some(pose) = &self.pose else {
            return;
        };
        if self.track.is_empty() {
            return;
        }
        let batch = PoseTrackBatch {
            source: pose.source(),
            estimates: std::mem::take(&mut self.track),
        };
        self.out.pose_track(&batch).await;
    }

    /// The live-map tick: changed tiles, the map status, the sweep history
    /// and the status block.
    pub(crate) async fn on_slow_tick(&mut self) {
        self.log_nav_feed();
        let inputs = TickInputs {
            evict_dir: self.scratch_dir.clone(),
            rss_mb: crate::nav::read_vm_rss_mb(),
            pose_provider: self.pose.as_ref().map(|p| p.source()),
        };
        let mut out = self.nav.tick(&inputs);
        out.status.imu_feed = self.imu_feed_status();
        if let Some(o) = &out.occupancy {
            self.out.occupancy(o).await;
        }
        if let Some(c) = &out.pointcloud {
            self.shared.sweep.lock().push(c);
        }
        self.out.map_status(&out.status).await;
        let latest = self.pose.as_ref().and_then(|p| p.latest());
        let now = ados_mapping_pose::clock::mono_ns();
        let mut view = self.shared.view.lock();
        view.provider = out.status.pose_provider;
        view.health = self.pose.as_ref().map(|p| p.health());
        view.pose_age_ms = latest.map(|e| now.saturating_sub(e.t_ns) / 1_000_000);
        view.map = Some(out.status);
    }

    fn on_bus(&self, ev: BusEvent) {
        if let (BusEvent::Imu { src, batch }, Some(p)) = (ev, &self.pose) {
            p.feed_imu(&src, &batch);
        }
    }

    fn on_frame(&self, f: CapturedFrame) {
        if let Some(p) = &self.pose {
            p.feed_frame(&f);
        }
    }

    fn whole_sweep(&self, reply: WholeSweep) {
        let (points, voxel_m) = self.nav.sweep_points();
        let _ = reply.send((points, voxel_m, self.shared.sweep.lock().seq()));
    }
}

/// Run one navigation run until `cancel`.
pub async fn run_lane(mut deps: LaneDeps, cancel: Shutdown) {
    let mut lane = Lane::new(&mut deps);
    lane.attach_fc_nav();
    tracing::info!(
        provider = ?lane.pose.as_ref().map(|p| p.source()),
        placement = ?deps.settings.placement,
        budget_mb = deps.settings.budget.total_mb(),
        "navigation_lane_started"
    );
    // Frames are pulled on their own task so a source read is never
    // cancelled mid-frame by the select below.
    let (frame_tx, mut frame_rx) = mpsc::channel::<CapturedFrame>(8);
    let reader = deps.frames.take().map(|mut frames| {
        let cancel = cancel.clone();
        tokio::spawn(async move {
            loop {
                let next = tokio::select! {
                    _ = cancel.wait() => return,
                    f = frames.next() => f,
                };
                match next {
                    Some(f) => {
                        if frame_tx.send(f).await.is_err() {
                            return;
                        }
                    }
                    None => tokio::select! {
                        _ = cancel.wait() => return,
                        _ = tokio::time::sleep(Duration::from_millis(50)) => {}
                    },
                }
            }
        })
    });
    let (whole_tx, mut whole_rx) = mpsc::channel::<WholeSweep>(4);
    *deps.shared.whole.lock() = Some(whole_tx);
    let mut nav_tick = tokio::time::interval(NAV_POLL);
    nav_tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut track_tick = tokio::time::interval(TRACK_EVERY);
    track_tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut slow_tick = tokio::time::interval(SLOW_EVERY);
    slow_tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tokio::select! {
            biased;
            _ = cancel.wait() => break,
            _ = nav_tick.tick() => lane.on_nav_tick().await,
            _ = track_tick.tick() => lane.on_track_tick().await,
            _ = slow_tick.tick() => lane.on_slow_tick().await,
            Some(req) = whole_rx.recv() => lane.whole_sweep(req),
            Some(ev) = deps.bus_rx.recv() => lane.on_bus(ev),
            Some(scan) = async { match deps.lidar_rx.as_mut() { Some(rx) => rx.recv().await, None => None } } => {
                lane.on_lidar(scan).await;
            }
            Some(f) = frame_rx.recv() => lane.on_frame(f),
        }
    }
    if let Some(r) = reader {
        r.abort();
    }
    *deps.shared.whole.lock() = None;
    lane.detach_fc_nav();
    tracing::info!("navigation_lane_stopped");
}

mod fc_feed;
#[cfg(test)]
mod tests;

use fc_feed::NavFeedStats;
