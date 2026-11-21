//! The 3D voxel map for the pilot: one thread owns an [`ados_voxmap::VoxMap`]
//! and publishes it over the voxel-map seam at 10 Hz of ADOS time.
//!
//! Inputs reach the thread over a bounded queue that drops its oldest entry
//! when full ([`queue`]), so the LiDAR path never waits for the map:
//!
//! - a 3D LiDAR scan, the whole cloud before the 360-bin flattening, thinned
//!   to the map's measured frame budget ([`feeder::CLOUD_POINTS_PER_SCAN`]);
//! - a planar LiDAR scan (with `navigation.voxmap.planar`), placed at its
//!   true 3D position from the sensor's attitude, only its returns within
//!   the flight band trusted;
//! - a depth camera frame, turned into body points through the camera mount
//!   by its own reader thread and placed with the body pose nearest its
//!   time.
//!
//! Each scan is placed with the sensor's pose at the scan time (the pose the
//! live map places it with). The published tier is the best one that fed
//! the map within the last second: 3D LiDAR, then depth, then planar.

pub mod config;
mod convert;
mod depth;
pub mod feeder;
pub mod queue;

#[cfg(test)]
mod tests;

use std::collections::VecDeque;
use std::path::PathBuf;
use std::sync::Arc;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use ados_mapping_pose::lidar_scan::LidarScan;
use ados_mapping_protocol::capture::{PolarScan, PoseDescriptor, VioHealth};
use ados_protocol::voxmap::{VoxmapWriter, VOXMAP_GROUP, VOXMAP_SHM_PATH};
use ados_voxmap::{Pose, VoxTier};
use parking_lot::Mutex;
use serde::Serialize;

use crate::depth_source::{DepthSettings, DepthSourceKind, DepthStatus};

pub use config::{Sensors, VoxWindow, VoxmapSettings};
use feeder::{Feeder, Input};
use queue::{Queue, QUEUE_DEPTH};

/// The map publishes once per this much ADOS time (10 Hz).
pub const PUBLISH_PERIOD_NS: u64 = 100_000_000;
/// The map thread's longest wait for an input, wall time.
const POLL: Duration = Duration::from_millis(10);
/// A seam file that could not be created is retried after this.
const WRITER_RETRY: Duration = Duration::from_secs(5);
/// Frame costs kept for the p95.
const COST_WINDOW: usize = 50;

/// The ADOS time source, nanoseconds.
pub type Now = Arc<dyn Fn() -> u64 + Send + Sync>;

/// How one feed runs.
pub struct FeedOptions {
    pub settings: VoxmapSettings,
    /// Planar scans feed the map.
    pub planar: bool,
    /// The depth camera; `source = off` runs no reader.
    pub depth: DepthSettings,
    /// The seam file and the group it is handed to.
    pub path: PathBuf,
    pub group: Option<String>,
    pub now: Now,
}

impl FeedOptions {
    /// The plugin's feed: the agent's seam file, ADOS time.
    pub fn for_run(settings: &VoxmapSettings, sensors: Sensors, depth: &DepthSettings) -> Self {
        let clock = ados_sdk::clock();
        Self {
            settings: settings.clone(),
            planar: sensors.planar && settings.planar,
            depth: depth.clone(),
            path: PathBuf::from(VOXMAP_SHM_PATH),
            group: Some(VOXMAP_GROUP.to_string()),
            now: Arc::new(move || clock.now_ns()),
        }
    }
}

/// The sensing tier behind the published map.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum TierName {
    #[default]
    None,
    Lidar3d,
    Depth,
    Camera,
    Planar,
}

impl From<VoxTier> for TierName {
    fn from(t: VoxTier) -> Self {
        match t {
            VoxTier::Lidar3d => TierName::Lidar3d,
            VoxTier::Depth => TierName::Depth,
            VoxTier::Camera => TierName::Camera,
            VoxTier::Planar => TierName::Planar,
            VoxTier::Unknown => TierName::None,
        }
    }
}

/// The map's block in `GET /navigation/status`.
#[derive(Debug, Clone, Default, PartialEq, Serialize)]
pub struct VoxmapHealth {
    pub enabled: bool,
    pub tier: TierName,
    /// Publications per second of ADOS time.
    pub rate_hz: f32,
    /// 95th percentile of a frame's work (its insertions, decay, distance
    /// field and publication) over the last 50 frames, ms.
    pub update_ms_p95: f32,
    /// Inputs dropped because the map thread was behind.
    pub drops: u64,
    pub published: u64,
    /// Depth frames left out for want of a body pose near their time.
    pub depth_unposed: u64,
    /// Why the map is not published, when it is not.
    pub error: Option<String>,
    pub depth: Option<DepthStatus>,
}

/// A body pose in the map frame at host monotonic `t_ns`.
#[derive(Debug, Clone, Copy)]
struct StampedPose {
    t_ns: u64,
    pose: Pose,
}

struct Shared {
    queue: Queue,
    health: Mutex<VoxmapHealth>,
    /// The two newest body poses, oldest first, for the depth reader.
    poses: Mutex<[Option<StampedPose>; 2]>,
    /// Point buffers the depth reader fills and the map thread hands back.
    spare: Mutex<Vec<Vec<[f32; 3]>>>,
    planar: bool,
}

/// The lane's side of the feed: cheap to clone, never blocks.
#[derive(Clone)]
pub struct VoxFeedTx(Arc<Shared>);

impl VoxFeedTx {
    /// Hand over one placed LiDAR scan: `polar` is the record the live map
    /// placed it with (its sensor pose at the scan time), `mount` the rig's
    /// body-from-sensor transform, `blocked` the rig's blocked sectors.
    pub fn lidar(
        &self,
        scan: LidarScan,
        polar: &PolarScan,
        mount: [f64; 16],
        blocked: Vec<[f64; 2]>,
        planar: bool,
    ) {
        if polar.health == VioHealth::Lost || (planar && !self.0.planar) {
            return;
        }
        self.0.queue.push(Input::Lidar {
            scan,
            sensor: Pose {
                t: polar.t,
                q: polar.q,
            },
            mount: convert::mount_pose(&mount),
            blocked,
            planar,
        });
    }

    /// The newest published body pose; a lost one places nothing.
    pub fn pose(&self, pd: &PoseDescriptor) {
        let mut poses = self.0.poses.lock();
        if pd.health == VioHealth::Lost {
            *poses = [None, None];
            return;
        }
        let p = StampedPose {
            t_ns: pd.t_ns,
            pose: convert::pose_of(&pd.pose.r, pd.pose.t),
        };
        *poses = [poses[1], Some(p)];
    }

    pub fn health(&self) -> VoxmapHealth {
        let mut h = self.0.health.lock().clone();
        h.drops = self.0.queue.drops();
        h
    }
}

/// The running feed; [`VoxFeed::stop`] ends its threads.
pub struct VoxFeed {
    tx: VoxFeedTx,
    threads: Vec<JoinHandle<()>>,
}

impl VoxFeed {
    pub fn start(opts: FeedOptions) -> Result<Self, ados_voxmap::ConfigError> {
        let feeder = Feeder::new(opts.settings.map_config(), opts.settings.depth_max_range_m)?;
        let shared = Arc::new(Shared {
            queue: Queue::new(QUEUE_DEPTH),
            health: Mutex::new(VoxmapHealth {
                enabled: true,
                ..VoxmapHealth::default()
            }),
            poses: Mutex::new([None, None]),
            spare: Mutex::new(Vec::new()),
            planar: opts.planar,
        });
        let mut threads = Vec::new();
        let worker = Worker {
            shared: shared.clone(),
            feeder,
            path: opts.path,
            group: opts.group,
            now: opts.now,
        };
        threads.push(spawn("voxmap", move || worker.run()));
        if opts.depth.source != DepthSourceKind::Off {
            let (s, d) = (shared.clone(), opts.depth);
            let max = opts.settings.depth_max_range_m;
            threads.push(spawn("voxmap-depth", move || depth::run(s, d, max)));
        }
        Ok(Self {
            tx: VoxFeedTx(shared),
            threads,
        })
    }

    pub fn tx(&self) -> VoxFeedTx {
        self.tx.clone()
    }

    /// Stop the threads and wait for them (up to a depth read's timeout).
    pub fn stop(mut self) {
        self.tx.0.queue.stop();
        for t in std::mem::take(&mut self.threads) {
            let _ = t.join();
        }
    }
}

impl Drop for VoxFeed {
    fn drop(&mut self) {
        self.tx.0.queue.stop();
    }
}

fn spawn(name: &str, f: impl FnOnce() + Send + 'static) -> JoinHandle<()> {
    std::thread::Builder::new()
        .name(name.into())
        .spawn(f)
        .expect("spawn the voxel-map thread")
}

struct Worker {
    shared: Arc<Shared>,
    feeder: Feeder,
    path: PathBuf,
    group: Option<String>,
    now: Now,
}

impl Worker {
    fn run(mut self) {
        let mut writer: Option<VoxmapWriter> = None;
        let mut retry_at = Instant::now();
        let mut error: Option<String> = None;
        let mut inbox = VecDeque::with_capacity(QUEUE_DEPTH);
        let mut costs: VecDeque<f32> = VecDeque::with_capacity(COST_WINDOW);
        let mut pending = Duration::ZERO;
        let mut next_due: Option<u64> = None;
        let (mut rate_from, mut rate_frames, mut rate_hz) = (None::<u64>, 0u32, 0.0f32);
        let mut published = 0u64;
        loop {
            let running = self.shared.queue.take(&mut inbox, POLL);
            if !running {
                break;
            }
            for input in inbox.drain(..) {
                let t0 = Instant::now();
                if let Some(mut buf) = self.feeder.ingest((self.now)(), input) {
                    buf.clear();
                    let mut spare = self.shared.spare.lock();
                    if spare.len() < QUEUE_DEPTH + 1 {
                        spare.push(buf);
                    }
                }
                pending += t0.elapsed();
            }
            if writer.is_none() && Instant::now() >= retry_at {
                let dims = self.feeder.map().dims();
                match VoxmapWriter::create(&self.path, dims, self.group.as_deref()) {
                    Ok(w) => {
                        tracing::info!(path = %self.path.display(), ?dims, "voxmap_publishing");
                        writer = Some(w);
                        error = None;
                    }
                    Err(e) => {
                        if error.is_none() {
                            tracing::warn!(path = %self.path.display(), error = %e, "voxmap_writer_unavailable");
                        }
                        error = Some(format!("seam: {e}"));
                        retry_at = Instant::now() + WRITER_RETRY;
                    }
                }
            }
            let now = (self.now)();
            if next_due.is_some_and(|d| now < d) {
                continue;
            }
            next_due = Some(match next_due {
                // On the schedule; re-anchored after a gap so frames do not
                // burst to catch up.
                Some(d) if now < d + PUBLISH_PERIOD_NS => d + PUBLISH_PERIOD_NS,
                _ => now + PUBLISH_PERIOD_NS,
            });
            let t0 = Instant::now();
            match self.feeder.frame(now, writer.as_mut()) {
                Ok(true) => {
                    published += 1;
                    rate_frames += 1;
                }
                Ok(false) => {}
                Err(e) => {
                    tracing::warn!(error = %e, "voxmap_publish_failed");
                    error = Some(format!("seam: {e}"));
                    writer = None;
                    retry_at = Instant::now() + WRITER_RETRY;
                }
            }
            if costs.len() == COST_WINDOW {
                costs.pop_front();
            }
            costs.push_back((pending + t0.elapsed()).as_secs_f32() * 1e3);
            pending = Duration::ZERO;
            let from = *rate_from.get_or_insert(now);
            if now.saturating_sub(from) >= 1_000_000_000 {
                rate_hz = rate_frames as f32 * 1e9 / (now - from) as f32;
                (rate_from, rate_frames) = (Some(now), 0);
            }
            let mut h = self.shared.health.lock();
            h.tier = self.feeder.tier(now).into();
            h.rate_hz = rate_hz;
            h.update_ms_p95 = p95(&costs);
            h.published = published;
            if h.error != error {
                h.error.clone_from(&error);
            }
        }
    }
}

/// The 95th percentile of `v` (0 when empty), sorted on the stack.
fn p95(v: &VecDeque<f32>) -> f32 {
    let mut buf = [0.0f32; COST_WINDOW];
    let n = v.len().min(COST_WINDOW);
    if n == 0 {
        return 0.0;
    }
    for (b, c) in buf.iter_mut().zip(v) {
        *b = *c;
    }
    let s = &mut buf[..n];
    s.sort_unstable_by(f32::total_cmp);
    s[((n - 1) as f32 * 0.95).round() as usize]
}
