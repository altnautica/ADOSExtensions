//! The `lidar-slam` service's lifecycle as a plugin process.
//!
//! [`supervise`] polls the `navigation.*` plugin config every
//! [`CONFIG_POLL`]. While the config resolves, one run is alive: the
//! sensor-bus ingest, the rig's LiDAR (when it declares one), the pose
//! provider the config and rig ask for, and the lane. A changed config
//! restarts the run; a changed rig document restarts it unless a capture
//! session is recording. An invalid config leaves nothing running and says
//! why on `GET /navigation/status`.
//!
//! Subscriptions that cannot be withdrawn (the FC messages, the capture
//! state, the frames VIO tracks on) are made once and routed to whichever
//! run is current.

use std::collections::HashSet;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use ados_mapping_pose::budget::{self, Budget};
use ados_mapping_pose::fc_ekf::FcEkfPose;
use ados_mapping_pose::frame_source::{FrameFeed, FrameRouter};
use ados_mapping_pose::lidar_scan::LidarScan;
use ados_mapping_pose::sensor_bus::{run_ingest, SensorState};
use ados_mapping_protocol::capture::{LidarStatus, PoseSource};
use ados_protocol::shutdown::Shutdown;
use ados_sdk::PluginContext;
use parking_lot::Mutex;
use tokio::sync::{mpsc, watch};
use tokio::task::JoinHandle;

use crate::capture_state::CaptureWatch;
use crate::lane::{run_lane, LaneDeps};
use crate::lidar_source::{LidarRunner, SCAN_CHANNEL_DEPTH};
use crate::nav::{NavSettings, TILES_DIR};
use crate::publish::Publisher;
use crate::settings::{NavConfig, CONFIG_KEYS};
use crate::status::Shared;
use crate::topics;

/// How often the plugin config is re-read.
pub const CONFIG_POLL: Duration = Duration::from_secs(5);

/// The FC messages the map and the planar matcher read. `ATTITUDE` is there
/// because the drone stream table carries it and not the quaternion outside
/// a capture session ([`ados_mapping_pose::mavlink_msg`]); `GPS_RAW_INT` says
/// when the EKF's position is GNSS-referenced ([`ados_mapping_pose::align`]);
/// `RAW_IMU` is the gyro the planar matcher's `fc` IMU integrates
/// ([`crate::nav::fc_imu`]); `OPTICAL_FLOW` over the downward
/// `DISTANCE_SENSOR` range is the ground velocity it follows along a corridor
/// ([`ados_mapping_pose::fc_flow`]); the autopilot's `HEARTBEAT` says whether
/// the vehicle is armed (a map restarted on the take-off spot starts over
/// only while it is not, [`crate::nav::NavMap::restart_in_flight`]).
const FC_MESSAGES: [&str; 9] = [
    "ATTITUDE",
    "ATTITUDE_QUATERNION",
    "LOCAL_POSITION_NED",
    "GPS_GLOBAL_ORIGIN",
    "GPS_RAW_INT",
    "RAW_IMU",
    "OPTICAL_FLOW",
    "DISTANCE_SENSOR",
    "HEARTBEAT",
];

/// The process-lifetime feeds a run attaches to.
#[derive(Clone)]
struct Feeds {
    router: Arc<FrameRouter>,
    fc: FcEkfPose,
    capture: CaptureWatch,
    out: Publisher,
    shared: Arc<Shared>,
}

/// The settings a run's map is built with.
pub fn nav_settings(cfg: &NavConfig) -> NavSettings {
    NavSettings {
        placement: cfg.map_placement,
        budget: budget_for(cfg),
        pose_interval_ms: cfg.pose_publish_interval_ms,
        takeoff_heading_rad: cfg.takeoff_heading_deg.map(f64::to_radians),
    }
}

fn budget_for(cfg: &NavConfig) -> Budget {
    budget::resolve(
        cfg.memory_budget_mb,
        std::env::var(budget::MEMORY_MAX_ENV).ok().as_deref(),
        std::path::Path::new("/"),
    )
}

/// One running navigation.
struct Run {
    config: NavConfig,
    rig_doc: Option<Vec<u8>>,
    cancel: Shutdown,
    tasks: Vec<JoinHandle<()>>,
    lane: JoinHandle<()>,
}

impl Run {
    async fn start(
        ctx: &PluginContext,
        config: NavConfig,
        profile: &str,
        feeds: &Feeds,
        frames_subscribed: &mut bool,
    ) -> Self {
        let rig_path = ados_mapping_pose::rig::rig_path();
        let rig_doc = std::fs::read(&rig_path).ok();
        let rig = ados_mapping_pose::rig::load_rig(&rig_path);
        let work_dir = ctx
            .data_dir
            .clone()
            .unwrap_or_else(std::env::temp_dir)
            .join("navigation");
        let pose = crate::providers::wanted(&config, profile, rig.as_ref()).and_then(|want| {
            crate::providers::start(want, &config, rig.as_ref(), &work_dir)
                .map_err(|why| tracing::warn!(wanted = ?want, reason = %why, "navigation_pose_provider_unavailable"))
                .ok()
        });
        let cancel = Shutdown::new();
        let (bus_tx, bus_rx) = mpsc::channel(1024);
        let mut tasks = vec![tokio::spawn(run_ingest(
            ados_sensorbus::socket_path(),
            Arc::new(Mutex::new(SensorState::default())),
            bus_tx,
            cancel.clone(),
        ))];
        let lidar_rx =
            crate::providers::lidar_runner(&config, rig.as_ref(), &work_dir).map(|runner| {
                let (scan_tx, scan_rx) = mpsc::channel(SCAN_CHANNEL_DEPTH);
                tasks.push(tokio::spawn(lidar_task(runner, scan_tx, feeds.clone())));
                scan_rx
            });
        let frames = match pose.as_ref().map(|p| p.source()) {
            Some(PoseSource::Vio) => {
                subscribe_frames(ctx, feeds, frames_subscribed).await;
                let ids = HashSet::from([config.vio.left.clone(), config.vio.right.clone()]);
                Some(FrameFeed::Engine(feeds.router.attach(ids)))
            }
            _ => None,
        };
        let lidar_planar = rig.as_ref().is_some_and(|r| {
            ados_mapping_pose::rig::lidar_sensor(r, &config.lidar.lidar_id)
                .is_some_and(|(_, m)| m.pattern == ados_capture_contract::LidarModelPattern::Planar)
        });
        feeds.shared.reset_run();
        let deps = LaneDeps {
            settings: nav_settings(&config),
            cfg: config.clone(),
            pose,
            fc: feeds.fc.clone(),
            rig,
            lidar_planar,
            frames,
            bus_rx,
            lidar_rx,
            out: feeds.out.clone(),
            capture: feeds.capture.clone(),
            shared: feeds.shared.clone(),
            scratch_dir: Some(work_dir.join("map").join(TILES_DIR)),
        };
        let lane = tokio::spawn(run_lane(deps, cancel.clone()));
        Self {
            config,
            rig_doc,
            cancel,
            tasks,
            lane,
        }
    }

    /// The rig document changed since the run started and no capture
    /// session is recording.
    fn rig_stale(&self, capture: &CaptureWatch) -> bool {
        let now = std::fs::read(ados_mapping_pose::rig::rig_path()).ok();
        !capture.recording(std::time::Instant::now()) && self.rig_doc != now
    }

    async fn stop(self, feeds: &Feeds) {
        self.cancel.trigger();
        if let Err(e) = self.lane.await {
            tracing::error!(error = %e, "navigation lane ended abnormally");
        }
        for t in self.tasks {
            t.abort();
            let _ = t.await;
        }
        feeds.router.detach();
    }
}

/// The run's LiDAR: the driver with its start retry, and its link status
/// published and kept for the status route as it is produced.
async fn lidar_task(runner: LidarRunner, scans: mpsc::Sender<LidarScan>, feeds: Feeds) {
    let (status_tx, mut status_rx) = watch::channel(LidarStatus::default());
    let publish = async {
        while status_rx.changed().await.is_ok() {
            let status = status_rx.borrow_and_update().clone();
            feeds.out.lidar(&status).await;
            feeds.shared.view.lock().lidar = Some(status);
        }
    };
    tokio::join!(runner.run(scans, status_tx), publish);
}

async fn subscribe_frames(ctx: &PluginContext, feeds: &Feeds, done: &mut bool) {
    if !*done {
        *done = ctx
            .vision
            .subscribe_frames(None, feeds.router.callback())
            .await
            .map_err(|e| tracing::warn!(error = %e, "navigation_frame_subscription_failed"))
            .is_ok();
    }
}

/// The FC messages and the capture state; each is retried on the next poll
/// until it succeeds.
async fn subscribe_feeds(ctx: &PluginContext, feeds: &Feeds, done: &mut [bool; 2]) {
    if !done[0] {
        let mut ok = true;
        for name in FC_MESSAGES {
            let fc = feeds.fc.clone();
            let cb: ados_sdk::EventCallback = Arc::new(move |args: rmpv::Value| {
                let frame = args.as_map().and_then(|m| {
                    m.iter()
                        .find(|(k, _)| k.as_str() == Some("frame"))
                        .and_then(|(_, v)| v.as_slice().map(<[u8]>::to_vec))
                });
                if let Some(frame) = frame {
                    fc.on_frame(&frame, ados_mapping_pose::clock::mono_ns());
                }
            });
            if let Err(e) = ctx.mavlink.subscribe(name, cb).await {
                tracing::warn!(msg = name, error = %e, "navigation_fc_subscription_failed");
                ok = false;
            }
        }
        done[0] = ok;
    }
    if !done[1] {
        done[1] = ctx
            .events
            .subscribe(topics::CAPTURE_STATE, feeds.capture.callback())
            .await
            .map_err(|e| tracing::info!(error = %e, "navigation_capture_state_unavailable"))
            .is_ok();
    }
}

/// Bind the plugin's HTTP socket and serve the routes until `cancel`; a host
/// that set no socket (no `agent.http`) serves none.
fn serve_http(
    ctx: &PluginContext,
    shared: Arc<Shared>,
    cancel: Shutdown,
) -> Option<JoinHandle<()>> {
    let path: PathBuf = ados_sdk::http::socket_path()?;
    if let Some(parent) = path.parent() {
        let _ = std::fs::create_dir_all(parent);
    }
    let listener = match ados_sdk::http::bind(&path) {
        Ok(l) => l,
        Err(e) => {
            tracing::error!(socket = %path.display(), error = %e, "navigation_http_bind_failed");
            return None;
        }
    };
    let router = crate::http::router(shared, Arc::new(crate::http::HostConfig(ctx.clone())));
    tracing::info!(socket = %path.display(), "serving the Navigation API");
    Some(tokio::spawn(async move {
        ados_mapping_transport::serve_unix(listener, router, async move { cancel.wait().await })
            .await
    }))
}

/// Run the service until `cancel` fires. See the module docs.
pub async fn supervise(ctx: PluginContext, cancel: Shutdown) {
    let (out, out_rx) = Publisher::new();
    let feeds = Feeds {
        router: FrameRouter::new(),
        fc: FcEkfPose::new(),
        capture: CaptureWatch::default(),
        out,
        shared: Arc::new(Shared::default()),
    };
    let forwarder = tokio::spawn(crate::publish::forward(ctx.clone(), out_rx, cancel.clone()));
    let http = serve_http(&ctx, feeds.shared.clone(), cancel.clone());
    let profile = match ctx.node.info().await {
        Ok(info) => info.profile,
        Err(e) => {
            tracing::warn!(error = %e, "node_info_unavailable_assuming_drone");
            "drone".to_string()
        }
    };
    let (mut subscribed, mut frames_subscribed) = ([false; 2], false);
    let mut run: Option<Run> = None;
    let mut poll = tokio::time::interval(CONFIG_POLL);
    poll.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tokio::select! {
            _ = cancel.wait() => break,
            _ = poll.tick() => {}
        }
        subscribe_feeds(&ctx, &feeds, &mut subscribed).await;
        if run.as_ref().is_some_and(|r| r.lane.is_finished()) {
            tracing::error!("navigation lane exited unexpectedly; restarting it");
            if let Some(r) = run.take() {
                r.stop(&feeds).await;
            }
        }
        let values = match ados_mapping_pose::config::read_values(&ctx, CONFIG_KEYS).await {
            Ok(v) => v,
            Err(e) => {
                tracing::warn!(error = %e, "navigation config read failed; keeping current state");
                continue;
            }
        };
        let desired = values.and_then(|v| NavConfig::from_values(&v));
        if let Some(r) = &run {
            let same = desired.as_ref().is_ok_and(|d| *d == r.config);
            if same && !r.rig_stale(&feeds.capture) {
                continue;
            }
        }
        if let Some(r) = run.take() {
            tracing::info!("navigation restarting for a changed config or rig");
            r.stop(&feeds).await;
        }
        match desired {
            Ok(config) => {
                run = Some(Run::start(&ctx, config, &profile, &feeds, &mut frames_subscribed).await)
            }
            Err(e) => {
                tracing::warn!(error = %e, "navigation config invalid; idle");
                feeds
                    .shared
                    .idle(&format!("navigation config invalid: {e}"));
            }
        }
    }
    if let Some(r) = run.take() {
        r.stop(&feeds).await;
    }
    for t in [Some(forwarder), http].into_iter().flatten() {
        t.abort();
        let _ = t.await;
    }
    tracing::info!("navigation lidar-slam service stopped");
}
