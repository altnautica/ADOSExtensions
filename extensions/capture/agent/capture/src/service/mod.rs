//! The capture loop: the single owner of the open session.
//!
//! It selects over operator control (socket and front-panel buttons), camera
//! frames, sensor-bus events and a 10 Hz tick, and for each:
//!
//! - feeds the pose provider (frames, IMU);
//! - while recording, converts to the session timebase and hands the
//!   recorder every frame (trigger-joined), IMU batch, trigger, GNSS byte
//!   stream and fix, LiDAR scan, clock sample, pose estimate and a 1 Hz
//!   health record;
//! - measures quality (blur, texture, flicker per frame; coverage per posed
//!   frame) for the HUD;
//! - runs the keyframe selector as a consumer of the same stream, publishing
//!   keyframes on the private bus and writing their previews under
//!   `keyframes/`.

mod control;
mod frame;
mod tick;
mod view;

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use ados_capture_contract::{Manifest, Rig, Timebase};
use ados_mapping_protocol::capture_ctl::{CaptureStateView, CtlReply, ExposureView};
use ados_protocol::shutdown::Shutdown;
use parking_lot::Mutex;
use tokio::sync::{mpsc, oneshot, watch};
use tokio::task::JoinHandle;

use crate::camera_control::CameraControl;
use crate::control::ControlMsg;
use crate::hud::HudAction;
use crate::navigation_track::NavigationTrack;
use crate::publish::CapturePublisher;
use crate::quality::coverage::Coverage;
use crate::quality::lidar_depth::LidarDepth;
use crate::quality::QualityMonitor;
use crate::recording::ActiveSession;
use crate::runtime::{CaptureRuntimeConfig, PoseSourceChoice, SharedSteps};
use crate::scan_raw::ScanRaw;
use crate::session::CaptureSession;
use ados_mapping_pose::fc_ekf::FcEkfPose;
use ados_mapping_pose::frame_source::{CapturedFrame, FrameFeed, FrameRouter};
use ados_mapping_pose::lidar_scan::LidarScan;
use ados_mapping_pose::pose::PoseProvider;
use ados_mapping_pose::sensor_bus::{BusEvent, SensorState};
use ados_mapping_pose::vehicle::VehicleFeed;

/// The loop's tick: pose drain and state refresh.
pub const TICK: Duration = Duration::from_millis(100);
/// Ticks between health records, HUD pages and capture-state publishes.
const SLOW_TICKS: u32 = 10;
/// The keyframe queue's share of the memory budget, percent.
pub const KEYFRAME_SHARE: u64 = 80;
/// The LiDAR scan queue's share of the memory budget, percent.
pub const SCAN_SHARE: u64 = 20;

/// Where the loop reads and writes outside its own state.
#[derive(Debug, Clone)]
pub struct Paths {
    pub storage_sidecar: PathBuf,
    pub clock_sidecar: PathBuf,
    pub rig: PathBuf,
    pub active_marker: Option<PathBuf>,
}

impl Paths {
    pub fn from_env() -> Self {
        Self {
            storage_sidecar: crate::storage::sidecar_path(),
            clock_sidecar: ados_mapping_pose::clock::clock_sidecar_path(),
            rig: ados_mapping_pose::rig::rig_path(),
            active_marker: Some(crate::storage::active_marker()),
        }
    }
}

/// Everything the loop runs with.
pub struct LoopDeps {
    pub cfg: CaptureRuntimeConfig,
    /// The node profile (`drone` or `visor`).
    pub profile: String,
    pub frames: FrameFeed,
    /// The engine router, for its per-camera drop counts.
    pub router: Option<Arc<FrameRouter>>,
    pub pose: Arc<dyn PoseProvider>,
    /// The FC EKF provider when it is the active one (it takes the anchor).
    pub fc: Option<FcEkfPose>,
    pub vehicle: VehicleFeed,
    pub sensors: Arc<Mutex<SensorState>>,
    pub bus_rx: mpsc::Receiver<BusEvent>,
    pub publisher: CapturePublisher,
    pub control_rx: mpsc::Receiver<ControlMsg>,
    pub hud_rx: Option<mpsc::Receiver<HudAction>>,
    /// The HUD task renders the latest state from here.
    pub view_tx: Option<watch::Sender<CaptureStateView>>,
    /// The host's camera control the exposure lock is applied through;
    /// `None` where there is none (the lock is then never applied).
    pub camera: Option<Arc<dyn CameraControl>>,
    pub paths: Paths,
    /// The live `capture.reconstruct_steps`, stamped on every capture status.
    pub reconstruct_steps: SharedSteps,
    /// What Navigation streams: its pose and the scans to record.
    pub navigation: NavInputs,
}

/// What Navigation streams into a run; empty without Navigation.
#[derive(Default)]
pub struct NavInputs {
    /// Navigation's pose, a session's pose source by `capture.pose_source`.
    pub track: Option<Arc<NavigationTrack>>,
    /// Whole scans to record, and the feed that counts incomplete ones.
    pub scans: Option<(Arc<ScanRaw>, mpsc::Receiver<LidarScan>)>,
}

/// How recent a Navigation batch must be for `auto` to pose a session on it.
const AUTO_NAV_FRESH: Duration = Duration::from_secs(2);

/// Per-camera frame accounting.
#[derive(Debug, Default, Clone)]
struct CamStats {
    width: u32,
    height: u32,
    last_ts_ns: Option<u64>,
    fps: f64,
    frames: u64,
    dropped: u64,
    /// Host ns of this camera's last keyframe (its IMU window starts there).
    last_kf_ns: Option<u64>,
    /// The roster geometry the camera's latest frame carried.
    geometry: ados_mapping_pose::frame_source::CameraGeometry,
}

/// A finished exposure apply, back on the loop.
pub(crate) struct ExposureDone {
    seq: u64,
    view: ExposureView,
    reply: Option<oneshot::Sender<CtlReply>>,
}

/// The loop's state.
pub(crate) struct Engine {
    cfg: CaptureRuntimeConfig,
    profile: String,
    router: Option<Arc<FrameRouter>>,
    pose: Arc<dyn PoseProvider>,
    fc: Option<FcEkfPose>,
    vehicle: VehicleFeed,
    sensors: Arc<Mutex<SensorState>>,
    publisher: CapturePublisher,
    view_tx: Option<watch::Sender<CaptureStateView>>,
    paths: Paths,
    keyframes: CaptureSession,
    session: Option<ActiveSession>,
    /// The rig the open session records with.
    rig: Option<Rig>,
    closing: Option<Closing>,
    quality: QualityMonitor,
    coverage: Coverage,
    /// The latest LiDAR scan's depth summary (keyframe baseline, coverage
    /// rays).
    lidar_depth: Option<LidarDepth>,
    exposure: ExposureView,
    camera: Option<Arc<dyn CameraControl>>,
    /// Bumped per apply; only the latest one's result is kept.
    exposure_seq: u64,
    exposure_tx: mpsc::UnboundedSender<ExposureDone>,
    cams: HashMap<String, CamStats>,
    last_fix_seq: u64,
    kf_seq: u64,
    slow: u32,
    record_retry_at: Option<u64>,
    /// The clock service's timebase (its sidecar), refreshed on the slow tick.
    bus_timebase: Option<Timebase>,
    /// Sim-stamped frames whose exposure could not be mapped to the host
    /// clock (the bus clock is not on `sim`); they keep their arrival stamp.
    sim_unmapped: u64,
    /// The keyframe and scan queues' budget.
    budget: ados_mapping_pose::budget::Budget,
    /// The live `capture.reconstruct_steps` (see [`LoopDeps::reconstruct_steps`]).
    reconstruct_steps: SharedSteps,
    /// The pose a session takes without Navigation (the FC EKF).
    default_pose: Arc<dyn PoseProvider>,
    /// Navigation's pose, when it streams one.
    nav_pose: Option<Arc<NavigationTrack>>,
    /// The scan feed, for its incomplete count.
    scans: Option<Arc<ScanRaw>>,
}

impl Engine {
    /// The pose a session starting now takes, by `capture.pose_source`:
    /// `navigation` always (when Navigation streams), `fc_ekf` never, `auto`
    /// when a batch arrived in the last [`AUTO_NAV_FRESH`].
    fn session_pose(&self) -> Arc<dyn PoseProvider> {
        match (self.cfg.pose_source, &self.nav_pose) {
            (PoseSourceChoice::Navigation, Some(nav)) => nav.clone(),
            (PoseSourceChoice::Auto, Some(nav))
                if nav.fresh(std::time::Instant::now(), AUTO_NAV_FRESH) =>
            {
                nav.clone()
            }
            _ => self.default_pose.clone(),
        }
    }
}

/// Run the capture loop until `cancel`. An open session is closed (manifest
/// and checksums written) before returning.
pub async fn run_capture_loop(deps: LoopDeps, cancel: Shutdown) {
    let LoopDeps {
        cfg,
        profile,
        mut frames,
        router,
        pose,
        fc,
        vehicle,
        sensors,
        mut bus_rx,
        publisher,
        mut control_rx,
        mut hud_rx,
        view_tx,
        camera,
        paths,
        reconstruct_steps,
        navigation,
    } = deps;
    let (scans, mut scan_rx) = match navigation.scans {
        Some((feed, rx)) => (Some(feed), Some(rx)),
        None => (None, None),
    };
    if let Some(fc) = &fc {
        fc.set_covariance(cfg.pose_prior.fc_covariance());
    }
    // No session is open yet: a capture-active marker left by a crashed run
    // would keep the FC at capture rates.
    if let Some(m) = &paths.active_marker {
        match std::fs::remove_file(m) {
            Ok(()) => tracing::warn!(path = %m.display(), "stale_capture_active_marker_removed"),
            Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
            Err(e) => {
                tracing::warn!(path = %m.display(), error = %e, "capture_active_marker_remove_failed")
            }
        }
    }
    let (exposure_tx, mut exposure_rx) = mpsc::unbounded_channel();
    let bus_timebase =
        ados_mapping_pose::clock::ClockSidecar::read(&paths.clock_sidecar).map(|c| c.timebase());
    let budget = ados_mapping_pose::budget::resolve(
        cfg.memory_budget_mb,
        std::env::var(ados_mapping_pose::budget::MEMORY_MAX_ENV)
            .ok()
            .as_deref(),
        std::path::Path::new("/"),
    );
    tracing::info!(budget_mb = budget.total_mb(), "capture_capture_budget");
    let mut engine = Engine {
        keyframes: CaptureSession::new(cfg.capture.clone()),
        exposure: cfg.exposure.clone(),
        camera,
        exposure_seq: 0,
        exposure_tx,
        record_retry_at: cfg
            .record_on_start
            .then(|| ados_mapping_pose::clock::mono_ns() + 2_000_000_000),
        cfg,
        profile,
        router,
        pose: pose.clone(),
        fc,
        vehicle,
        sensors,
        publisher,
        view_tx,
        paths,
        session: None,
        rig: None,
        closing: None,
        quality: QualityMonitor::new(),
        coverage: Coverage::new(),
        lidar_depth: None,
        cams: HashMap::new(),
        last_fix_seq: 0,
        kf_seq: 0,
        slow: 0,
        bus_timebase,
        sim_unmapped: 0,
        budget,
        reconstruct_steps,
        default_pose: pose,
        nav_pose: navigation.track,
        scans,
    };
    engine.pose = engine.session_pose();
    // Frames are pulled on their own task so a source read is never
    // cancelled mid-frame by the select below.
    let (frame_tx, mut frame_rx) = mpsc::channel::<CapturedFrame>(8);
    let reader_cancel = cancel.clone();
    let reader = tokio::spawn(async move {
        loop {
            let next = tokio::select! {
                _ = reader_cancel.wait() => return,
                f = frames.next() => f,
            };
            match next {
                Some(f) => {
                    if frame_tx.send(f).await.is_err() {
                        return;
                    }
                }
                None => tokio::select! {
                    _ = reader_cancel.wait() => return,
                    _ = tokio::time::sleep(Duration::from_millis(50)) => {}
                },
            }
        }
    });
    let mut tick = tokio::time::interval(TICK);
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    engine.publish_status().await;
    // A configured lock is applied once per run; unlocked leaves the cameras
    // on their own auto exposure.
    if engine.exposure.locked {
        engine.apply_exposure(None);
    }
    loop {
        tokio::select! {
            biased;
            _ = cancel.wait() => break,
            Some(msg) = control_rx.recv() => engine.handle(msg).await,
            Some(done) = exposure_rx.recv() => engine.exposure_done(done),
            Some(action) = async { match hud_rx.as_mut() { Some(rx) => rx.recv().await, None => None } } => {
                engine.on_hud(action).await;
            }
            _ = tick.tick() => engine.on_tick().await,
            Some(ev) = bus_rx.recv() => engine.on_bus(ev),
            Some(scan) = async { match scan_rx.as_mut() { Some(rx) => rx.recv().await, None => None } } => {
                engine.on_scan(scan);
            }
            f = frame_rx.recv() => match f {
                Some(f) => engine.on_frame(f).await,
                None => break,
            },
        }
    }
    reader.abort();
    engine.shutdown().await;
    tracing::info!("capture loop stopped");
}

#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_timebase;

/// A session being finished on a blocking thread; its id stays reported in
/// the state view until the finish completes.
struct Closing {
    session_id: String,
    handle: JoinHandle<anyhow::Result<Manifest>>,
}
