//! The capture service's lifecycle as a plugin process.
//!
//! [`supervise`] polls the `atlas.*` plugin config every [`CONFIG_POLL`].
//! While capture is enabled and the config validates, one [`CaptureRun`] is
//! alive: the atlas bus and the control socket bound, the sensor-bus ingest,
//! the pose provider, the HUD page and the capture loop running. A changed
//! config stops the run (closing an open session) and starts a new one; a
//! disabled or invalid config leaves nothing bound, so the link reports the
//! service as not running.
//!
//! Subscriptions that cannot be withdrawn (frames, vehicle state, the FC
//! EKF messages, front-panel buttons) are made once and routed to whichever
//! run is current.

use std::collections::{BTreeMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use ados_protocol::shutdown::Shutdown;
use ados_sdk::{ClientError, PluginContext};
use parking_lot::Mutex;
use tokio::sync::{mpsc, watch};
use tokio::task::JoinHandle;
use world_engine_protocol::paths::{ipc_dir, ATLAS_BUS_SOCKET, ATLAS_CONTROL_SOCKET};

use crate::control::serve_control;
use crate::frame_source::{AtlasFrameSource, FrameRouter};
use crate::hud::{button_action, HudAction};
use crate::pose::fc_ekf::FcEkfPose;
use crate::publish::AtlasPublisher;
use crate::runtime::{AtlasRuntimeConfig, ConfigError, CONFIG_KEYS};
use crate::sensor_bus::{run_ingest, SensorState};
use crate::service::{run_capture_loop, LoopDeps, Paths};
use crate::vehicle::VehicleFeed;

/// How often the plugin config is re-read.
pub const CONFIG_POLL: Duration = Duration::from_secs(5);

/// The FC messages the EKF pose reads.
const FC_MESSAGES: [&str; 3] = [
    "ATTITUDE_QUATERNION",
    "LOCAL_POSITION_NED",
    "GPS_GLOBAL_ORIGIN",
];

/// The sockets a capture run binds, in the extension's private IPC directory.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sockets {
    pub bus: PathBuf,
    pub control: PathBuf,
}

impl Sockets {
    pub fn in_dir(dir: &Path) -> Self {
        Self {
            bus: dir.join(ATLAS_BUS_SOCKET),
            control: dir.join(ATLAS_CONTROL_SOCKET),
        }
    }
}

/// The config a capture run should be running with, or why none should run.
pub fn capture_config(
    resolved: Result<AtlasRuntimeConfig, ConfigError>,
) -> Result<AtlasRuntimeConfig, String> {
    let config = resolved.map_err(|e| format!("atlas config invalid: {e}"))?;
    if !config.enabled {
        return Err("atlas is not enabled".to_string());
    }
    config
        .capture
        .validate()
        .map_err(|e| format!("invalid capture config: {e}"))?;
    Ok(config)
}

/// Read every `atlas.*` key from the host and resolve them.
pub async fn read_config(
    ctx: &PluginContext,
) -> Result<Result<AtlasRuntimeConfig, ConfigError>, ClientError> {
    let mut values = BTreeMap::new();
    for &key in CONFIG_KEYS {
        let raw = ctx.config.get(key, rmpv::Value::Nil).await?;
        match rmpv::ext::from_value::<serde_json::Value>(raw) {
            Ok(v) => {
                values.insert(key.to_string(), v);
            }
            Err(e) => {
                return Ok(Err(ConfigError {
                    key,
                    reason: e.to_string(),
                }))
            }
        }
    }
    Ok(AtlasRuntimeConfig::from_values(&ctx.agent_id, &values))
}

/// Where front-panel actions go and which pins mean REC and MARK.
pub type HudSlot = Arc<Mutex<Option<(mpsc::Sender<HudAction>, u32, u32)>>>;

/// The process-lifetime feeds a run attaches to.
#[derive(Clone)]
pub struct Feeds {
    pub router: Arc<FrameRouter>,
    pub vehicle: VehicleFeed,
    pub fc: FcEkfPose,
    /// Where front-panel actions go: the current run, if any.
    pub hud: HudSlot,
}

/// One running capture.
pub struct CaptureRun {
    config: AtlasRuntimeConfig,
    cancel: Shutdown,
    tasks: Vec<JoinHandle<()>>,
    capture: JoinHandle<()>,
    control_path: PathBuf,
    _lidar: Option<crate::lidar_source::LidarSource>,
}

impl CaptureRun {
    pub async fn start(
        ctx: &PluginContext,
        config: AtlasRuntimeConfig,
        profile: String,
        feeds: &Feeds,
        sockets: &Sockets,
    ) -> anyhow::Result<Self> {
        let publisher = AtlasPublisher::bind(&sockets.bus.to_string_lossy()).await?;
        let (control_tx, control_rx) = mpsc::channel(32);
        let control = serve_control(&sockets.control.to_string_lossy(), control_tx)
            .await
            .map_err(|e| {
                anyhow::anyhow!("bind control socket {}: {e}", sockets.control.display())
            })?;
        let cancel = Shutdown::new();
        let paths = Paths::from_env();
        let work_dir = ctx
            .data_dir
            .clone()
            .unwrap_or_else(std::env::temp_dir)
            .join("capture");
        let rig = crate::rig_source::load_rig(&paths.rig);
        let chosen =
            crate::providers::pose_provider(&config, &profile, rig.as_ref(), &feeds.fc, &work_dir);
        let lidar = crate::providers::lidar_source(&config, rig.as_ref(), &work_dir);
        let (lidar_src, lidar_rx) = match lidar {
            Some((s, rx)) => (Some(s), Some(rx)),
            None => (None, None),
        };
        let sensors = Arc::new(Mutex::new(SensorState::default()));
        let (bus_tx, bus_rx) = mpsc::channel(1024);
        let mut tasks = vec![
            control,
            tokio::spawn(run_ingest(
                ados_sensorbus::socket_path(),
                sensors.clone(),
                bus_tx,
                cancel.clone(),
            )),
        ];
        let (hud_tx, hud_rx) = mpsc::channel(8);
        *feeds.hud.lock() = Some((hud_tx, config.hud.rec_pin, config.hud.mark_pin));
        let (view_tx, view_rx) = watch::channel(Default::default());
        tasks.push(tokio::spawn(hud_page(ctx.clone(), view_rx, cancel.clone())));
        let enabled: HashSet<String> = config
            .capture
            .enabled_cameras()
            .map(|c| c.id.clone())
            .collect();
        let frames = AtlasFrameSource::Engine(feeds.router.attach(enabled));
        let deps = LoopDeps {
            cfg: config.clone(),
            profile,
            frames,
            router: Some(feeds.router.clone()),
            pose: chosen.pose,
            fc: chosen.fc,
            vehicle: feeds.vehicle.clone(),
            sensors,
            bus_rx,
            lidar_rx,
            publisher,
            control_rx,
            hud_rx: Some(hud_rx),
            view_tx: Some(view_tx),
            paths,
        };
        let capture = tokio::spawn(run_capture_loop(deps, cancel.clone()));
        Ok(Self {
            config,
            cancel,
            tasks,
            capture,
            control_path: sockets.control.clone(),
            _lidar: lidar_src,
        })
    }

    pub fn config(&self) -> &AtlasRuntimeConfig {
        &self.config
    }

    pub fn is_finished(&self) -> bool {
        self.capture.is_finished()
    }

    /// Stop: the loop closes an open session, then everything is torn down.
    pub async fn stop(self, feeds: &Feeds) {
        self.cancel.trigger();
        if let Err(e) = self.capture.await {
            tracing::error!(error = %e, "atlas capture loop ended abnormally");
        }
        for t in self.tasks {
            t.abort();
            let _ = t.await;
        }
        *feeds.hud.lock() = None;
        feeds.router.detach();
        let _ = std::fs::remove_file(&self.control_path);
    }
}

/// Keep the display's plugin page in step with the capture state (about 1 Hz,
/// only on change). A host without a display or the grant answers an error,
/// logged once.
async fn hud_page(
    ctx: PluginContext,
    mut rx: watch::Receiver<world_engine_protocol::capture_ctl::CaptureStateView>,
    cancel: Shutdown,
) {
    let mut last: Option<Vec<(String, String)>> = None;
    let mut warned = false;
    loop {
        tokio::select! {
            _ = cancel.wait() => return,
            r = rx.changed() => if r.is_err() { return },
        }
        let rows = crate::hud::page_rows(&rx.borrow_and_update());
        if last.as_ref() == Some(&rows) {
            continue;
        }
        match ctx
            .display
            .set_page(crate::hud::PAGE_TITLE, &rows, &[])
            .await
        {
            Ok(_) => last = Some(rows),
            Err(e) if !warned => {
                tracing::info!(error = %e, "capture_hud_page_unavailable");
                warned = true;
            }
            Err(_) => {}
        }
    }
}

/// Make the process-lifetime subscriptions a run needs; each is retried on
/// the next poll until it succeeds.
async fn subscribe_feeds(ctx: &PluginContext, feeds: &Feeds, done: &mut [bool; 4]) {
    if !done[0] {
        done[0] = ctx
            .vision
            .subscribe_frames(None, feeds.router.callback())
            .await
            .map_err(|e| tracing::warn!(error = %e, "atlas_frame_subscription_failed"))
            .is_ok();
    }
    if !done[1] {
        done[1] = feeds
            .vehicle
            .subscribe(ctx)
            .await
            .map_err(|e| tracing::warn!(error = %e, "atlas_vehicle_subscription_failed"))
            .is_ok();
    }
    if !done[2] {
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
                    fc.on_frame(&frame, crate::clock::mono_ns());
                }
            });
            if let Err(e) = ctx.mavlink.subscribe(name, cb).await {
                tracing::warn!(msg = name, error = %e, "atlas_fc_subscription_failed");
                ok = false;
            }
        }
        done[2] = ok;
    }
    if !done[3] {
        let slot = feeds.hud.clone();
        let cb = Arc::new(move |press: ados_protocol::buttons::ButtonPress| {
            if let Some((tx, rec, mark)) = slot.lock().as_ref() {
                if let Some(a) = button_action(&press, *rec, *mark) {
                    let _ = tx.try_send(a);
                }
            }
        });
        // A node without a panel or the grant: front-panel control is off.
        done[3] = ctx
            .buttons
            .subscribe(cb)
            .await
            .map_err(|e| tracing::info!(error = %e, "atlas_button_subscription_unavailable"))
            .is_ok();
    }
}

/// Run the capture service until `cancel` fires. See the module docs.
pub async fn supervise(ctx: PluginContext, cancel: Shutdown) {
    let sockets = Sockets::in_dir(&ipc_dir());
    let feeds = Feeds {
        router: FrameRouter::new(),
        vehicle: VehicleFeed::new(),
        fc: FcEkfPose::new(),
        hud: Arc::new(Mutex::new(None)),
    };
    let profile = match ctx.node.info().await {
        Ok(info) => info.profile,
        Err(e) => {
            tracing::warn!(error = %e, "node_info_unavailable_assuming_drone");
            "drone".to_string()
        }
    };
    let mut subscribed = [false; 4];
    let mut run: Option<CaptureRun> = None;
    let mut last_idle: Option<String> = None;
    let mut poll = tokio::time::interval(CONFIG_POLL);
    poll.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tokio::select! {
            _ = cancel.wait() => break,
            _ = poll.tick() => {}
        }
        if run.as_ref().is_some_and(CaptureRun::is_finished) {
            tracing::error!("atlas capture loop exited unexpectedly; restarting it");
            if let Some(r) = run.take() {
                r.stop(&feeds).await;
            }
        }
        let resolved = match read_config(&ctx).await {
            Ok(r) => r,
            Err(e) => {
                tracing::warn!(error = %e, "atlas config read failed; keeping current state");
                continue;
            }
        };
        let desired = capture_config(resolved);
        let idle = desired.as_ref().err().cloned();
        if idle != last_idle {
            match &idle {
                Some(reason) => tracing::info!(reason = %reason, "atlas capture idle"),
                None => tracing::info!("atlas capture enabled"),
            }
            last_idle = idle;
        }
        if run.as_ref().map(CaptureRun::config) == desired.as_ref().ok() {
            continue;
        }
        if let Some(r) = run.take() {
            tracing::info!("atlas capture stopping for a config change");
            r.stop(&feeds).await;
        }
        let Ok(config) = desired else { continue };
        subscribe_feeds(&ctx, &feeds, &mut subscribed).await;
        if !subscribed[0] {
            continue;
        }
        match CaptureRun::start(&ctx, config, profile.clone(), &feeds, &sockets).await {
            Ok(r) => run = Some(r),
            Err(e) => {
                tracing::error!(error = %e, "atlas capture failed to start; retrying");
                feeds.router.detach();
            }
        }
    }
    if let Some(r) = run.take() {
        r.stop(&feeds).await;
    }
    tracing::info!("atlas capture service stopped");
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{CameraConfig, CaptureConfig};
    use world_engine_protocol::atlas::CameraRole;

    fn enabled_config() -> AtlasRuntimeConfig {
        AtlasRuntimeConfig {
            enabled: true,
            capture: CaptureConfig {
                cameras: vec![CameraConfig {
                    id: "front".into(),
                    role: CameraRole::Primary,
                    enabled: true,
                    reconstruct: true,
                }],
                ..Default::default()
            },
            ..Default::default()
        }
    }

    #[test]
    fn only_an_enabled_valid_config_runs() {
        assert!(capture_config(Ok(enabled_config())).is_ok());
        let disabled = AtlasRuntimeConfig {
            enabled: false,
            ..enabled_config()
        };
        assert!(capture_config(Ok(disabled)).is_err());
        let no_cameras = AtlasRuntimeConfig {
            capture: CaptureConfig::default(),
            ..enabled_config()
        };
        assert!(capture_config(Ok(no_cameras)).is_err());
        let why = capture_config(Err(ConfigError {
            key: "atlas.cameras",
            reason: "bad role".into(),
        }))
        .unwrap_err();
        assert!(why.contains("atlas.cameras"), "{why}");
    }

    #[test]
    fn sockets_live_in_the_private_dir() {
        let s = Sockets::in_dir(Path::new("/run/x"));
        assert_eq!(s.control, Path::new("/run/x/atlas-control.sock"));
        assert_eq!(s.bus, Path::new("/run/x/atlas.sock"));
    }
}
