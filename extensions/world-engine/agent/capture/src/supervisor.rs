//! The capture service's lifecycle as a plugin process.
//!
//! [`supervise`] polls the `atlas.*` plugin config every [`CONFIG_POLL`]. While
//! capture is enabled and the capture config validates, it keeps one
//! [`CaptureRun`] alive: the atlas bus bound, the control socket served, and
//! the capture loop running. When the resolved config changes the run is
//! stopped and started again with the new config; while capture is disabled or
//! the config is invalid, nothing is bound. The link reports the capture
//! service as running from whether the control socket answers, so an idle
//! service must not leave one behind.
//!
//! The frame and vehicle-state subscriptions cannot be withdrawn, so each is
//! made once, on the first run that needs it, and routed to whichever run is
//! current.

use std::collections::{BTreeMap, HashSet};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use ados_protocol::shutdown::Shutdown;
use ados_sdk::{ClientError, PluginContext};
use tokio::sync::mpsc;
use tokio::task::JoinHandle;
use world_engine_protocol::paths::{
    ipc_dir, ATLAS_BUS_SOCKET, ATLAS_CONTROL_SOCKET, ATLAS_POSE_OFFLOAD_SOCKET,
};

use crate::control::{serve_control, AtlasControlCmd};
use crate::frame_source::{AtlasFrameSource, FrameRouter};
use crate::pose_source::{build_pose_provider, PoseProvider, TelemetryPoseFeed};
use crate::publish::AtlasPublisher;
use crate::runtime::{select_pose_tier, AtlasRuntimeConfig, ConfigError, PoseTier, CONFIG_KEYS};
use crate::service::{new_session_id, run_capture_loop};
use crate::session::CaptureSession;

/// How often the plugin config is re-read.
pub const CONFIG_POLL: Duration = Duration::from_secs(5);

/// The sockets a capture run binds and reads, all in the extension's private
/// IPC directory.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Sockets {
    pub bus: PathBuf,
    pub control: PathBuf,
    pub offload: PathBuf,
}

impl Sockets {
    pub fn in_dir(dir: &Path) -> Self {
        Self {
            bus: dir.join(ATLAS_BUS_SOCKET),
            control: dir.join(ATLAS_CONTROL_SOCKET),
            offload: dir.join(ATLAS_POSE_OFFLOAD_SOCKET),
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
    // A capture config with no enabled camera can never produce a keyframe;
    // stay idle rather than run a loop that selects nothing.
    config
        .capture
        .validate()
        .map_err(|e| format!("invalid capture config: {e}"))?;
    Ok(config)
}

/// Read every `atlas.*` key from the host and resolve them. The outer error is
/// the host being unreachable (the caller keeps its current state); the inner
/// one is a key whose value does not parse.
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

/// One running capture: the loop plus the control socket that drives it.
pub struct CaptureRun {
    config: AtlasRuntimeConfig,
    cancel: Shutdown,
    capture: JoinHandle<()>,
    control: JoinHandle<()>,
    control_path: PathBuf,
}

impl CaptureRun {
    /// Bind the atlas bus and the control socket, then start the capture loop.
    /// Either bind failing fails the start with nothing left bound: a capture
    /// the link cannot see (no control socket) would be reported as not
    /// running while it captures.
    pub async fn start(
        config: AtlasRuntimeConfig,
        frames: AtlasFrameSource,
        pose: Arc<dyn PoseProvider>,
        sockets: &Sockets,
    ) -> anyhow::Result<Self> {
        let publisher = AtlasPublisher::bind(&sockets.bus.to_string_lossy()).await?;
        let control_path = sockets.control.clone();
        let (control_tx, control_rx) = mpsc::channel::<AtlasControlCmd>(32);
        let control = serve_control(&control_path.to_string_lossy(), control_tx)
            .await
            .map_err(|e| anyhow::anyhow!("bind control socket {}: {e}", control_path.display()))?;

        let session = CaptureSession::with_pose_interval(
            config.capture.clone(),
            config.pose_publish_interval_ms,
        );
        let session_id = new_session_id(&config.device_id);
        let cancel = Shutdown::new();
        let capture = tokio::spawn(run_capture_loop(
            frames,
            pose,
            publisher,
            session,
            config.clone(),
            session_id,
            control_rx,
            cancel.clone(),
        ));
        Ok(Self {
            config,
            cancel,
            capture,
            control,
            control_path,
        })
    }

    pub fn config(&self) -> &AtlasRuntimeConfig {
        &self.config
    }

    /// The capture loop only returns on cancel, so a finished loop on a live
    /// run means it died (a panic); the caller stops and restarts it.
    pub fn is_finished(&self) -> bool {
        self.capture.is_finished()
    }

    /// Stop the loop (a live session is finalized and bagged so the final
    /// state reaches the bus), then take the control socket down so nothing
    /// answers on it while idle.
    pub async fn stop(self) {
        self.cancel.trigger();
        if let Err(e) = self.capture.await {
            tracing::error!(error = %e, "atlas capture loop ended abnormally");
        }
        self.control.abort();
        let _ = self.control.await;
        let _ = std::fs::remove_file(&self.control_path);
    }
}

/// Run the capture service until `cancel` fires. See the module docs.
pub async fn supervise(ctx: PluginContext, cancel: Shutdown) {
    let sockets = Sockets::in_dir(&ipc_dir());
    let frames = FrameRouter::new();
    let vehicle = TelemetryPoseFeed::new();
    let mut frames_subscribed = false;
    let mut vehicle_subscribed = false;
    let mut run: Option<CaptureRun> = None;
    let mut last_idle_reason: Option<String> = None;

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
                r.stop().await;
            }
        }

        let resolved = match read_config(&ctx).await {
            Ok(resolved) => resolved,
            Err(e) => {
                // The host did not answer; keep whatever is running.
                tracing::warn!(error = %e, "atlas config read failed; keeping current state");
                continue;
            }
        };
        let desired = capture_config(resolved);
        let idle_reason = desired.as_ref().err().cloned();
        if idle_reason != last_idle_reason {
            match &idle_reason {
                Some(reason) => tracing::info!(reason = %reason, "atlas capture idle"),
                None => tracing::info!("atlas capture enabled"),
            }
            last_idle_reason = idle_reason;
        }

        if run.as_ref().map(CaptureRun::config) == desired.as_ref().ok() {
            continue;
        }
        if let Some(r) = run.take() {
            tracing::info!("atlas capture stopping for a config change");
            r.stop().await;
            frames.detach();
        }
        let Ok(config) = desired else {
            continue;
        };

        if !frames_subscribed {
            match ctx.vision.subscribe_frames(None, frames.callback()).await {
                Ok(()) => frames_subscribed = true,
                Err(e) => {
                    tracing::warn!(error = %e, "atlas frame subscription failed; retrying");
                    continue;
                }
            }
        }
        // A compute node is not paired in this build and the board's
        // accelerator is not probed, so `Auto` resolves to the always-available
        // flight-controller pose unless the operator pins a tier.
        let tier = select_pose_tier(config.pose_tier, false, false);
        if tier != PoseTier::Offload && !vehicle_subscribed {
            match vehicle.subscribe(&ctx).await {
                Ok(()) => vehicle_subscribed = true,
                Err(e) => {
                    tracing::warn!(error = %e, "atlas vehicle-state subscription failed; retrying");
                    continue;
                }
            }
        }

        let enabled: HashSet<String> = config
            .capture
            .enabled_cameras()
            .map(|c| c.id.clone())
            .collect();
        let source = AtlasFrameSource::Engine(frames.attach(enabled));
        let pose = build_pose_provider(tier, &config, &vehicle, sockets.offload.clone());
        tracing::info!(
            ?tier,
            cameras = config.capture.enabled_camera_count(),
            "atlas capture starting"
        );
        match CaptureRun::start(config, source, pose, &sockets).await {
            Ok(r) => run = Some(r),
            Err(e) => {
                tracing::error!(error = %e, "atlas capture failed to start; retrying");
                frames.detach();
            }
        }
    }

    if let Some(r) = run.take() {
        r.stop().await;
    }
    frames.detach();
    tracing::info!("atlas capture service stopped");
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::{CameraConfig, CaptureConfig};
    use crate::frame_source::SyntheticFrameSource;
    use crate::pose_source::PoseSample;
    use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
    use tokio::net::UnixStream;
    use world_engine_protocol::atlas::{CameraRole, CaptureState, CaptureStatus};

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
        let unparsed = ConfigError {
            key: "atlas.cameras",
            reason: "bad role".into(),
        };
        let why = capture_config(Err(unparsed)).unwrap_err();
        assert!(
            why.contains("atlas.cameras"),
            "the reason names the key: {why}"
        );
    }

    struct NoPose;
    impl PoseProvider for NoPose {
        fn latest(&self) -> Option<PoseSample> {
            None
        }
    }

    async fn control_status(path: &Path) -> std::io::Result<CaptureStatus> {
        let mut stream = UnixStream::connect(path).await?;
        stream.write_all(b"{\"cmd\":\"status\"}\n").await?;
        let mut line = String::new();
        BufReader::new(stream).read_line(&mut line).await?;
        serde_json::from_str(&line).map_err(std::io::Error::other)
    }

    #[tokio::test]
    async fn a_run_answers_on_its_control_socket_until_stopped() {
        let dir = tempfile::tempdir().unwrap();
        let sockets = Sockets::in_dir(dir.path());
        let run = CaptureRun::start(
            enabled_config(),
            AtlasFrameSource::Synthetic(SyntheticFrameSource::new(Vec::new())),
            Arc::new(NoPose),
            &sockets,
        )
        .await
        .unwrap();

        // Running: the control socket answers with the live session.
        let status = tokio::time::timeout(Duration::from_secs(3), control_status(&sockets.control))
            .await
            .expect("status within 3 s")
            .expect("control socket answers");
        assert_eq!(status.state, CaptureState::Capturing);
        assert!(sockets.bus.exists());

        // Stopped: nothing answers and neither socket is left behind, which is
        // how the link tells an idle capture service from a running one.
        run.stop().await;
        assert!(control_status(&sockets.control).await.is_err());
        assert!(!sockets.control.exists());
        assert!(!sockets.bus.exists());
    }
}
