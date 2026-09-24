//! `world-engine-capture`: the extension's on-drone `capture` service.
//!
//! Runs as a plugin process (`bin:world-engine-capture com.altnautica.world-engine`,
//! so argv[1] is the plugin id; the socket and token come from the host's
//! environment). `on_start` spawns the capture supervisor, which follows the
//! `atlas.*` plugin config: while capture is enabled it tags each engine frame
//! with the flight controller's pose (or an offloaded SLAM pose), selects
//! keyframes, and publishes the keyframe + pose + capture-state streams on the
//! extension's private atlas bus. `on_stop` stops it. Runs until SIGTERM /
//! SIGINT.

use std::collections::BTreeMap;

use ados_protocol::shutdown::Shutdown;
use ados_sdk::{ClientError, Plugin, PluginContext};
use async_trait::async_trait;
use tokio::task::JoinHandle;
use world_engine_capture::supervisor::supervise;

struct CapturePlugin {
    cancel: Shutdown,
    supervisor: Option<JoinHandle<()>>,
}

#[async_trait]
impl Plugin for CapturePlugin {
    fn new() -> Self {
        Self {
            cancel: Shutdown::new(),
            supervisor: None,
        }
    }

    async fn on_start(&mut self, ctx: &PluginContext) -> Result<(), ClientError> {
        self.supervisor = Some(tokio::spawn(supervise(ctx.clone(), self.cancel.clone())));
        Ok(())
    }

    async fn on_stop(&mut self, _ctx: &PluginContext) -> Result<(), ClientError> {
        self.cancel.trigger();
        if let Some(task) = self.supervisor.take() {
            if let Err(e) = task.await {
                tracing::error!(error = %e, "world-engine-capture supervisor ended abnormally");
            }
        }
        Ok(())
    }
}

fn init_tracing() {
    use tracing_subscriber::EnvFilter;
    let filter = EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info"));
    let _ = tracing_subscriber::fmt()
        .with_env_filter(filter)
        .with_writer(std::io::stderr)
        .try_init();
}

#[tokio::main]
async fn main() {
    init_tracing();
    let result = ados_sdk::run_plugin::<CapturePlugin, _>(
        env!("CARGO_PKG_VERSION"),
        BTreeMap::new(),
        shutdown_signal(),
    )
    .await;
    if let Err(e) = result {
        tracing::error!(error = %e, "world-engine-capture exited with an error");
        std::process::exit(1);
    }
}

/// Resolve when the process receives SIGTERM or SIGINT.
async fn shutdown_signal() {
    use tokio::signal::unix::{signal, SignalKind};
    let mut term = signal(SignalKind::terminate()).expect("install SIGTERM handler");
    let mut int = signal(SignalKind::interrupt()).expect("install SIGINT handler");
    tokio::select! {
        _ = term.recv() => {}
        _ = int.recv() => {}
    }
}
