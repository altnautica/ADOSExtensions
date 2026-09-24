//! World Engine link: the extension's main process on every profile.
//!
//! The host runs it as `world-engine-link com.altnautica.world-engine` and the
//! mode follows the node profile the host exports (`ADOS_NODE_PROFILE`):
//!
//! - **drone**: the Atlas forwarder ([`forward`]), the shared-topic republish
//!   and the `status` telemetry ([`monitor`]), the perception-offload
//!   reconciler ([`offload`]), and the drone HTTP API ([`api`]).
//! - **ground-station**: the radio aux-lane relay ([`relay`]) and the ground
//!   HTTP API.
//! - **workstation / compute**: the `node` service owns the HTTP socket and the
//!   `status` telemetry, so the link binds and publishes nothing and only waits
//!   for shutdown.

pub mod api;
pub mod compute_node;
pub mod control;
pub mod credentials;
pub mod forward;
pub mod host;
pub mod monitor;
pub mod offload;
pub mod relay;
pub mod sdk;
#[cfg(test)]
mod testing;

use std::sync::Arc;
use std::time::Duration;

use ados_sdk::{ClientError, Plugin, PluginContext};
use parking_lot::Mutex;
use tokio::sync::watch;
use tokio::task::JoinHandle;
use world_engine_protocol::paths::{self, ATLAS_BUS_SOCKET, ATLAS_CONTROL_SOCKET};
use world_engine_transport::serve_unix;

use crate::api::DroneApi;
use crate::control::ControlClient;
use crate::credentials::CredentialStore;
use crate::forward::{ForwardStatus, Forwarder};
use crate::host::Host;
use crate::monitor::CaptureState;
use crate::offload::Reconciler;
use crate::relay::Relay;

/// How long `on_stop` waits for each task to wind down.
const STOP_GRACE: Duration = Duration::from_secs(5);

/// What the link does on this node.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Mode {
    Drone,
    GroundStation,
    /// A profile whose HTTP and state another process owns.
    Passive(String),
}

impl Mode {
    /// The mode for a node profile string.
    pub fn from_profile(profile: &str) -> Self {
        match profile {
            "drone" => Self::Drone,
            "ground-station" | "ground_station" => Self::GroundStation,
            other => Self::Passive(other.to_string()),
        }
    }
}

/// Sleep for `delay` unless shutdown is signalled first (or its sender is
/// gone). Returns `true` when the caller should stop.
pub(crate) async fn sleep_or_shutdown(
    shutdown: &mut watch::Receiver<bool>,
    delay: Duration,
) -> bool {
    tokio::select! {
        r = shutdown.changed() => r.is_err() || *shutdown.borrow(),
        _ = tokio::time::sleep(delay) => false,
    }
}

/// Resolve once shutdown is signalled (or its sender is gone).
async fn stopped(mut shutdown: watch::Receiver<bool>) {
    let _ = shutdown.wait_for(|stop| *stop).await;
}

/// Resolve on SIGTERM or SIGINT.
pub async fn shutdown_signal() {
    use tokio::signal::unix::{signal, SignalKind};
    let (Ok(mut term), Ok(mut int)) = (
        signal(SignalKind::terminate()),
        signal(SignalKind::interrupt()),
    ) else {
        let _ = tokio::signal::ctrl_c().await;
        return;
    };
    tokio::select! {
        _ = term.recv() => {}
        _ = int.recv() => {}
    }
}

/// Bind the plugin's HTTP socket and serve `router` on it until shutdown.
fn serve_http(
    router: axum::Router,
    shutdown: watch::Receiver<bool>,
) -> std::io::Result<JoinHandle<()>> {
    let path = paths::http_socket();
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let listener = ados_sdk::http::bind(&path)?;
    tracing::info!(socket = %path.display(), "serving the World Engine API");
    Ok(tokio::spawn(serve_unix(
        listener,
        router,
        stopped(shutdown),
    )))
}

/// The plugin: starts the mode's tasks on `on_start`, stops them on `on_stop`.
pub struct LinkPlugin {
    stop: Option<watch::Sender<bool>>,
    tasks: Vec<JoinHandle<()>>,
}

impl LinkPlugin {
    fn start_drone(
        &mut self,
        ctx: &PluginContext,
        shutdown: watch::Receiver<bool>,
    ) -> std::io::Result<()> {
        let host: Arc<dyn Host> = Arc::new(ctx.clone());
        let credentials = CredentialStore::new(data_dir(ctx).as_deref());
        let bus_socket = paths::ipc_dir().join(ATLAS_BUS_SOCKET);
        let forward = Arc::new(Mutex::new(ForwardStatus::default()));
        let capture = Arc::new(CaptureState::default());

        let router = api::drone_router(
            DroneApi {
                host: host.clone(),
                control: ControlClient::new(paths::ipc_dir().join(ATLAS_CONTROL_SOCKET)),
                forward: forward.clone(),
                profile: "drone".to_string(),
            },
            credentials.clone(),
        );
        self.tasks.push(serve_http(router, shutdown.clone())?);
        self.tasks.push(tokio::spawn(
            Forwarder {
                host: host.clone(),
                device_id: ctx.agent_id.clone(),
                credentials: credentials.clone(),
                bus_socket: bus_socket.clone(),
                status: forward.clone(),
            }
            .run(shutdown.clone()),
        ));
        self.tasks.push(tokio::spawn(monitor::run_bus_monitor(
            host.clone(),
            bus_socket,
            capture.clone(),
            shutdown.clone(),
        )));
        self.tasks.push(tokio::spawn(monitor::run_status_writer(
            host.clone(),
            capture,
            forward,
            shutdown.clone(),
        )));
        self.tasks.push(tokio::spawn(
            Reconciler {
                host,
                agent_id: ctx.agent_id.clone(),
                credentials,
            }
            .run(shutdown),
        ));
        Ok(())
    }

    fn start_ground(
        &mut self,
        ctx: &PluginContext,
        shutdown: watch::Receiver<bool>,
    ) -> std::io::Result<()> {
        let host: Arc<dyn Host> = Arc::new(ctx.clone());
        let credentials = CredentialStore::new(data_dir(ctx).as_deref());
        let snapshot = relay::SharedSnapshot::default();
        let router = api::ground_router(snapshot.clone(), credentials.clone());
        self.tasks.push(serve_http(router, shutdown.clone())?);
        self.tasks.push(tokio::spawn(
            Relay {
                host,
                credentials,
                snapshot,
            }
            .run(shutdown),
        ));
        Ok(())
    }
}

/// The plugin data dir the host gave this unit.
fn data_dir(ctx: &PluginContext) -> Option<std::path::PathBuf> {
    ctx.data_dir.clone().or_else(paths::data_dir)
}

#[async_trait::async_trait]
impl Plugin for LinkPlugin {
    fn new() -> Self {
        Self {
            stop: None,
            tasks: Vec::new(),
        }
    }

    async fn on_start(&mut self, ctx: &PluginContext) -> Result<(), ClientError> {
        let (stop, shutdown) = watch::channel(false);
        self.stop = Some(stop);
        match Mode::from_profile(&ados_sdk::node_profile()) {
            Mode::Drone => {
                tracing::info!("world engine link: drone mode");
                self.start_drone(ctx, shutdown)?;
            }
            Mode::GroundStation => {
                tracing::info!("world engine link: ground-station mode");
                self.start_ground(ctx, shutdown)?;
            }
            Mode::Passive(profile) => {
                tracing::info!(profile = %profile, "world engine link: the node service serves this profile; waiting for shutdown");
            }
        }
        Ok(())
    }

    async fn on_stop(&mut self, _ctx: &PluginContext) -> Result<(), ClientError> {
        if let Some(stop) = self.stop.take() {
            let _ = stop.send(true);
        }
        for task in self.tasks.drain(..) {
            let abort = task.abort_handle();
            if tokio::time::timeout(STOP_GRACE, task).await.is_err() {
                abort.abort();
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_mode_follows_the_node_profile() {
        assert_eq!(Mode::from_profile("drone"), Mode::Drone);
        assert_eq!(Mode::from_profile("ground-station"), Mode::GroundStation);
        assert_eq!(
            Mode::from_profile("workstation"),
            Mode::Passive("workstation".into())
        );
        assert_eq!(
            Mode::from_profile("compute"),
            Mode::Passive("compute".into())
        );
    }
}
