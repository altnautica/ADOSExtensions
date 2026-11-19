//! `reconstruct-offload`: Reconstruct's process on a drone.
//!
//! The host runs it as `reconstruct-offload com.altnautica.reconstruct`; the
//! mode follows the node profile the host exports (`ADOS_NODE_PROFILE`):
//!
//! - **drone**: perception offload to a paired node ([`offload`]), the live
//!   scan stream for a node-built map ([`stream`]), the reconstruction MCP
//!   tools ([`tools`]) and the drone routes ([`api`]).
//! - **workstation / compute**: the `node` service owns the HTTP socket and
//!   the status telemetry, so this process binds nothing and waits for
//!   shutdown.
//! - any other profile: nothing to do; it says so once and waits.

pub mod api;
pub mod offload;
pub mod stream;
pub mod tools;

use std::sync::Arc;
use std::time::Duration;

use ados_mapping_transport::credentials::CredentialStore;
use ados_mapping_transport::host::Host;
use ados_mapping_transport::serve_unix;
use ados_sdk::{ClientError, Plugin, PluginContext};
use tokio::sync::watch;
use tokio::task::JoinHandle;

use crate::offload::Reconciler;
use crate::stream::{SharedStream, Streamer};

/// The plugin config key that pins the compute node this extension pairs
/// with.
pub const PIN_KEY: &str = "reconstruct.offload.compute_node_addr";
/// `auto` | `on` | `off`: whether perception runs on the paired node.
pub const ENABLED_KEY: &str = "reconstruct.offload.enabled";
/// The id the vision engine captures the primary camera under, when it is
/// not the `main` tap.
pub const CAMERA_KEY: &str = "reconstruct.offload.camera";

/// How long `on_stop` waits for each task to wind down.
const STOP_GRACE: Duration = Duration::from_secs(5);

/// What this process does on this node.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Mode {
    Drone,
    /// A profile whose HTTP and state the `node` service owns.
    Node(String),
    /// A profile Reconstruct has nothing to run on.
    Idle(String),
}

impl Mode {
    pub fn from_profile(profile: &str) -> Self {
        match profile {
            "drone" => Self::Drone,
            "workstation" | "compute" => Self::Node(profile.to_string()),
            other => Self::Idle(other.to_string()),
        }
    }
}

/// The plugin: starts the mode's tasks on `on_start`, stops them on `on_stop`.
pub struct OffloadPlugin {
    stop: Option<watch::Sender<bool>>,
    tasks: Vec<JoinHandle<()>>,
}

impl OffloadPlugin {
    fn start_drone(
        &mut self,
        ctx: &PluginContext,
        shutdown: watch::Receiver<bool>,
    ) -> std::io::Result<()> {
        let host: Arc<dyn Host> = Arc::new(ctx.clone());
        let credentials = CredentialStore::new(ctx.data_dir.as_deref(), PIN_KEY);
        let stream = SharedStream::default();
        let api = api::DroneApi {
            host: host.clone(),
            credentials: credentials.clone(),
            stream: stream.clone(),
        };
        ctx.tools
            .serve(tools::handler(ados_mapping_transport::jobs::NodeJobs {
                host: host.clone(),
                credentials: credentials.clone(),
            }));
        if let Some(path) = ados_sdk::http::socket_path() {
            if let Some(parent) = path.parent() {
                std::fs::create_dir_all(parent)?;
            }
            let listener = ados_sdk::http::bind(&path)?;
            let mut stop = shutdown.clone();
            self.tasks.push(tokio::spawn(serve_unix(
                listener,
                api::router(api),
                async move {
                    let _ = stop.wait_for(|s| *s).await;
                },
            )));
        }
        self.tasks.push(tokio::spawn(
            Streamer {
                host: host.clone(),
                credentials: credentials.clone(),
                device_id: ctx.agent_id.clone(),
                view: stream,
            }
            .run(shutdown.clone()),
        ));
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
}

#[async_trait::async_trait]
impl Plugin for OffloadPlugin {
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
                tracing::info!("reconstruct offload: drone mode");
                self.start_drone(ctx, shutdown)?;
            }
            Mode::Node(profile) => {
                tracing::info!(profile = %profile, "reconstruct offload: the node service serves this profile; waiting for shutdown");
            }
            Mode::Idle(profile) => {
                tracing::info!(profile = %profile, "reconstruct offload: nothing to run on this profile; waiting for shutdown");
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
        assert_eq!(Mode::from_profile("compute"), Mode::Node("compute".into()));
        assert_eq!(Mode::from_profile("visor"), Mode::Idle("visor".into()));
    }
}
