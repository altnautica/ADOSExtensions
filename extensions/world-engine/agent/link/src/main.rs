//! `world-engine-link`: the World Engine extension's main process. The host runs
//! it as `world-engine-link com.altnautica.world-engine` with the plugin socket
//! and token in the environment.

use std::collections::BTreeMap;
use std::process::ExitCode;

use tracing_subscriber::EnvFilter;
use world_engine_link::{shutdown_signal, LinkPlugin};

#[tokio::main]
async fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();
    match ados_sdk::run_plugin::<LinkPlugin, _>(
        env!("CARGO_PKG_VERSION"),
        BTreeMap::new(),
        shutdown_signal(),
    )
    .await
    {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            tracing::error!(error = %e, "world engine link exited");
            ExitCode::FAILURE
        }
    }
}
