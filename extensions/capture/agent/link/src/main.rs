//! `capture-link`: the Capture extension's main process. The host runs
//! it as `capture-link com.altnautica.capture` with the plugin socket
//! and token in the environment.

use std::collections::BTreeMap;
use std::process::ExitCode;

use capture_link::{shutdown_signal, LinkPlugin};
use tracing_subscriber::EnvFilter;

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
            tracing::error!(error = %e, "capture link exited");
            ExitCode::FAILURE
        }
    }
}
