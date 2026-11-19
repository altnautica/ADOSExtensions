//! `reconstruct-offload`: Reconstruct's process on every profile it targets.
//! The host runs it as `reconstruct-offload com.altnautica.reconstruct` with
//! the plugin socket and token in the environment.

use std::collections::BTreeMap;
use std::process::ExitCode;

use reconstruct_offload::OffloadPlugin;
use tracing_subscriber::EnvFilter;

/// Resolve on SIGTERM or SIGINT.
async fn shutdown_signal() {
    use tokio::signal::unix::{signal, SignalKind};
    let mut term = signal(SignalKind::terminate()).expect("install SIGTERM handler");
    let mut int = signal(SignalKind::interrupt()).expect("install SIGINT handler");
    tokio::select! {
        _ = term.recv() => {}
        _ = int.recv() => {}
    }
}

#[tokio::main]
async fn main() -> ExitCode {
    tracing_subscriber::fmt()
        .with_env_filter(
            EnvFilter::try_from_default_env().unwrap_or_else(|_| EnvFilter::new("info")),
        )
        .with_writer(std::io::stderr)
        .init();
    match ados_sdk::run_plugin::<OffloadPlugin, _>(
        env!("CARGO_PKG_VERSION"),
        BTreeMap::new(),
        shutdown_signal(),
    )
    .await
    {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            tracing::error!(error = %e, "reconstruct offload exited");
            ExitCode::FAILURE
        }
    }
}
