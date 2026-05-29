//! Minimal Rust extension agent half.
//!
//! This is the smallest plugin that exercises the whole Rust extension path:
//! it links `ados-sdk`, implements the [`Plugin`] lifecycle trait with no
//! behaviour, and hands the type to [`run_plugin`]. The runner reads
//! `--socket` (with the `ADOS_PLUGIN_SOCKET` env fallback) plus the capability
//! token and agent id, connects to the per-plugin Unix socket, and drives the
//! lifecycle hooks until the unit is stopped.
//!
//! There are no permissions to use and no capabilities to call, so every hook
//! is the trait default no-op. A real plugin overrides `on_start` to subscribe
//! to events, register a model, inject pose, and so on through the
//! `PluginContext` the runner builds.

use std::collections::BTreeMap;

use ados_sdk::{run_plugin, Plugin, RunnerError};
use async_trait::async_trait;

/// An empty plugin. Construction cannot fail and no hook does any work, so the
/// type holds no state and relies entirely on the trait's default hooks.
struct HelloPlugin;

#[async_trait]
impl Plugin for HelloPlugin {
    fn new() -> Self {
        HelloPlugin
    }
}

/// Resolve when the process is asked to stop. The plugin host stops the unit
/// with SIGTERM; SIGINT covers a foreground run. Whichever arrives first ends
/// the wait and the runner tears the plugin down through `on_stop` /
/// `on_disable`.
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
async fn main() {
    // No static config: the empty plugin reads nothing from the manifest.
    let static_config: BTreeMap<String, rmpv::Value> = BTreeMap::new();

    match run_plugin::<HelloPlugin, _>(env!("CARGO_PKG_VERSION"), static_config, shutdown_signal())
        .await
    {
        Ok(()) => {}
        // Running the binary by hand (no host) lands here: there is no socket
        // or token to connect to. Report it and exit non-zero rather than
        // panicking, so the binary is safe to invoke standalone while checking
        // a build.
        Err(RunnerError::NoBridge) => {
            eprintln!(
                "hello-rust: no plugin host socket/token supplied; \
                 this binary is launched by the agent plugin host"
            );
            std::process::exit(1);
        }
        Err(err) => {
            eprintln!("hello-rust: {err}");
            std::process::exit(1);
        }
    }
}
