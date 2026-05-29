//! Vision Navigation plugin binary.
//!
//! The plugin host execs this binary directly under systemd and hands
//! it the per-plugin socket on the command line plus the capability
//! token + agent id through the unit environment. It builds the
//! [`VisionNavPlugin`] lifecycle and drives it until SIGTERM/SIGINT.

use std::collections::BTreeMap;

use ados_sdk::{run_plugin, RunnerError};
use vision_nav::pipeline::VisionNavPlugin;

/// Resolve when the host asks the process to stop.
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
    // No manifest-supplied static config block is consumed here; the
    // plugin reads its per-drone config via on_configure.
    let static_config: BTreeMap<String, rmpv::Value> = BTreeMap::new();

    match run_plugin::<VisionNavPlugin, _>(
        env!("CARGO_PKG_VERSION"),
        static_config,
        shutdown_signal(),
    )
    .await
    {
        Ok(()) => {}
        // Running by hand (no host socket/token) lands here: report and
        // exit non-zero rather than panicking, so the binary is safe to
        // invoke while checking a build.
        Err(RunnerError::NoBridge) => {
            eprintln!(
                "vision-nav: no plugin host socket/token supplied; this binary \
                 is launched by the agent plugin host"
            );
            std::process::exit(1);
        }
        Err(err) => {
            eprintln!("vision-nav: {err}");
            std::process::exit(1);
        }
    }
}
