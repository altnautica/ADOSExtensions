//! `world-engine-node`: the World Engine reconstruction node.
//!
//! - Under the plugin host (Linux / macOS), started as
//!   `world-engine-node com.altnautica.world-engine`, it is the extension's
//!   `node` service ([`host`]).
//! - `world-engine-node --standalone --data-dir <dir> [--listen <addr>]
//!   [--token <t>] [--public-url <url>] [--workers <n>] [--tools-dir <dir>]`
//!   runs without a plugin host (the Windows workstation build only runs this
//!   way): the job API, the job files, the session upload lane and the
//!   artifacts on one listener, admitted by the owner bearer token kept in
//!   `<data-dir>/node/owner-token` or by a node credential the owner issued.

#[cfg(unix)]
#[path = "service/host/mod.rs"]
mod host;

use world_engine_node::standalone::{self, StandaloneOptions};

fn init_logging() {
    use tracing_subscriber::EnvFilter;
    let filter = std::env::var("RUST_LOG").unwrap_or_else(|_| "info".to_string());
    let _ = tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::new(filter))
        .try_init();
}

const USAGE: &str = "usage: world-engine-node --standalone --data-dir <dir> \
[--listen <addr>] [--token <t>] [--public-url <url>] [--workers <n>] [--tools-dir <dir>]";

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    init_logging();
    let args: Vec<String> = std::env::args().collect();
    match StandaloneOptions::from_args(&args) {
        Ok(Some(opts)) => standalone::run(opts).await,
        Ok(None) => run_service().await,
        Err(e) => {
            eprintln!("{e}\n{USAGE}");
            std::process::exit(2);
        }
    }
}

#[cfg(unix)]
async fn run_service() -> Result<(), Box<dyn std::error::Error>> {
    host::run().await
}

#[cfg(not(unix))]
async fn run_service() -> Result<(), Box<dyn std::error::Error>> {
    eprintln!("this build runs only as a standalone node\n{USAGE}");
    std::process::exit(2);
}
