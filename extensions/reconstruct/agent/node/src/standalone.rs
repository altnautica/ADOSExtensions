//! The standalone node: the reconstruction node without a plugin host (a
//! Windows workstation, or a Linux box run by hand). It serves the job API,
//! the job files and measurement, the session upload lane and the artifacts
//! on one TCP listener, and runs the worker. With no operator socket, the
//! owner is whoever presents the node's bearer token; the owner issues other
//! nodes credentials (`/api/compute/node-credentials`), kept in
//! `node/node-credentials.json`, which admit them to their lanes.

use std::io::Write as _;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use tokio::net::TcpListener;
use tokio::sync::Mutex;
use tower_http::cors::CorsLayer;

use crate::bulk::{bulk_router, CommitHook, SessionStore};
use crate::job_files::job_files_router;
use crate::lanes::upload_and_artifact_lanes;
use crate::worker::{worker_loop, WorkerCtx};
use crate::{
    build_router_with_base, derive_public_base, Cluster, ComputeAuth, Engine, JobStore, NoDetector,
    Scheduler, SelectingReconstructor, SessionRegistry,
};

/// Command-line options of a standalone run.
#[derive(Debug, Clone, PartialEq)]
pub struct StandaloneOptions {
    /// State directory: `node/jobs.db`, `node/work/` (job outputs),
    /// `node/sessions/` (uploaded capture sessions), `node/owner-token`,
    /// `node/node-credentials.json` (credentials issued to other nodes),
    /// `node/node-id`.
    pub data_dir: PathBuf,
    /// Listen address, e.g. `0.0.0.0:8092`.
    pub listen: String,
    /// The owner bearer token; generated into `node/owner-token` when absent.
    pub token: Option<String>,
    /// Base URL artifacts are fetched from (derived from `listen` when absent).
    pub public_url: Option<String>,
    pub workers: u32,
}

impl StandaloneOptions {
    /// Parse `--standalone --data-dir <dir> --listen <addr> [--token <t>]
    /// [--public-url <url>] [--workers <n>] [--tools-dir <dir>]`. Both
    /// directories are made absolute here (tools run with other working
    /// directories, and inside WSL, where a relative Windows path means
    /// nothing). `--tools-dir` is applied to the environment
    /// (`ADOS_WE_TOOLS_DIR`). `None` when `--standalone` is absent.
    pub fn from_args(args: &[String]) -> Result<Option<Self>, String> {
        if !args.iter().any(|a| a == "--standalone") {
            return Ok(None);
        }
        let value = |flag: &str| -> Result<Option<String>, String> {
            match args.iter().position(|a| a == flag) {
                Some(i) => args
                    .get(i + 1)
                    .filter(|v| !v.starts_with("--"))
                    .cloned()
                    .map(Some)
                    .ok_or_else(|| format!("{flag} needs a value")),
                None => Ok(None),
            }
        };
        let absolute = |flag: &str, dir: String| {
            crate::tools::absolute_path(Path::new(&dir)).map_err(|e| format!("{flag} {dir}: {e}"))
        };
        let data_dir = value("--data-dir")?.ok_or("--data-dir is required")?;
        let data_dir = absolute("--data-dir", data_dir)?;
        if let Some(t) = value("--tools-dir")? {
            std::env::set_var("ADOS_WE_TOOLS_DIR", absolute("--tools-dir", t)?);
        }
        Ok(Some(Self {
            data_dir,
            listen: value("--listen")?.unwrap_or_else(|| "127.0.0.1:8092".into()),
            token: value("--token")?.or_else(|| std::env::var("ADOS_COMPUTE_TOKEN").ok()),
            public_url: value("--public-url")?,
            workers: value("--workers")?
                .map(|w| {
                    w.parse()
                        .map_err(|_| "--workers needs a number".to_string())
                })
                .transpose()?
                .unwrap_or(1),
        }))
    }
}

/// The owner token: given, else read from (or generated into) `path`. The
/// file is owner-only (an older, wider one is narrowed when read).
fn owner_token(given: Option<String>, path: &Path) -> std::io::Result<String> {
    if let Some(t) = given.filter(|t| !t.trim().is_empty()) {
        return Ok(t.trim().to_string());
    }
    if let Ok(t) = std::fs::read_to_string(path) {
        if !t.trim().is_empty() {
            crate::owner_only::restrict(path)?;
            return Ok(t.trim().to_string());
        }
    }
    let mut bytes = [0u8; 24];
    getrandom::fill(&mut bytes).map_err(|e| std::io::Error::other(e.to_string()))?;
    let token = hex::encode(bytes);
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)?;
    }
    crate::owner_only::create(path)?.write_all(token.as_bytes())?;
    Ok(token)
}

/// This node's id (`standalone-<16 hex>`), generated once into
/// `<node>/node-id` and stable across restarts: the issuer a capture device
/// files its credential under, distinct for every standalone node.
pub fn node_id(node: &Path) -> std::io::Result<String> {
    let path = node.join("node-id");
    if let Ok(id) = std::fs::read_to_string(&path) {
        if !id.trim().is_empty() {
            return Ok(id.trim().to_string());
        }
    }
    let mut bytes = [0u8; 8];
    getrandom::fill(&mut bytes).map_err(|e| std::io::Error::other(e.to_string()))?;
    let id = format!("standalone-{}", hex::encode(bytes));
    std::fs::create_dir_all(node)?;
    std::fs::write(&path, &id)?;
    Ok(id)
}

/// The standalone node's auth over `<node>`: the owner bearer `token`, and the
/// credentials this node (`node_id`) issued, kept in
/// `<node>/node-credentials.json`.
pub fn auth(node: &Path, node_id: &str, token: String) -> ComputeAuth {
    ComputeAuth::new(crate::NodeCredentialStore::open(
        node.join("node-credentials.json"),
        node_id,
    ))
    .with_owner_token(token)
}

fn hostname() -> Option<String> {
    std::env::var("COMPUTERNAME")
        .or_else(|_| std::env::var("HOSTNAME"))
        .ok()
        .filter(|h| !h.trim().is_empty() && !h.eq_ignore_ascii_case("localhost"))
}

/// Everything the standalone node serves on its listener: the job API (with
/// credential issuing), the job files, the session upload lane and the
/// artifacts, each behind `auth`.
pub fn router(
    state: crate::ApiState,
    auth: Arc<ComputeAuth>,
    public_base: &str,
    work_root: PathBuf,
    sessions_root: PathBuf,
) -> axum::Router {
    let on_commit: CommitHook = Arc::new(|s| {
        tracing::info!(session = %s.session_id, files = s.files, bytes = s.bytes, "session committed");
        Ok(())
    });
    build_router_with_base(
        state,
        auth.clone(),
        Arc::from(public_base),
        Arc::new(SessionRegistry::new()),
    )
    .merge(job_files_router(auth.clone(), work_root.clone()))
    .merge(upload_and_artifact_lanes(
        &auth,
        bulk_router(Arc::new(SessionStore::new(sessions_root)), on_commit),
        work_root,
    ))
    .layer(CorsLayer::permissive())
}

/// Run the standalone node until the listener stops.
pub async fn run(opts: StandaloneOptions) -> Result<(), Box<dyn std::error::Error>> {
    let node = opts.data_dir.join("node");
    let work_root = node.join("work");
    let sessions_root = node.join("sessions");
    std::fs::create_dir_all(&sessions_root)?;
    let token = owner_token(opts.token.clone(), &node.join("owner-token"))?;
    let node_id = node_id(&node)?;
    let public_base = derive_public_base(
        &opts.listen,
        opts.public_url.as_deref(),
        hostname().as_deref(),
    );

    let store = JobStore::open(&node.join("jobs.db").to_string_lossy())?;
    let scheduler = Scheduler::new(
        store,
        Arc::new(SelectingReconstructor::new(work_root.clone())),
        Arc::new(NoDetector),
    )
    .with_sessions_root(sessions_root.clone());
    let engine = Engine::new(
        scheduler,
        Cluster::new_master(node_id.clone()),
        opts.workers,
    );
    let runner = engine.runner();
    let state = Arc::new(Mutex::new(engine));
    {
        let engine = state.lock().await;
        let n = engine
            .scheduler()
            .store()
            .requeue_stale_running(crate::worker::now_ms())?;
        if n > 0 {
            tracing::info!(requeued = n, "requeued interrupted jobs");
        }
    }

    let auth = Arc::new(auth(&node, &node_id, token));
    let router = router(
        state.clone(),
        auth,
        &public_base,
        work_root.clone(),
        sessions_root,
    );

    let ctx = WorkerCtx {
        state: state.clone(),
        work_root: work_root.clone(),
        public_base: public_base.clone(),
        runner,
        tour_viewer: crate::tools::tour_viewer_dir(),
        person_model: std::env::var_os("ADOS_WE_PERSON_MODEL").map(PathBuf::from),
    };
    for _ in 0..opts.workers.max(1) {
        tokio::spawn(worker_loop(ctx.clone()));
    }
    let retention = state.clone();
    tokio::spawn(async move {
        loop {
            tokio::time::sleep(Duration::from_secs(60)).await;
            let cutoff = crate::worker::now_ms() - 7 * 86_400_000;
            let engine = retention.lock().await;
            if let Err(e) = engine.scheduler().store().purge_terminal_before(cutoff) {
                tracing::warn!(error = %e, "retention purge failed");
            }
        }
    });

    let listener = TcpListener::bind(&opts.listen).await?;
    tracing::info!(
        listen = %opts.listen,
        public_base = %public_base,
        token_file = %node.join("owner-token").display(),
        tools = %crate::tools::tools_root().display(),
        "standalone reconstruction node listening"
    );
    axum::serve(listener, router).await?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn args(s: &str) -> Vec<String> {
        s.split_whitespace().map(String::from).collect()
    }

    #[test]
    fn standalone_flags_parse() {
        assert_eq!(StandaloneOptions::from_args(&args("node")).unwrap(), None);
        let o = StandaloneOptions::from_args(&args(
            "node --standalone --data-dir D:/x --listen 0.0.0.0:8092 --workers 2 --token abc",
        ))
        .unwrap()
        .unwrap();
        assert!(o.data_dir.is_absolute());
        assert!(o.data_dir.ends_with("x"));
        assert_eq!(o.listen, "0.0.0.0:8092");
        assert_eq!(o.workers, 2);
        assert_eq!(o.token.as_deref(), Some("abc"));
        assert!(StandaloneOptions::from_args(&args("node --standalone")).is_err());
        assert!(StandaloneOptions::from_args(&args("node --standalone --data-dir")).is_err());
    }

    #[test]
    fn a_relative_data_dir_yields_absolute_tool_paths() {
        use crate::recon::{process::command, ResolvedTool, ToolInvocation};
        let o = StandaloneOptions::from_args(&args("node --standalone --data-dir data"))
            .unwrap()
            .unwrap();
        assert_eq!(o.data_dir, std::env::current_dir().unwrap().join("data"));
        let job = o.data_dir.join("node").join("work").join("jobs").join("j1");
        let root = if cfg!(windows) { "/mnt/" } else { "/" };
        let tool = ResolvedTool {
            name: "gsplat_train".into(),
            version: "1".into(),
            program: "/opt/v/bin/python".into(),
            prefix_args: Vec::new(),
            home: PathBuf::new(),
            wsl: true,
        };
        let inv = ToolInvocation::new("gsplat_train", &job).arg(job.join("splat.ply"));
        let argv: Vec<String> = command(&tool, &inv)
            .unwrap()
            .get_args()
            .map(|a| a.to_string_lossy().into_owned())
            .collect();
        assert!(argv[3].starts_with(root), "--cd {}", argv[3]);
        assert!(argv.last().unwrap().starts_with(root), "{argv:?}");
        assert!(argv
            .last()
            .unwrap()
            .ends_with("/node/work/jobs/j1/splat.ply"));
        let inv = crate::calib::container::kalibr_bagcreater(&job, "img");
        let volume = inv.args_lossy()[3].clone();
        assert!(volume.starts_with(root), "{volume}");
        assert!(volume.ends_with("/node/work/jobs/j1:/data"), "{volume}");
    }

    #[test]
    fn the_owner_token_is_generated_once_and_reused() {
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("node/owner-token");
        let a = owner_token(None, &p).unwrap();
        assert_eq!(a.len(), 48);
        assert_eq!(owner_token(None, &p).unwrap(), a);
        crate::owner_only::assert_owner_only(&p);
        assert_eq!(owner_token(Some("given".into()), &p).unwrap(), "given");
    }

    #[test]
    fn an_existing_wider_owner_token_is_narrowed_when_read() {
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("owner-token");
        std::fs::write(&p, "abc").unwrap();
        assert_eq!(owner_token(None, &p).unwrap(), "abc");
        crate::owner_only::assert_owner_only(&p);
    }

    #[test]
    fn each_node_keeps_its_own_stable_id() {
        let (a, b) = (tempfile::tempdir().unwrap(), tempfile::tempdir().unwrap());
        let id_a = node_id(a.path()).unwrap();
        let id_b = node_id(b.path()).unwrap();
        assert!(
            id_a.starts_with("standalone-") && id_a.len() == 27,
            "{id_a}"
        );
        assert_ne!(id_a, id_b);
        assert_eq!(node_id(a.path()).unwrap(), id_a);
    }
}
