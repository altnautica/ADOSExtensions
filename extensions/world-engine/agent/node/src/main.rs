//! `world-engine-node`: the World Engine extension's `node` service on a
//! workstation or compute node. It opens the job store, builds the node engine,
//! runs a worker loop that drains the queue and periodically reclaims terminal
//! jobs, and serves the REST job API plus the drone lanes on one TCP listener
//! and, for the operator, on the plugin's `http.sock`.
//!
//! Host connection: the plugin host starts it as
//! `world-engine-node com.altnautica.world-engine` with `ADOS_PLUGIN_SOCKET` +
//! `ADOS_PLUGIN_TOKEN` set; it then connects to the host and uses the plugin
//! config (`serving.*`, `atlas.live_reconstruct`), publishes its compute status
//! on the `status` telemetry channel, and mirrors its reconstruct jobs into the
//! `jobs` cloud records. Without them (a standalone dev run) it runs on
//! defaults and makes no host call.
//!
//! Reach: the operator reaches the node only through `http.sock` (ados-control
//! authenticated them, so every request there is the owner). The TCP listener
//! is for other nodes: each lane admits only a credential this node issued for
//! it, and owner-only routes are refused over TCP whatever is presented, so
//! binding a non-loopback address is safe. It still defaults to `127.0.0.1`;
//! the installer opts a node into serving the LAN with `ADOS_COMPUTE_BIND`.
//!
//! Worker note: the worker claims the next job under the engine lock, then
//! releases the lock and runs the (real, possibly minutes-long) backend
//! WITHOUT it, so a long reconstruction never blocks the API. It re-acquires
//! the lock only briefly to record the terminal state; a cancel that lands
//! during the run wins (`Scheduler::finalize` refuses to overwrite a job that
//! is no longer `Running`).
//!
//! Environment (every default under `<ADOS_PLUGIN_DATA_DIR>/node/` applies when
//! the host set a data dir; the `/var/ados/compute` defaults otherwise):
//! - `ADOS_COMPUTE_DB`        job store path (default `<data>/node/jobs.db`)
//! - `ADOS_COMPUTE_WORK`      dataset + artifact work root (default
//!   `<data>/node/work`); the persister writes keyframe datasets here, the
//!   reconstructor writes artifacts here, and the artifact route serves from here
//! - `ADOS_COMPUTE_NODE_CREDENTIALS` the store of credentials issued to drones
//!   (default `<data>/node/node-credentials.json`)
//! - `ADOS_COMPUTE_BIND`      bind address (default `127.0.0.1:8092`, loopback)
//! - `ADOS_COMPUTE_PUBLIC_URL` base URL the GCS fetches artifacts from (default
//!   derived from the bind address, substituting the node hostname for a wildcard
//!   bind); the artifact URL is `<public_url>/artifacts/<relpath>`
//! - `ADOS_COMPUTE_NODE_ID`   this node's id (default derived from the machine id)
//! - `ADOS_COMPUTE_WORKERS`   worker slots (default `1`)
//! - `ADOS_COMPUTE_RETENTION_S` terminal-job retention seconds (default `86400`)
//! - `ADOS_ATLAS_LIVE`        overrides the `atlas.live_reconstruct` plugin config
//!   (live in-flight reconstruction, default off)

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use ados_sdk::{PluginContext, PluginIpcClient, RunnerArgs};
use tokio::net::TcpListener;
use tokio::sync::{mpsc, Mutex};
use tower_http::cors::CorsLayer;
use world_engine_node::{
    build_atlas_jobs, build_rerun_output, build_router_with_base, derive_descriptors,
    derive_occupancy, derive_public_base, lane_router, load_serving_config, operator_socket_router,
    put_accepted, rewrite_output_to_artifact_url, status_payload, submit_reconstruct_job,
    AtlasIngest, AtlasLanes, BackendResult, Cluster, ComputeAuth, ComputeHeartbeatSidecar,
    ComputeJobState, DetectionBroadcaster, Detector, Engine, JobStore, JobsCloudSync, LaneRoutes,
    LatestComputeStatus, LiveReconstructConfig, MockDetector, NodeCredentialStore,
    OffloadSessionManager, Prepared, PreparedInput, Scheduler, SelectingReconstructor, SessionSpec,
    WorldDescriptorSet, DEFAULT_NODE_CREDENTIALS_PATH, JOBS_COLLECTION,
};
use world_engine_protocol::STATUS_CHANNEL;
use world_engine_transport::{AtlasEvent, WorldBroadcaster};

/// Bounded capacity of the Atlas event receive channel. When it fills, the event
/// router returns `503` so the sender's failover ladder retries or drops — the
/// reconstructor running behind never grows an unbounded in-memory queue.
const ATLAS_EVENT_CHANNEL_CAP: usize = 256;
/// How often the live-reconstruction cadence is evaluated (the interval trigger's
/// granularity + the skip-while-running reconcile). The cadence's own thresholds
/// are much coarser (tens of seconds / keyframes); this is just the poll period.
const LIVE_CADENCE_TICK_SECS: u64 = 2;
/// How often the receiver looks for capture sessions that went silent without
/// a `Bagged` frame.
const IDLE_SWEEP: Duration = Duration::from_secs(60);
/// Broadcaster/channel depth (batches) for the streaming-offload detection return
/// lane. A slow WS subscriber past this buffer lags and skips (never blocking the
/// detector); the pump keeps pace with the detector on this bound.
const OFFLOAD_WS_CHANNEL_CAP: usize = 64;
/// Per-subscriber buffer depth (descriptors) on the world-model descriptor
/// stream. A descriptor is a few hundred bytes and one generation emits at most
/// four, so 64 gives a subscriber room to miss several generations of a fast
/// live cadence before it lags and skips.
const WORLD_WS_CHANNEL_CAP: usize = 64;
/// How often the compute status is sampled and published (`status` channel).
const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(5);
/// How often the reconstruct jobs are mirrored into the cloud records. Jobs are
/// slow-moving and the local-first path is primary, so a coarser cadence than
/// the heartbeat; only new or changed records are put.
const JOBS_SYNC_INTERVAL: Duration = Duration::from_secs(15);
/// The plugin config key gating live (in-flight) reconstruction.
const ATLAS_LIVE_KEY: &str = "atlas.live_reconstruct";

fn init_logging() {
    use tracing_subscriber::EnvFilter;
    let filter = std::env::var("RUST_LOG").unwrap_or_else(|_| "info".to_string());
    let _ = tracing_subscriber::fmt()
        .with_env_filter(EnvFilter::new(filter))
        .try_init();
}

fn env_or(key: &str, default: &str) -> String {
    std::env::var(key).unwrap_or_else(|_| default.to_string())
}

/// Pick the perception-offload detector. When the `onnx` feature is built AND
/// `ADOS_COMPUTE_DETECTOR_MODEL` (or the `serving.detector_model` plugin config)
/// names an `.onnx` file, load the real ONNX detector (CoreML-accelerated on
/// macOS via the `coreml` feature); otherwise fall back to the mock so the
/// offload path stays exercised with no model. A load failure logs and falls
/// back to the mock rather than refusing to start (the node still comes up).
fn select_detector(config_model_path: Option<&str>) -> Arc<dyn Detector> {
    // Used only in the onnx build (the mock path ignores it).
    #[cfg(not(feature = "onnx"))]
    let _ = config_model_path;
    #[cfg(feature = "onnx")]
    {
        // The env wins (the bench override); else the operator-picked model from
        // `serving.detector_model` (resolved to a path). Empty ⇒ mock.
        let path = match env_or("ADOS_COMPUTE_DETECTOR_MODEL", "") {
            p if !p.is_empty() => p,
            _ => config_model_path.unwrap_or("").to_string(),
        };
        if !path.is_empty() {
            let (iw, ih) = parse_input_dims(&env_or("ADOS_COMPUTE_DETECTOR_INPUT", "640x640"));
            let classes: Vec<String> = std::env::var("ADOS_COMPUTE_DETECTOR_CLASSES")
                .ok()
                .map(|s| {
                    s.split(',')
                        .map(|c| c.trim().to_string())
                        .filter(|c| !c.is_empty())
                        .collect()
                })
                .unwrap_or_default();
            let head = match env_or("ADOS_COMPUTE_DETECTOR_HEAD", "yolo8").as_str() {
                "yolo5" => ados_protocol::framebus::DetectionHead::Yolo5,
                _ => ados_protocol::framebus::DetectionHead::Yolo8,
            };
            let meta = ados_protocol::framebus::ModelMetadata {
                id: "offload-detector".into(),
                kind: ados_protocol::framebus::ModelKind::Detection,
                execution: ados_protocol::framebus::ModelExecution::EngineRun,
                input_width: iw,
                input_height: ih,
                input_format: ados_protocol::framebus::FrameFormat::Rgb24,
                output_classes: classes,
                model_path: Some(path.clone()),
                head,
            };
            match world_engine_node::OnnxDetector::from_model(&meta) {
                Ok(det) => {
                    tracing::info!(model = %path, input = format!("{iw}x{ih}"), "perception offload: real ONNX detector loaded");
                    return Arc::new(det);
                }
                Err(e) => {
                    tracing::warn!(model = %path, error = %e, "perception offload: ONNX load failed, using mock");
                }
            }
        }
    }
    Arc::new(MockDetector)
}

/// Parse a `WxH` input-size string (e.g. `640x640`) into `(width, height)`,
/// defaulting to 640 square on a malformed value.
#[cfg(feature = "onnx")]
fn parse_input_dims(s: &str) -> (u32, u32) {
    let mut it = s.split(['x', 'X']);
    let w = it.next().and_then(|v| v.trim().parse().ok()).unwrap_or(640);
    let h = it.next().and_then(|v| v.trim().parse().ok()).unwrap_or(w);
    (w, h)
}

/// Resolve a stable per-node id: the `ADOS_COMPUTE_NODE_ID` override if set,
/// else derived from the host `machine-id` so two compute nodes never collide on
/// the mDNS instance / deviceId / cluster identity, else a generic fallback.
/// Pure (the inputs are injected) so the derivation is unit-tested.
fn derive_node_id(env: Option<String>, machine_id: Option<String>) -> String {
    if let Some(id) = env {
        let id = id.trim();
        if !id.is_empty() {
            return id.to_string();
        }
    }
    if let Some(mid) = machine_id {
        let mid = mid.trim();
        if !mid.is_empty() {
            return format!("compute-{}", &mid[..mid.len().min(12)]);
        }
    }
    "compute-node".to_string()
}

fn resolve_node_id() -> String {
    derive_node_id(
        std::env::var("ADOS_COMPUTE_NODE_ID").ok(),
        std::fs::read_to_string("/etc/machine-id").ok(),
    )
}

fn now_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0)
}

/// True when a config flag string is an affirmative (`1`/`true`/`yes`/`on`).
fn is_truthy(v: &str) -> bool {
    matches!(
        v.trim().to_ascii_lowercase().as_str(),
        "1" | "true" | "yes" | "on"
    )
}

/// Where the node keeps its state.
#[derive(Debug, Clone, PartialEq)]
struct StatePaths {
    db: String,
    work_root: PathBuf,
    credentials: PathBuf,
}

impl StatePaths {
    /// Each path is its `ADOS_COMPUTE_*` override when set; else under
    /// `<ADOS_PLUGIN_DATA_DIR>/node/` when the host set a data dir; else the
    /// `/var/ados/compute` default. Pure (the env lookup is injected).
    fn resolve(env: impl Fn(&str) -> Option<String>) -> Self {
        let data = env(world_engine_protocol::paths::DATA_DIR_ENV)
            .filter(|d| !d.trim().is_empty())
            .map(|d| Path::new(&d).join("node"));
        let under = |leaf: &str, fallback: &str| match &data {
            Some(dir) => dir.join(leaf),
            None => PathBuf::from(fallback),
        };
        let pick = |key: &str, leaf: &str, fallback: &str| {
            env(key)
                .filter(|v| !v.trim().is_empty())
                .map(PathBuf::from)
                .unwrap_or_else(|| under(leaf, fallback))
        };
        StatePaths {
            db: pick("ADOS_COMPUTE_DB", "jobs.db", "/var/ados/compute/jobs.db")
                .to_string_lossy()
                .into_owned(),
            work_root: pick("ADOS_COMPUTE_WORK", "work", "/var/ados/compute/work"),
            credentials: pick(
                "ADOS_COMPUTE_NODE_CREDENTIALS",
                "node-credentials.json",
                DEFAULT_NODE_CREDENTIALS_PATH,
            ),
        }
    }
}

/// Whether live (in-flight) reconstruction is on: the `ADOS_ATLAS_LIVE` env
/// override wins, else the `atlas.live_reconstruct` plugin config. Opt-in
/// (default off). Pure so the precedence is unit-tested.
fn live_enabled(env: Option<&str>, config: Option<bool>) -> bool {
    match env {
        Some(v) => is_truthy(v),
        None => config.unwrap_or(false),
    }
}

/// Build the live-reconstruction cadence config. The cadence defaults (every 30
/// keyframes / 20 s / 8-keyframe floor) come from
/// [`LiveReconstructConfig::default`]; `enabled` is set from the gate, and the
/// thresholds can be tuned via env.
async fn live_reconstruct_config(ctx: Option<&PluginContext>) -> LiveReconstructConfig {
    let config = match ctx {
        Some(ctx) => match ctx
            .config
            .get(ATLAS_LIVE_KEY, rmpv::Value::Boolean(false))
            .await
        {
            Ok(v) => v.as_bool(),
            Err(e) => {
                tracing::warn!(key = ATLAS_LIVE_KEY, error = %e, "plugin config read failed; using the default");
                None
            }
        },
        None => None,
    };
    let mut cfg = LiveReconstructConfig {
        enabled: live_enabled(std::env::var("ADOS_ATLAS_LIVE").ok().as_deref(), config),
        ..LiveReconstructConfig::default()
    };
    if let Some(v) = env_u64("ADOS_ATLAS_LIVE_EVERY_KEYFRAMES") {
        cfg.every_keyframes = v.max(1);
    }
    if let Some(s) = env_u64("ADOS_ATLAS_LIVE_INTERVAL_S") {
        cfg.interval_ms = (s as i64).saturating_mul(1000);
    }
    if let Some(v) = env_u64("ADOS_ATLAS_LIVE_MIN_KEYFRAMES") {
        cfg.min_keyframes = v.max(1);
    }
    cfg
}

/// Parse a `u64` env var, or `None` when unset / unparseable.
fn env_u64(key: &str) -> Option<u64> {
    std::env::var(key).ok().and_then(|v| v.trim().parse().ok())
}

/// Connect to the plugin host when the node was started as the extension's
/// service (the plugin id in argv[1] plus `ADOS_PLUGIN_SOCKET` and
/// `ADOS_PLUGIN_TOKEN`). `Ok(None)` is a standalone dev run: no host, defaults
/// everywhere. A host that is named but refuses the connection is an error, so
/// the service fails loudly instead of silently running unmanaged.
async fn connect_host() -> Result<Option<PluginContext>, ados_sdk::RunnerError> {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    let env = |k: &str| std::env::var(k).ok().filter(|v| !v.trim().is_empty());
    let Ok(args) = RunnerArgs::parse(&argv, env) else {
        return Ok(None);
    };
    let (Some(socket), Some(token)) = (args.socket_path.clone(), args.token.clone()) else {
        return Ok(None);
    };
    let ipc = Arc::new(PluginIpcClient::new(args.plugin_id.clone(), token, socket));
    ipc.connect().await?;
    if let Some(dir) = args.data_dir.as_deref() {
        if let Err(e) = std::fs::create_dir_all(dir) {
            tracing::warn!(dir, error = %e, "could not create plugin data dir");
        }
    }
    Ok(Some(PluginContext::new(
        ipc,
        env!("CARGO_PKG_VERSION"),
        args.agent_id,
        args.data_dir,
        BTreeMap::new(),
    )))
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    init_logging();

    let ctx = connect_host().await?;
    match &ctx {
        Some(c) => tracing::info!(plugin_id = %c.plugin_id, "connected to the plugin host"),
        None => tracing::info!("no plugin host (standalone run): host calls skipped"),
    }

    let paths = StatePaths::resolve(|k| std::env::var(k).ok());
    let bind = env_or("ADOS_COMPUTE_BIND", "127.0.0.1:8092");
    let node_id = resolve_node_id();
    let workers: u32 = env_or("ADOS_COMPUTE_WORKERS", "1").parse().unwrap_or(1);
    let retention_ms: i64 = env_or("ADOS_COMPUTE_RETENTION_S", "86400")
        .parse::<i64>()
        .unwrap_or(86_400)
        .saturating_mul(1000);

    // The base URL the GCS fetches artifacts from: the explicit override, else
    // derived from the bind (the node hostname stands in for a wildcard bind so
    // the URL is reachable off-box). The artifact host matches the mDNS target
    // exactly — both resolve through `ados_protocol::reach` — and a host with
    // no dialable name falls back to loopback rather than to a fabricated one.
    let public_base = derive_public_base(
        &bind,
        std::env::var("ADOS_COMPUTE_PUBLIC_URL").ok().as_deref(),
        ados_protocol::reach::system_hostname().as_deref(),
    );

    if paths.db != ":memory:" {
        if let Some(parent) = Path::new(&paths.db).parent() {
            let _ = std::fs::create_dir_all(parent);
        }
    }
    // The work root holds keyframe datasets and reconstruction artifacts.
    let work_root = paths.work_root.clone();
    let _ = std::fs::create_dir_all(&work_root);

    let store = JobStore::open(&paths.db)?;
    // The offload-serving config: the toggle (auto-serve unless disabled) + the
    // operator-picked detector model. Read once at startup (a change takes effect
    // on the next service restart).
    let serving = load_serving_config(ctx.as_ref()).await;
    if !serving.serve_offload {
        tracing::info!("perception offload serving disabled by config (serving.enabled=off)");
    }
    // The detector is shared between the one-shot job path (the scheduler) and the
    // streaming-session path (the session manager below), so both run the same
    // real ONNX model (or the mock in CI) over their frames.
    let detector = select_detector(serving.detector_model_path.as_deref());
    // The reconstructor picks the real backend per job (Brush when installed),
    // falling back to the mock (CI / no-GPU), and writes artifacts under work_root.
    let scheduler = Scheduler::new(
        store,
        Arc::new(SelectingReconstructor::new(work_root.clone())),
        detector.clone(),
    );
    let mut engine = Engine::new(scheduler, Cluster::new_master(node_id.clone()), workers);

    // The streaming perception-offload lane: a PerceptionOffload job carrying a
    // `session` block (an NPU-less drone streaming its live camera) starts a
    // continuous frames->detections session whose batches fan out over this
    // broadcaster; the drone subscribes on the WS route mounted below. The
    // one-shot per-frame offload job path is unchanged.
    let offload_broadcaster = Arc::new(DetectionBroadcaster::new(OFFLOAD_WS_CHANNEL_CAP));
    let offload_sessions = Arc::new(OffloadSessionManager::new(
        offload_broadcaster.clone(),
        detector.clone(),
        serving.serve_offload,
    ));
    // Share the session counter so the node heartbeat reflects live offload
    // sessions (a node streaming to N drones reports N, not 0).
    engine.set_session_counter(offload_sessions.session_counter());
    let state = Arc::new(Mutex::new(engine));

    // Startup recovery: a job left in Running (the node crashed mid-backend)
    // is neither claimable nor purgeable, so requeue it before the workers start.
    {
        let engine = state.lock().await;
        match engine.scheduler().store().requeue_stale_running(now_ms()) {
            Ok(n) if n > 0 => {
                tracing::info!(requeued = n, "requeued stale running jobs at startup")
            }
            Ok(_) => {}
            Err(e) => tracing::error!(error = %e, "startup requeue failed"),
        }
    }

    // Fan-out for world-model descriptors: the workers publish each completed
    // generation into it and `/ws/atlas/<device_id>` subscribers read it.
    let world_broadcaster = Arc::new(WorldBroadcaster::new(WORLD_WS_CHANNEL_CAP));

    // One worker task per configured slot. Each claims a distinct job atomically
    // (claim_next_queued), runs its backend WITHOUT the engine lock, and
    // finalizes under the lock, so N backends run in parallel while the API stays
    // responsive. A separate task runs retention on a fixed cadence.
    for _ in 0..workers.max(1) {
        let ws = state.clone();
        let wr = work_root.clone();
        let pb = public_base.clone();
        let sessions = offload_sessions.clone();
        let world = world_broadcaster.clone();
        tokio::spawn(async move { worker_loop(ws, wr, pb, sessions, world).await });
    }
    let rs = state.clone();
    tokio::spawn(async move { retention_loop(rs, retention_ms).await });
    // The stall watchdog flips a Live session that went quiet (its source froze /
    // the drone paused, with no reader error) to Stalled, so the session surfaces
    // report the honest quiet state rather than a stale Live.
    offload_sessions.spawn_stall_watchdog();

    // The compute status: sampled every heartbeat tick, served on
    // `GET /compute/status` (http.sock), and published on the `status`
    // telemetry channel when a host is connected.
    let latest_status = Arc::new(LatestComputeStatus::new());
    {
        let hs = state.clone();
        let latest = latest_status.clone();
        let ctx = ctx.clone();
        tokio::spawn(async move { heartbeat_loop(hs, latest, ctx).await });
    }
    // The reconstruct jobs, mirrored into the `jobs` cloud records. The poster
    // and compute-node identity is this node's paired device id, else its node id.
    if let Some(ctx) = ctx.clone() {
        let compute_node_id = if ctx.agent_id.trim().is_empty() {
            node_id.clone()
        } else {
            ctx.agent_id.clone()
        };
        let js = state.clone();
        tokio::spawn(async move { jobs_sync_loop(js, ctx, compute_node_id).await });
    }

    // The credentials this node issues drones for its lanes, persisted beside
    // the job store.
    let credentials = NodeCredentialStore::open(paths.credentials.clone(), node_id.clone());
    let auth = Arc::new(ComputeAuth::new(credentials));

    // Atlas world-model receiver: POST /api/atlas/event is mounted beside the job
    // API and decoded events drain into the job queue (a bagged capture-state
    // submits the reconstruct job the workers pick up), and every completed
    // generation's descriptors are served per device at /ws/atlas/<device_id>.
    // Installing the extension on a node is the opt-in, so both always mount.
    let live_config = live_reconstruct_config(ctx.as_ref()).await;
    let (atlas_tx, atlas_rx) = mpsc::channel::<AtlasEvent>(ATLAS_EVENT_CHANNEL_CAP);
    {
        let rs = state.clone();
        let wr = work_root.clone();
        tokio::spawn(async move { atlas_receiver_loop(atlas_rx, rs, wr, live_config).await });
    }
    tracing::info!(
        channel_cap = ATLAS_EVENT_CHANNEL_CAP,
        live_reconstruct = live_config.enabled,
        "world-model event receiver mounted at POST /api/atlas/event, \
         descriptor stream at GET /ws/atlas/<device_id>"
    );

    // The owner's job API, plus the lanes a drone uses on the same listener
    // (artifacts, the offload detection return stream, the Atlas ingest and
    // world stream), each behind its own credential gate.
    let shared = build_router_with_base(
        state.clone(),
        auth.clone(),
        std::sync::Arc::from(public_base.as_str()),
        // The live session registry so /api/compute/status carries the state
        // breakdown + /api/compute/sessions serves the live records.
        offload_sessions.registry(),
    )
    .merge(lane_router(
        auth,
        LaneRoutes {
            work_root: work_root.clone(),
            offload: offload_broadcaster.clone(),
            atlas: AtlasLanes {
                events: atlas_tx,
                world: world_broadcaster.clone(),
            },
        },
    ));

    // The operator socket: the same router, served as the owner, plus
    // `GET /compute/status`. Best-effort: a node that cannot bind it still
    // serves its drones over TCP.
    serve_operator_socket(operator_socket_router(shared.clone(), latest_status));

    // Permissive CORS so the GCS can read this listener cross-origin from the
    // browser on an HTTP origin. Outermost layer: OPTIONS preflights are
    // answered ahead of the per-route gates.
    let router = shared.layer(CorsLayer::permissive());

    let listener = TcpListener::bind(&bind).await?;
    tracing::info!(bind = %bind, workers, "compute job API listening (node credentials per lane)");
    // Advertise on mDNS so the GCS Add-a-Node card auto-discovers this node for
    // LAN pairing. Best-effort: a None means no auto-discovery, manual
    // add-by-IP still works. Held for the process lifetime (unregisters on exit).
    let job_port = bind
        .rsplit(':')
        .next()
        .and_then(|p| p.parse::<u16>().ok())
        .unwrap_or(8092);
    let _mdns_advert = world_engine_transport::advertise_compute(&node_id, job_port);
    axum::serve(listener, router).await?;
    Ok(())
}

/// Bind the plugin's `http.sock` and serve `router` on it for the process
/// lifetime. A bind failure is logged; the TCP listener carries on.
fn serve_operator_socket(router: axum::Router) {
    let path = world_engine_protocol::paths::http_socket();
    if let Some(parent) = path.parent() {
        if let Err(e) = std::fs::create_dir_all(parent) {
            tracing::error!(dir = %parent.display(), error = %e, "http.sock dir create failed; operator socket not served");
            return;
        }
    }
    match ados_sdk::http::bind(&path) {
        Ok(listener) => {
            tracing::info!(path = %path.display(), "operator socket listening");
            tokio::spawn(world_engine_transport::serve_unix(
                listener,
                router,
                std::future::pending::<()>(),
            ));
        }
        Err(e) => {
            tracing::error!(path = %path.display(), error = %e, "http.sock bind failed; operator socket not served")
        }
    }
}

/// One worker: claim a job under the lock, run the backend WITHOUT it, finalize
/// under the lock. Idles 500 ms when the queue is empty. A cancel that lands
/// during the backend run wins inside `finalize`. A real `file://` artifact under
/// the work root is rewritten to a fetchable LAN URL (the GCS reads it as the
/// output URL) before finalize, keeping the local path for any pipeline chaining.
async fn worker_loop(
    state: Arc<Mutex<Engine>>,
    work_root: PathBuf,
    public_base: String,
    sessions: Arc<OffloadSessionManager>,
    world: Arc<WorldBroadcaster>,
) {
    loop {
        let (prepared, reconstructor, detector) = {
            let engine = state.lock().await;
            let prepared = engine.scheduler().claim_and_prepare(now_ms());
            let (reconstructor, detector) = engine.scheduler().backends();
            (prepared, reconstructor, detector)
        };
        match prepared {
            Ok(Prepared::Ready { job, input }) => {
                // A PerceptionOffload job carrying a `session` block requests a
                // continuous streaming session, not one-shot inference: start the
                // session (the node pulls the drone's RTSP feed -> detector ->
                // broadcaster -> the WS the drone subscribes to) and finalize the
                // job as accepted. The one-shot per-frame path below is unchanged.
                if let Some(spec) = SessionSpec::from_job_params(&job.params) {
                    let session_id = spec.id.clone();
                    sessions.start(spec).await;
                    let outcome = {
                        let engine = state.lock().await;
                        engine.scheduler().finalize(
                            &job,
                            BackendResult {
                                outputs: Vec::new(),
                                detections: Vec::new(),
                                error: None,
                            },
                            now_ms(),
                        )
                    };
                    match outcome {
                        Ok(o) => {
                            tracing::info!(job = %o.job_id, session = %session_id, "offload streaming session started")
                        }
                        Err(e) => {
                            tracing::error!(job = %job.id, error = %e, "offload session job finalize failed")
                        }
                    }
                    continue;
                }
                // Reconstruction is best-effort batch work and runs off this worker
                // on a blocking thread; a streaming perception-offload session is
                // latency-critical and runs on its OWN task, outside this batch
                // worker pool. They do not queue behind each other, so batch cannot
                // starve streaming of a worker slot. Before starting the heavy
                // reconstruction backend we still yield once when a session is live,
                // so any ready streaming work is polled first (WorkPriority::Streaming
                // > Batch). This is a cooperative nudge; fine-grained CPU/GPU
                // preemption of an in-flight reconstruction is out of scope.
                if matches!(input, PreparedInput::Reconstruct(_))
                    && sessions.should_yield_to_streaming()
                {
                    tokio::task::yield_now().await;
                }
                // Run the (real, possibly minutes-long) backend off the async
                // runtime thread so a long reconstruction never starves the HTTP
                // surface. The owned job + input move into the blocking task and
                // back out (PreparedInput is not Clone, and both are needed after
                // the run — input for the rerun/world-model write, job for finalize).
                let now = now_ms();
                let failed_job = job.clone();
                let joined = tokio::task::spawn_blocking(move || {
                    let r = Scheduler::run_backend(&*reconstructor, &*detector, &job, &input, now);
                    (r, job, input)
                })
                .await;
                // A panicking backend fails its job and the worker keeps
                // claiming: letting the panic unwind this task would take the
                // slot down for the life of the daemon and strand the job in
                // Running until the next restart.
                let (mut result, job, input) = match joined {
                    Ok(done) => done,
                    Err(e) => {
                        tracing::error!(job = %failed_job.id, error = %e, "backend task panicked");
                        let engine = state.lock().await;
                        if let Err(e) = engine.scheduler().finalize(
                            &failed_job,
                            BackendResult {
                                outputs: Vec::new(),
                                detections: Vec::new(),
                                error: Some(format!("backend panicked: {e}")),
                            },
                            now_ms(),
                        ) {
                            tracing::error!(job = %failed_job.id, error = %e, "finalize failed");
                        }
                        continue;
                    }
                };
                // Write the Rerun world-model .rrd from the real capture + the
                // reconstruction geometry so the GCS World viewer renders real data
                // (camera trajectory + the reconstructed point cloud). Reconstruct
                // jobs only; an offload job has no world model. Best-effort: a write
                // fault is logged and the job still completes with its other output.
                if let PreparedInput::Reconstruct(dataset) = &input {
                    let input_path = dataset
                        .meta
                        .get("input_path")
                        .and_then(|v| v.as_str())
                        .map(std::path::Path::new);
                    let geometry = result
                        .outputs
                        .first()
                        .map(|o| (o.kind.as_str(), o.uri.as_str()));
                    match build_rerun_output(&work_root, &job.id, input_path, geometry, now_ms()) {
                        Ok(Some(rerun_out)) => result.outputs.push(rerun_out),
                        Ok(None) => {}
                        Err(e) => {
                            tracing::warn!(job = %job.id, error = %e, "rerun world-model write failed")
                        }
                    }
                }
                for output in &mut result.outputs {
                    rewrite_output_to_artifact_url(output, &work_root, &public_base);
                }
                // Publish the generation as shared world-model DATA before
                // finalize consumes the result. This is the step that turns Atlas
                // from a private pipeline ending in a viewer into a consumable
                // data product: a plugin, a planner, or the GCS reads the
                // descriptors and pulls only the artifact it needs.
                publish_world_generation(&world, &job, &result.outputs, &work_root, &public_base)
                    .await;
                let outcome = {
                    let engine = state.lock().await;
                    engine.scheduler().finalize(&job, result, now_ms())
                };
                match outcome {
                    Ok(o) => tracing::info!(job = %o.job_id, state = ?o.state, "ran job"),
                    Err(e) => tracing::error!(job = %job.id, error = %e, "finalize failed"),
                }
            }
            Ok(Prepared::Failed(o)) => {
                tracing::info!(job = %o.job_id, state = ?o.state, "job failed at prepare");
            }
            Ok(Prepared::Empty) => tokio::time::sleep(Duration::from_millis(500)).await,
            Err(e) => {
                tracing::error!(error = %e, "worker claim failed");
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        }
    }
}

/// Publish one completed reconstruction generation onto the world-model
/// descriptor stream, tagged with the drone that captured it.
///
/// Derived facts only: a gaussian count comes from the backend's own metadata or
/// from parsing the real `.ply`, bounds come from the real points, and the ESDF
/// is computed from the real geometry — a generation with nothing readable
/// publishes nothing rather than an empty world. Best-effort throughout: a
/// publish reaching zero subscribers is normal (no consumer connected), and a
/// derivation fault is logged without touching the job's own outcome.
async fn publish_world_generation(
    world: &Arc<WorldBroadcaster>,
    job: &world_engine_node::JobRecord,
    outputs: &[world_engine_node::Output],
    work_root: &std::path::Path,
    public_base: &str,
) {
    // A reconstruct job always carries its capture session; without one there is
    // nothing to attribute a world model to.
    let Some(session_id) = job.params.get("session_id").and_then(|v| v.as_str()) else {
        return;
    };
    // The drone the descriptors belong to. Absent on a pre-attribution capture,
    // and the per-device stream cannot route without it, so skip rather than
    // publish a world model under an empty device id.
    let Some(device_id) = job.params.get("device_id").and_then(|v| v.as_str()) else {
        tracing::debug!(job = %job.id, "world-model generation has no device attribution; not published");
        return;
    };
    let generation = job
        .params
        .get("generation")
        .and_then(|v| v.as_u64())
        .unwrap_or(0);

    let mut set: WorldDescriptorSet =
        derive_descriptors(session_id, generation, outputs, work_root);
    // The ESDF is a real distance transform over the reconstruction, so it runs
    // off the async runtime like the backend does.
    let (sid, outs, wr, jid, pb) = (
        session_id.to_string(),
        outputs.to_vec(),
        work_root.to_path_buf(),
        job.id.clone(),
        public_base.to_string(),
    );
    match tokio::task::spawn_blocking(move || {
        derive_occupancy(&sid, generation, &outs, &wr, &jid, &pb)
    })
    .await
    {
        Ok(Ok(Some((desc, grid)))) => {
            tracing::info!(
                job = %job.id,
                generation,
                dims = ?desc.dims,
                resolution_m = desc.resolution_m,
                voxels = grid.distances.len(),
                "atlas esdf derived"
            );
            set.occupancy = Some(desc);
        }
        Ok(Ok(None)) => {}
        Ok(Err(e)) => tracing::warn!(job = %job.id, error = %e, "atlas_esdf_derive_failed"),
        Err(e) => tracing::warn!(job = %job.id, error = %e, "atlas_esdf_task_panicked"),
    }

    if set.is_empty() {
        return;
    }
    let mut published = 0usize;
    for (topic, payload) in [
        (
            world_engine_protocol::atlas::PLUGIN_ATLAS_SPLAT_TOPIC,
            set.splat.as_ref().map(|d| d.to_msgpack()),
        ),
        (
            world_engine_protocol::atlas::PLUGIN_ATLAS_POINTCLOUD_TOPIC,
            set.pointcloud.as_ref().map(|d| d.to_msgpack()),
        ),
        (
            world_engine_protocol::atlas::PLUGIN_ATLAS_MESH_TOPIC,
            set.mesh.as_ref().map(|d| d.to_msgpack()),
        ),
        (
            world_engine_protocol::atlas::PLUGIN_ATLAS_OCCUPANCY_TOPIC,
            set.occupancy.as_ref().map(|d| d.to_msgpack()),
        ),
    ] {
        match payload {
            Some(Ok(body)) => {
                world.publish(
                    device_id,
                    AtlasEvent::new(topic, Some(device_id.to_string()), body),
                );
                published += 1;
            }
            Some(Err(e)) => {
                tracing::warn!(topic, error = %e, "world_descriptor_encode_failed")
            }
            None => {}
        }
    }
    tracing::info!(
        job = %job.id,
        device = %device_id,
        session = %session_id,
        generation,
        descriptors = published,
        subscribers = world.subscriber_count(),
        "world-model generation published as shared data"
    );
}

/// Periodically reclaim terminal jobs older than the retention window.
async fn retention_loop(state: Arc<Mutex<Engine>>, retention_ms: i64) {
    loop {
        tokio::time::sleep(Duration::from_secs(60)).await;
        let engine = state.lock().await;
        match engine
            .scheduler()
            .store()
            .purge_terminal_before(now_ms() - retention_ms)
        {
            Ok(n) if n > 0 => tracing::info!(removed = n, "retention purge"),
            Ok(_) => {}
            Err(e) => tracing::warn!(error = %e, "retention purge failed"),
        }
    }
}

/// Every 5 s, snapshot the engine heartbeat + the host-GPU block, keep it as
/// the latest compute status (`GET /compute/status`), and publish it on the
/// `status` telemetry channel when a host is connected. The GPU read shells out
/// to `system_profiler`/`powermetrics`, so it runs on a blocking thread to keep
/// the async runtime responsive; a join failure degrades to an all-null GPU
/// block. Best-effort: a store or publish error is logged, never fatal.
async fn heartbeat_loop(
    state: Arc<Mutex<Engine>>,
    latest: Arc<LatestComputeStatus>,
    ctx: Option<PluginContext>,
) {
    loop {
        tokio::time::sleep(HEARTBEAT_INTERVAL).await;
        let now = now_ms();
        let hb = state.lock().await.heartbeat();
        let hb = match hb {
            Ok(hb) => hb,
            Err(e) => {
                tracing::warn!(error = %e, "compute heartbeat snapshot failed");
                continue;
            }
        };
        let gpu = tokio::task::spawn_blocking(world_engine_node::gpu::sample)
            .await
            .unwrap_or_default();
        let status = ComputeHeartbeatSidecar::from_heartbeat(&hb, gpu, now);
        latest.set(status.clone());
        let Some(ctx) = &ctx else { continue };
        match rmpv::ext::to_value(status_payload(&status)) {
            Ok(payload) => {
                if let Err(e) = ctx.telemetry.extend(STATUS_CHANNEL, payload).await {
                    tracing::warn!(error = %e, "status telemetry publish failed");
                }
            }
            Err(e) => tracing::warn!(error = %e, "status telemetry encode failed"),
        }
    }
}

/// Every [`JOBS_SYNC_INTERVAL`], put each reconstruct-job record that is new or
/// changed since the cloud last accepted it (collection `jobs`, key = job id,
/// subject = the capturing drone). A put the cloud did not accept (the relay is
/// down on an unpaired or offline node) is logged at debug and retried on the
/// next tick.
async fn jobs_sync_loop(state: Arc<Mutex<Engine>>, ctx: PluginContext, compute_node_id: String) {
    let mut sync = JobsCloudSync::new();
    loop {
        tokio::time::sleep(JOBS_SYNC_INTERVAL).await;
        let jobs = {
            let engine = state.lock().await;
            build_atlas_jobs(engine.scheduler().store())
        };
        let jobs = match jobs {
            Ok(jobs) => jobs,
            Err(e) => {
                tracing::warn!(error = %e, "compute jobs snapshot failed");
                continue;
            }
        };
        for record in sync.pending(&jobs, &compute_node_id) {
            let data = match rmpv::ext::to_value(&record.data) {
                Ok(v) => v,
                Err(e) => {
                    tracing::warn!(job = %record.key, error = %e, "job record encode failed");
                    continue;
                }
            };
            match ctx
                .cloud
                .put_record(JOBS_COLLECTION, &record.key, data, Some(&record.device_id))
                .await
            {
                Ok(reply) if put_accepted(&reply) => {
                    tracing::debug!(job = %record.key, "job record synced to cloud");
                    sync.mark_synced(record);
                }
                Ok(reply) => {
                    tracing::debug!(job = %record.key, reply = %reply, "job record not accepted; retrying next tick")
                }
                Err(e) => {
                    tracing::debug!(job = %record.key, error = %e, "job record put failed; retrying next tick")
                }
            }
        }
    }
}

/// Drain the Atlas event receiver into the job queue. One [`AtlasIngest`] lives
/// for the task: it persists each keyframe's image to the work-root dataset (no
/// store, no lock) and, on the terminal `Bagged` state, finalizes the dataset
/// (writes `transforms.json`) and yields the reconstruct job. The disk writes run
/// lock-free; the engine lock is held only briefly for the store submit (the same
/// lock-briefly discipline the worker uses). A malformed frame is swallowed; a
/// real filesystem or store fault is logged and the loop continues. The loop ends
/// when the event channel closes (the receiver router dropped its senders), which
/// only happens on shutdown.
///
/// When live reconstruction is enabled, the loop ALSO runs the per-session cadence
/// ([`run_live_cycles`]) on a tick and after each keyframe: it periodically
/// snapshots the growing capture and enqueues a real reconstruct so the world
/// model updates during the flight, not only at the end. In both modes a slow
/// sweep finalizes any session that went silent without a `Bagged` frame (see
/// [`AtlasIngest::idle_bags`]), so a capture whose bag was lost is still
/// reconstructed and its state released.
async fn atlas_receiver_loop(
    mut rx: mpsc::Receiver<AtlasEvent>,
    state: Arc<Mutex<Engine>>,
    work_root: PathBuf,
    live_config: LiveReconstructConfig,
) {
    let mut ingest = AtlasIngest::with_live_config(work_root, live_config);

    let mut tick = tokio::time::interval(Duration::from_secs(LIVE_CADENCE_TICK_SECS));
    tick.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    let mut idle_sweep = tokio::time::interval(IDLE_SWEEP);
    idle_sweep.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);
    loop {
        tokio::select! {
            maybe_event = rx.recv() => {
                let Some(event) = maybe_event else { break };
                drain_event(&mut ingest, &state, &event).await;
                // A keyframe may have hit the count trigger; check promptly.
                if live_config.enabled {
                    run_live_cycles(&mut ingest, &state).await;
                }
            }
            _ = tick.tick(), if live_config.enabled => {
                // The interval trigger + the skip-while-running reconcile.
                run_live_cycles(&mut ingest, &state).await;
            }
            _ = idle_sweep.tick() => match ingest.idle_bags(now_ms()) {
                Ok(bags) => {
                    for (dataset, job) in bags {
                        let engine = state.lock().await;
                        match submit_reconstruct_job(engine.scheduler().store(), &dataset, &job) {
                            Ok(job_id) => tracing::info!(job = %job_id, "atlas idle session finalized: reconstruct job enqueued"),
                            Err(e) => tracing::error!(error = %e, "atlas reconstruct submit failed"),
                        }
                    }
                }
                Err(e) => tracing::error!(error = %e, "atlas idle session finalize failed"),
            },
        }
    }
    tracing::info!("atlas receiver loop ended (event channel closed)");
}

/// Handle one received Atlas event: persist a keyframe or, on the terminal bag,
/// enqueue the final reconstruct. A malformed frame is swallowed; a real disk or
/// store fault is logged and the caller continues.
async fn drain_event(ingest: &mut AtlasIngest, state: &Arc<Mutex<Engine>>, event: &AtlasEvent) {
    match ingest.step(event, now_ms()) {
        Ok(Some((dataset, job))) => {
            let keyframes_received = ingest.keyframes_seen();
            let submitted = {
                let engine = state.lock().await;
                submit_reconstruct_job(engine.scheduler().store(), &dataset, &job)
            };
            match submitted {
                Ok(job_id) => tracing::info!(
                    job = %job_id,
                    keyframes_received,
                    "atlas capture bagged: reconstruct job enqueued"
                ),
                Err(e) => tracing::error!(error = %e, "atlas reconstruct submit failed"),
            }
        }
        Ok(None) => {}
        Err(e) => tracing::error!(error = %e, "atlas ingest disk write failed"),
    }
}

/// Drive the live-reconstruction cadence: reconcile the skip-while-running guard
/// against the store (a cycle whose job reached a terminal state, or vanished,
/// releases the guard), then enqueue a fresh periodic reconstruct for every
/// session now due. The store is the source of truth for "is the cycle done", so
/// cycles coalesce instead of piling up. A snapshot or submit fault is logged; the
/// cadence keeps running.
async fn run_live_cycles(ingest: &mut AtlasIngest, state: &Arc<Mutex<Engine>>) {
    // 1. Release the guard for any session whose in-flight cycle finished.
    let in_flight = ingest.in_flight_cycles();
    if !in_flight.is_empty() {
        let finished: Vec<String> = {
            let engine = state.lock().await;
            let store = engine.scheduler().store();
            in_flight
                .into_iter()
                .filter_map(|(session, job_id)| match store.get_job(&job_id) {
                    Ok(Some(job)) if is_terminal(job.state) => Some(session),
                    // The job was purged (retention) — treat as finished so the
                    // session is never stuck with a guard that never releases.
                    Ok(None) => Some(session),
                    Ok(Some(_)) => None, // still queued / running: keep the guard
                    Err(e) => {
                        tracing::warn!(error = %e, "live cycle reconcile read failed");
                        None
                    }
                })
                .collect()
        };
        for session in finished {
            ingest.note_cycle_finished(&session);
        }
    }

    // 2. Enqueue a fresh periodic reconstruct for every session now due.
    match ingest.due_reconstructs(now_ms()) {
        Ok(jobs) => {
            for (dataset, job) in jobs {
                let submitted = {
                    let engine = state.lock().await;
                    submit_reconstruct_job(engine.scheduler().store(), &dataset, &job)
                };
                let keyframes = dataset
                    .meta
                    .get("keyframes")
                    .and_then(|v| v.as_u64())
                    .unwrap_or(0);
                match submitted {
                    Ok(id) => tracing::info!(
                        job = %id,
                        keyframes,
                        "atlas live reconstruct cycle enqueued"
                    ),
                    Err(e) => tracing::error!(error = %e, "atlas live reconstruct submit failed"),
                }
            }
        }
        Err(e) => tracing::error!(error = %e, "atlas live snapshot write failed"),
    }
}

/// Whether a job has reached a terminal state (the live cadence's
/// skip-while-running guard releases on a terminal in-flight cycle).
fn is_terminal(state: ComputeJobState) -> bool {
    matches!(
        state,
        ComputeJobState::Completed | ComputeJobState::Failed | ComputeJobState::Cancelled
    )
}

#[cfg(test)]
mod tests {
    use super::{derive_node_id, live_enabled, StatePaths};
    use std::path::PathBuf;

    #[test]
    fn node_id_prefers_the_env_override() {
        assert_eq!(
            derive_node_id(Some("rtx-box".to_string()), Some("mid".to_string())),
            "rtx-box"
        );
    }

    #[test]
    fn node_id_derives_from_machine_id_when_env_unset_and_is_unique_per_host() {
        let a = derive_node_id(None, Some("aaaaaaaaaaaaaaaa1111".to_string()));
        let b = derive_node_id(None, Some("bbbbbbbbbbbbbbbb2222".to_string()));
        assert_eq!(a, "compute-aaaaaaaaaaaa");
        assert_ne!(a, b, "distinct machine-ids must yield distinct node ids");
    }

    #[test]
    fn node_id_falls_back_when_nothing_is_available() {
        assert_eq!(derive_node_id(None, None), "compute-node");
        // A blank env / blank machine-id both fall through to the next source.
        assert_eq!(derive_node_id(Some("  ".to_string()), None), "compute-node");
    }

    #[test]
    fn live_reconstruct_is_opt_in_and_the_env_overrides_the_config() {
        assert!(!live_enabled(None, None), "absent everywhere reads off");
        assert!(live_enabled(None, Some(true)));
        assert!(!live_enabled(None, Some(false)));
        // The env wins over the config, both ways.
        assert!(!live_enabled(Some("0"), Some(true)));
        assert!(live_enabled(Some("true"), Some(false)));
    }

    #[test]
    fn state_paths_default_under_the_plugin_data_dir_and_keep_the_env_overrides() {
        let with_data = |k: &str| (k == "ADOS_PLUGIN_DATA_DIR").then(|| "/data/we".to_string());
        let p = StatePaths::resolve(with_data);
        assert_eq!(p.db, "/data/we/node/jobs.db");
        assert_eq!(p.work_root, PathBuf::from("/data/we/node/work"));
        assert_eq!(
            p.credentials,
            PathBuf::from("/data/we/node/node-credentials.json")
        );

        // An explicit override wins over the data dir.
        let overridden = |k: &str| match k {
            "ADOS_PLUGIN_DATA_DIR" => Some("/data/we".to_string()),
            "ADOS_COMPUTE_DB" => Some(":memory:".to_string()),
            "ADOS_COMPUTE_WORK" => Some("/scratch/work".to_string()),
            _ => None,
        };
        let p = StatePaths::resolve(overridden);
        assert_eq!(p.db, ":memory:");
        assert_eq!(p.work_root, PathBuf::from("/scratch/work"));
        assert_eq!(
            p.credentials,
            PathBuf::from("/data/we/node/node-credentials.json")
        );

        // No host data dir: the standalone defaults.
        let p = StatePaths::resolve(|_| None);
        assert_eq!(p.db, "/var/ados/compute/jobs.db");
        assert_eq!(p.work_root, PathBuf::from("/var/ados/compute/work"));
    }
}
