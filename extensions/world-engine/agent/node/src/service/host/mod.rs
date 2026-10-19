//! The World Engine extension's `node` service on a workstation or compute
//! node, run by the plugin host. It opens the job store, builds the node
//! engine, runs a worker loop that drains the queue and periodically reclaims
//! terminal jobs, and serves the REST job API plus the drone lanes on one TCP
//! listener and, for the operator, on the plugin's `http.sock`.
//!
//! Host connection: the plugin host starts it as
//! `world-engine-node com.altnautica.world-engine` with its launch values
//! (socket, token, data dir) in the unit's token credential; it then connects
//! to the host and uses the plugin
//! config (`serving.*`, `atlas.live_reconstruct`), publishes its compute status
//! on the `status` telemetry channel, and mirrors its reconstruct jobs into the
//! `jobs` cloud records. Without them (a standalone dev run) it runs on
//! defaults and makes no host call. A lost host session (a plugin-host
//! restart) ends the process with an error, so its unit's `Restart=` brings it
//! back with a fresh socket and token.
//!
//! Reach: the operator reaches the node only through `http.sock` (ados-control
//! authenticated them, so every request there is the owner). The TCP listener
//! is for other nodes: each lane admits only a credential this node issued for
//! it, and owner-only routes are refused over TCP whatever is presented, so
//! binding a non-loopback address is safe. Run by the plugin host (the
//! extension's `node` service, whose manifest declares `listen_ports: [8092]`)
//! it binds `0.0.0.0:8092` so drones on the LAN reach it; a standalone run
//! stays on loopback. `ADOS_COMPUTE_BIND` overrides both.
//!
//! Worker note: the worker claims the next job under the engine lock, then
//! releases the lock and runs the (real, possibly minutes-long) backend
//! WITHOUT it, so a long reconstruction never blocks the API. It re-acquires
//! the lock only briefly to record the terminal state; a cancel that lands
//! during the run wins (`Scheduler::finalize` refuses to overwrite a job that
//! is no longer `Running`).
//!
//! Environment (every default under `<plugin data dir>/node/` applies when
//! the host gave a data dir; the `/var/ados/compute` defaults otherwise):
//! - `ADOS_COMPUTE_DB`        job store path (default `<data>/node/jobs.db`)
//! - `ADOS_COMPUTE_WORK`      dataset + artifact work root (default
//!   `<data>/node/work`); the persister writes keyframe datasets here, the
//!   reconstructor writes artifacts here, and the artifact route serves from here
//! - `ADOS_COMPUTE_NODE_CREDENTIALS` the store of credentials issued to drones
//!   (default `<data>/node/node-credentials.json`)
//! - `ADOS_COMPUTE_BIND`      bind address (default `0.0.0.0:8092` under the
//!   plugin host, `127.0.0.1:8092` standalone)
//! - `ADOS_COMPUTE_PUBLIC_URL` base URL the GCS fetches artifacts from (default
//!   derived from the bind address, substituting the node hostname for a wildcard
//!   bind); the artifact URL is `<public_url>/artifacts/<relpath>`
//! - `ADOS_COMPUTE_NODE_ID`   this node's id (default derived from the machine id)
//! - `ADOS_COMPUTE_WORKERS`   worker slots (default `1`)
//! - `ADOS_COMPUTE_RETENTION_S` terminal-job retention seconds (default `86400`)
//! - `ADOS_ATLAS_LIVE`        overrides the `atlas.live_reconstruct` plugin config
//!   (live in-flight reconstruction, default off)

mod loops;
mod setup;
mod worker;

use loops::*;
use setup::*;
use worker::*;

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
    LatestComputeStatus, LiveReconstructConfig, NoDetector, NodeCredentialStore,
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

/// Run the plugin-host service until the listener stops or the host session
/// ends.
pub async fn run() -> Result<(), Box<dyn std::error::Error>> {
    let launch = launch_args();
    let data_dir = launch.as_ref().and_then(|a| a.data_dir.clone());
    let host = connect_host(launch).await?;
    let host_ipc = host.as_ref().map(|(_, ipc)| ipc.clone());
    let ctx = host.map(|(ctx, _)| ctx);
    match &ctx {
        Some(c) => tracing::info!(plugin_id = %c.plugin_id, "connected to the plugin host"),
        None => tracing::info!("no plugin host (standalone run): host calls skipped"),
    }

    let paths = StatePaths::resolve(data_dir.as_deref(), |k| std::env::var(k).ok());
    // Under the plugin host the node serves the LAN: that is what the declared
    // listen port is for, and the sandbox admits a bind on exactly that port.
    let default_bind = if ctx.is_some() {
        "0.0.0.0:8092"
    } else {
        "127.0.0.1:8092"
    };
    let bind = env_or("ADOS_COMPUTE_BIND", default_bind);
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
    // The live-lane reconstructor picks the pinned trainer per job and writes
    // artifacts under work_root; staged session jobs find their uploads under
    // `sessions/` beside the work root, outside the artifact-served tree.
    let sessions_root = work_root
        .parent()
        .map(|p| p.join("sessions"))
        .unwrap_or_else(|| work_root.join("..").join("sessions"));
    let _ = std::fs::create_dir_all(&sessions_root);
    let scheduler = Scheduler::new(
        store,
        Arc::new(SelectingReconstructor::new(work_root.clone())),
        detector.clone(),
    )
    .with_sessions_root(sessions_root.clone());
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
    let staged_ctx = {
        let runner = state.lock().await.runner();
        world_engine_node::worker::WorkerCtx {
            state: state.clone(),
            work_root: work_root.clone(),
            public_base: public_base.clone(),
            runner,
            tour_viewer: world_engine_node::tools::tour_viewer_dir(),
            person_model: std::env::var_os("ADOS_WE_PERSON_MODEL").map(PathBuf::from),
        }
    };
    for _ in 0..workers.max(1) {
        let ws = state.clone();
        let wr = work_root.clone();
        let pb = public_base.clone();
        let sessions = offload_sessions.clone();
        let world = world_broadcaster.clone();
        let staged = staged_ctx.clone();
        tokio::spawn(async move { worker_loop(ws, wr, pb, sessions, world, staged).await });
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

    // The owner's job API and job files, plus the lanes a drone or a capture
    // device uses on the same listener (artifacts, session upload, the offload
    // detection return stream, the Atlas ingest and world stream), each behind
    // its own credential gate.
    let on_commit: world_engine_node::bulk::CommitHook = Arc::new(|s| {
        tracing::info!(session = %s.session_id, files = s.files, bytes = s.bytes, "capture session committed");
        Ok(())
    });
    let shared = build_router_with_base(
        state.clone(),
        auth.clone(),
        std::sync::Arc::from(public_base.as_str()),
        // The live session registry so /api/compute/status carries the state
        // breakdown + /api/compute/sessions serves the live records.
        offload_sessions.registry(),
    )
    .merge(world_engine_node::job_files::job_files_router(
        auth.clone(),
        work_root.clone(),
    ))
    .merge(lane_router(
        auth,
        LaneRoutes {
            work_root: work_root.clone(),
            uploads: world_engine_node::bulk::bulk_router(
                Arc::new(world_engine_node::bulk::SessionStore::new(sessions_root)),
                on_commit,
            ),
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
    // Advertise the job API on the LAN so a drone finds this node without a
    // pinned address. The host publishes it (the sandbox refuses the node a
    // responder of its own) and withdraws it when this connection ends. The GCS
    // finds the node through the agent's own `_ados._tcp` record, not this one.
    // A standalone run serves loopback only, so it advertises nothing.
    let job_port = bind
        .rsplit(':')
        .next()
        .and_then(|p| p.parse::<u16>().ok())
        .unwrap_or(8092);
    if let Some(ctx) = ctx.clone() {
        let node_id = node_id.clone();
        tokio::spawn(async move { advertise_job_api(ctx, node_id, job_port).await });
    }
    let serve = async { axum::serve(listener, router).await };
    match host_ipc {
        // Nothing reconnects a lost host session, so stop and let the unit
        // restart this service against the restarted host.
        Some(ipc) => tokio::select! {
            served = serve => served?,
            () = ipc.session_lost() => {
                let lost = ados_sdk::RunnerError::SessionLost;
                tracing::error!(error = %lost, "world-engine-node stopping");
                return Err(lost.into());
            }
        },
        None => serve.await?,
    }
    Ok(())
}

/// How long the job-API advert waits before asking the host again.
const ADVERT_RETRY: Duration = Duration::from_secs(5);

/// Publish the job API as [`world_engine_transport::COMPUTE_SERVICE`] through
/// the host, retrying on a fixed cadence until it is published: a node whose
/// hostname is not set yet, or whose host responder is not up, gets its advert
/// the moment that clears, with no restart. A repeated failure is logged once
/// per distinct reason.
async fn advertise_job_api(ctx: PluginContext, node_id: String, port: u16) {
    let txt = world_engine_transport::compute_advert_txt(&node_id);
    let mut last_error: Option<String> = None;
    loop {
        match ctx
            .mdns
            .advertise(world_engine_transport::COMPUTE_SERVICE, port, &txt)
            .await
        {
            Ok(advert) => {
                tracing::info!(
                    record = %advert.fullname,
                    host = %advert.hostname,
                    port,
                    "job API advertised on mDNS"
                );
                return;
            }
            Err(e) => {
                let reason = e.to_string();
                if last_error.as_deref() != Some(reason.as_str()) {
                    tracing::warn!(error = %reason, "job API mDNS advert failed; retrying");
                }
                last_error = Some(reason);
            }
        }
        tokio::time::sleep(ADVERT_RETRY).await;
    }
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
