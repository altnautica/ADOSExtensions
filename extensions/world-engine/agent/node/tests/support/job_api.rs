//! A node with stand-in tools behind its job API, for the pipeline tests:
//! upload a session, submit a reconstruction and wait for it.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use axum::body::Body;
use axum::http::{Request, StatusCode};
use axum::Router;
use tokio::sync::Mutex;
use tower::ServiceExt;
use world_engine_node::bulk::{bulk_router, CommitHook, SessionStore};
use world_engine_node::recon::{StandInRunner, ToolRunner};
use world_engine_node::worker::{worker_loop, WorkerCtx};
use world_engine_node::{
    build_router, job_files::job_files_router, upload_and_artifact_lanes, ApiState, Cluster,
    ComputeAuth, Engine, JobStore, MockDetector, MockReconstructor, Scheduler,
};

const TOKEN: &str = "lidar-test-token";

fn auth(dir: &Path) -> Arc<ComputeAuth> {
    let creds = world_engine_node::NodeCredentialStore::open(dir.join("creds.json"), "t");
    Arc::new(ComputeAuth::new(creds).with_owner_token(TOKEN.into()))
}

async fn call(router: &Router, method: &str, uri: &str, body: Vec<u8>) -> (StatusCode, Vec<u8>) {
    let req = Request::builder()
        .method(method)
        .uri(uri)
        .header("authorization", format!("Bearer {TOKEN}"))
        .header("content-type", "application/json")
        .body(Body::from(body))
        .unwrap();
    let resp = router.clone().oneshot(req).await.unwrap();
    let status = resp.status();
    let bytes = axum::body::to_bytes(resp.into_body(), usize::MAX)
        .await
        .unwrap();
    (status, bytes.to_vec())
}

async fn json(
    router: &Router,
    method: &str,
    uri: &str,
    body: serde_json::Value,
) -> serde_json::Value {
    let (_, b) = call(router, method, uri, serde_json::to_vec(&body).unwrap()).await;
    serde_json::from_slice(&b).unwrap_or(serde_json::Value::Null)
}

/// Upload the session `build` writes (`track`: with a recorded pose track),
/// submit a `tour` reconstruction (plus `extra` request fields) with `runner`
/// and wait for it to finish. Returns the job view and the job directory, or
/// the refused submission's status and body.
pub async fn run_job(
    dir: &Path,
    runner: &StandInRunner,
    build: fn(&Path, bool) -> PathBuf,
    track: bool,
    extra: serde_json::Value,
) -> Result<(serde_json::Value, PathBuf), (StatusCode, serde_json::Value)> {
    let src = build(&dir.join("src"), track);
    let sid = src.file_name().unwrap().to_string_lossy().into_owned();
    let work = dir.join("work");
    let sessions = dir.join("sessions");
    std::fs::create_dir_all(&sessions).unwrap();
    let viewer = dir.join("viewer");
    std::fs::create_dir_all(&viewer).unwrap();
    std::fs::write(viewer.join("index.html"), "<html></html>").unwrap();

    let scheduler = Scheduler::new(
        JobStore::open_in_memory().unwrap(),
        Arc::new(MockReconstructor),
        Arc::new(MockDetector),
    )
    .with_sessions_root(sessions.clone());
    let mut engine = Engine::new(scheduler, Cluster::new_master("n"), 1);
    let shared: Arc<dyn ToolRunner> = Arc::new(runner.clone());
    engine.set_runner(shared.clone());
    let state: ApiState = Arc::new(Mutex::new(engine));
    let auth = auth(dir);
    let hook: CommitHook = Arc::new(|_| Ok(()));
    let router = build_router(state.clone(), auth.clone())
        .merge(job_files_router(auth.clone(), work.clone()))
        .merge(upload_and_artifact_lanes(
            &auth,
            bulk_router(Arc::new(SessionStore::new(sessions)), hook),
            work.clone(),
        ));
    tokio::spawn(worker_loop(WorkerCtx {
        state,
        work_root: work.clone(),
        public_base: "http://127.0.0.1:8092".into(),
        runner: shared,
        tour_viewer: viewer,
        person_model: None,
    }));

    let manifest = std::fs::read(src.join("manifest.json")).unwrap();
    let (s, _) = call(
        &router,
        "POST",
        &format!("/api/compute/sessions/{sid}"),
        manifest,
    )
    .await;
    assert!(s.is_success());
    for rel in ["rig.json", "segments/000000.mcap", "checksums.b3"] {
        let (s, _) = call(
            &router,
            "PUT",
            &format!("/api/compute/sessions/{sid}/files/{rel}"),
            std::fs::read(src.join(rel)).unwrap(),
        )
        .await;
        assert!(s.is_success(), "{rel}: {s}");
    }
    let (s, body) = call(
        &router,
        "POST",
        &format!("/api/compute/sessions/{sid}/commit"),
        vec![],
    )
    .await;
    assert_eq!(s, StatusCode::OK, "{}", String::from_utf8_lossy(&body));

    let mut req = serde_json::json!({"kind": "reconstruct", "session_id": sid, "preset": "tour"});
    for (k, v) in extra.as_object().into_iter().flatten() {
        req[k] = v.clone();
    }
    let (s, body) = call(
        &router,
        "POST",
        "/api/compute/jobs",
        serde_json::to_vec(&req).unwrap(),
    )
    .await;
    let sub: serde_json::Value = serde_json::from_slice(&body).unwrap();
    if !s.is_success() {
        return Err((s, sub));
    }
    let job = sub["job_id"].as_str().expect("submitted").to_string();
    let mut done = serde_json::Value::Null;
    for _ in 0..900 {
        done = json(
            &router,
            "GET",
            &format!("/api/compute/jobs/{job}"),
            serde_json::Value::Null,
        )
        .await;
        if matches!(done["state"].as_str(), Some("completed" | "failed")) {
            break;
        }
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    Ok((done, work.join("jobs").join(&job)))
}
