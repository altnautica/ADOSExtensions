//! The staged pipeline end to end on the contract's tiny session, with the
//! stand-in tool runner (feature `mock`): resumable upload over HTTP, a `tour`
//! reconstruction through every stage to its deliverables, the tool check at
//! submit, a resumed job that reruns only its unfinished stage, and change
//! detection between two reconstructions.

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
    ComputeAuth, ComputeJobState, Engine, JobStore, MockDetector, MockReconstructor, Scheduler,
};

const TOKEN: &str = "test-owner-token";
const SESSION: &str = "sim-1-20261019T000000Z-0001";

fn fixture() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../contract/fixtures/tiny-session")
        .join(SESSION)
}

fn auth(dir: &Path) -> Arc<ComputeAuth> {
    let creds = world_engine_node::NodeCredentialStore::open(dir.join("creds.json"), "t");
    Arc::new(ComputeAuth::new(creds).with_owner_token(TOKEN.into()))
}

struct Node {
    router: Router,
    state: ApiState,
    work: PathBuf,
    _dir: tempfile::TempDir,
}

fn node(runner: StandInRunner) -> (Node, WorkerCtx) {
    let dir = tempfile::tempdir().unwrap();
    let work = dir.path().join("work");
    let sessions = work.join("sessions");
    std::fs::create_dir_all(&sessions).unwrap();
    let viewer = dir.path().join("viewer");
    std::fs::create_dir_all(viewer.join("assets")).unwrap();
    std::fs::write(viewer.join("index.html"), "<html></html>").unwrap();
    std::fs::write(viewer.join("assets/app.js"), "//").unwrap();

    let scheduler = Scheduler::new(
        JobStore::open_in_memory().unwrap(),
        Arc::new(MockReconstructor),
        Arc::new(MockDetector),
    )
    .with_sessions_root(sessions.clone());
    let mut engine = Engine::new(scheduler, Cluster::new_master("n"), 1);
    let runner: Arc<dyn ToolRunner> = Arc::new(runner);
    engine.set_runner(runner.clone());
    let state: ApiState = Arc::new(Mutex::new(engine));
    let auth = auth(dir.path());
    let hook: CommitHook = Arc::new(|_| Ok(()));
    let router = build_router(state.clone(), auth.clone())
        .merge(job_files_router(auth.clone(), work.clone()))
        .merge(upload_and_artifact_lanes(
            &auth,
            bulk_router(Arc::new(SessionStore::new(sessions)), hook),
            work.clone(),
        ));
    let ctx = WorkerCtx {
        state: state.clone(),
        work_root: work.clone(),
        public_base: "http://127.0.0.1:8092".into(),
        runner,
        tour_viewer: viewer,
        person_model: None,
    };
    (
        Node {
            router,
            state,
            work,
            _dir: dir,
        },
        ctx,
    )
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
) -> (StatusCode, serde_json::Value) {
    let (s, b) = call(router, method, uri, serde_json::to_vec(&body).unwrap()).await;
    (
        s,
        serde_json::from_slice(&b).unwrap_or(serde_json::Value::Null),
    )
}

/// Upload the fixture session over the lane and commit it.
async fn upload(router: &Router) {
    let dir = fixture();
    let manifest = std::fs::read(dir.join("manifest.json")).unwrap();
    let (s, _) = call(
        router,
        "POST",
        &format!("/api/compute/sessions/{SESSION}"),
        manifest,
    )
    .await;
    assert!(s.is_success(), "post manifest: {s}");
    for rel in ["rig.json", "segments/000000.mcap", "checksums.b3"] {
        let bytes = std::fs::read(dir.join(rel)).unwrap();
        let (s, _) = call(
            router,
            "PUT",
            &format!("/api/compute/sessions/{SESSION}/files/{rel}"),
            bytes,
        )
        .await;
        assert!(s.is_success(), "put {rel}: {s}");
    }
    let (s, body) = json(
        router,
        "POST",
        &format!("/api/compute/sessions/{SESSION}/commit"),
        serde_json::json!({}),
    )
    .await;
    assert_eq!(s, StatusCode::OK, "commit: {body}");
}

async fn wait_terminal(router: &Router, job: &str) -> serde_json::Value {
    let mut last = serde_json::Value::Null;
    for _ in 0..600 {
        let (_, v) = json(
            router,
            "GET",
            &format!("/api/compute/jobs/{job}"),
            serde_json::Value::Null,
        )
        .await;
        if matches!(
            v["state"].as_str(),
            Some("completed" | "failed" | "cancelled")
        ) {
            return v;
        }
        last = v;
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    panic!("job {job} did not finish: {last}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_tour_reconstruction_runs_every_stage_and_resumes() {
    let _ = tracing_subscriber::fmt().with_test_writer().try_init();
    let (n, ctx) = node(StandInRunner::default());
    upload(&n.router).await;
    tokio::spawn(worker_loop(ctx));

    let (s, sub) = json(
        &n.router,
        "POST",
        "/api/compute/jobs",
        serde_json::json!({"kind": "reconstruct", "session_id": SESSION, "preset": "tour"}),
    )
    .await;
    assert_eq!(s, StatusCode::CREATED, "{sub}");
    let job = sub["job_id"].as_str().unwrap().to_string();
    let done = wait_terminal(&n.router, &job).await;
    assert_eq!(done["state"], "completed", "{done}");
    assert_eq!(done["stage"], "done");
    let stages: Vec<(&str, &str)> = done["stages"]
        .as_array()
        .unwrap()
        .iter()
        .map(|s| (s["id"].as_str().unwrap(), s["state"].as_str().unwrap()))
        .collect();
    assert_eq!(
        stages,
        [
            ("prepare", "done"),
            ("sfm", "done"),
            ("train", "done"),
            ("mesh", "done"),
            ("products", "done")
        ]
    );
    assert!(done["tools"]["colmap"].as_str().is_some());
    let train = &done["stages"][2]["detail"];
    assert_eq!(train["trainer"], "gsplat", "{train}");
    assert_eq!(train["psnr"], 31.5, "{train}");
    assert_eq!(train["psnr_cc"], 32.25, "{train}");

    let (_, outs) = json(
        &n.router,
        "GET",
        &format!("/api/compute/jobs/{job}/outputs"),
        serde_json::Value::Null,
    )
    .await;
    let kinds: Vec<&str> = outs
        .as_array()
        .unwrap()
        .iter()
        .map(|o| o["kind"].as_str().unwrap())
        .collect();
    for k in [
        "splat",
        "splat_lod",
        "mesh",
        "pointcloud",
        "occupancy",
        "floorplan",
        "tour",
        "rig",
    ] {
        assert!(kinds.contains(&k), "missing {k} in {kinds:?}");
    }
    let spz = outs
        .as_array()
        .unwrap()
        .iter()
        .find(|o| o["format"] == "spz")
        .unwrap();
    assert_eq!(spz["place_json"], "scene.spz.place.json");
    assert!(spz["size_bytes"].as_u64().unwrap() > 0);
    assert!(spz["uri"]
        .as_str()
        .unwrap()
        .starts_with("http://127.0.0.1:8092/artifacts/jobs/"));

    // Deliverables are served; the placement sidecar is a contract descriptor.
    let (s, body) = call(
        &n.router,
        "GET",
        &format!("/api/compute/jobs/{job}/files/scene.spz.place.json"),
        vec![],
    )
    .await;
    assert_eq!(s, StatusCode::OK);
    let desc: ados_capture_contract::ArtifactDescriptor = serde_json::from_slice(&body).unwrap();
    assert_eq!(desc.source_session, SESSION);
    assert_eq!(desc.generation, 1);
    // splat-transform writes SPZ positions unchanged (verified round trip), so
    // the SPZ sits in the same ENU frame as the PLY.
    assert_eq!(
        desc.r_enu_artifact,
        [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    );
    let export = n.work.join("jobs").join(&job).join("out/export.zip");
    let mut zip = zip::ZipArchive::new(std::fs::File::open(&export).unwrap()).unwrap();
    let names: Vec<String> = (0..zip.len())
        .map(|i| zip.by_index(i).unwrap().name().unwrap().to_string())
        .collect();
    assert!(names.iter().any(|n| n == "transforms.json"), "{names:?}");
    assert!(
        names.iter().any(|n| n.starts_with("sparse/0/")),
        "{names:?}"
    );
    let images: Vec<&String> = names.iter().filter(|n| n.starts_with("images/")).collect();
    assert!(!images.is_empty(), "{names:?}");
    let transforms: serde_json::Value =
        serde_json::from_reader(zip.by_name("transforms.json").unwrap()).unwrap();
    for f in transforms["frames"].as_array().unwrap() {
        let p = f["file_path"].as_str().unwrap();
        assert!(names.iter().any(|n| n == p), "{p} not in the export");
    }
    let (s, zip) = call(
        &n.router,
        "GET",
        &format!("/api/compute/jobs/{job}/tour.zip"),
        vec![],
    )
    .await;
    assert_eq!(s, StatusCode::OK);
    assert_eq!(&zip[..2], b"PK");

    // Measurement on the reconstructed mesh.
    let (s, m) = json(
        &n.router,
        "POST",
        "/api/compute/measure",
        serde_json::json!({"job_id": job, "a": [0.0, 2.0, 1.0], "b": [6.0, 2.0, 1.0]}),
    )
    .await;
    assert_eq!(s, StatusCode::OK, "{m}");
    assert!(
        (m["distance_m"].as_f64().unwrap() - 6.0).abs() < 0.05,
        "{m}"
    );

    // Resume: the node "crashed" during products; only products reruns.
    let prepare_started = done["stages"][0]["started_ns"].as_i64().unwrap();
    {
        let engine = n.state.lock().await;
        let store = engine.scheduler().store();
        store
            .set_job_state(&job, ComputeJobState::Running, 0.9, None, None, 1)
            .unwrap();
        store
            .set_stage(
                &job,
                "products",
                world_engine_node::store_stages::StageState::Running,
                5,
                None,
            )
            .unwrap();
        store.requeue_stale_running(2).unwrap();
    }
    let again = wait_terminal(&n.router, &job).await;
    assert_eq!(again["state"], "completed", "{again}");
    // The resumed products rerun keeps the generation fixed at submit.
    let (_, body) = call(
        &n.router,
        "GET",
        &format!("/api/compute/jobs/{job}/files/scene.spz.place.json"),
        vec![],
    )
    .await;
    let desc: ados_capture_contract::ArtifactDescriptor = serde_json::from_slice(&body).unwrap();
    assert_eq!(desc.generation, 1);
    assert_eq!(
        again["stages"][0]["started_ns"].as_i64().unwrap(),
        prepare_started
    );
    assert!(again["stages"][4]["started_ns"].as_i64().unwrap() > 5);
    assert!(n
        .work
        .join("jobs")
        .join(&job)
        .join("out/outputs.json")
        .is_file());

    // Change detection between two reconstructions of the session.
    let (s, sub2) = json(
        &n.router,
        "POST",
        "/api/compute/jobs",
        serde_json::json!({"kind": "reconstruct", "session_id": SESSION, "preset": "preview"}),
    )
    .await;
    assert_eq!(s, StatusCode::CREATED);
    let second = wait_terminal(&n.router, sub2["job_id"].as_str().unwrap()).await;
    assert_eq!(second["state"], "completed", "{second}");
    assert_eq!(second["stages"][3]["state"], "skipped");
    let (s, ch) = json(
        &n.router,
        "POST",
        "/api/compute/jobs",
        serde_json::json!({"kind": "change", "a": SESSION, "b": SESSION}),
    )
    .await;
    assert_eq!(s, StatusCode::CREATED, "{ch}");
    let change = wait_terminal(&n.router, ch["job_id"].as_str().unwrap()).await;
    assert_eq!(change["state"], "completed", "{change}");
    assert_eq!(change["stages"][1]["detail"]["points_changed"], 0);
}

#[tokio::test]
async fn submissions_are_refused_with_the_reason() {
    let (n, _ctx) = node(StandInRunner {
        missing: vec!["colmap".into()],
        ..Default::default()
    });
    // Not uploaded yet.
    let (s, body) = json(
        &n.router,
        "POST",
        "/api/compute/jobs",
        serde_json::json!({"kind": "reconstruct", "session_id": SESSION, "preset": "hq"}),
    )
    .await;
    assert_eq!(s, StatusCode::CONFLICT);
    assert_eq!(body["error"], "E_NOT_UPLOADED");
    upload(&n.router).await;
    let (s, body) = json(
        &n.router,
        "POST",
        "/api/compute/jobs",
        serde_json::json!({"kind": "reconstruct", "session_id": SESSION, "preset": "hq"}),
    )
    .await;
    assert_eq!(s, StatusCode::UNPROCESSABLE_ENTITY);
    assert_eq!(body["error"], "E_TOOL_MISSING");
    assert_eq!(body["tool"], "colmap");
    let (s, body) = json(
        &n.router,
        "POST",
        "/api/compute/jobs",
        serde_json::json!({"kind": "reconstruct", "session_id": SESSION, "preset": "ultra"}),
    )
    .await;
    assert_eq!(s, StatusCode::BAD_REQUEST);
    for bad in ["../../x", "a/b", ".hidden", "a\\b"] {
        let (s, body) = json(
            &n.router,
            "POST",
            "/api/compute/jobs",
            serde_json::json!({"job_id": bad, "kind": "reconstruct", "session_id": SESSION, "preset": "preview"}),
        )
        .await;
        assert_eq!(s, StatusCode::BAD_REQUEST, "{bad}: {body}");
        assert_eq!(body["error"], "E_INPUT");
    }
    assert_eq!(body["error"], "E_INPUT");
    let (s, body) = json(
        &n.router,
        "POST",
        "/api/compute/jobs",
        serde_json::json!({"kind": "change", "a": SESSION, "b": SESSION}),
    )
    .await;
    assert_eq!(s, StatusCode::CONFLICT);
    assert_eq!(body["error"], "E_NOT_RECONSTRUCTED");
    // No bearer, no entry.
    let req = Request::builder()
        .uri("/api/compute/jobs")
        .body(Body::empty())
        .unwrap();
    assert_eq!(
        n.router.clone().oneshot(req).await.unwrap().status(),
        StatusCode::UNAUTHORIZED
    );
}
