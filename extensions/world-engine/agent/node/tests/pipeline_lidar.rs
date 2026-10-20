//! A LiDAR session with operator rooms through the `tour` preset on a node
//! whose gsplat kernels do not run (stand-in runner, feature `mock`): GLIM
//! poses the keyframes (aligned onto the recorded track) before SfM, depth is
//! computed before the masks so the mirror/glass pass runs, Brush stands in
//! for gsplat at full length, and the tour ships with no hero stills. With no
//! recorded track GLIM's poses keep the model local (no geo anchor). Without
//! GLIM (or, for GPU odometry, the NVIDIA runtime) the submit is refused.

#[path = "support/lidar_session.rs"]
mod lidar_session;

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use axum::body::Body;
use axum::http::{Request, StatusCode};
use axum::Router;
use lidar_session::{body_at, build_session, T0};
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

/// Upload the session (`track`: with a recorded pose track), submit a `tour`
/// reconstruction (plus `extra` request fields) with `runner` and wait for it
/// to finish. Returns the job view and the job directory, or the refused
/// submission's status and body.
async fn run_job(
    dir: &Path,
    runner: &StandInRunner,
    track: bool,
    extra: serde_json::Value,
) -> Result<(serde_json::Value, PathBuf), (StatusCode, serde_json::Value)> {
    let src = build_session(&dir.join("src"), track);
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

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_lidar_tour_masks_mirrors_and_falls_back_to_brush() {
    let dir = tempfile::tempdir().unwrap();
    let runner = StandInRunner {
        missing: vec!["gsplat_kernels".into()],
        ..Default::default()
    };
    let (done, job_dir) = run_job(dir.path(), &runner, true, serde_json::json!({}))
        .await
        .unwrap();
    assert_eq!(done["state"], "completed", "{done}");

    // GLIM ran over the exported bag with the IMU, its trajectory was aligned
    // onto the recorded track, and COLMAP triangulated against those poses.
    let sfm = &done["stages"][1]["detail"];
    assert_eq!(sfm["glim"]["used"], true, "{sfm}");
    assert_eq!(sfm["glim"]["odometry"], "cpu", "{sfm}");
    assert!(sfm["glim"].get("rejected").is_none(), "{sfm}");
    assert_eq!(sfm["glim"]["trajectory_poses"], 6, "{sfm}");
    assert!(
        sfm["glim"]["align_rmse_m"].as_f64().unwrap() < 1e-6,
        "{sfm}"
    );
    assert_eq!(sfm["method"], "known_poses", "{sfm}");
    assert!(runner
        .runs_of("glim")
        .iter()
        .any(|i| i.args_lossy().iter().any(|a| a == "glim_rosbag")));
    let glim = job_dir.join("glim");
    let meta = std::fs::read_to_string(glim.join("bag/metadata.yaml")).unwrap();
    assert!(
        meta.contains("name: /imu\n") && meta.contains("message_count: 300\n"),
        "{meta}"
    );
    let ros: serde_json::Value =
        serde_json::from_slice(&std::fs::read(glim.join("config/config_ros.json")).unwrap())
            .unwrap();
    assert_eq!(ros["glim_ros"]["imu_topic"], "/imu");
    let global: serde_json::Value =
        serde_json::from_slice(&std::fs::read(glim.join("config/config.json")).unwrap()).unwrap();
    assert_eq!(
        global["global"]["config_odometry"],
        "config_odometry_cpu.json"
    );
    // Frames after the last sweep (+50 ms) keep their recorded pose; all sit
    // on the walk either way.
    let frames: Vec<serde_json::Value> =
        serde_json::from_slice(&std::fs::read(job_dir.join("dataset/frames.json")).unwrap())
            .unwrap();
    let late = frames
        .iter()
        .filter(|f| f["t_ns"].as_u64().unwrap() > T0 + 2_550_000_000)
        .count();
    assert!(late > 0, "{frames:?}");
    assert_eq!(sfm["glim"]["frames_recorded"], late, "{sfm}");
    assert_eq!(sfm["glim"]["frames_dropped"], 0, "{sfm}");
    assert_eq!(
        sfm["glim"]["frames_glim"].as_u64().unwrap() as usize + late,
        frames.len()
    );
    for f in &frames {
        let i = (f["t_ns"].as_u64().unwrap() - T0) / 100_000_000;
        let t = f["T_world_cam"].as_array().expect("posed");
        let want = body_at(i);
        for (k, idx) in [3, 7, 11].into_iter().enumerate() {
            assert!((t[idx].as_f64().unwrap() - want[k]).abs() < 1e-5, "{f}");
        }
    }

    // Depth ran before the masks, so the mirror/glass pass ran with LiDAR.
    let prep = &done["stages"][0]["detail"];
    assert!(prep["lidar_points"].as_u64().unwrap() > 1000, "{prep}");
    assert_eq!(prep["depths"], true);
    assert_eq!(prep["masks"]["mirror_glass"], true, "{prep}");
    assert_eq!(prep["masks"]["person"], false);
    assert!(prep["masks"]["off"][0]
        .as_str()
        .unwrap()
        .starts_with("person"));
    let masks = runner.runs_of("gsplat_train");
    let mask_run = masks
        .iter()
        .find(|i| i.args_lossy().iter().any(|a| a.ends_with("masks.py")))
        .expect("masks.py ran");
    let args = mask_run.args_lossy();
    assert!(args.iter().any(|a| a == "--depths") && args.iter().any(|a| a == "--lidar"));
    let mask_files = std::fs::read_dir(job_dir.join("dataset/masks/cam0"))
        .unwrap()
        .count();
    assert!(mask_files > 0);

    // Brush stood in for gsplat, at full length, and said why.
    let train = &done["stages"][2]["detail"];
    assert_eq!(train["trainer"], "brush");
    assert!(train["fallback_reason"]
        .as_str()
        .unwrap()
        .contains("gsplat"));
    assert_eq!(train["steps"], 30000);

    // The tour shipped; with no gsplat there are no hero stills.
    let tour_json = {
        let mut z =
            zip::ZipArchive::new(std::fs::File::open(job_dir.join("out/tour.zip")).unwrap())
                .unwrap();
        let v: serde_json::Value =
            serde_json::from_reader(z.by_name("tour.json").unwrap()).unwrap();
        v
    };
    for r in tour_json["rooms"].as_array().unwrap() {
        assert!(r["still"].is_null(), "{r}");
    }
    assert!(!runner
        .runs_of("gsplat_train")
        .iter()
        .any(|i| i.args_lossy().iter().any(|a| a.ends_with("stills.py"))));
    // Aligned onto the recorded ENU track, so the products carry the anchor.
    assert_eq!(done["stages"][1]["detail"]["world_frame"], true);
    assert_eq!(place_anchor(&job_dir)["lat_deg"], 12.9716);
    assert_eq!(tour_json["anchor"]["lat_deg"], 12.9716);
}

/// The `anchor` of the scene's placement sidecar.
fn place_anchor(job_dir: &Path) -> serde_json::Value {
    let p = job_dir.join("out/scene.spz.place.json");
    let v: serde_json::Value = serde_json::from_slice(&std::fs::read(p).unwrap()).unwrap();
    v["anchor"].clone()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn glim_poses_without_a_track_keep_the_model_local() {
    let dir = tempfile::tempdir().unwrap();
    let runner = StandInRunner::default();
    let (done, job_dir) = run_job(dir.path(), &runner, false, serde_json::json!({}))
        .await
        .unwrap();
    assert_eq!(done["state"], "completed", "{done}");
    assert!(done["stages"][0]["detail"]["pose_prior"].is_null());
    let sfm = &done["stages"][1]["detail"];
    assert_eq!(sfm["glim"]["used"], true, "{sfm}");
    assert!(sfm["glim"]["align_rmse_m"].is_null(), "{sfm}");
    assert_eq!(sfm["method"], "known_poses", "{sfm}");
    assert_eq!(sfm["world_frame"], false, "{sfm}");
    assert_eq!(done["stages"][4]["detail"]["world_frame"], false);
    // The manifest is anchored, but GLIM's own frame is not ENU.
    assert!(place_anchor(&job_dir).is_null());
    let mut z =
        zip::ZipArchive::new(std::fs::File::open(job_dir.join("out/tour.zip")).unwrap()).unwrap();
    let tour: serde_json::Value = serde_json::from_reader(z.by_name("tour.json").unwrap()).unwrap();
    assert!(tour["anchor"].is_null(), "{tour}");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_lidar_session_without_glim_is_refused_at_submit() {
    let dir = tempfile::tempdir().unwrap();
    let runner = StandInRunner {
        missing: vec!["glim".into()],
        ..Default::default()
    };
    let (s, body) = run_job(dir.path(), &runner, true, serde_json::json!({}))
        .await
        .unwrap_err();
    assert_eq!(s, StatusCode::UNPROCESSABLE_ENTITY);
    assert_eq!(
        (&body["error"], &body["tool"]),
        (&"E_TOOL_MISSING".into(), &"glim".into())
    );
    assert!(runner.runs_of("colmap").is_empty());

    // GPU odometry also needs docker's NVIDIA runtime; CPU odometry does not.
    let dir = tempfile::tempdir().unwrap();
    let runner = StandInRunner {
        missing: vec!["nvidia-container-runtime".into()],
        ..Default::default()
    };
    let gpu = serde_json::json!({"glim_odometry": "gpu"});
    let (s, body) = run_job(dir.path(), &runner, true, gpu).await.unwrap_err();
    assert_eq!(s, StatusCode::UNPROCESSABLE_ENTITY);
    assert_eq!(body["tool"], "nvidia-container-runtime", "{body}");
}
