//! A posed `warehouse` session larger than one chunk, with the stand-in tool
//! runner (feature `mock`): uploaded over HTTP, reconstructed in spatial
//! chunks, and merged so each gaussian comes from exactly one chunk's core
//! and the merged sparse model feeds the products unchanged.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use ados_capture_contract::foxglove::{Pose, PoseInFrame, Quaternion, RawImage, Vector3};
use ados_capture_contract::*;
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

const TOKEN: &str = "chunks-test-token";
const T0: u64 = 1_000_000_000;
const FRAMES: u64 = 80;
/// Camera looking along body +x (OpenCV axes: x right = -y body, y down =
/// -z body).
const T_BODY_CAM: [f64; 16] = [
    0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

fn rig() -> Rig {
    Rig {
        contract: ContractId::AdosCapture1,
        rig_id: "aisle_rig".into(),
        calib_version: "1".into(),
        sensors: vec![RigSensor {
            id: "cam0".into(),
            kind: RigSensorKind::Camera,
            role: RigSensorRole::FrontLeft,
            t_body_sensor: T_BODY_CAM.into(),
            camera: Some(CameraModel {
                model: CameraModelModel::Pinhole,
                width: 64,
                height: 48,
                fx: 40.0,
                fy: 40.0,
                cx: 32.0,
                cy: 24.0,
                dist: vec![],
                calibrated: false,
                readout_ns: 0,
            }),
            imu_noise: None,
            time_offset_ns: 0,
            lever_arm_m: None,
        }],
    }
}

/// The body at arc length `s` of a 60 m serpentine: three 16 m aisles 6 m
/// apart, joined by 6 m cross runs, 1.5 m up.
fn body_at(s: f64) -> [f64; 3] {
    let (x, y) = match s {
        s if s <= 16.0 => (s, 0.0),
        s if s <= 22.0 => (16.0, s - 16.0),
        s if s <= 38.0 => (16.0 - (s - 22.0), 6.0),
        s if s <= 44.0 => (0.0, 6.0 + (s - 38.0)),
        s => (s - 44.0, 12.0),
    };
    [x, y, 1.5]
}

fn build_session(root: &Path) -> PathBuf {
    let manifest = Manifest {
        contract: ContractId::AdosCapture1,
        session_id: session_id("visor-2", 1_792_368_000, 9),
        device_id: "visor-2".into(),
        profile: ManifestProfile::Visor,
        timebase: Timebase::HubFreerun,
        t0_ns: T0,
        utc_at_t0_ns: None,
        sigma_ns: -1,
        anchor: None,
        rig: RigRef {
            rig_id: "aisle_rig".into(),
            calib_version: "1".into(),
        },
        operator: Operator {
            site: Some("Test warehouse".into()),
            floor: Some(0),
            rooms: vec![],
        },
        segments: vec![],
        health: SessionHealth {
            blur: false,
            texture: false,
            flicker: false,
            corridor: false,
            frames_dropped_pct: 0.0,
        },
    };
    let mut w = SessionWriter::create(root, manifest, &rig()).unwrap();
    for i in 0..FRAMES {
        let t = T0 + i * 100_000_000;
        let p = body_at(60.0 * i as f64 / (FRAMES - 1) as f64);
        let pose = PoseInFrame {
            timestamp: None,
            frame_id: "world".into(),
            pose: Some(Pose {
                position: Some(Vector3 {
                    x: p[0],
                    y: p[1],
                    z: p[2],
                }),
                orientation: Some(Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                }),
            }),
        };
        w.write_proto(&topics::pose("vio"), t, &pose).unwrap();
        let data: Vec<u8> = (0..64 * 48)
            .map(|k| (((k % 64) / 4 + (k / 64) / 4 + i as usize) % 2 * 200) as u8)
            .collect();
        let img = RawImage {
            timestamp: None,
            frame_id: "cam0".into(),
            width: 64,
            height: 48,
            encoding: "mono8".into(),
            step: 64,
            data,
        };
        w.write_proto(&topics::cam_image("cam0"), t, &img).unwrap();
    }
    let dir = w.dir().to_path_buf();
    w.finish().unwrap();
    dir
}

fn auth(dir: &Path) -> Arc<ComputeAuth> {
    #[cfg(unix)]
    {
        let creds = world_engine_node::NodeCredentialStore::open(dir.join("creds.json"), "t");
        Arc::new(ComputeAuth::new(creds).with_owner_token(TOKEN.into()))
    }
    #[cfg(not(unix))]
    {
        let _ = dir;
        Arc::new(ComputeAuth::standalone(TOKEN.into()))
    }
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

/// Every session file but the manifest, `/`-separated, checksums last.
fn session_files(dir: &Path) -> Vec<String> {
    let mut out = Vec::new();
    let mut stack = vec![dir.to_path_buf()];
    while let Some(d) = stack.pop() {
        for e in std::fs::read_dir(&d).unwrap().flatten() {
            let p = e.path();
            if p.is_dir() {
                stack.push(p);
            } else {
                let rel = p
                    .strip_prefix(dir)
                    .unwrap()
                    .to_string_lossy()
                    .replace('\\', "/");
                if rel != "manifest.json" {
                    out.push(rel);
                }
            }
        }
    }
    out.sort_by_key(|r| (r == "checksums.b3", r.clone()));
    out
}

/// Upload the session over the lane and commit it.
async fn upload(router: &Router, src: &Path, sid: &str) {
    let manifest = std::fs::read(src.join("manifest.json")).unwrap();
    let (s, _) = call(
        router,
        "POST",
        &format!("/api/compute/sessions/{sid}"),
        manifest,
    )
    .await;
    assert!(s.is_success(), "post manifest: {s}");
    for rel in session_files(src) {
        let bytes = std::fs::read(src.join(&rel)).unwrap();
        let uri = format!("/api/compute/sessions/{sid}/files/{rel}");
        let (s, _) = call(router, "PUT", &uri, bytes).await;
        assert!(s.is_success(), "put {rel}: {s}");
    }
    let uri = format!("/api/compute/sessions/{sid}/commit");
    let (s, body) = json(router, "POST", &uri, serde_json::json!({})).await;
    assert_eq!(s, StatusCode::OK, "commit: {body}");
}

async fn wait_terminal(router: &Router, job: &str) -> serde_json::Value {
    let mut last = serde_json::Value::Null;
    for _ in 0..900 {
        let uri = format!("/api/compute/jobs/{job}");
        let (_, v) = json(router, "GET", &uri, serde_json::Value::Null).await;
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

fn ply_vertices(path: &Path) -> u64 {
    let bytes = std::fs::read(path).unwrap();
    String::from_utf8_lossy(&bytes)
        .lines()
        .find_map(|l| l.strip_prefix("element vertex "))
        .unwrap()
        .trim()
        .parse()
        .unwrap()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_large_posed_warehouse_reconstructs_in_chunks() {
    let dir = tempfile::tempdir().unwrap();
    let src = build_session(&dir.path().join("src"));
    let sid = src.file_name().unwrap().to_string_lossy().into_owned();
    let work = dir.path().join("work");
    let sessions = dir.path().join("sessions");
    std::fs::create_dir_all(&sessions).unwrap();
    let viewer = dir.path().join("viewer");
    std::fs::create_dir_all(&viewer).unwrap();
    std::fs::write(viewer.join("index.html"), "<html></html>").unwrap();

    let runner = StandInRunner::default();
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
    let auth = auth(dir.path());
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
    upload(&router, &src, &sid).await;

    let (s, sub) = json(
        &router,
        "POST",
        "/api/compute/jobs",
        serde_json::json!({"kind": "reconstruct", "session_id": sid, "preset": "warehouse",
            "params": {"chunk_frames": 20}}),
    )
    .await;
    assert_eq!(s, StatusCode::CREATED, "{sub}");
    let job = sub["job_id"].as_str().unwrap().to_string();
    let done = wait_terminal(&router, &job).await;
    assert_eq!(done["state"], "completed", "{done}");
    let job_dir = work.join("jobs").join(&job);

    // SfM ran per chunk in the session frame and merged one model.
    let sfm = &done["stages"][1]["detail"];
    let chunks = sfm["chunks"].as_array().expect("chunked sfm");
    assert!(chunks.len() >= 3, "{sfm}");
    assert!(chunks
        .iter()
        .all(|c| c["core"].as_array().unwrap().len() == 4));
    assert_eq!(sfm["method"], "known_poses");
    assert_eq!(sfm["world_frame"], true);
    assert_eq!(sfm["registered_images"], FRAMES);
    assert!(sfm["shared_frames"].as_u64().unwrap() > 0, "{sfm}");
    // Every chunk triangulated against the same priors.
    assert!(
        sfm["max_shared_pose_delta_m"].as_f64().unwrap() < 1e-9,
        "{sfm}"
    );
    for c in chunks {
        let id = c["id"].as_u64().unwrap().to_string();
        assert!(job_dir
            .join("dataset/chunks")
            .join(&id)
            .join("frames.json")
            .is_file());
        assert!(job_dir
            .join("sfm/chunks")
            .join(&id)
            .join("txt/points3D.txt")
            .is_file());
    }
    assert!(job_dir.join("dataset/sparse/0").is_dir());
    let converted = runner
        .runs_of("colmap")
        .iter()
        .filter(|i| i.args_lossy().iter().any(|a| a == "BIN"))
        .count();
    assert_eq!(converted, 1);

    // The merged splat holds each chunk's core gaussians once.
    let train = &done["stages"][2]["detail"];
    let tchunks = train["chunks"].as_array().expect("chunked train");
    assert_eq!(tchunks.len(), chunks.len());
    let core: u64 = tchunks
        .iter()
        .map(|c| c["core_gaussians"].as_u64().unwrap())
        .sum();
    let all: u64 = tchunks
        .iter()
        .map(|c| c["gaussians"].as_u64().unwrap())
        .sum();
    let merged = ply_vertices(&job_dir.join("train/splat.ply"));
    assert_eq!(merged, core, "{train}");
    assert_eq!(train["gaussians"], merged);
    assert!(all > merged, "overlap was not cropped: {train}");
    for c in tchunks {
        let id = c["id"].as_u64().unwrap().to_string();
        let ply = job_dir.join("train/chunks").join(&id).join("splat.ply");
        assert_eq!(ply_vertices(&ply), c["gaussians"].as_u64().unwrap());
    }
    assert_eq!(train["shared_frames"], sfm["shared_frames"]);

    let (_, outs) = json(
        &router,
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
    for k in ["splat", "pointcloud", "aisles", "export"] {
        assert!(kinds.contains(&k), "missing {k} in {kinds:?}");
    }
}
