//! A LiDAR session with operator rooms through the `tour` preset on a node
//! whose gsplat kernels do not run (stand-in runner, feature `mock`): GLIM
//! poses the keyframes (aligned onto the recorded track) before SfM, depth is
//! computed before the masks so the mirror/glass pass runs, Brush stands in
//! for gsplat at full length, and the tour ships with no hero stills. Without
//! GLIM the job fails at start with `E_TOOL_MISSING`.

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::Duration;

use ados_capture_contract::foxglove::{
    PackedElementField, PointCloud, Pose, PoseInFrame, Quaternion, RawImage, Vector3,
};
use ados_capture_contract::v1::{ImuBatch, ImuSample};
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

const TOKEN: &str = "lidar-test-token";
const T0: u64 = 1_000_000_000;
const IDENTITY: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];
/// Camera looking along +x (OpenCV axes: x right = -y body, y down = -z body).
const T_BODY_CAM: [f64; 16] = [
    0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

fn sensor(id: &str, kind: RigSensorKind, role: RigSensorRole, t: [f64; 16]) -> RigSensor {
    RigSensor {
        id: id.into(),
        kind,
        role,
        t_body_sensor: t.into(),
        camera: None,
        imu_noise: None,
        time_offset_ns: 0,
        lever_arm_m: None,
    }
}

fn rig() -> Rig {
    let mut cam = sensor(
        "cam0",
        RigSensorKind::Camera,
        RigSensorRole::FrontLeft,
        T_BODY_CAM,
    );
    cam.camera = Some(CameraModel {
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
    });
    Rig {
        contract: ContractId::AdosCapture1,
        rig_id: "lidar_rig".into(),
        calib_version: "1".into(),
        sensors: vec![
            cam,
            sensor("lidar0", RigSensorKind::Lidar, RigSensorRole::Up, IDENTITY),
            sensor("imu0", RigSensorKind::Imu, RigSensorRole::Nav, IDENTITY),
        ],
    }
}

/// Body position at frame `i`: walking along x inside a 6 x 4 m room.
fn body_at(i: u64) -> [f64; 3] {
    [0.5 + 0.1 * i as f64, 2.0, 1.5]
}

/// A 6 x 4 x 2.5 m room in the body frame of a body at `at` (identity
/// rotation), every 10 cm.
fn room_points(at: [f64; 3]) -> Vec<u8> {
    let mut data = Vec::new();
    let mut push = |p: [f64; 3]| {
        for v in [p[0] - at[0], p[1] - at[1], p[2] - at[2]] {
            data.extend_from_slice(&(v as f32).to_le_bytes());
        }
    };
    for xi in 0..=60 {
        for yi in 0..=40 {
            push([xi as f64 * 0.1, yi as f64 * 0.1, 0.0]);
        }
        for zi in 0..=25 {
            push([xi as f64 * 0.1, 0.0, zi as f64 * 0.1]);
            push([xi as f64 * 0.1, 4.0, zi as f64 * 0.1]);
        }
    }
    for yi in 0..=40 {
        for zi in 0..=25 {
            push([0.0, yi as f64 * 0.1, zi as f64 * 0.1]);
            push([6.0, yi as f64 * 0.1, zi as f64 * 0.1]);
        }
    }
    data
}

fn build_session(root: &Path) -> PathBuf {
    let manifest = Manifest {
        contract: ContractId::AdosCapture1,
        session_id: session_id("visor-1", 1_792_368_000, 7),
        device_id: "visor-1".into(),
        profile: ManifestProfile::Visor,
        timebase: Timebase::HubFreerun,
        t0_ns: T0,
        utc_at_t0_ns: None,
        sigma_ns: -1,
        anchor: None,
        rig: RigRef {
            rig_id: "lidar_rig".into(),
            calib_version: "1".into(),
        },
        operator: Operator {
            site: Some("Test flat".into()),
            floor: Some(0),
            rooms: vec![Room {
                name: "Living".into(),
                floor: 0,
                t_start_ns: T0,
                t_end_ns: T0 + 30 * 100_000_000,
            }],
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
    for i in 0..30u64 {
        let t = T0 + i * 100_000_000;
        let p = body_at(i);
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
        w.write_proto(&topics::pose("lidar"), t, &pose).unwrap();
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
        let samples = (0..10u64)
            .map(|k| ImuSample {
                t_ns: t + k * 10_000_000,
                seq: i * 10 + k,
                gyro: vec![0.0, 0.0, 0.0],
                accel: vec![0.0, 0.0, 9.81],
                temp_c: None,
            })
            .collect();
        w.write_proto(&topics::imu("imu0"), t, &ImuBatch { samples })
            .unwrap();
        if i % 5 == 0 {
            let field = |name: &str, offset: u32| PackedElementField {
                name: name.into(),
                offset,
                r#type: 7,
            };
            let pc = PointCloud {
                timestamp: None,
                frame_id: "lidar0".into(),
                pose: None,
                point_stride: 12,
                fields: vec![field("x", 0), field("y", 4), field("z", 8)],
                data: room_points(p),
            };
            w.write_proto(&topics::lidar_points("lidar0"), t, &pc)
                .unwrap();
        }
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
) -> serde_json::Value {
    let (_, b) = call(router, method, uri, serde_json::to_vec(&body).unwrap()).await;
    serde_json::from_slice(&b).unwrap_or(serde_json::Value::Null)
}

/// Upload the session, submit a `tour` reconstruction with `runner` and wait
/// for it to finish. Returns the job view and the job directory.
async fn run_job(dir: &Path, runner: &StandInRunner) -> (serde_json::Value, PathBuf) {
    let src = build_session(&dir.join("src"));
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

    let sub = json(
        &router,
        "POST",
        "/api/compute/jobs",
        serde_json::json!({"kind": "reconstruct", "session_id": sid, "preset": "tour"}),
    )
    .await;
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
    (done, work.join("jobs").join(&job))
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_lidar_tour_masks_mirrors_and_falls_back_to_brush() {
    let dir = tempfile::tempdir().unwrap();
    let runner = StandInRunner {
        missing: vec!["gsplat_kernels".into()],
        ..Default::default()
    };
    let (done, job_dir) = run_job(dir.path(), &runner).await;
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
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_lidar_session_without_glim_fails_with_tool_missing() {
    let dir = tempfile::tempdir().unwrap();
    let runner = StandInRunner {
        missing: vec!["glim".into()],
        ..Default::default()
    };
    let (done, _) = run_job(dir.path(), &runner).await;
    assert_eq!(done["state"], "failed", "{done}");
    assert_eq!(done["error"]["code"], "E_TOOL_MISSING", "{done}");
    assert_eq!(done["error"]["tool"], "glim", "{done}");
    assert!(runner.runs_of("colmap").is_empty());
}
