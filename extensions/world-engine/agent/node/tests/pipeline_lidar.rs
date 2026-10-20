//! A LiDAR session with operator rooms through the `tour` preset on a node
//! whose gsplat kernels do not run (stand-in runner, feature `mock`): GLIM
//! poses the keyframes (aligned onto the recorded track) before SfM, depth is
//! computed before the masks so the mirror/glass pass runs, Brush stands in
//! for gsplat at full length, and the tour ships with no hero stills. With no
//! recorded track GLIM's poses keep the model local (no geo anchor). Without
//! GLIM (or, for GPU odometry, the NVIDIA runtime) the submit is refused.

#[path = "support/job_api.rs"]
mod job_api;
#[path = "support/lidar_session.rs"]
mod lidar_session;

use std::path::Path;

use axum::http::StatusCode;
use job_api::run_job;
use lidar_session::{body_at, build_session, T0};
use world_engine_node::recon::StandInRunner;

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_lidar_tour_masks_mirrors_and_falls_back_to_brush() {
    let dir = tempfile::tempdir().unwrap();
    let runner = StandInRunner {
        missing: vec!["gsplat_kernels".into()],
        ..Default::default()
    };
    let (done, job_dir) = run_job(
        dir.path(),
        &runner,
        build_session,
        true,
        serde_json::json!({}),
    )
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
    let (done, job_dir) = run_job(
        dir.path(),
        &runner,
        build_session,
        false,
        serde_json::json!({}),
    )
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
    let (s, body) = run_job(
        dir.path(),
        &runner,
        build_session,
        true,
        serde_json::json!({}),
    )
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
    let (s, body) = run_job(dir.path(), &runner, build_session, true, gpu)
        .await
        .unwrap_err();
    assert_eq!(s, StatusCode::UNPROCESSABLE_ENTITY);
    assert_eq!(body["tool"], "nvidia-container-runtime", "{body}");
}
