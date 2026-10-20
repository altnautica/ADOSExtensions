//! A planar LiDAR session through the `tour` preset on a stand-in node
//! (feature `mock`): no GLIM is needed, the scans are matched in-process, the
//! keyframes get `lidar2d` priors the pose-prior mapper refines, the sweeps
//! are registered along the matched trajectory, and training keeps the
//! monocular depth prior. With a recorded track the trajectory is aligned
//! onto it and the model keeps the session's world frame.

#[path = "support/job_api.rs"]
mod job_api;
#[path = "support/lidar_session.rs"]
mod lidar_session;

use std::path::Path;

use job_api::run_job;
use lidar_session::{body_at, build_planar_session, T0};
use world_engine_node::recon::StandInRunner;

/// The `pose_priors` position variances `(x, y, z)` of a job's SfM database.
fn prior_variances(job_dir: &Path) -> Vec<[f64; 3]> {
    let conn = rusqlite::Connection::open(job_dir.join("sfm/database.db")).unwrap();
    let mut stmt = conn
        .prepare("SELECT position_covariance FROM pose_priors")
        .unwrap();
    stmt.query_map([], |r| r.get::<_, Vec<u8>>(0))
        .unwrap()
        .map(|b| {
            let b = b.unwrap();
            let v: Vec<f64> = b
                .as_chunks::<8>()
                .0
                .iter()
                .map(|c| f64::from_le_bytes(*c))
                .collect();
            [v[0], v[4], v[8]]
        })
        .collect()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_planar_lidar_session_is_matched_in_process_and_keeps_the_depth_prior() {
    let dir = tempfile::tempdir().unwrap();
    // No GLIM on this node: a planar LiDAR does not need it.
    let runner = StandInRunner {
        missing: vec!["glim".into()],
        ..Default::default()
    };
    let (done, job_dir) = run_job(
        dir.path(),
        &runner,
        build_planar_session,
        false,
        serde_json::json!({}),
    )
    .await
    .unwrap();
    assert_eq!(done["state"], "completed", "{done}");
    assert!(runner.runs_of("glim").is_empty());
    assert!(!job_dir.join("glim").exists());

    // The scan matcher posed every keyframe; its poses are SfM priors with
    // the planar uncertainty, refined by the pose-prior mapper.
    let sfm = &done["stages"][1]["detail"];
    let l2d = &sfm["lidar2d"];
    assert_eq!(l2d["used"], true, "{sfm}");
    assert_eq!(l2d["scans"], 30, "{sfm}");
    assert_eq!(l2d["imu_samples"], 300, "{sfm}");
    assert!(l2d["submaps"].as_u64().unwrap() >= 1, "{sfm}");
    assert!(l2d["align_rmse_m"].is_null(), "{sfm}");
    assert_eq!(sfm["prior_source"], "lidar2d", "{sfm}");
    assert_eq!(sfm["method"], "pose_prior_mapper", "{sfm}");
    assert_eq!(sfm["world_frame"], false, "{sfm}");
    assert!(sfm.get("glim").is_none(), "{sfm}");
    let variances = prior_variances(&job_dir);
    assert!(!variances.is_empty());
    for v in variances {
        let want = [0.03f64.powi(2), 0.03f64.powi(2), 0.25f64.powi(2)];
        assert!(
            v.iter().zip(want).all(|(a, b)| (a - b).abs() < 1e-12),
            "{v:?}"
        );
    }

    // The trajectory: body poses in the level frame of the first scan, at one
    // height; the keyframes follow the walk.
    let tum = std::fs::read_to_string(job_dir.join("lidar2d/trajectory.tum")).unwrap();
    let rows: Vec<Vec<f64>> = tum
        .lines()
        .map(|l| l.split_whitespace().map(|v| v.parse().unwrap()).collect())
        .collect();
    assert!(rows.len() >= 25, "{tum}");
    assert!(rows.iter().all(|r| r[3] == rows[0][3]), "{tum}");
    let summary: serde_json::Value =
        serde_json::from_slice(&std::fs::read(job_dir.join("lidar2d/summary.json")).unwrap())
            .unwrap();
    for k in [
        "submaps",
        "loops_accepted",
        "loops_rejected",
        "scans",
        "mean_score",
    ] {
        assert_eq!(summary[k], l2d[k], "{k}");
    }
    let frames: Vec<serde_json::Value> =
        serde_json::from_slice(&std::fs::read(job_dir.join("dataset/frames.json")).unwrap())
            .unwrap();
    assert_eq!(l2d["frames_posed"].as_u64().unwrap() as usize, frames.len());
    let start = body_at(0);
    for f in &frames {
        let i = (f["t_ns"].as_u64().unwrap() - T0) / 100_000_000;
        let t = f["T_world_cam"].as_array().expect("posed");
        let want = body_at(i);
        let got = [3, 7, 11].map(|k| t[k].as_f64().unwrap());
        assert!((got[0] - (want[0] - start[0])).abs() < 0.05, "{f}");
        assert!(got[1].abs() < 0.05 && got[2].abs() < 0.05, "{f}");
        assert!(f["prior_sigma_m"].is_null(), "{f}");
    }
    // The sweeps were registered along the matched trajectory.
    let origins: Vec<[f64; 3]> =
        serde_json::from_slice(&std::fs::read(job_dir.join("dataset/lidar_origins.json")).unwrap())
            .unwrap();
    assert!(origins.len() >= 25);
    assert!(job_dir.join("dataset/lidar_points.laz").is_file());
    assert!(l2d["lidar_points"].as_u64().unwrap() > 500, "{sfm}");

    // Training kept the monocular depth prior.
    let prep = &done["stages"][0]["detail"];
    assert_eq!(prep["depths"], true, "{prep}");
    let train = &done["stages"][2]["detail"];
    assert_eq!(train["depth_prior"], true, "{train}");
    assert!(runner.runs_of("gsplat_train").iter().any(|i| i
        .args_lossy()
        .iter()
        .any(|a| a.ends_with("train.py"))
        && i.args_lossy().iter().any(|a| a == "--depths")));
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_planar_trajectory_is_aligned_onto_the_recorded_track() {
    let dir = tempfile::tempdir().unwrap();
    let runner = StandInRunner::default();
    let (done, job_dir) = run_job(
        dir.path(),
        &runner,
        build_planar_session,
        true,
        serde_json::json!({}),
    )
    .await
    .unwrap();
    assert_eq!(done["state"], "completed", "{done}");
    let sfm = &done["stages"][1]["detail"];
    assert_eq!(sfm["lidar2d"]["used"], true, "{sfm}");
    assert!(
        sfm["lidar2d"]["align_rmse_m"].as_f64().unwrap() < 0.05,
        "{sfm}"
    );
    // The recorded track is a planar LiDAR's: never fixed poses.
    assert_eq!(sfm["prior_source"], "lidar2d", "{sfm}");
    assert_eq!(sfm["method"], "pose_prior_mapper", "{sfm}");
    assert_eq!(sfm["world_frame"], true, "{sfm}");
    let frames: Vec<serde_json::Value> =
        serde_json::from_slice(&std::fs::read(job_dir.join("dataset/frames.json")).unwrap())
            .unwrap();
    for f in &frames {
        let i = (f["t_ns"].as_u64().unwrap() - T0) / 100_000_000;
        let t = f["T_world_cam"].as_array().expect("posed");
        let want = body_at(i);
        for (k, idx) in [3, 7, 11].into_iter().enumerate() {
            assert!((t[idx].as_f64().unwrap() - want[k]).abs() < 0.05, "{f}");
        }
    }
    assert_eq!(done["stages"][2]["detail"]["depth_prior"], true);
}
