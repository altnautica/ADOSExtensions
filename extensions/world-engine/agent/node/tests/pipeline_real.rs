//! The staged pipeline with the real pinned tools (COLMAP, Brush or gsplat,
//! splat-transform, build-lod). Ignored by default: it needs the tools
//! fetched by `node/tools/fetch-tools`. The session defaults to a synthetic
//! posed room rendered at test time (`support/synth_session.rs`);
//! `ADOS_WE_REAL_SESSION` names another session directory instead.
//! `ADOS_WE_REAL_PRESET` picks the preset (default `preview`), and
//! `ADOS_WE_REAL_KEEP=1` keeps the work directory for inspection.
//!
//! ```text
//! ADOS_WE_REAL_PRESET=preview \
//!   cargo test -p world-engine-node --test pipeline_real -- --ignored --nocapture
//! ```

#[path = "support/synth_session.rs"]
mod synth_session;

use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use tokio::sync::Mutex;
use world_engine_node::recon::{ProcessRunner, ToolRunner};
use world_engine_node::worker::{worker_loop, WorkerCtx};
use world_engine_node::{
    ApiState, Cluster, ComputeJobKind, ComputeJobState, Engine, JobRecord, JobStore, MockDetector,
    Scheduler, SelectingReconstructor,
};

fn copy_dir(src: &Path, dst: &Path) {
    std::fs::create_dir_all(dst).unwrap();
    for e in std::fs::read_dir(src).unwrap() {
        let e = e.unwrap();
        let to = dst.join(e.file_name());
        if e.path().is_dir() {
            copy_dir(&e.path(), &to);
        } else {
            std::fs::copy(e.path(), to).unwrap();
        }
    }
}

/// Every file under `dir` with its size, relative paths `/`-separated.
fn list_files(dir: &Path, base: &Path, out: &mut Vec<(String, u64)>) {
    for e in std::fs::read_dir(dir).unwrap() {
        let e = e.unwrap();
        let p = e.path();
        if p.is_dir() {
            list_files(&p, base, out);
        } else {
            let rel = p
                .strip_prefix(base)
                .unwrap()
                .to_string_lossy()
                .replace('\\', "/");
            out.push((rel, e.metadata().unwrap().len()));
        }
    }
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "needs the pinned tools"]
async fn a_real_session_reconstructs_with_the_pinned_tools() {
    let preset = std::env::var("ADOS_WE_REAL_PRESET").unwrap_or_else(|_| "preview".into());
    let dir = tempfile::tempdir().unwrap();
    let synthetic = std::env::var_os("ADOS_WE_REAL_SESSION").is_none();
    let session = match std::env::var("ADOS_WE_REAL_SESSION") {
        Ok(s) => PathBuf::from(s),
        Err(_) => {
            let started = Instant::now();
            let s = synth_session::build(&dir.path().join("synth"));
            println!(
                "synthetic session {} rendered in {:.1}s",
                s.display(),
                started.elapsed().as_secs_f64()
            );
            s
        }
    };
    let session_id = session.file_name().unwrap().to_string_lossy().into_owned();
    let work = dir.path().join("work");
    let sessions = work.join("sessions");
    copy_dir(&session, &sessions.join(&session_id));
    std::fs::write(sessions.join(&session_id).join(".committed"), "{}").unwrap();

    let scheduler = Scheduler::new(
        JobStore::open_in_memory().unwrap(),
        Arc::new(SelectingReconstructor::new(work.clone())),
        Arc::new(MockDetector),
    )
    .with_sessions_root(sessions);
    let engine = Engine::new(scheduler, Cluster::new_master("real"), 1);
    let runner: Arc<dyn ToolRunner> = Arc::new(ProcessRunner::default());
    let state: ApiState = Arc::new(Mutex::new(engine));
    let job = JobRecord {
        id: "real-1".into(),
        kind: ComputeJobKind::Reconstruct,
        dataset_id: Some(session_id.clone()),
        state: ComputeJobState::Queued,
        progress: 0.0,
        params: serde_json::json!({ "session_id": session_id, "preset": preset }),
        result_ref: None,
        error: None,
        created_ms: 1,
        updated_ms: 1,
    };
    state
        .lock()
        .await
        .scheduler()
        .store()
        .submit_job(&job)
        .unwrap();
    tokio::spawn(worker_loop(WorkerCtx {
        state: state.clone(),
        work_root: work.clone(),
        public_base: "http://127.0.0.1:8092".into(),
        runner,
        tour_viewer: world_engine_node::tools::tour_viewer_dir(),
        person_model: None,
    }));
    let started = Instant::now();
    let finished = loop {
        tokio::time::sleep(Duration::from_secs(2)).await;
        let engine = state.lock().await;
        let j = engine
            .scheduler()
            .store()
            .get_job("real-1")
            .unwrap()
            .unwrap();
        if j.state.is_terminal() {
            let stages = engine.scheduler().store().stages("real-1").unwrap();
            let err = engine.scheduler().store().job_error("real-1").unwrap();
            break (j, stages, err);
        }
    };
    let (j, stages, err) = finished;
    println!(
        "preset {preset}: job {:?} after {:.1}s",
        j.state,
        started.elapsed().as_secs_f64()
    );
    for s in &stages {
        let secs = match (s.started_ns, s.ended_ns) {
            (Some(a), Some(b)) => format!("{:.1}s", (b - a) as f64 / 1e9),
            _ => "-".into(),
        };
        println!("stage {} {:?} {secs} {}", s.id, s.state, s.detail);
    }
    let job_dir = work.join("jobs/real-1");
    let out = job_dir.join("out");
    if out.is_dir() {
        let mut files = Vec::new();
        list_files(&out, &out, &mut files);
        files.sort();
        for (f, len) in &files {
            println!("deliverable {f} {len} B");
        }
    }
    if std::env::var_os("ADOS_WE_REAL_KEEP").is_some() {
        println!("kept work dir {}", dir.keep().join("work").display());
    }
    let detail = |id: &str| {
        stages
            .iter()
            .find(|s| s.id == id)
            .map(|s| s.detail.clone())
            .unwrap_or_default()
    };
    let sfm = detail("sfm");
    if synthetic && sfm["method"].is_string() {
        // The ground-truth poses are the prior: every frame is posed, the
        // model is in the session frame, the room yields a dense cloud, and
        // the calibrated intrinsics survive SfM.
        let prep = detail("prepare");
        assert_eq!(prep["pose_prior"], "vio", "{prep}");
        assert!(
            prep["frames"].as_u64().unwrap_or(0) * 4 >= synth_session::FRAMES,
            "{prep}"
        );
        // Preview only triangulates against the prior; the other presets map
        // with it as a weighted constraint.
        let method = match preset.as_str() {
            "preview" => "known_poses",
            _ => "pose_prior_mapper",
        };
        assert_eq!(sfm["method"], method, "{sfm}");
        assert!(sfm["pairs_rejected_by_prior"].is_u64(), "{sfm}");
        assert_eq!(sfm["world_frame"], true, "{sfm}");
        assert!(sfm["points"].as_u64().unwrap_or(0) >= 1000, "{sfm}");
        let cams = std::fs::read_to_string(job_dir.join("sfm/txt/cameras.txt")).unwrap();
        for line in cams
            .lines()
            .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        {
            let p: Vec<f64> = line
                .split_whitespace()
                .skip(4)
                .map(|v| v.parse().unwrap())
                .collect();
            assert!(
                (p[0] / synth_session::FX - 1.0).abs() < 0.01
                    && (p[1] / synth_session::FY - 1.0).abs() < 0.01,
                "camera intrinsics drifted: {line}"
            );
        }
    }
    assert_eq!(j.state, ComputeJobState::Completed, "{err:?}");
    for f in ["scene.spz", "scene.ply", "cloud.laz", "outputs.json"] {
        assert!(out.join(f).is_file(), "{f} missing");
    }
    // The SPZ is version 4 (plaintext `NGSP` header, then the version), the
    // layout the viewer's Spark and the world-engine tiler read.
    let spz = std::fs::read(out.join("scene.spz")).unwrap();
    assert_eq!(&spz[..4], b"NGSP", "scene.spz is not a version 4 SPZ");
    assert_eq!(u32::from_le_bytes(spz[4..8].try_into().unwrap()), 4);
}
