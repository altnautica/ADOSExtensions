//! The staged pipeline with the real pinned tools (COLMAP, Brush or gsplat,
//! splat-transform, build-lod). Ignored by default: it needs the tools
//! fetched by `node/tools/fetch-tools` and a real capture session.
//!
//! ```text
//! ADOS_WE_REAL_SESSION=<session dir> ADOS_WE_REAL_PRESET=hq \
//!   cargo test -p world-engine-node --test pipeline_real -- --ignored --nocapture
//! ```

use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use tokio::sync::Mutex;
use world_engine_node::recon::{ProcessRunner, ToolRunner};
use world_engine_node::worker::{worker_loop, WorkerCtx};
use world_engine_node::{
    ApiState, Cluster, ComputeJobKind, ComputeJobState, Engine, JobRecord, JobStore, MockDetector,
    Scheduler, SelectingReconstructor,
};

fn copy_dir(src: &std::path::Path, dst: &std::path::Path) {
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

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
#[ignore = "needs the pinned tools and a real capture session"]
async fn a_real_session_reconstructs_with_the_pinned_tools() {
    let session = PathBuf::from(
        std::env::var("ADOS_WE_REAL_SESSION").expect("set ADOS_WE_REAL_SESSION to a session dir"),
    );
    let preset = std::env::var("ADOS_WE_REAL_PRESET").unwrap_or_else(|_| "preview".into());
    let session_id = session.file_name().unwrap().to_string_lossy().into_owned();
    let dir = tempfile::tempdir().unwrap();
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
    for s in &stages {
        println!("{} {:?} {}", s.id, s.state, s.detail);
    }
    assert_eq!(j.state, ComputeJobState::Completed, "{err:?}");
    let out = work.join("jobs/real-1/out");
    for f in ["scene.spz", "scene.ply", "cloud.laz", "outputs.json"] {
        assert!(out.join(f).is_file(), "{f} missing");
    }
}
