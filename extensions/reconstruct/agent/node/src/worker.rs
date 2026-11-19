//! The worker: claim a job under the engine lock, run it without the lock,
//! record the result under the lock.
//!
//! Staged jobs (a session reconstruction, change detection, a calibration)
//! run on a blocking thread through [`crate::recon::run_staged`], persisting
//! every stage as it goes; a live-lane keyframe reconstruction and a one-shot
//! offload run through [`Scheduler::run_backend`]. The plugin-host service
//! adds its own handling (streaming offload sessions, world-model fan-out) on
//! top of [`run_staged_job`]; the standalone node runs [`worker_loop`].

use std::path::PathBuf;
use std::sync::{Arc, Mutex as StdMutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use serde::Serialize;

use crate::recon::pipeline::RunCtx;
use crate::recon::{self, ArtifactOut, StageSink, StagedJob, ToolRunner};
use crate::store_stages::{StageRow, StageState};
use crate::{
    rewrite_output_to_artifact_url, ApiState, BackendResult, ComputeJobState, JobRecord, Output,
    Prepared, PreparedInput, Scheduler,
};

/// Wall-clock milliseconds.
pub fn now_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0)
}

fn now_ns() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_nanos() as i64)
        .unwrap_or(0)
}

/// What the worker needs.
#[derive(Clone)]
pub struct WorkerCtx {
    pub state: ApiState,
    pub work_root: PathBuf,
    pub public_base: String,
    pub runner: Arc<dyn ToolRunner>,
    pub tour_viewer: PathBuf,
    pub person_model: Option<PathBuf>,
}

/// Persists a staged job's stages through the engine lock from the blocking
/// thread the job runs on.
struct EngineSink {
    state: ApiState,
    job_id: String,
}

impl StageSink for EngineSink {
    fn stages(&self) -> Vec<StageRow> {
        let engine = self.state.blocking_lock();
        engine
            .scheduler()
            .store()
            .stages(&self.job_id)
            .unwrap_or_default()
    }

    fn init(&self, stages: &[&str]) {
        let engine = self.state.blocking_lock();
        if let Err(e) = engine.scheduler().store().init_stages(&self.job_id, stages) {
            tracing::warn!(job = %self.job_id, error = %e, "stage init failed");
        }
    }

    fn begin(&self, stage: &str) {
        let engine = self.state.blocking_lock();
        let r = engine.scheduler().store().set_stage(
            &self.job_id,
            stage,
            StageState::Running,
            now_ns(),
            None,
        );
        if let Err(e) = r {
            tracing::warn!(job = %self.job_id, stage, error = %e, "stage begin failed");
        }
        tracing::info!(job = %self.job_id, stage, "stage start");
    }

    fn progress(&self, stage: &str, stage_progress: f32, job_progress: f32) {
        let engine = self.state.blocking_lock();
        let _ = engine.scheduler().store().set_stage_progress(
            &self.job_id,
            stage,
            stage_progress,
            job_progress,
            now_ms(),
        );
    }

    fn finish(&self, stage: &str, state: StageState, detail: &serde_json::Value) {
        let engine = self.state.blocking_lock();
        let r = engine.scheduler().store().set_stage(
            &self.job_id,
            stage,
            state,
            now_ns(),
            Some(detail),
        );
        if let Err(e) = r {
            tracing::warn!(job = %self.job_id, stage, error = %e, "stage finish failed");
        }
        tracing::info!(job = %self.job_id, stage, state = ?state, "stage end");
    }

    fn cancelled(&self) -> bool {
        let engine = self.state.blocking_lock();
        matches!(
            engine.scheduler().store().get_job(&self.job_id),
            Ok(Some(j)) if j.state == ComputeJobState::Cancelled
        )
    }
}

/// The deliverables of a staged job as output rows, their URIs fetchable LAN
/// URLs (the local path kept in `meta.local_uri`).
pub fn artifact_outputs(
    job_id: &str,
    arts: &[ArtifactOut],
    work_root: &std::path::Path,
    public_base: &str,
    now_ms: i64,
) -> Vec<Output> {
    let Ok(out_dir) = recon::job_dir(work_root, job_id).map(|d| d.join("out")) else {
        return Vec::new();
    };
    arts.iter()
        .enumerate()
        .map(|(i, a)| {
            let path = out_dir.join(&a.file);
            let mut o = Output::new(
                format!("{job_id}-o{i}"),
                job_id.to_string(),
                a.kind.clone(),
                crate::path_to_file_uri(&path.to_string_lossy()),
                now_ms,
            );
            let mut meta = match &a.meta {
                serde_json::Value::Object(m) => m.clone(),
                _ => serde_json::Map::new(),
            };
            meta.insert("file".into(), a.file.clone().into());
            meta.insert("format".into(), a.format.clone().into());
            meta.insert(
                "place_json".into(),
                a.place_json
                    .clone()
                    .map(Into::into)
                    .unwrap_or(serde_json::Value::Null),
            );
            o.meta = serde_json::Value::Object(meta);
            rewrite_output_to_artifact_url(&mut o, work_root, public_base);
            o
        })
        .collect()
}

/// Run one claimed staged job to its terminal state.
pub async fn run_staged_job(ctx: &WorkerCtx, job: JobRecord, staged: StagedJob) {
    let job_id = job.id.clone();
    {
        let engine = ctx.state.lock().await;
        let _ = engine.scheduler().store().clear_job_error(&job_id);
    }
    let c = ctx.clone();
    let jid = job_id.clone();
    let joined = tokio::task::spawn_blocking(move || {
        let sink = EngineSink {
            state: c.state.clone(),
            job_id: jid.clone(),
        };
        let record = |tool: &recon::ResolvedTool| {
            let engine = c.state.blocking_lock();
            let store = engine.scheduler().store();
            if let Err(e) = store.record_job_tool(&jid, &tool.name, &tool.version) {
                tracing::warn!(job = %jid, tool = %tool.name, error = %e, "recording tool failed");
            }
        };
        let runner = recon::RecordingRunner {
            inner: c.runner.as_ref(),
            on_run: &record,
        };
        let run = RunCtx {
            work_root: &c.work_root,
            runner: &runner,
            sink: &sink,
            tour_viewer: c.tour_viewer.clone(),
            vram_mb: nvidia_gpu().and_then(|g| g.vram_total_mb),
            person_model: c.person_model.clone(),
        };
        recon::run_staged(&jid, &staged, &run)
    })
    .await;
    let result = match joined {
        Ok(r) => r,
        Err(e) => Err(recon::StageError::new(
            "E_PANIC",
            format!("job task panicked: {e}"),
        )),
    };
    let engine = ctx.state.lock().await;
    let backend = match result {
        Ok(arts) => {
            if let Err(e) = engine.scheduler().store().delete_outputs(&job_id) {
                tracing::warn!(job = %job_id, error = %e, "clearing prior outputs failed");
            }
            BackendResult {
                outputs: artifact_outputs(
                    &job_id,
                    &arts,
                    &ctx.work_root,
                    &ctx.public_base,
                    now_ms(),
                ),
                detections: Vec::new(),
                error: None,
            }
        }
        Err(e) => {
            tracing::warn!(job = %job_id, error = %e, "staged job failed");
            if e.code != "E_CANCELLED" {
                let _ = engine.scheduler().store().set_job_error(&job_id, &e);
            }
            BackendResult {
                outputs: Vec::new(),
                detections: Vec::new(),
                error: Some(e.to_string()),
            }
        }
    };
    match engine.scheduler().finalize(&job, backend, now_ms()) {
        Ok(o) => tracing::info!(job = %o.job_id, state = ?o.state, "staged job finished"),
        Err(e) => tracing::error!(job = %job_id, error = %e, "finalize failed"),
    }
}

/// The standalone worker loop: staged jobs, live-lane keyframe
/// reconstructions and one-shot offloads.
pub async fn worker_loop(ctx: WorkerCtx) {
    loop {
        let (prepared, reconstructor, detector) = {
            let engine = ctx.state.lock().await;
            let prepared = engine.scheduler().claim_and_prepare(now_ms());
            let (r, d) = engine.scheduler().backends();
            (prepared, r, d)
        };
        match prepared {
            Ok(Prepared::Ready {
                job,
                input: PreparedInput::Staged(staged),
            }) => run_staged_job(&ctx, job, staged).await,
            Ok(Prepared::Ready { job, input }) => {
                let now = now_ms();
                let j = job.clone();
                let joined = tokio::task::spawn_blocking(move || {
                    Scheduler::run_backend(&*reconstructor, &*detector, &j, &input, now)
                })
                .await;
                let mut result = joined.unwrap_or_else(|e| BackendResult {
                    outputs: Vec::new(),
                    detections: Vec::new(),
                    error: Some(format!("backend panicked: {e}")),
                });
                for o in &mut result.outputs {
                    rewrite_output_to_artifact_url(o, &ctx.work_root, &ctx.public_base);
                }
                let engine = ctx.state.lock().await;
                if let Err(e) = engine.scheduler().finalize(&job, result, now_ms()) {
                    tracing::error!(job = %job.id, error = %e, "finalize failed");
                }
            }
            Ok(Prepared::Failed(o)) => tracing::info!(job = %o.job_id, "job failed at prepare"),
            Ok(Prepared::Empty) => tokio::time::sleep(Duration::from_millis(500)).await,
            Err(e) => {
                tracing::error!(error = %e, "worker claim failed");
                tokio::time::sleep(Duration::from_secs(1)).await;
            }
        }
    }
}

/// One NVIDIA GPU sample (`nvidia-smi`), as surfaced on a running job.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct NvidiaGpu {
    pub name: String,
    pub util_pct: Option<f32>,
    pub vram_used_mb: Option<u64>,
    #[serde(skip)]
    pub vram_total_mb: Option<u64>,
}

/// Parse one `nvidia-smi --query-gpu=name,utilization.gpu,memory.used,
/// memory.total --format=csv,noheader,nounits` line.
pub fn parse_nvidia_line(line: &str) -> Option<NvidiaGpu> {
    let f: Vec<&str> = line.split(',').map(str::trim).collect();
    if f.len() < 4 || f[0].is_empty() {
        return None;
    }
    Some(NvidiaGpu {
        name: f[0].to_string(),
        util_pct: f[1].parse().ok(),
        vram_used_mb: f[2].parse().ok(),
        vram_total_mb: f[3].parse().ok(),
    })
}

/// The first NVIDIA GPU, sampled at most every 5 s; `None` without one.
pub fn nvidia_gpu() -> Option<NvidiaGpu> {
    static CACHE: StdMutex<Option<(Instant, Option<NvidiaGpu>)>> = StdMutex::new(None);
    let mut c = CACHE.lock().unwrap_or_else(|p| p.into_inner());
    if let Some((at, v)) = c.as_ref() {
        if at.elapsed() < Duration::from_secs(5) {
            return v.clone();
        }
    }
    let v = std::process::Command::new("nvidia-smi")
        .args([
            "--query-gpu=name,utilization.gpu,memory.used,memory.total",
            "--format=csv,noheader,nounits",
        ])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| {
            String::from_utf8_lossy(&o.stdout)
                .lines()
                .next()
                .and_then(parse_nvidia_line)
        });
    *c = Some((Instant::now(), v.clone()));
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nvidia_lines_parse() {
        let g = parse_nvidia_line("NVIDIA GeForce RTX 5070 Laptop GPU, 37, 2104, 8151").unwrap();
        assert_eq!(g.name, "NVIDIA GeForce RTX 5070 Laptop GPU");
        assert_eq!(g.util_pct, Some(37.0));
        assert_eq!(g.vram_used_mb, Some(2104));
        assert_eq!(g.vram_total_mb, Some(8151));
        assert!(parse_nvidia_line("").is_none());
    }

    #[test]
    fn artifacts_become_fetchable_outputs() {
        let dir = tempfile::tempdir().unwrap();
        let out = recon::job_dir(dir.path(), "j1").unwrap().join("out");
        std::fs::create_dir_all(&out).unwrap();
        std::fs::write(out.join("scene.spz"), b"x").unwrap();
        let arts = vec![ArtifactOut {
            kind: "splat".into(),
            file: "scene.spz".into(),
            format: "spz".into(),
            place_json: Some("scene.spz.place.json".into()),
            meta: serde_json::json!({ "size_bytes": 1 }),
        }];
        let outs = artifact_outputs("j1", &arts, dir.path(), "http://node:8092", 5);
        assert_eq!(outs[0].kind, "splat");
        assert_eq!(
            outs[0].uri,
            "http://node:8092/artifacts/jobs/j1/out/scene.spz"
        );
        assert_eq!(outs[0].meta["file"], "scene.spz");
        assert_eq!(outs[0].meta["place_json"], "scene.spz.place.json");
        assert_eq!(outs[0].meta["size_bytes"], 1);
    }
}
