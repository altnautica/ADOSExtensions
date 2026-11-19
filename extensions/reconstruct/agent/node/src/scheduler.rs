//! The scheduler: claim the next queued job, run it on the right backend, write
//! the output, and record the terminal state.
//!
//! The work is split into three steps so a real backend (which runs for
//! minutes) never holds the engine lock across its run:
//!
//! 1. [`Scheduler::claim_and_prepare`] — atomically claim the oldest queued job
//!    (flipping it to `Running`) and gather its input. Runs under the engine
//!    lock; cheap and store-only.
//! 2. [`Scheduler::run_backend`] — an associated fn (no `&self`, no store) that
//!    runs the reconstructor/detector. The worker calls this WITHOUT the lock,
//!    so a long run does not block the API.
//! 3. [`Scheduler::finalize`] — re-read the job and record the terminal state.
//!    If the job is no longer `Running` (cancelled or removed during the run)
//!    the cancel wins: the backend result is discarded, not written.
//!
//! [`Scheduler::run_one`] composes all three synchronously for the tests and
//! the instant mock path; the daemon drives the three steps itself so it can
//! drop the lock around the backend run.

use std::sync::Arc;

use crate::{
    ComputeError, ComputeJobKind, ComputeJobState, Dataset, Detection, Detector, FrameRef,
    JobRecord, JobStore, Output, Reconstructor,
};

/// The result of running one job.
#[derive(Debug, Clone, PartialEq)]
pub struct JobOutcome {
    pub job_id: String,
    pub state: ComputeJobState,
    pub outputs: Vec<Output>,
    /// Detections returned by a perception offload job (empty otherwise).
    pub detections: Vec<Detection>,
}

/// The prepared input a backend needs, gathered while the lock is held so the
/// backend run itself touches no store state.
#[derive(Debug)]
pub enum PreparedInput {
    /// A live-lane reconstruction over a resolved keyframe dataset.
    Reconstruct(Dataset),
    /// A perception offload over one frame.
    Offload { frame: FrameRef },
    /// A staged job (a session reconstruction, change detection or a
    /// calibration) the pipeline worker runs stage by stage.
    Staged(crate::recon::StagedJob),
}

/// The outcome of [`Scheduler::claim_and_prepare`].
#[derive(Debug)]
pub enum Prepared {
    /// The queue was empty; there was nothing to claim.
    Empty,
    /// The claimed job failed at the prepare step (e.g. a missing dataset or a
    /// malformed frame param). Its terminal `Failed` state is already recorded.
    Failed(JobOutcome),
    /// A job is claimed (now `Running`) and ready for the backend.
    Ready {
        job: JobRecord,
        input: PreparedInput,
    },
}

/// What a backend produced, before it is committed to the store. Held by value
/// so it can cross the lock boundary (the backend runs unlocked, finalize
/// commits this under the lock).
#[derive(Debug)]
pub struct BackendResult {
    pub outputs: Vec<Output>,
    pub detections: Vec<Detection>,
    /// `Some` if the backend failed; the message is recorded on the job.
    pub error: Option<String>,
}

/// Ties the store to the backends. Holds the reconstructor and detector behind
/// `Arc` trait objects so the worker can clone a handle, drop the engine lock,
/// and run the backend without holding it.
pub struct Scheduler {
    store: JobStore,
    reconstructor: Arc<dyn Reconstructor>,
    detector: Arc<dyn Detector>,
    /// Where the upload lane keeps committed capture sessions.
    sessions_root: Option<std::path::PathBuf>,
}

impl Scheduler {
    pub fn new(
        store: JobStore,
        reconstructor: Arc<dyn Reconstructor>,
        detector: Arc<dyn Detector>,
    ) -> Self {
        Self {
            store,
            reconstructor,
            detector,
            sessions_root: None,
        }
    }

    /// Set where committed capture sessions live (the upload lane's root).
    pub fn with_sessions_root(mut self, root: std::path::PathBuf) -> Self {
        self.sessions_root = Some(root);
        self
    }

    /// Where committed capture sessions live.
    pub fn sessions_root(&self) -> Option<&std::path::Path> {
        self.sessions_root.as_deref()
    }

    /// Borrow the store (the API layer reads jobs/outputs through it).
    pub fn store(&self) -> &JobStore {
        &self.store
    }

    /// Clone the backend handles so the worker can run them after dropping the
    /// engine lock.
    pub fn backends(&self) -> (Arc<dyn Reconstructor>, Arc<dyn Detector>) {
        (self.reconstructor.clone(), self.detector.clone())
    }

    /// Claim the oldest queued job (flipping it to `Running`) and gather the
    /// input its backend needs. Returns [`Prepared::Empty`] when the queue is
    /// empty. A job that cannot be prepared (missing dataset, malformed frame)
    /// is failed here and returned as [`Prepared::Failed`] — bad input fails the
    /// job, it never propagates, so one bad job never stalls the worker. Run
    /// this under the engine lock; it is store-only and cheap.
    pub fn claim_and_prepare(&self, now_ms: i64) -> Result<Prepared, ComputeError> {
        let job = match self.store.claim_next_queued(now_ms)? {
            Some(j) => j,
            None => return Ok(Prepared::Empty),
        };

        match job.kind {
            ComputeJobKind::Reconstruct | ComputeJobKind::Change | ComputeJobKind::Calibrate
                if crate::recon::is_staged(&job) =>
            {
                match crate::recon::prepare_staged(&self.store, self.sessions_root(), &job) {
                    Ok(staged) => Ok(Prepared::Ready {
                        job,
                        input: PreparedInput::Staged(staged),
                    }),
                    Err(e) => Ok(Prepared::Failed(self.fail_staged(&job.id, &e, now_ms)?)),
                }
            }
            ComputeJobKind::Change | ComputeJobKind::Calibrate => Ok(Prepared::Failed(self.fail(
                &job.id,
                "job is missing its session parameters",
                now_ms,
            )?)),
            ComputeJobKind::Reconstruct => {
                let dataset = match job.dataset_id.as_deref() {
                    Some(id) => self.store.get_dataset(id)?,
                    None => None,
                };
                let Some(dataset) = dataset else {
                    return Ok(Prepared::Failed(self.fail(
                        &job.id,
                        "reconstruct job has no dataset",
                        now_ms,
                    )?));
                };
                Ok(Prepared::Ready {
                    job,
                    input: PreparedInput::Reconstruct(dataset),
                })
            }
            ComputeJobKind::PerceptionOffload => {
                // The frame to process rides the job params on this lane; absent
                // one, a default frame keeps the mock path exercised. A
                // malformed `frame` is bad input: fail the job (recorded).
                let frame: FrameRef = match job.params.get("frame") {
                    Some(v) => match serde_json::from_value(v.clone()) {
                        Ok(f) => f,
                        Err(e) => {
                            return Ok(Prepared::Failed(self.fail(
                                &job.id,
                                &format!("invalid frame param: {e}"),
                                now_ms,
                            )?));
                        }
                    },
                    None => FrameRef {
                        camera_id: "front".into(),
                        width: 1280,
                        height: 720,
                        ts_ms: now_ms,
                    },
                };
                Ok(Prepared::Ready {
                    job,
                    input: PreparedInput::Offload { frame },
                })
            }
        }
    }

    /// Run the backend for a prepared job. An associated fn (no `&self`, no
    /// store access) so the worker can call it WITHOUT the engine lock — the
    /// whole point of the claim/run/finalize split. A backend error is captured
    /// in [`BackendResult::error`] rather than propagating, so finalize records
    /// it as a failed job.
    pub fn run_backend(
        reconstructor: &dyn Reconstructor,
        detector: &dyn Detector,
        job: &JobRecord,
        input: &PreparedInput,
        now_ms: i64,
    ) -> BackendResult {
        match input {
            PreparedInput::Reconstruct(dataset) => {
                match reconstructor.reconstruct(dataset, &job.params) {
                    Ok(out) => {
                        let gaussian_count = out.gaussian_count;
                        // Capture the honest backend name before `out` is moved
                        // into the Output below; `mock` marks a placeholder.
                        let backend = out.backend;
                        let mut output = Output::new(
                            format!("{}-out", job.id),
                            job.id.clone(),
                            out.kind,
                            out.uri,
                            now_ms,
                        );
                        // Surface the backend's result metadata to clients: the
                        // gaussian count and the tool that produced it.
                        output.meta = serde_json::json!({ "gaussian_count": gaussian_count, "backend": backend });
                        BackendResult {
                            outputs: vec![output],
                            detections: Vec::new(),
                            error: None,
                        }
                    }
                    Err(e) => BackendResult {
                        outputs: Vec::new(),
                        detections: Vec::new(),
                        error: Some(e.to_string()),
                    },
                }
            }
            PreparedInput::Offload { frame } => {
                // Pixels arrive on the streaming transport; the job-params lane
                // carries only frame metadata, so pass None. A real detector
                // errors without pixels rather than fabricating a box.
                match detector.infer(frame, None) {
                    Ok(detections) => {
                        let output = Output::new(
                            format!("{}-out", job.id),
                            job.id.clone(),
                            "detection".into(),
                            format!("offload://detection/{}", job.id),
                            now_ms,
                        );
                        BackendResult {
                            outputs: vec![output],
                            detections,
                            error: None,
                        }
                    }
                    Err(e) => BackendResult {
                        outputs: Vec::new(),
                        detections: Vec::new(),
                        error: Some(e.to_string()),
                    },
                }
            }
            // A staged job runs on the pipeline worker ([`crate::worker`]),
            // never through this one-shot path.
            PreparedInput::Staged(_) => BackendResult {
                outputs: Vec::new(),
                detections: Vec::new(),
                error: Some("a staged job runs on the pipeline worker".into()),
            },
        }
    }

    /// Record the terminal state for a job whose backend has run. Re-reads the
    /// job first: if it is no longer `Running` (cancelled or removed while the
    /// backend ran), the cancel WINS — the backend result is discarded and the
    /// current state is returned, never overwritten. Otherwise a backend error
    /// fails the job, and a success inserts every output and completes the job
    /// with the first output's uri as the result ref. Run this under the lock.
    pub fn finalize(
        &self,
        job: &JobRecord,
        result: BackendResult,
        now_ms: i64,
    ) -> Result<JobOutcome, ComputeError> {
        // The backend ran without the lock; a cancel (or a retention purge)
        // could have landed in the meantime. If the job is no longer Running,
        // do not overwrite its terminal state — the cancel wins.
        match self.store.get_job(&job.id)? {
            Some(current) if current.state != ComputeJobState::Running => {
                return Ok(JobOutcome {
                    job_id: job.id.clone(),
                    state: current.state,
                    outputs: Vec::new(),
                    detections: Vec::new(),
                });
            }
            None => {
                return Ok(JobOutcome {
                    job_id: job.id.clone(),
                    state: ComputeJobState::Cancelled,
                    outputs: Vec::new(),
                    detections: Vec::new(),
                });
            }
            Some(_) => {}
        }

        if let Some(message) = result.error {
            self.store.set_job_state(
                &job.id,
                ComputeJobState::Failed,
                0.0,
                None,
                Some(&message),
                now_ms,
            )?;
            return Ok(JobOutcome {
                job_id: job.id.clone(),
                state: ComputeJobState::Failed,
                outputs: Vec::new(),
                detections: Vec::new(),
            });
        }

        for output in &result.outputs {
            self.store.insert_output(output)?;
        }
        let result_ref = result.outputs.first().map(|o| o.uri.clone());
        self.store.set_job_state(
            &job.id,
            ComputeJobState::Completed,
            1.0,
            result_ref.as_deref(),
            None,
            now_ms,
        )?;
        Ok(JobOutcome {
            job_id: job.id.clone(),
            state: ComputeJobState::Completed,
            outputs: result.outputs,
            detections: result.detections,
        })
    }

    /// Claim and run the next queued job synchronously, all in one call. Returns
    /// `None` when the queue is empty. This is the instant path the tests and
    /// the engine `tick` use; the daemon drives `claim_and_prepare` /
    /// `run_backend` / `finalize` itself so it can drop the lock around the
    /// backend run.
    pub fn run_one(&self, now_ms: i64) -> Result<Option<JobOutcome>, ComputeError> {
        match self.claim_and_prepare(now_ms)? {
            Prepared::Empty => Ok(None),
            Prepared::Failed(outcome) => Ok(Some(outcome)),
            Prepared::Ready { job, input } => {
                let result =
                    Self::run_backend(&*self.reconstructor, &*self.detector, &job, &input, now_ms);
                Ok(Some(self.finalize(&job, result, now_ms)?))
            }
        }
    }

    /// Mark a job failed with `message` and return the failed outcome. Used by
    /// the prepare step for bad input.
    fn fail(&self, job_id: &str, message: &str, now_ms: i64) -> Result<JobOutcome, ComputeError> {
        self.store.set_job_state(
            job_id,
            ComputeJobState::Failed,
            0.0,
            None,
            Some(message),
            now_ms,
        )?;
        Ok(JobOutcome {
            job_id: job_id.to_string(),
            state: ComputeJobState::Failed,
            outputs: Vec::new(),
            detections: Vec::new(),
        })
    }

    /// Fail a staged job at prepare, recording its structured error.
    fn fail_staged(
        &self,
        job_id: &str,
        e: &crate::recon::StageError,
        now_ms: i64,
    ) -> Result<JobOutcome, ComputeError> {
        self.store.set_job_error(job_id, e)?;
        self.fail(job_id, &e.to_string(), now_ms)
    }
}

#[cfg(test)]
#[path = "scheduler_tests.rs"]
mod tests;
