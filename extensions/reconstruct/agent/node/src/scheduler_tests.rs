//! Tests of `scheduler`.

use super::*;
use crate::{MockDetector, MockReconstructor};

fn scheduler() -> Scheduler {
    Scheduler::new(
        JobStore::open_in_memory().unwrap(),
        Arc::new(MockReconstructor),
        Arc::new(MockDetector),
    )
}

fn queued(
    id: &str,
    kind: ComputeJobKind,
    dataset_id: Option<&str>,
    params: serde_json::Value,
) -> JobRecord {
    JobRecord {
        id: id.into(),
        kind,
        dataset_id: dataset_id.map(Into::into),
        state: ComputeJobState::Queued,
        progress: 0.0,
        params,
        result_ref: None,
        error: None,
        created_ms: 100,
        updated_ms: 100,
    }
}

fn with_dataset(s: &Scheduler, id: &str) {
    s.store()
        .insert_dataset(&Dataset {
            id: id.into(),
            kind: "bag".into(),
            created_ms: 100,
            meta: serde_json::json!({ "cameras": 1 }),
        })
        .unwrap();
}

#[test]
fn empty_queue_runs_nothing() {
    assert!(scheduler().run_one(1).unwrap().is_none());
}

#[test]
fn reconstruct_job_runs_queue_to_worker_to_output() {
    let s = scheduler();
    with_dataset(&s, "ds-1");
    s.store()
        .submit_job(&queued(
            "job-1",
            ComputeJobKind::Reconstruct,
            Some("ds-1"),
            serde_json::json!({}),
        ))
        .unwrap();

    let outcome = s.run_one(200).unwrap().unwrap();
    assert_eq!(outcome.state, ComputeJobState::Completed);
    assert_eq!(outcome.outputs.len(), 1);
    assert_eq!(outcome.outputs[0].kind, "splat");
    assert_eq!(outcome.outputs[0].uri, "mock://splat/ds-1");

    // The store reflects the terminal state and the artifact.
    let job = s.store().get_job("job-1").unwrap().unwrap();
    assert_eq!(job.state, ComputeJobState::Completed);
    assert_eq!(job.result_ref.as_deref(), Some("mock://splat/ds-1"));
    assert_eq!(s.store().outputs_for_job("job-1").unwrap().len(), 1);
}

#[test]
fn reconstruct_output_carries_gaussian_count_meta() {
    let s = scheduler();
    with_dataset(&s, "ds-1");
    s.store()
        .submit_job(&queued(
            "job-1",
            ComputeJobKind::Reconstruct,
            Some("ds-1"),
            serde_json::json!({}),
        ))
        .unwrap();

    let outcome = s.run_one(200).unwrap().unwrap();
    // The backend's gaussian_count rides the output meta (the MockReconstructor reports 1000).
    assert_eq!(outcome.outputs[0].meta["gaussian_count"], 1000);
    // The honest backend name rides the same meta so a client can badge a
    // placeholder (the mock path here reports `mock`).
    assert_eq!(outcome.outputs[0].meta["backend"], "mock");
    // And both persisted to the store, not just the in-memory outcome.
    let outs = s.store().outputs_for_job("job-1").unwrap();
    assert_eq!(outs[0].meta["gaussian_count"], 1000);
    assert_eq!(outs[0].meta["backend"], "mock");
}

#[test]
fn perception_offload_returns_a_detection() {
    let s = scheduler();
    s.store()
        .submit_job(&queued(
            "job-off",
            ComputeJobKind::PerceptionOffload,
            None,
            serde_json::json!({ "frame": { "camera_id": "front", "width": 640, "height": 480, "ts_ms": 7 } }),
        ))
        .unwrap();

    let outcome = s.run_one(300).unwrap().unwrap();
    assert_eq!(outcome.state, ComputeJobState::Completed);
    assert_eq!(outcome.detections.len(), 1);
    assert_eq!(outcome.detections[0].class, "object");
    assert_eq!(outcome.detections[0].track_id, Some(7));
    assert_eq!(outcome.outputs[0].kind, "detection");
}

#[test]
fn reconstruct_without_dataset_fails_the_job_not_the_worker() {
    let s = scheduler();
    s.store()
        .submit_job(&queued(
            "job-bad",
            ComputeJobKind::Reconstruct,
            Some("missing"),
            serde_json::json!({}),
        ))
        .unwrap();
    let outcome = s.run_one(200).unwrap().unwrap();
    assert_eq!(outcome.state, ComputeJobState::Failed);
    let job = s.store().get_job("job-bad").unwrap().unwrap();
    assert_eq!(job.state, ComputeJobState::Failed);
    assert!(job.error.is_some());
    // The worker is still usable for the next job.
    assert!(s.run_one(201).unwrap().is_none());
}

#[test]
fn offload_with_malformed_frame_fails_the_job_not_the_worker() {
    let s = scheduler();
    // A `frame` that is present but malformed is bad input: it must fail the
    // job (recorded), not propagate and orphan the job in Running.
    s.store()
        .submit_job(&queued(
            "job-bad-frame",
            ComputeJobKind::PerceptionOffload,
            None,
            serde_json::json!({ "frame": { "width": "oops" } }),
        ))
        .unwrap();
    let outcome = s.run_one(200).unwrap().unwrap();
    assert_eq!(outcome.state, ComputeJobState::Failed);
    let job = s.store().get_job("job-bad-frame").unwrap().unwrap();
    assert_eq!(job.state, ComputeJobState::Failed);
    assert!(job
        .error
        .as_deref()
        .unwrap()
        .contains("invalid frame param"));
    // The worker is not stalled.
    assert!(s.run_one(201).unwrap().is_none());
}

#[test]
fn a_staged_job_without_a_committed_session_fails_with_a_structured_error() {
    let s = scheduler();
    s.store()
        .submit_job(&queued(
            "job-stage",
            ComputeJobKind::Reconstruct,
            None,
            serde_json::json!({ "session_id": "dev-20261019T000000Z-0001", "preset": "hq" }),
        ))
        .unwrap();
    let outcome = s.run_one(300).unwrap().unwrap();
    assert_eq!(outcome.state, ComputeJobState::Failed);
    let err = s.store().job_error("job-stage").unwrap().unwrap();
    assert_eq!(err.code, "E_NOT_UPLOADED");
}
#[test]
fn jobs_run_in_fifo_order() {
    let s = scheduler();
    for (id, t) in [("job-2", 200), ("job-1", 100)] {
        s.store()
            .insert_dataset(&Dataset {
                id: format!("ds-{id}"),
                kind: "bag".into(),
                created_ms: t,
                meta: serde_json::json!({}),
            })
            .unwrap();
        let mut j = queued(
            id,
            ComputeJobKind::Reconstruct,
            Some(&format!("ds-{id}")),
            serde_json::json!({}),
        );
        j.created_ms = t;
        j.updated_ms = t;
        s.store().submit_job(&j).unwrap();
    }
    // job-1 (created 100) before job-2 (created 200).
    assert_eq!(s.run_one(300).unwrap().unwrap().job_id, "job-1");
    assert_eq!(s.run_one(301).unwrap().unwrap().job_id, "job-2");
}

#[test]
fn cancel_of_a_running_job_wins() {
    let s = scheduler();
    with_dataset(&s, "ds-1");
    s.store()
        .submit_job(&queued(
            "job-1",
            ComputeJobKind::Reconstruct,
            Some("ds-1"),
            serde_json::json!({}),
        ))
        .unwrap();

    // Claim the job: it is now Running, modeling the moment a long backend
    // run begins (without the lock).
    let (job, input) = match s.claim_and_prepare(200).unwrap() {
        Prepared::Ready { job, input } => (job, input),
        other => panic!("expected a ready job, got {other:?}"),
    };
    assert_eq!(
        s.store().get_job("job-1").unwrap().unwrap().state,
        ComputeJobState::Running
    );

    // A cancel lands while the backend is running.
    assert!(s.store().cancel_job("job-1", 210).unwrap());

    // The backend finishes and we finalize — the cancel must win.
    let (reconstructor, detector) = s.backends();
    let result = Scheduler::run_backend(&*reconstructor, &*detector, &job, &input, 220);
    let outcome = s.finalize(&job, result, 220).unwrap();

    assert_eq!(outcome.state, ComputeJobState::Cancelled);
    assert!(outcome.outputs.is_empty());
    // The store holds Cancelled, not Completed, and no output was written.
    let job = s.store().get_job("job-1").unwrap().unwrap();
    assert_eq!(job.state, ComputeJobState::Cancelled);
    assert!(s.store().outputs_for_job("job-1").unwrap().is_empty());
}
