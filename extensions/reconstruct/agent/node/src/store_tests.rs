//! Tests of `store`.

use super::*;
use crate::ComputeJobKind;

fn job(id: &str, kind: ComputeJobKind, now: i64) -> JobRecord {
    JobRecord {
        id: id.into(),
        kind,
        dataset_id: Some("ds-1".into()),
        state: ComputeJobState::Queued,
        progress: 0.0,
        params: serde_json::json!({}),
        result_ref: None,
        error: None,
        created_ms: now,
        updated_ms: now,
    }
}

#[test]
fn dataset_round_trips() {
    let s = JobStore::open_in_memory().unwrap();
    let d = Dataset {
        id: "ds-1".into(),
        kind: "bag".into(),
        created_ms: 100,
        meta: serde_json::json!({ "cameras": 1 }),
    };
    s.insert_dataset(&d).unwrap();
    assert_eq!(s.get_dataset("ds-1").unwrap(), Some(d));
    assert_eq!(s.get_dataset("nope").unwrap(), None);
}

#[test]
fn queue_is_fifo_by_created_ms() {
    let s = JobStore::open_in_memory().unwrap();
    s.submit_job(&job("job-b", ComputeJobKind::Reconstruct, 200))
        .unwrap();
    s.submit_job(&job("job-a", ComputeJobKind::Reconstruct, 100))
        .unwrap();
    // The older job comes first regardless of insertion order.
    assert_eq!(s.next_queued_job().unwrap().unwrap().id, "job-a");
}

#[test]
fn state_transition_and_outputs() {
    let s = JobStore::open_in_memory().unwrap();
    s.submit_job(&job("job-1", ComputeJobKind::Reconstruct, 100))
        .unwrap();
    s.set_job_state("job-1", ComputeJobState::Running, 0.5, None, None, 110)
        .unwrap();
    s.set_job_state(
        "job-1",
        ComputeJobState::Completed,
        1.0,
        Some("mock://splat/ds-1"),
        None,
        120,
    )
    .unwrap();
    let j = s.get_job("job-1").unwrap().unwrap();
    assert_eq!(j.state, ComputeJobState::Completed);
    assert_eq!(j.result_ref.as_deref(), Some("mock://splat/ds-1"));

    s.insert_output(&Output::new(
        "out-1".into(),
        "job-1".into(),
        "splat".into(),
        "mock://splat/ds-1".into(),
        120,
    ))
    .unwrap();
    assert_eq!(s.outputs_for_job("job-1").unwrap().len(), 1);
}

#[test]
fn cancel_only_non_terminal() {
    let s = JobStore::open_in_memory().unwrap();
    s.submit_job(&job("job-1", ComputeJobKind::Reconstruct, 100))
        .unwrap();
    assert!(s.cancel_job("job-1", 110).unwrap());
    // A second cancel is a no-op (already terminal).
    assert!(!s.cancel_job("job-1", 120).unwrap());
    assert_eq!(
        s.get_job("job-1").unwrap().unwrap().state,
        ComputeJobState::Cancelled
    );
}

#[test]
fn retention_purges_terminal() {
    let s = JobStore::open_in_memory().unwrap();
    s.submit_job(&job("old", ComputeJobKind::Reconstruct, 100))
        .unwrap();
    s.set_job_state("old", ComputeJobState::Completed, 1.0, None, None, 100)
        .unwrap();
    s.submit_job(&job("fresh", ComputeJobKind::Reconstruct, 500))
        .unwrap();
    // Both jobs have an output; the purge must cascade-delete the old job's
    // output and leave the fresh one's intact (no orphaned outputs).
    let out = |id: &str, t: i64| {
        Output::new(
            format!("{id}-out"),
            id.into(),
            "splat".into(),
            format!("mock://splat/{id}"),
            t,
        )
    };
    s.insert_output(&out("old", 100)).unwrap();
    s.insert_output(&out("fresh", 500)).unwrap();

    let removed = s.purge_terminal_before(200).unwrap();
    assert_eq!(removed, 1);
    assert!(s.get_job("old").unwrap().is_none());
    assert!(s.get_job("fresh").unwrap().is_some());
    // The cascade fired for the purged job, not for the surviving one.
    assert!(s.outputs_for_job("old").unwrap().is_empty());
    assert_eq!(s.outputs_for_job("fresh").unwrap().len(), 1);
}

#[test]
fn count_in_state_counts_per_state() {
    let s = JobStore::open_in_memory().unwrap();
    s.submit_job(&job("a", ComputeJobKind::Reconstruct, 1))
        .unwrap();
    s.submit_job(&job("b", ComputeJobKind::Reconstruct, 2))
        .unwrap();
    s.set_job_state("b", ComputeJobState::Running, 0.0, None, None, 3)
        .unwrap();
    assert_eq!(s.count_in_state(ComputeJobState::Queued).unwrap(), 1);
    assert_eq!(s.count_in_state(ComputeJobState::Running).unwrap(), 1);
    assert_eq!(s.count_in_state(ComputeJobState::Completed).unwrap(), 0);
}

#[test]
fn duplicate_id_is_a_conflict_not_a_store_fault() {
    let s = JobStore::open_in_memory().unwrap();
    s.submit_job(&job("dup", ComputeJobKind::Reconstruct, 1))
        .unwrap();
    let err = s
        .submit_job(&job("dup", ComputeJobKind::Reconstruct, 2))
        .unwrap_err();
    assert!(matches!(err, ComputeError::Conflict(_)));
}

#[test]
fn requeue_stale_running_reclaims_orphans() {
    let s = JobStore::open_in_memory().unwrap();
    // Simulate a crash mid-backend: a job stranded in Running.
    s.submit_job(&job("stranded", ComputeJobKind::Reconstruct, 1))
        .unwrap();
    s.claim_next_queued(2).unwrap(); // -> Running
    s.submit_job(&job("done", ComputeJobKind::Reconstruct, 3))
        .unwrap();
    s.set_job_state("done", ComputeJobState::Completed, 1.0, None, None, 4)
        .unwrap();

    let n = s.requeue_stale_running(10).unwrap();
    assert_eq!(n, 1); // only the Running one
                      // The stranded job is claimable again; the completed one is untouched.
    assert_eq!(
        s.get_job("stranded").unwrap().unwrap().state,
        ComputeJobState::Queued
    );
    assert_eq!(
        s.get_job("done").unwrap().unwrap().state,
        ComputeJobState::Completed
    );
    assert_eq!(s.claim_next_queued(11).unwrap().unwrap().id, "stranded");
}

#[test]
fn set_state_unknown_job_is_not_found() {
    let s = JobStore::open_in_memory().unwrap();
    let err = s
        .set_job_state("ghost", ComputeJobState::Running, 0.0, None, None, 1)
        .unwrap_err();
    assert!(matches!(err, ComputeError::NotFound(_)));
}
