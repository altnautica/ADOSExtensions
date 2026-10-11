use super::*;
use crate::offload_ws::DetectionBroadcaster;
use crate::{ComputeError, Detection, Detector, FrameRef, MockDetector, VecFrameStream};
use std::sync::atomic::{AtomicU32, Ordering};

fn manager() -> OffloadSessionManager {
    OffloadSessionManager::new(
        Arc::new(DetectionBroadcaster::new(64)),
        Arc::new(MockDetector),
        true,
    )
}

#[tokio::test]
async fn a_disabled_manager_declines_a_session() {
    let mgr = OffloadSessionManager::new(
        Arc::new(DetectionBroadcaster::new(64)),
        Arc::new(MockDetector),
        false, // serving disabled
    );
    mgr.start_with_stream(spec("s-off"), VecFrameStream::new(vec![]))
        .await;
    assert_eq!(
        mgr.active_count().await,
        0,
        "a disabled node starts no session"
    );
}

fn spec(id: &str) -> SessionSpec {
    SessionSpec {
        id: id.into(),
        rtsp_url: "rtsp://drone.local:8554/main".into(),
        camera_id: "front".into(),
        width: 64,
        height: 48,
    }
}

#[test]
fn from_job_params_reads_a_session_block() {
    let params = serde_json::json!({
        "session": { "id": "s1", "rtsp_url": "rtsp://d:8554/main", "camera_id": "front", "width": 640, "height": 480 }
    });
    let s = SessionSpec::from_job_params(&params).unwrap();
    assert_eq!(s.id, "s1");
    assert_eq!(s.rtsp_url, "rtsp://d:8554/main");
    assert_eq!(s.camera_id, "front");
    assert_eq!(s.width, 640);
    assert_eq!(s.height, 480);
}

#[test]
fn from_job_params_defaults_the_frame_size_when_omitted() {
    let params = serde_json::json!({
        "session": { "id": "s1", "rtsp_url": "rtsp://d:8554/main", "camera_id": "front" }
    });
    let s = SessionSpec::from_job_params(&params).unwrap();
    assert_eq!(s.width, DEFAULT_WIDTH);
    assert_eq!(s.height, DEFAULT_HEIGHT);
}

#[test]
fn a_per_frame_job_carries_no_session() {
    // The one-shot per-frame offload job (a `frame`, no `session`) yields None
    // so the worker takes the unchanged per-frame path.
    let params = serde_json::json!({ "frame": { "camera_id": "front", "width": 640, "height": 480, "ts_ms": 1 } });
    assert!(SessionSpec::from_job_params(&params).is_none());
    // A session block missing required fields is treated as absent, not fatal.
    let bad = serde_json::json!({ "session": { "camera_id": "front" } });
    assert!(SessionSpec::from_job_params(&bad).is_none());
    let empty_id = serde_json::json!({ "session": { "id": "", "rtsp_url": "rtsp://d/main", "camera_id": "front" } });
    assert!(SessionSpec::from_job_params(&empty_id).is_none());
}

#[test]
fn work_priority_prefers_streaming_over_batch() {
    assert!(
        OffloadSessionManager::STREAMING_PRIORITY > OffloadSessionManager::BATCH_PRIORITY,
        "streaming outranks batch"
    );
}

#[tokio::test]
async fn should_yield_to_streaming_only_when_a_session_is_live() {
    let mgr = manager();
    assert!(
        !mgr.should_yield_to_streaming(),
        "no session live -> no yield"
    );
    mgr.start_with_stream(
        spec("s1"),
        VecFrameStream::solid("front", 16, 16, 100_000, 0),
    )
    .await;
    assert!(
        mgr.should_yield_to_streaming(),
        "a live session -> batch yields to it"
    );
    assert!(mgr.stop("s1").await);
}

#[tokio::test]
async fn start_with_stream_runs_a_session_and_reaps_it_on_end() {
    let mgr = manager();
    let mut rx = mgr.broadcaster().subscribe();
    // A finite synthetic stream: the session runs, emits, then ends.
    let stream = VecFrameStream::solid("front", 64, 48, 3, 1000);
    mgr.start_with_stream(spec("s1"), stream).await;

    // Three batches fan out over the broadcaster.
    for i in 0..3 {
        let b = tokio::time::timeout(std::time::Duration::from_secs(5), rx.recv())
            .await
            .expect("a batch within 5s")
            .expect("broadcaster open");
        assert_eq!(b.session_id, "s1");
        assert_eq!(b.seq, i);
    }

    // The stream exhausted; the session self-reaps from the registry.
    for _ in 0..200 {
        if mgr.active_count().await == 0 {
            break;
        }
        tokio::time::sleep(std::time::Duration::from_millis(5)).await;
    }
    assert_eq!(mgr.active_count().await, 0, "the ended session was reaped");
}

#[tokio::test]
async fn a_resubmit_of_a_live_session_is_ignored() {
    let mgr = manager();
    // A long stream so the first session stays live across the second submit.
    mgr.start_with_stream(
        spec("s1"),
        VecFrameStream::solid("front", 16, 16, 100_000, 0),
    )
    .await;
    // It registers as live synchronously on start.
    assert_eq!(mgr.active_count().await, 1);
    // A re-submit of the same id must not start a second session.
    mgr.start_with_stream(
        spec("s1"),
        VecFrameStream::solid("front", 16, 16, 100_000, 0),
    )
    .await;
    assert_eq!(mgr.active_count().await, 1, "re-submit deduped");
    // Stop it cleanly.
    assert!(mgr.stop("s1").await);
}

#[tokio::test]
async fn a_session_restarts_under_the_same_id_after_the_previous_one_ended() {
    let mgr = manager();
    let mut rx = mgr.broadcaster().subscribe();

    // First session over a finite stream: it runs, emits, then ends + self-reaps.
    mgr.start_with_stream(spec("s1"), VecFrameStream::solid("front", 64, 48, 2, 1000))
        .await;
    for i in 0..2 {
        let b = tokio::time::timeout(std::time::Duration::from_secs(5), rx.recv())
            .await
            .expect("a batch within 5s")
            .expect("broadcaster open");
        assert_eq!(b.seq, i);
    }
    for _ in 0..200 {
        if mgr.active_count().await == 0 {
            break;
        }
        tokio::time::sleep(std::time::Duration::from_millis(5)).await;
    }
    assert_eq!(mgr.active_count().await, 0, "the first session was reaped");

    // Re-open the SAME id: it must start a FRESH session (not be deduped
    // against the ended one), so batches flow again with seq restarting at 0.
    mgr.start_with_stream(spec("s1"), VecFrameStream::solid("front", 64, 48, 2, 2000))
        .await;
    let b = tokio::time::timeout(std::time::Duration::from_secs(5), rx.recv())
        .await
        .expect("the re-opened session emits")
        .expect("broadcaster open");
    assert_eq!(b.session_id, "s1");
    assert_eq!(b.seq, 0, "a fresh session restarts the sequence");
}

#[tokio::test]
async fn the_session_counter_tracks_live_sessions() {
    let mgr = manager();
    let counter = mgr.session_counter();
    assert_eq!(counter.load(Ordering::Relaxed), 0);

    mgr.start_with_stream(
        spec("s1"),
        VecFrameStream::solid("front", 16, 16, 100_000, 0),
    )
    .await;
    // The registration is synchronous with the start, so it is already 1 here.
    assert_eq!(counter.load(Ordering::Relaxed), 1);
    assert_eq!(
        mgr.active_count().await,
        1,
        "counter agrees with the registry"
    );

    assert!(mgr.stop("s1").await);
    // The decrement lands when the supervisor closes the session.
    for _ in 0..200 {
        if counter.load(Ordering::Relaxed) == 0 {
            break;
        }
        tokio::time::sleep(std::time::Duration::from_millis(5)).await;
    }
    assert_eq!(
        counter.load(Ordering::Relaxed),
        0,
        "the counter decremented when the session ended"
    );
}

#[tokio::test]
async fn stop_all_cancels_live_sessions() {
    let mgr = manager();
    mgr.start_with_stream(
        spec("a"),
        VecFrameStream::solid("front", 16, 16, 100_000, 0),
    )
    .await;
    mgr.start_with_stream(
        spec("b"),
        VecFrameStream::solid("front", 16, 16, 100_000, 0),
    )
    .await;
    assert_eq!(mgr.active_count().await, 2);
    mgr.stop_all().await;
    for _ in 0..200 {
        if mgr.active_count().await == 0 {
            break;
        }
        tokio::time::sleep(std::time::Duration::from_millis(5)).await;
    }
    assert_eq!(
        mgr.active_count().await,
        0,
        "all sessions cancelled + reaped"
    );
}

/// A detector that always fails, counting its invocations, to drive the
/// supervisor's backend-fault restart budget.
struct FaultDetector {
    calls: Arc<AtomicU32>,
}
impl Detector for FaultDetector {
    fn name(&self) -> &str {
        "fault"
    }
    fn infer(
        &self,
        _frame: &FrameRef,
        _pixels: Option<&[u8]>,
    ) -> Result<Vec<Detection>, ComputeError> {
        self.calls.fetch_add(1, Ordering::Relaxed);
        Err(ComputeError::Backend {
            backend: "fault".into(),
            message: "always fails".into(),
        })
    }
}

#[tokio::test]
async fn a_backend_faulting_session_restarts_under_budget_then_gives_up() {
    let calls = Arc::new(AtomicU32::new(0));
    let mgr = OffloadSessionManager::new(
        Arc::new(DetectionBroadcaster::new(64)),
        Arc::new(FaultDetector {
            calls: calls.clone(),
        }),
        true,
    );
    // A factory that yields a FRESH one-frame stream each (re)start, so the
    // detector runs once per run and faults, exercising the restart budget.
    mgr.start_with_factory(spec("s-fault"), || {
        Some(VecFrameStream::solid("front", 8, 8, 1, 0))
    })
    .await;

    // The session gives up (never restart-loops forever) and closes.
    for _ in 0..600 {
        if mgr.active_count().await == 0 {
            break;
        }
        tokio::time::sleep(std::time::Duration::from_millis(10)).await;
    }
    assert_eq!(
        mgr.active_count().await,
        0,
        "the persistently-faulting session gave up + closed"
    );
    // The detector ran once per run: 1 initial run + MAX_RESTARTS restarts.
    assert_eq!(
        calls.load(Ordering::Relaxed),
        1 + OffloadSessionManager::MAX_RESTARTS,
        "restarts are bounded by the budget"
    );
}
