//! Pose-stream, ingest-rate and status tests.

use super::tests::{cam, config, pose_at, started};
use super::*;
use world_engine_protocol::atlas::CameraRole;

#[test]
fn the_pose_stream_is_decimated_to_one_publish_per_interval_for_the_whole_rig() {
    // The regression: a pose descriptor was produced per accepted frame per
    // camera, so a 3-camera rig at 30 fps published 90 poses/sec onto the same
    // 16-deep bus that carries multi-MB keyframes — against a documented
    // ~10 Hz contract, and raising keyframe eviction pressure threefold.
    //
    // The invariant that matters is that the publish count is bounded by the
    // TIME span and the interval, and is INDEPENDENT of how many cameras the
    // rig has. The exact count on a 33 ms frame grid is not 10: a publish can
    // only land on a frame boundary, so the effective period is 132 ms.
    fn publishes(cameras: &[&str], frames: i64, frame_ms: i64) -> usize {
        let cfg = config(
            cameras
                .iter()
                .map(|id| cam(id, CameraRole::Primary, true))
                .collect(),
        );
        let mut s = CaptureSession::with_pose_interval(cfg, 100);
        s.start("sess-decimate".into());
        s.set_frame_fixed(true);
        let mut n = 0usize;
        for i in 0..frames {
            let ts = i * frame_ms;
            for camera in cameras {
                if s.on_pose_only(camera, pose_at([0.0, 0.0, 0.0]), ts)
                    .is_some()
                {
                    n += 1;
                }
            }
        }
        n
    }

    // ~1 s at 30 fps.
    let one_cam = publishes(&["front"], 30, 33);
    let three_cam = publishes(&["front", "down", "back"], 30, 33);
    assert_eq!(
        one_cam, three_cam,
        "the rig has ONE pose, so the publish rate must not scale with camera count"
    );
    // Bounded by the span and the interval, nowhere near the 90 frames fed in.
    let span_ms = 29 * 33;
    let ceiling = (span_ms / 100) as usize + 1;
    assert!(
        three_cam <= ceiling,
        "expected at most {ceiling} rig-wide pose publishes over {span_ms} ms, got {three_cam}"
    );
    assert!(
        three_cam >= 7,
        "the cadence must still run at roughly the documented rate, got {three_cam}"
    );
}

#[test]
fn a_camera_that_stops_delivering_reads_zero_hz_not_its_last_rate() {
    let mut s = started(
        config(vec![cam("front", CameraRole::Primary, true)]),
        "sess-stale",
    );
    for i in 0..10i64 {
        s.on_pose_only("front", pose_at([0.0, 0.0, 0.0]), i * 100);
    }
    assert!(
        s.status().ingest_rate_hz > 5.0,
        "a live feed reads its rate"
    );
    // The last accepted frame is now older than the staleness window.
    s.last_accept = Some(Instant::now() - INGEST_STALE - Duration::from_millis(100));
    assert_eq!(s.status().ingest_rate_hz, 0.0);
}

#[test]
fn a_pose_only_frame_is_still_counted_in_the_ingest_rate() {
    // The pose-only path must not become a second definition of "accepted":
    // the ingest rate is the camera feed rate, so a non-keyframe frame counts.
    let mut s = started(
        config(vec![cam("front", CameraRole::Primary, true)]),
        "sess-rate",
    );
    for i in 0..11 {
        s.on_pose_only("front", pose_at([0.0, 0.0, 0.0]), i * 100);
    }
    let st = s.status();
    assert_eq!(st.keyframes, 0, "pose-only never selects a keyframe");
    assert!(
        (st.ingest_rate_hz - 10.0).abs() < 0.5,
        "rate {}",
        st.ingest_rate_hz
    );
    // A disabled/unknown camera is refused on this path too.
    assert!(s
        .on_pose_only("nope", pose_at([0.0, 0.0, 0.0]), 2000)
        .is_none());
}

#[test]
fn undelivered_keyframes_are_reported_on_the_status() {
    let mut s = started(
        config(vec![cam("front", CameraRole::Primary, true)]),
        "sess-drop",
    );
    assert_eq!(s.status().dropped_keyframes, 0);
    s.note_keyframe_dropped(2);
    s.note_keyframe_dropped(1);
    assert_eq!(
        s.status().dropped_keyframes,
        3,
        "a keyframe count that claims frames nothing received is a false reading"
    );
}

#[test]
fn the_active_pose_tier_is_reported() {
    let mut s = started(
        config(vec![cam("front", CameraRole::Primary, true)]),
        "sess-tier",
    );
    assert_eq!(s.status().pose_tier, PoseSource::FcEkf);
    s.set_pose_tier(PoseSource::Vio);
    assert_eq!(
        s.status().pose_tier,
        PoseSource::Vio,
        "a switch of provider must be visible on the status"
    );
}
