//! Ingest-rate and status tests.

use super::tests::{cam, config, started};
use super::*;
use world_engine_protocol::atlas::CameraRole;

#[test]
fn a_camera_that_stops_delivering_reads_zero_hz_not_its_last_rate() {
    let mut s = started(
        config(vec![cam("front", CameraRole::Primary, true)]),
        "sess-stale",
    );
    for i in 0..10i64 {
        s.on_pose_only("front", i * 100);
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
        s.on_pose_only("front", i * 100);
    }
    let st = s.status();
    assert_eq!(st.keyframes, 0, "pose-only never selects a keyframe");
    assert!(
        (st.ingest_rate_hz - 10.0).abs() < 0.5,
        "rate {}",
        st.ingest_rate_hz
    );
    // A disabled/unknown camera is refused on this path too.
    assert!(!s.on_pose_only("nope", 2000));
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
