use super::*;
use crate::config::{CameraConfig, CaptureProfile, SelectionParams};
use ados_capture_contract::{CameraModelModel, GeoAnchorDatum, GeoAnchorSource};
use world_engine_protocol::atlas::{CameraRole, ImageEncoding};

pub(super) fn cam(id: &str, role: CameraRole, enabled: bool) -> CameraConfig {
    CameraConfig {
        id: id.to_string(),
        role,
        enabled,
        reconstruct: enabled,
    }
}

pub(super) fn config(cameras: Vec<CameraConfig>) -> CaptureConfig {
    CaptureConfig {
        cameras,
        profile: CaptureProfile::Freeform,
        selection: SelectionParams::default(),
    }
}

pub(super) fn anchor() -> GeoAnchor {
    GeoAnchor {
        lat_deg: 12.97,
        lon_deg: 77.59,
        h_ellipsoid_m: 920.0,
        geoid_undulation_m: 0.0,
        datum: GeoAnchorDatum::Wgs84,
        source: GeoAnchorSource::GnssMean,
    }
}

pub(super) fn frame_at(t: [f64; 3]) -> FrameInput {
    FrameInput {
        image: KeyframeImage {
            encoding: ImageEncoding::Jpeg,
            width: 1280,
            height: 720,
            bytes: vec![0xAB; 8],
        },
        camera: CameraModel {
            model: CameraModelModel::Opencv,
            width: 1280,
            height: 720,
            fx: 900.0,
            fy: 900.0,
            cx: 640.0,
            cy: 360.0,
            dist: vec![0.0; 5],
            calibrated: true,
            readout_ns: 0,
        },
        pose: Pose {
            r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            t,
            cov: None,
        },
        pose_cov: vec![0.25; 36],
        pose_source: PoseSource::FcEkf,
        time: TimeAlignment::unmeasured(),
        anchor: Some(anchor()),
        imu_window: vec![ImuSample {
            t_ns: 1_000_000,
            seq: 0,
            gyro: vec![0.0, 0.0, 0.0],
            accel: vec![0.0, 0.0, 9.81],
            temp_c: None,
        }],
    }
}

pub(super) fn pose_at(t: [f64; 3]) -> PoseInput {
    PoseInput {
        pose: Pose {
            r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            t,
            cov: None,
        },
        anchor: Some(anchor()),
    }
}

/// A started session with its world frame fixed: the precondition for
/// keyframe selection.
pub(super) fn started(config: CaptureConfig, id: &str) -> CaptureSession {
    let mut s = CaptureSession::new(config);
    s.start(id.into());
    s.set_frame_fixed(true);
    s
}

#[test]
fn single_camera_flow_produces_keyframes() {
    let mut s = started(
        config(vec![cam("front", CameraRole::Primary, true)]),
        "sess-a",
    );

    // First frame → first keyframe.
    let out = s.on_frame("front", frame_at([0.0, 0.0, 0.0]), 0).unwrap();
    let kf = out.keyframe.expect("first frame is a keyframe");
    assert_eq!(kf.kf_id, 0);
    assert_eq!(kf.session_id, "sess-a");
    assert_eq!(kf.camera_role, CameraRole::Primary);
    assert_eq!(kf.tier, KeyframeTier::Full);
    assert!(kf.flags.is_session_start);
    // The reconstruction inputs the envelope now carries ride through.
    assert_eq!(kf.pose_cov.len(), 36);
    assert_eq!(kf.time, TimeAlignment::unmeasured());

    // A tiny move under threshold → pose only, no keyframe.
    let out = s
        .on_frame("front", frame_at([0.05, 0.0, 0.0]), 100)
        .unwrap();
    assert!(out.keyframe.is_none());

    // A move past the translation threshold → second keyframe.
    let out = s.on_frame("front", frame_at([0.7, 0.0, 0.0]), 200).unwrap();
    let kf = out.keyframe.expect("baseline move is a keyframe");
    assert_eq!(kf.kf_id, 1);
    assert!(!kf.flags.is_session_start);
}

#[test]
fn multi_camera_flow_runs_per_camera_selection() {
    let mut s = started(
        config(vec![
            cam("front", CameraRole::Primary, true),
            cam("down", CameraRole::Down, true),
            cam("back", CameraRole::Back, false),
        ]),
        "sess-b",
    );
    assert_eq!(s.enabled_camera_count(), 2);

    // Each enabled camera selects its own first keyframe independently.
    let f = s.on_frame("front", frame_at([0.0, 0.0, 0.0]), 0).unwrap();
    assert_eq!(f.keyframe.unwrap().camera_role, CameraRole::Primary);
    let d = s.on_frame("down", frame_at([0.0, 0.0, 0.0]), 0).unwrap();
    let dkf = d
        .keyframe
        .expect("down camera selects its own first keyframe");
    assert_eq!(dkf.camera_role, CameraRole::Down);
    assert_eq!(dkf.camera_id, "down");
    // is_session_start marks the SESSION's first keyframe, not each camera's
    // first: the down camera's keyframe is the session's second, so false.
    assert!(!dkf.flags.is_session_start);

    // A disabled camera yields nothing.
    assert!(s.on_frame("back", frame_at([0.0, 0.0, 0.0]), 0).is_none());
    // An unknown camera yields nothing.
    assert!(s.on_frame("nope", frame_at([0.0, 0.0, 0.0]), 0).is_none());

    // Two keyframes (one per enabled camera), and is_session_start only on
    // the very first keyframe of the session.
    assert_eq!(s.status().keyframes, 2);
}

#[test]
fn would_select_mirrors_on_frame_keyframe_decision() {
    let mut s = CaptureSession::new(config(vec![
        cam("front", CameraRole::Primary, true),
        cam("down", CameraRole::Down, false),
    ]));
    // Idle: nothing selects.
    assert!(!s.would_select("front", &frame_at([0.0, 0.0, 0.0]).pose, 0));
    s.start("sess-w".into());
    s.set_frame_fixed(true);
    // A disabled / unknown camera never selects.
    assert!(!s.would_select("down", &frame_at([0.0, 0.0, 0.0]).pose, 0));
    assert!(!s.would_select("nope", &frame_at([0.0, 0.0, 0.0]).pose, 0));
    // First frame: peek says select; committing via on_frame agrees.
    assert!(s.would_select("front", &frame_at([0.0, 0.0, 0.0]).pose, 0));
    assert!(s
        .on_frame("front", frame_at([0.0, 0.0, 0.0]), 0)
        .unwrap()
        .keyframe
        .is_some());
    // A tiny move: peek says no, and on_frame produces no keyframe.
    assert!(!s.would_select("front", &frame_at([0.05, 0.0, 0.0]).pose, 100));
    assert!(s
        .on_frame("front", frame_at([0.05, 0.0, 0.0]), 100)
        .unwrap()
        .keyframe
        .is_none());
    // A baseline move: peek says yes, and on_frame produces a keyframe.
    assert!(s.would_select("front", &frame_at([0.7, 0.0, 0.0]).pose, 200));
    assert!(s
        .on_frame("front", frame_at([0.7, 0.0, 0.0]), 200)
        .unwrap()
        .keyframe
        .is_some());
}

#[test]
fn on_frame_before_start_returns_none() {
    let mut s = CaptureSession::new(config(vec![cam("front", CameraRole::Primary, true)]));
    // State is Idle; no frames accepted.
    assert!(s.on_frame("front", frame_at([0.0, 0.0, 0.0]), 0).is_none());
    assert_eq!(s.state(), CaptureState::Idle);
}

#[test]
fn status_reflects_count_cameras_and_state() {
    let mut s = started(
        config(vec![
            cam("front", CameraRole::Primary, true),
            cam("down", CameraRole::Down, false),
        ]),
        "sess-c",
    );
    s.on_frame("front", frame_at([0.0, 0.0, 0.0]), 0);
    s.on_frame("front", frame_at([1.0, 0.0, 0.0]), 100);

    let st = s.status();
    assert_eq!(st.state, CaptureState::Capturing);
    assert_eq!(st.keyframes, 2);
    assert_eq!(st.camera_count, 1);
    assert_eq!(st.session_id, "sess-c");
    // Two frames 100 ms apart → ~10 Hz ingest.
    assert!(
        (st.ingest_rate_hz - 10.0).abs() < 0.5,
        "rate {}",
        st.ingest_rate_hz
    );
}

#[test]
fn first_keyframe_marks_session_start_only_once() {
    let mut s = started(
        config(vec![cam("front", CameraRole::Primary, true)]),
        "sess-d",
    );
    let first = s
        .on_frame("front", frame_at([0.0, 0.0, 0.0]), 0)
        .unwrap()
        .keyframe
        .unwrap();
    assert!(first.flags.is_session_start);
    let second = s
        .on_frame("front", frame_at([1.0, 0.0, 0.0]), 100)
        .unwrap()
        .keyframe
        .unwrap();
    assert!(!second.flags.is_session_start);
}

#[test]
fn lifecycle_transitions_are_guarded() {
    let mut s = CaptureSession::new(config(vec![cam("front", CameraRole::Primary, true)]));
    // finalize/pause are no-ops while idle.
    s.finalize();
    assert_eq!(s.state(), CaptureState::Idle);

    s.start("sess-e".into());
    s.set_frame_fixed(true);
    s.pause();
    assert_eq!(s.state(), CaptureState::Paused);
    // No frames accepted while paused.
    assert!(s.on_frame("front", frame_at([0.0, 0.0, 0.0]), 0).is_none());
    s.resume();
    assert_eq!(s.state(), CaptureState::Capturing);

    s.finalize();
    assert_eq!(s.state(), CaptureState::Finalizing);
    // No frames accepted while finalizing.
    assert!(s.on_frame("front", frame_at([0.0, 0.0, 0.0]), 0).is_none());
    s.mark_bagged();
    assert_eq!(s.state(), CaptureState::Bagged);
}

#[test]
fn session_wide_keyframe_cap_stops_selecting_past_the_cap() {
    // A capped session stops laying down keyframes once the cap is reached,
    // even as more frames arrive that would each cross the motion threshold —
    // the fix for a looping flight (an orbit revisits the same viewpoints each
    // lap) that would otherwise re-cross the per-camera thresholds forever and
    // bloat the dataset without bound.
    let capped = CaptureConfig {
        cameras: vec![cam("front", CameraRole::Primary, true)],
        profile: CaptureProfile::Orbit,
        selection: SelectionParams {
            max_keyframes: 3,
            ..SelectionParams::default()
        },
    };
    let mut s = started(capped, "sess-cap");
    assert!(!s.status().capped, "not capped before any keyframe");

    // Ten frames, each 1.0 m past the last (well over the 0.5 m threshold) so
    // the per-camera selector would otherwise fire on every one.
    for i in 0..10 {
        s.on_frame("front", frame_at([i as f64, 0.0, 0.0]), i * 100);
    }
    // Exactly the cap, never more, no matter how many frames arrive.
    assert_eq!(s.status().keyframes, 3);

    // The pose stream keeps flowing past the cap (the live-capture model is
    // preserved): an eleventh frame still yields an output, just no keyframe.
    let out = s
        .on_frame("front", frame_at([11.0, 0.0, 0.0]), 1100)
        .unwrap();
    assert!(out.keyframe.is_none());
    // The peek agrees with the commit: at the cap it never selects.
    assert!(!s.would_select("front", &frame_at([12.0, 0.0, 0.0]).pose, 1200));
    assert_eq!(s.status().keyframes, 3);
}

#[test]
fn the_cap_is_reported_so_a_frozen_count_is_not_read_as_live_progress() {
    // The regression: at the cap, state stays `Capturing` and `keyframes`
    // stops moving, so nothing in the status changed and the transition was
    // never republished. The operator surface then showed "capturing" against
    // a frozen count forever, indistinguishable from a stalled camera.
    let capped = CaptureConfig {
        cameras: vec![cam("front", CameraRole::Primary, true)],
        profile: CaptureProfile::Orbit,
        selection: SelectionParams {
            max_keyframes: 2,
            ..SelectionParams::default()
        },
    };
    let mut s = started(capped, "sess-capflag");
    s.on_frame("front", frame_at([0.0, 0.0, 0.0]), 0);
    assert!(!s.status().capped);
    s.on_frame("front", frame_at([1.0, 0.0, 0.0]), 100);
    assert!(s.status().capped, "the cap is reported the moment it bites");
    assert!(s.is_capped());
}

#[test]
fn zero_keyframe_cap_is_unlimited() {
    // The default cap of 0 preserves today's behaviour: no session-wide bound,
    // so every moving frame past the first still lays down a keyframe.
    let uncapped = CaptureConfig {
        cameras: vec![cam("front", CameraRole::Primary, true)],
        profile: CaptureProfile::Orbit,
        selection: SelectionParams::default(), // max_keyframes == 0
    };
    assert_eq!(uncapped.selection.max_keyframes, 0);
    let mut s = started(uncapped, "sess-unlimited");
    for i in 0..50 {
        s.on_frame("front", frame_at([i as f64, 0.0, 0.0]), i * 100);
    }
    assert_eq!(s.status().keyframes, 50);
    assert!(!s.status().capped);
}

#[test]
fn no_keyframe_is_selected_before_the_frame_is_fixed() {
    // Keyframes selected before the world frame is fixed would sit in a
    // different frame of reference than later ones under one session id.
    let mut s = CaptureSession::new(config(vec![cam("front", CameraRole::Primary, true)]));
    s.start("sess-frame".into());

    // Frames arrive; the pose stream flows, but nothing is selected.
    for i in 0..5 {
        let out = s
            .on_frame("front", frame_at([i as f64, 0.0, 0.0]), i * 100)
            .expect("the frame is still accepted and counted");
        assert!(
            out.keyframe.is_none(),
            "no keyframe may be selected without a world frame"
        );
    }
    assert_eq!(s.status().keyframes, 0);
    assert!(!s.would_select("front", &frame_at([9.0, 0.0, 0.0]).pose, 900));

    // The frame is fixed; selection starts immediately.
    s.set_frame_fixed(true);
    let out = s.on_frame("front", frame_at([9.0, 0.0, 0.0]), 900).unwrap();
    assert!(
        out.keyframe.is_some(),
        "selection starts once the frame is fixed"
    );
}

#[test]
fn anchored_reports_the_geo_anchor_and_does_not_gate_selection() {
    // A session with a fixed local frame but no geo anchor selects keyframes
    // and reports `anchored: false`; it must never claim an anchor it lacks.
    let mut s = started(
        config(vec![cam("front", CameraRole::Primary, true)]),
        "sess-a",
    );
    s.set_anchored(false);
    assert!(!s.status().anchored);
    let out = s.on_frame("front", frame_at([0.0, 0.0, 0.0]), 0).unwrap();
    assert!(
        out.keyframe.is_some(),
        "an unanchored local frame still selects"
    );
    s.set_anchored(true);
    assert!(s.status().anchored);
}

#[test]
fn a_new_session_does_not_inherit_the_previous_latches() {
    let mut s = started(config(vec![cam("front", CameraRole::Primary, true)]), "one");
    s.set_anchored(true);
    s.start("two".into());
    assert!(!s.status().anchored, "the anchor is per-session");
    assert!(
        !s.would_select("front", &frame_at([0.0, 0.0, 0.0]).pose, 0),
        "inheriting the frame latch would select before this run has a world frame"
    );
}
