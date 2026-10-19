use super::control::{stop_and_bag, unwritten_pct};
use crate::config::{CameraConfig, CaptureConfig};
use crate::session::CaptureSession;
use world_engine_protocol::atlas::{CameraRole, CaptureState};

#[test]
fn stop_and_bag_moves_a_live_session_once() {
    let mut s = CaptureSession::new(CaptureConfig {
        cameras: vec![CameraConfig {
            id: "front".into(),
            role: CameraRole::Primary,
            enabled: true,
            reconstruct: true,
        }],
        ..Default::default()
    });
    assert!(!stop_and_bag(&mut s), "idle stays idle");
    s.start("sess".into());
    assert!(stop_and_bag(&mut s));
    assert_eq!(s.state(), CaptureState::Bagged);
    assert!(!stop_and_bag(&mut s), "a second stop does not re-announce");
}

#[test]
fn a_truncated_session_reports_its_unwritten_frames() {
    assert_eq!(unwritten_pct(0, 0), 0.0);
    assert_eq!(unwritten_pct(200, 50), 75.0);
    assert_eq!(unwritten_pct(10, 12), 0.0);
}
