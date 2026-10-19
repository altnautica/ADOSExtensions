use super::control::stop_and_bag;
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
