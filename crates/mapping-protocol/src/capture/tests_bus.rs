//! Bearer and envelope tests.

use super::*;

#[test]
fn node_event_round_trips_and_carries_a_struct_payload() {
    // The bus wrapper carries an already-encoded topic struct as its opaque
    // payload, so a subscriber demuxes by topic and decodes the inner struct.
    let status = CaptureStatus {
        session_id: "sess-1".into(),
        state: CaptureState::Capturing,
        keyframes: 3,
        vio_health: VioHealth::Good,
        camera_count: 2,
        ingest_rate_hz: 9.5,
        capped: false,
        anchored: true,
        pose_tier: PoseSource::FcEkf,
        dropped_keyframes: 0,
        reconstruct_steps: None,
    };
    let ev = NodeEvent::new(CAPTURE_STATE_TOPIC, None, status.to_msgpack().unwrap());
    let back = NodeEvent::decode(&ev.encode().unwrap()).unwrap();
    assert_eq!(back, ev);
    assert_eq!(back.topic, "capture.state");
    let inner = CaptureStatus::from_msgpack(&back.payload).unwrap();
    assert_eq!(inner, status);
}

#[test]
fn device_id_round_trips_and_is_skipped_when_absent() {
    // Absent (the local publish-bus shape): the key is omitted on the wire and
    // a decode defaults to None.
    let bare = NodeEvent::new(CAPTURE_KEYFRAME_TOPIC, None, vec![1, 2, 3]);
    let bare_json = serde_json::to_value(&bare).unwrap();
    assert_eq!(
        bare_json["v"], EVENT_ENVELOPE_VERSION,
        "the envelope version rides on the wire as `v`"
    );
    assert!(
        bare_json.get("device_id").is_none(),
        "device_id is skipped when None"
    );
    assert_eq!(NodeEvent::decode(&bare.encode().unwrap()).unwrap(), bare);

    // Stamped (the egress shape): the drone id round-trips on the wire so the
    // compute node can attribute the job to the capturing drone.
    let stamped = NodeEvent::new(CAPTURE_KEYFRAME_TOPIC, Some("drone-42".into()), vec![9]);
    let stamped_json = serde_json::to_value(&stamped).unwrap();
    assert_eq!(stamped_json["v"], EVENT_ENVELOPE_VERSION);
    assert_eq!(stamped_json["device_id"], "drone-42");
    assert_eq!(
        NodeEvent::decode(&stamped.encode().unwrap()).unwrap(),
        stamped
    );
}

#[test]
fn the_wfb_relay_bearer_reports_that_it_cannot_carry_keyframes() {
    // The field topology that has no LAN degrades to pose + status only, and
    // the surface must say so instead of reporting a working bearer.
    assert!(!bearer_carries_keyframes("wfb-relay"));
    assert!(bearer_keyframe_degraded_reason("wfb-relay").is_some());
    for bearer in ["direct-lan", "cloud"] {
        assert!(bearer_carries_keyframes(bearer));
        assert_eq!(bearer_keyframe_degraded_reason(bearer), None);
    }
}

#[test]
fn forward_status_defaults_version_to_zero_for_an_old_file() {
    // A file written before the field existed reads back as version 0 (the
    // serde default), so the reader warns best-effort rather than failing.
    let old: KeyframeForwardStatus = serde_json::from_str(r#"{"generatedAtMs":5}"#).unwrap();
    assert_eq!(old.version, 0);
    assert_eq!(old.generated_at_ms, 5);
}

#[test]
fn rejects_any_other_envelope_version() {
    // A producer on another version must be refused at decode rather than
    // silently mis-parsed (the pose-bug class), in both directions.
    for v in [EVENT_ENVELOPE_VERSION - 1, EVENT_ENVELOPE_VERSION + 1] {
        let other = NodeEvent {
            v,
            topic: CAPTURE_KEYFRAME_TOPIC.into(),
            device_id: None,
            payload: vec![1, 2, 3],
        };
        let err = NodeEvent::decode(&other.encode().unwrap()).unwrap_err();
        assert!(matches!(
            err,
            EnvelopeError::Version { got, ours } if got == v && ours == EVENT_ENVELOPE_VERSION
        ));
    }
}
