//! Topic, capability, bearer and envelope tests.

use super::*;

#[test]
fn topic_names_are_stable() {
    assert_eq!(ATLAS_CAPTURE_STATE_TOPIC, "atlas.capture.state");
    assert_eq!(ATLAS_KEYFRAME_TOPIC, "atlas.keyframe");
    assert_eq!(ATLAS_POSE_OFFLOAD_TOPIC, "atlas.pose.offload");
    assert_eq!(PLUGIN_ATLAS_POSE_TOPIC, "plugin.world-engine.pose");
    assert_eq!(
        PLUGIN_ATLAS_POINTCLOUD_TOPIC,
        "plugin.world-engine.pointcloud"
    );
    assert_eq!(
        PLUGIN_ATLAS_OCCUPANCY_TOPIC,
        "plugin.world-engine.occupancy"
    );
    assert_eq!(PLUGIN_ATLAS_SPLAT_TOPIC, "plugin.world-engine.splat");
    assert_eq!(PLUGIN_ATLAS_MESH_TOPIC, "plugin.world-engine.mesh");
}

#[test]
fn atlas_event_round_trips_and_carries_a_struct_payload() {
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
        pose_tier: PoseSource::LocalVio,
        dropped_keyframes: 0,
    };
    let ev = AtlasEvent::new(
        ATLAS_CAPTURE_STATE_TOPIC,
        None,
        status.to_msgpack().unwrap(),
    );
    let back = AtlasEvent::decode(&ev.encode().unwrap()).unwrap();
    assert_eq!(back, ev);
    assert_eq!(back.topic, "atlas.capture.state");
    let inner = CaptureStatus::from_msgpack(&back.payload).unwrap();
    assert_eq!(inner, status);
}

#[test]
fn device_id_round_trips_and_is_skipped_when_absent() {
    // Absent (the local publish-bus shape): the key is omitted on the wire and
    // a decode defaults to None.
    let bare = AtlasEvent::new(ATLAS_KEYFRAME_TOPIC, None, vec![1, 2, 3]);
    let bare_json = serde_json::to_value(&bare).unwrap();
    assert_eq!(
        bare_json["v"], ATLAS_ENVELOPE_VERSION,
        "the envelope version rides on the wire as `v`"
    );
    assert!(
        bare_json.get("device_id").is_none(),
        "device_id is skipped when None"
    );
    assert_eq!(AtlasEvent::decode(&bare.encode().unwrap()).unwrap(), bare);

    // Stamped (the egress shape): the drone id round-trips on the wire so the
    // compute node can attribute the job to the capturing drone.
    let stamped = AtlasEvent::new(ATLAS_KEYFRAME_TOPIC, Some("drone-42".into()), vec![9]);
    let stamped_json = serde_json::to_value(&stamped).unwrap();
    assert_eq!(stamped_json["v"], ATLAS_ENVELOPE_VERSION);
    assert_eq!(stamped_json["device_id"], "drone-42");
    assert_eq!(
        AtlasEvent::decode(&stamped.encode().unwrap()).unwrap(),
        stamped
    );
}

#[test]
fn every_shared_data_topic_names_an_enforced_capability() {
    // The namespace is documented as "any plugin subscribes to", and the
    // host's generic rule grants a plugin only its own plugin.<id>. space,
    // so without this mapping the shared-data topics are reachable by
    // exactly one plugin. Every one must resolve to a capability, and no
    // agent-namespace topic may resolve to any.
    for topic in PLUGIN_ATLAS_TOPICS {
        assert!(
            atlas_topic_subscribe_capability(topic).is_some(),
            "shared-data topic `{topic}` names no capability, so it is either \
             unreachable or ungated"
        );
    }
    assert_eq!(
        atlas_topic_subscribe_capability(PLUGIN_ATLAS_SPLAT_TOPIC),
        Some(ATLAS_WORLD_READ_CAP)
    );
    assert_eq!(
        atlas_topic_subscribe_capability(PLUGIN_ATLAS_OCCUPANCY_TOPIC),
        Some(ATLAS_WORLD_READ_CAP)
    );
    assert_eq!(
        atlas_topic_subscribe_capability(PLUGIN_ATLAS_POSE_TOPIC),
        Some(ATLAS_POSE_READ_CAP)
    );
    // The internal namespace is not shared plugin data: the raw keyframe
    // lane and the capture-state lane are not offered to plugins here.
    for topic in [
        ATLAS_KEYFRAME_TOPIC,
        ATLAS_CAPTURE_STATE_TOPIC,
        ATLAS_POSE_OFFLOAD_TOPIC,
        "plugin.other.thing",
    ] {
        assert_eq!(atlas_topic_subscribe_capability(topic), None);
    }
}

#[test]
fn the_capability_mapping_is_exact_and_cannot_be_inherited_by_a_longer_topic() {
    // A prefix match would turn the gate into a naming convention: a plugin
    // could mint `plugin.world-engine.occupancy.<anything>` and be handed
    // the world-read capability for it. Every one of these must resolve to None so
    // it falls through to the host's ordinary namespace rule instead.
    for topic in [
        "plugin.world-engine.occupancy.evil",
        "plugin.world-engine.occupancy.",
        "plugin.world-engine.splat/x",
        "plugin.world-engine.pose.extra",
        "plugin.world-engine.meshy",
        "plugin.world-engine.",
        "plugin.world-engine",
        "Plugin.World-Engine.Occupancy",
        "",
    ] {
        assert_eq!(
            atlas_topic_subscribe_capability(topic),
            None,
            "`{topic}` must not inherit a shared-data capability"
        );
    }
    // And a shortened topic is not a match either.
    assert_eq!(
        atlas_topic_subscribe_capability(&PLUGIN_ATLAS_OCCUPANCY_TOPIC[..20]),
        None
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
fn atlas_forward_status_defaults_version_to_zero_for_an_old_file() {
    // A file written before the field existed reads back as version 0 (the
    // serde default), so the reader warns best-effort rather than failing.
    let old: AtlasForwardStatus = serde_json::from_str(r#"{"generatedAtMs":5}"#).unwrap();
    assert_eq!(old.version, 0);
    assert_eq!(old.generated_at_ms, 5);
}

#[test]
fn rejects_any_other_envelope_version() {
    // A producer on another version must be refused at decode rather than
    // silently mis-parsed (the pose-bug class), in both directions.
    for v in [ATLAS_ENVELOPE_VERSION - 1, ATLAS_ENVELOPE_VERSION + 1] {
        let other = AtlasEvent {
            v,
            topic: ATLAS_KEYFRAME_TOPIC.into(),
            device_id: None,
            payload: vec![1, 2, 3],
        };
        let err = AtlasEvent::decode(&other.encode().unwrap()).unwrap_err();
        assert!(matches!(
            err,
            AtlasError::Version { got, ours } if got == v && ours == ATLAS_ENVELOPE_VERSION
        ));
    }
}
