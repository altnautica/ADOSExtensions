//! The live navigation and mapping payloads: topic names, capabilities and
//! the exact wire keys the operator surfaces read.

use super::*;

#[test]
fn the_nav_topics_are_shared_and_gated_by_telemetry_read() {
    assert_eq!(PLUGIN_ATLAS_SCAN_TOPIC, "plugin.world-engine.scan");
    assert_eq!(
        PLUGIN_ATLAS_MAP_STATUS_TOPIC,
        "plugin.world-engine.map_status"
    );
    assert_eq!(PLUGIN_ATLAS_LIDAR_TOPIC, "plugin.world-engine.lidar");
    for topic in [
        PLUGIN_ATLAS_SCAN_TOPIC,
        PLUGIN_ATLAS_MAP_STATUS_TOPIC,
        PLUGIN_ATLAS_LIDAR_TOPIC,
    ] {
        assert!(PLUGIN_ATLAS_TOPICS.contains(&topic));
        assert_eq!(
            atlas_topic_subscribe_capability(topic),
            Some(ATLAS_POSE_READ_CAP)
        );
    }
    assert_eq!(
        atlas_topic_subscribe_capability("plugin.world-engine.scan.x"),
        None
    );
    assert_eq!(
        atlas_topic_subscribe_capability("plugin.world-engine.lidar.x"),
        None
    );
}

#[test]
fn the_pose_descriptor_carries_health_frame_and_source_as_words() {
    let pd = PoseDescriptor {
        pose: Pose {
            r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            t: [1.0, -2.0, 0.0],
            cov: None,
        },
        anchor: None,
        ts_ms: 7,
        health: VioHealth::Good,
        frame: NavFrame::MapNed,
        source: NavPoseSource::LidarPlanar,
        north: Some(MapNorth::Declared),
        sigma_xy_m: Some(0.35),
        imu_feed: Some(ImuFeedReason::Stalled),
    };
    let v = serde_json::to_value(&pd).unwrap();
    assert_eq!(v["health"], "good");
    assert_eq!(v["frame"], "map_ned");
    assert_eq!(v["source"], "lidar_planar");
    assert_eq!(v["north"], "declared");
    assert_eq!(v["sigma_xy_m"], 0.35);
    assert_eq!(v["imu_feed"], "stalled");
    assert_eq!(v["pose"]["R"][0], 1.0);
    assert_eq!(
        PoseDescriptor::from_msgpack(&pd.to_msgpack().unwrap()).unwrap(),
        pd
    );
    // An older publisher sends no north, sigma or IMU feed; they read as
    // unknown.
    let mut old = v.clone();
    for key in ["north", "sigma_xy_m", "imu_feed"] {
        old.as_object_mut().unwrap().remove(key);
    }
    let parsed: PoseDescriptor = serde_json::from_value(old).unwrap();
    assert_eq!(parsed.north, None);
    assert_eq!(parsed.sigma_xy_m, None);
    assert_eq!(parsed.imu_feed, None);
    let none = PoseDescriptor {
        north: None,
        sigma_xy_m: None,
        imu_feed: None,
        ..pd
    };
    let v = serde_json::to_value(&none).unwrap();
    assert!(v.get("north").is_none() && v.get("sigma_xy_m").is_none());
    assert!(v.get("imu_feed").is_none());
}

#[test]
fn live_and_product_payloads_are_told_apart_by_kind() {
    let live = LiveOccupancy {
        kind: MapPayloadKind::Live,
        seq: 12,
        frame: NavFrame::MapNed,
        res_m: 0.05,
        tile_cells: nav::OCCUPANCY_TILE_CELLS,
        tiles: vec![OccupancyTile {
            ix: 0,
            iy: -1,
            png_b64: "AA==".into(),
        }],
    };
    let v = serde_json::to_value(&live).unwrap();
    assert_eq!(v["kind"], "live");
    assert_eq!(v["tiles"][0]["iy"], -1);
    assert_eq!(
        LiveOccupancy::from_msgpack(&live.to_msgpack().unwrap()).unwrap(),
        live
    );

    let cloud = LivePointCloud {
        kind: MapPayloadKind::Live,
        seq: 3,
        frame: NavFrame::MapNed,
        voxel_m: 0.05,
        count: 0,
        points_b64: String::new(),
    };
    assert_eq!(serde_json::to_value(&cloud).unwrap()["kind"], "live");
    // A product descriptor that predates the field reads as a product.
    assert_eq!(MapPayloadKind::default(), MapPayloadKind::Product);
}

#[test]
fn scan_and_map_status_use_the_documented_keys() {
    let scan = ScanDescriptor {
        t_ns: 1,
        frame: NavFrame::BodyFrd,
        angle_start_rad: 0.0,
        angle_increment_rad: std::f32::consts::PI / 180.0,
        ranges_m: vec![0.0; nav::SCAN_BINS],
    };
    let v = serde_json::to_value(&scan).unwrap();
    assert_eq!(v["frame"], "body_frd");
    assert_eq!(v["ranges_m"].as_array().unwrap().len(), 360);

    let st = MapStatus {
        placement: ComputePlacement::PostFlight,
        ..MapStatus::default()
    };
    let v = serde_json::to_value(&st).unwrap();
    assert_eq!(v["placement"], "post_flight");
    assert_eq!(v["nav_pose"], "always");
    assert!(v["session_id"].is_null());
    assert_eq!(v["upload"]["state"], "idle");
    assert_eq!(v["stream"]["state"], "off");
    assert!(
        v["memory_used_mb"].is_null(),
        "unread resident memory is null, never 0"
    );
    for key in [
        "memory_used_mb",
        "memory_budget_mb",
        "voxel_m",
        "keyframes_dropped",
        "sweep_points",
        "area_m2",
    ] {
        assert!(v.get(key).is_some(), "{key}");
    }
}

/// `imu_feed` is null without an FC IMU, absent on an older device, and
/// otherwise `{state, reason, rate_hz}` in snake case: lost on the
/// attitude stand-in or a stalled feed, degraded on a starved one.
#[test]
fn map_status_carries_the_fc_imu_feed() {
    let v = serde_json::to_value(MapStatus::default()).unwrap();
    assert!(v["imu_feed"].is_null());
    let st = MapStatus {
        imu_feed: Some(ImuFeedStatus {
            state: ImuFeedState::Lost,
            reason: Some(ImuFeedReason::AttitudeStandIn),
            rate_hz: None,
        }),
        ..MapStatus::default()
    };
    let v = serde_json::to_value(&st).unwrap();
    assert_eq!(
        v["imu_feed"],
        serde_json::json!({"state": "lost", "reason": "attitude_stand_in", "rate_hz": null})
    );
    assert_eq!(ImuFeedReason::AttitudeStandIn.state(), ImuFeedState::Lost);
    assert_eq!(ImuFeedReason::Stalled.state(), ImuFeedState::Lost);
    assert_eq!(ImuFeedReason::Starved.state(), ImuFeedState::Degraded);
    let mut old = serde_json::to_value(MapStatus::default()).unwrap();
    old.as_object_mut().unwrap().remove("imu_feed");
    let back: MapStatus = serde_json::from_value(old).unwrap();
    assert_eq!(back.imu_feed, None);
}

#[test]
fn the_lidar_status_carries_exactly_the_documented_keys() {
    let keys = |v: &serde_json::Value| {
        let mut k: Vec<String> = v.as_object().unwrap().keys().cloned().collect();
        k.sort();
        k
    };
    let want = [
        "connected",
        "device",
        "dropped_scans",
        "health",
        "health_code",
        "rate_hz",
        "reconnects",
        "scans",
        "v",
    ];

    // Not running: nulls and zero counters, never invented values.
    let idle = LidarStatus::default();
    let v = serde_json::to_value(&idle).unwrap();
    assert_eq!(keys(&v), want);
    assert_eq!(v["v"], 1);
    assert_eq!(v["connected"], false);
    for key in ["device", "health", "health_code", "rate_hz"] {
        assert!(v[key].is_null(), "{key}");
    }
    for key in ["scans", "dropped_scans", "reconnects"] {
        assert_eq!(v[key], 0, "{key}");
    }

    let live = LidarStatus {
        device: Some("/dev/serial/by-id/usb-Silicon_Labs_CP2102N-if00-port0".into()),
        connected: true,
        health: Some(LidarHealth::Warning),
        health_code: Some(0x0102),
        rate_hz: Some(10.0),
        scans: 120,
        dropped_scans: 2,
        reconnects: 1,
        ..LidarStatus::default()
    };
    let v = serde_json::to_value(&live).unwrap();
    assert_eq!(keys(&v), want);
    assert_eq!(v["health"], "warning");
    assert_eq!(v["health_code"], 0x0102);
    assert_eq!(v["rate_hz"], 10.0);
    for (h, word) in [
        (LidarHealth::Good, "good"),
        (LidarHealth::Warning, "warning"),
        (LidarHealth::Error, "error"),
    ] {
        assert_eq!(serde_json::to_value(h).unwrap(), word);
    }

    // The bus carries it as a named msgpack map inside the envelope.
    let ev = AtlasEvent::new(PLUGIN_ATLAS_LIDAR_TOPIC, None, live.to_msgpack().unwrap());
    let back = AtlasEvent::decode(&ev.encode().unwrap()).unwrap();
    assert_eq!(back.topic, "plugin.world-engine.lidar");
    assert_eq!(LidarStatus::from_msgpack(&back.payload).unwrap(), live);
    let wire: serde_json::Value = rmp_serde::from_slice(&back.payload).unwrap();
    assert_eq!(keys(&wire), want);
    assert_eq!(wire["health"], "warning");
}

#[test]
fn each_placement_states_its_live_voxel_and_upload() {
    use ComputePlacement::*;
    assert_eq!(Onboard.live_voxel_m(), 0.05);
    assert_eq!(Hybrid.live_voxel_m(), 0.10);
    assert_eq!(Stream.live_voxel_m(), 0.20);
    assert_eq!(PostFlight.live_voxel_m(), 0.20);
    assert!(Hybrid.uploads_after_flight() && PostFlight.uploads_after_flight());
    assert!(!Onboard.uploads_after_flight() && !Stream.uploads_after_flight());
    assert!(Stream.streams_live() && !Hybrid.streams_live());
    assert_eq!(ComputePlacement::default(), Hybrid);
}
