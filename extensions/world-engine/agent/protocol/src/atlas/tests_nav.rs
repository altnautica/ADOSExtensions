//! The live navigation and mapping payloads: topic names, capabilities and
//! the exact wire keys the operator surfaces read.

use super::*;

#[test]
fn the_nav_topics_are_shared_and_gated_by_telemetry_read() {
    assert_eq!(PLUGIN_ATLAS_SCAN_TOPIC, "plugin.world-engine.scan");
    assert_eq!(PLUGIN_ATLAS_MAP_STATUS_TOPIC, "plugin.world-engine.map_status");
    for topic in [PLUGIN_ATLAS_SCAN_TOPIC, PLUGIN_ATLAS_MAP_STATUS_TOPIC] {
        assert!(PLUGIN_ATLAS_TOPICS.contains(&topic));
        assert_eq!(atlas_topic_subscribe_capability(topic), Some(ATLAS_POSE_READ_CAP));
    }
    assert_eq!(atlas_topic_subscribe_capability("plugin.world-engine.scan.x"), None);
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
        frame: NavFrame::MapFrd,
        source: NavPoseSource::LidarPlanar,
    };
    let v = serde_json::to_value(&pd).unwrap();
    assert_eq!(v["health"], "good");
    assert_eq!(v["frame"], "map_frd");
    assert_eq!(v["source"], "lidar_planar");
    assert_eq!(v["pose"]["R"][0], 1.0);
    assert_eq!(PoseDescriptor::from_msgpack(&pd.to_msgpack().unwrap()).unwrap(), pd);
}

#[test]
fn live_and_product_payloads_are_told_apart_by_kind() {
    let live = LiveOccupancy {
        kind: MapPayloadKind::Live,
        seq: 12,
        frame: NavFrame::MapFrd,
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
    assert_eq!(LiveOccupancy::from_msgpack(&live.to_msgpack().unwrap()).unwrap(), live);

    let cloud = LivePointCloud {
        kind: MapPayloadKind::Live,
        seq: 3,
        frame: NavFrame::MapFrd,
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
