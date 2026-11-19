use super::*;
use crate::pose::vio::pairing::VioScale;
use serde_json::json;

fn values(pairs: &[(&str, Value)]) -> BTreeMap<String, Value> {
    pairs
        .iter()
        .map(|(k, v)| (k.to_string(), v.clone()))
        .collect()
}

#[test]
fn vio_scale_is_one_or_a_half() {
    let load = |v: Value| NavConfig::from_values(&values(&[(KEY_VIO, v)]));
    assert_eq!(load(json!({})).unwrap().vio.scale, VioScale::Full);
    assert_eq!(load(json!({"scale": 1})).unwrap().vio.scale, VioScale::Full);
    assert_eq!(
        load(json!({"scale": 0.5})).unwrap().vio.scale,
        VioScale::Half
    );
    assert!(load(json!({"scale": 0.25})).is_err());
}

#[test]
fn takeoff_heading_is_an_optional_angle() {
    let load = |v: Value| NavConfig::from_values(&values(&[(KEY_TAKEOFF_HEADING_DEG, v)]));
    assert_eq!(load(json!(180)).unwrap().takeoff_heading_deg, Some(180.0));
    assert_eq!(load(json!(-12.5)).unwrap().takeoff_heading_deg, Some(-12.5));
    assert_eq!(load(Value::Null).unwrap().takeoff_heading_deg, None);
    assert_eq!(
        NavConfig::from_values(&BTreeMap::new())
            .unwrap()
            .takeoff_heading_deg,
        None
    );
    assert!(load(json!(400)).is_err());
    assert!(load(json!("north")).is_err());
    assert!(CONFIG_KEYS.contains(&KEY_TAKEOFF_HEADING_DEG));
}

#[test]
fn auto_picks_lidar_anywhere_then_vio_on_a_visor_and_none_elsewhere() {
    let r = |c, p, s, l| resolve_pose_provider(c, p, s, l);
    let auto = PoseProviderConfig::Auto;
    assert_eq!(r(auto, "visor", true, false), Some(PoseSource::Vio));
    assert_eq!(r(auto, "visor", false, true), Some(PoseSource::Lidar));
    // A LiDAR and IMU outrank the stereo pair.
    assert_eq!(r(auto, "visor", true, true), Some(PoseSource::Lidar));
    assert_eq!(r(auto, "visor", false, false), None);
    // A drone with a planar LiDAR and IMU navigates on the LiDAR pose.
    assert_eq!(r(auto, "drone", true, true), Some(PoseSource::Lidar));
    assert_eq!(
        r(auto, "drone", true, false),
        None,
        "VIO on a drone only when asked"
    );
    assert_eq!(
        r(PoseProviderConfig::Lidar, "drone", false, false),
        Some(PoseSource::Lidar)
    );
    assert_eq!(r(PoseProviderConfig::Off, "visor", true, true), None);
    assert_eq!(
        r(PoseProviderConfig::Vio, "visor", false, true),
        Some(PoseSource::Vio)
    );
    let e = NavConfig::from_values(&values(&[(KEY_POSE_PROVIDER, json!("fc_ekf"))])).unwrap_err();
    assert_eq!(
        e.key, KEY_POSE_PROVIDER,
        "the FC EKF is Capture's pose, not Navigation's"
    );
}

#[test]
fn placement_defaults_to_hybrid_and_names_a_bad_value() {
    let d = NavConfig::from_values(&BTreeMap::new()).unwrap();
    assert_eq!(d.map_placement, ComputePlacement::Hybrid);
    assert_eq!(
        (d.memory_budget_mb, d.pose_publish_interval_ms),
        (None, 100)
    );
    let set = values(&[
        (KEY_MAP_PLACEMENT, json!("post_flight")),
        (KEY_MEMORY_BUDGET_MB, json!(600)),
    ]);
    let c = NavConfig::from_values(&set).unwrap();
    assert_eq!(c.map_placement, ComputePlacement::PostFlight);
    assert_eq!(c.memory_budget_mb, Some(600));
    let bad = values(&[(KEY_MAP_PLACEMENT, json!("cloud"))]);
    assert_eq!(
        NavConfig::from_values(&bad).unwrap_err().key,
        KEY_MAP_PLACEMENT
    );
}

#[test]
fn lidar_settings_take_per_driver_blocks() {
    let load = |v: Value| NavConfig::from_values(&values(&[(KEY_LIDAR, v)]));
    let d = load(json!({})).unwrap().lidar;
    assert_eq!((d.lidar_id.as_str(), d.imu.as_str()), ("lidar0", "imu0"));
    assert_eq!(
        (d.rplidar.device.as_str(), d.rplidar.baud),
        ("auto", 460_800)
    );
    assert_eq!(d.livox.host_ip, None);
    assert!(d
        .livox
        .sdk_library
        .ends_with("liblivox_lidar_sdk_shared.so"));
    let c = load(json!({
        "lidar_id": "lidar1",
        "rplidar": {"device": "tcp:192.168.1.50:7700"},
        "livox": {"host_ip": "192.168.1.50", "lidar_ips": ["192.168.1.12"]},
    }))
    .unwrap()
    .lidar;
    assert_eq!(c.lidar_id, "lidar1");
    assert_eq!(c.rplidar.device, "tcp:192.168.1.50:7700");
    assert_eq!(c.rplidar.baud, 460_800);
    assert_eq!(c.livox.host_ip.as_deref(), Some("192.168.1.50"));
    assert_eq!(c.livox.lidar_ips, ["192.168.1.12"]);
    // The old flat Livox keys and unknown driver keys are refused, naming the key.
    assert_eq!(load(json!({"host_ip": "x"})).unwrap_err().key, KEY_LIDAR);
    assert_eq!(
        load(json!({"rplidar": {"port": "x"}})).unwrap_err().key,
        KEY_LIDAR
    );
}
