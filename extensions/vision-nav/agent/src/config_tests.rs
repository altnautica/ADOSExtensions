//! Tests for the config model in `config.rs`.

use super::*;

fn map(pairs: &[(&str, Value)]) -> Value {
    Value::Map(
        pairs
            .iter()
            .map(|(k, v)| (Value::from(*k), v.clone()))
            .collect(),
    )
}

fn json_to_rmpv(v: &serde_json::Value) -> Value {
    match v {
        serde_json::Value::Null => Value::Nil,
        serde_json::Value::Bool(b) => Value::Boolean(*b),
        serde_json::Value::Number(n) => match n.as_i64() {
            Some(i) => Value::from(i),
            None => Value::F64(n.as_f64().unwrap_or(0.0)),
        },
        serde_json::Value::String(s) => Value::from(s.as_str()),
        serde_json::Value::Array(a) => Value::Array(a.iter().map(json_to_rmpv).collect()),
        serde_json::Value::Object(o) => Value::Map(
            o.iter()
                .map(|(k, v)| (Value::from(k.as_str()), json_to_rmpv(v)))
                .collect(),
        ),
    }
}

/// Collect the `default` of every property of a schema object node,
/// following a local `$ref` and recursing into nested objects.
fn schema_defaults(
    node: &serde_json::Value,
    root: &serde_json::Value,
) -> Option<serde_json::Value> {
    let node = match node.get("$ref").and_then(|r| r.as_str()) {
        Some(r) => {
            let name = r.trim_start_matches("#/definitions/");
            &root["definitions"][name]
        }
        None => node,
    };
    if let Some(d) = node.get("default") {
        return Some(d.clone());
    }
    let props = node.get("properties")?.as_object()?;
    let mut out = serde_json::Map::new();
    for (key, prop) in props {
        if let Some(d) = schema_defaults(prop, root) {
            out.insert(key.clone(), d);
        }
    }
    Some(serde_json::Value::Object(out))
}

#[test]
fn empty_config_is_defaults() {
    let cfg = VisionNavConfig::from_value(&Value::Map(vec![])).unwrap();
    assert_eq!(cfg.mode, Mode::Off);
    assert_eq!(cfg.effective_mode(), Mode::Off);
    assert_eq!(cfg.flow_quality_min, 50);
    assert_eq!(cfg.firmware.firmware, Firmware::Ardupilot);
    assert_eq!(cfg.rangefinder.topology, Topology::Fc);
    assert_eq!(cfg.rangefinder.driver, "fc_relay");
    assert!(!cfg.outdoor);
}

#[test]
fn schema_defaults_equal_rust_default() {
    let schema: serde_json::Value =
        serde_json::from_str(include_str!("../../config-schema.json")).unwrap();
    let defaults = schema_defaults(&schema, &schema).unwrap();
    let Value::Map(entries) = json_to_rmpv(&defaults) else {
        panic!("schema defaults are not an object");
    };
    let map: BTreeMap<String, Value> = entries
        .into_iter()
        .filter_map(|(k, v)| k.as_str().map(|s| (s.to_string(), v)))
        .collect();
    // Every top-level property carries a default.
    for key in [
        "mode",
        "active",
        "flow_quality_min",
        "outdoor",
        "camera",
        "rangefinder",
        "firmware",
    ] {
        assert!(map.contains_key(key), "schema default missing for {key}");
    }
    let parsed = VisionNavConfig::from_map(&map).unwrap();
    assert_eq!(parsed, VisionNavConfig::default());
}

#[test]
fn flow_quality_min_is_clamped() {
    let high = map(&[("flow_quality_min", Value::from(900i64))]);
    assert_eq!(
        VisionNavConfig::from_value(&high).unwrap().flow_quality_min,
        255
    );
    let low = map(&[("flow_quality_min", Value::from(-4i64))]);
    assert_eq!(
        VisionNavConfig::from_value(&low).unwrap().flow_quality_min,
        0
    );
}

#[test]
fn outdoor_parses() {
    let v = map(&[("outdoor", Value::from(true))]);
    assert!(VisionNavConfig::from_value(&v).unwrap().outdoor);
}

#[test]
fn mode_this_build_does_not_offer_is_rejected() {
    let v = map(&[("mode", Value::from("estimator_from_an_older_build"))]);
    let err = VisionNavConfig::from_value(&v).unwrap_err();
    assert!(err.0.contains("unknown mode"), "{}", err.0);
}

#[test]
fn optical_flow_rejects_forward_camera() {
    let v = map(&[
        ("mode", Value::from("optical_flow")),
        ("camera", map(&[("orientation", Value::from("forward"))])),
    ]);
    assert!(VisionNavConfig::from_value(&v).is_err());
}

#[test]
fn disengaged_config_emits_nothing() {
    let cfg = VisionNavConfig::disengaged();
    assert_eq!(cfg.effective_mode(), Mode::Off);
}

#[test]
fn unknown_keys_ignored() {
    // A stored config from an older build may carry keys this build
    // dropped (pre_arm, firmware.ekf_source_set_index).
    let v = map(&[
        ("mode", Value::from("optical_flow")),
        ("pre_arm", map(&[("auto_set_origin", Value::from(true))])),
        (
            "firmware",
            map(&[
                ("type", Value::from("px4")),
                ("ekf_source_set_index", Value::from(2i64)),
            ]),
        ),
    ]);
    let cfg = VisionNavConfig::from_value(&v).unwrap();
    assert_eq!(cfg.mode, Mode::OpticalFlow);
    assert_eq!(cfg.firmware.firmware, Firmware::Px4);
}

#[test]
fn disengaged_effective_mode_is_off() {
    let v = map(&[
        ("mode", Value::from("optical_flow")),
        ("active", Value::from(false)),
    ]);
    let cfg = VisionNavConfig::from_value(&v).unwrap();
    assert!(!cfg.active);
    assert_eq!(cfg.mode, Mode::OpticalFlow);
    assert_eq!(cfg.effective_mode(), Mode::Off);
}
