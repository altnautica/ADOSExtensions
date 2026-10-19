//! Every golden JSON fixture validates against its schema (full constraints,
//! via the `jsonschema` crate) and round-trips through the generated type
//! unchanged; broken variants are refused by the schema.

use std::path::PathBuf;

use ados_capture_contract::{
    ArtifactDescriptor, Manifest, Placement, Rig, TransformsJson, SCHEMA_BASE_URI, SCHEMA_FILES,
};
use serde::de::DeserializeOwned;
use serde::Serialize;
use serde_json::{json, Value};

fn contract_dir() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../contract")
}

fn fixture(name: &str) -> Value {
    let path = contract_dir().join("fixtures").join(name);
    serde_json::from_str(&std::fs::read_to_string(&path).unwrap()).unwrap()
}

fn validator(schema_file: &str) -> jsonschema::Validator {
    let mut registry = jsonschema::Registry::new();
    for (name, body) in SCHEMA_FILES {
        let v: Value = serde_json::from_str(body).unwrap();
        assert_eq!(v["$id"], format!("{SCHEMA_BASE_URI}{name}"), "{name} $id");
        registry = registry.add(format!("{SCHEMA_BASE_URI}{name}"), v).unwrap();
    }
    let registry = registry.prepare().unwrap();
    jsonschema::options()
        .with_registry(&registry)
        .build(&json!({ "$ref": format!("{SCHEMA_BASE_URI}{schema_file}") }))
        .unwrap()
}

fn assert_valid(schema_file: &str, doc: &Value) {
    let v = validator(schema_file);
    let errors: Vec<String> = v.iter_errors(doc).map(|e| e.to_string()).collect();
    assert!(errors.is_empty(), "{schema_file}: {errors:#?}");
}

fn assert_invalid(schema_file: &str, doc: &Value, why: &str) {
    assert!(
        !validator(schema_file).is_valid(doc),
        "{schema_file} accepted {why}"
    );
}

fn round_trip<T: DeserializeOwned + Serialize>(doc: &Value) {
    let typed: T = serde_json::from_value(doc.clone()).unwrap();
    assert_eq!(&serde_json::to_value(&typed).unwrap(), doc);
}

#[test]
fn manifest_fixture_validates_and_round_trips() {
    let doc = fixture("manifest.json");
    assert_valid("manifest.schema.json", &doc);
    round_trip::<Manifest>(&doc);

    let mut bad = doc.clone();
    bad["timebase"] = json!("wall");
    assert_invalid("manifest.schema.json", &bad, "an unknown timebase");
    let mut bad = doc.clone();
    bad["session_id"] = json!("visor-1");
    assert_invalid(
        "manifest.schema.json",
        &bad,
        "a session id without time and suffix",
    );
    let mut bad = doc;
    bad.as_object_mut().unwrap().remove("anchor");
    assert_invalid(
        "manifest.schema.json",
        &bad,
        "a missing anchor (must be explicit null)",
    );
}

#[test]
fn rig_fixture_validates_and_round_trips() {
    let doc = fixture("rig.json");
    assert_valid("rig.schema.json", &doc);
    round_trip::<Rig>(&doc);

    let mut bad = doc.clone();
    bad["sensors"][0]["camera"]["dist"] = json!([0.1, 0.2]);
    assert_invalid("rig.schema.json", &bad, "a kb4 camera with 2 coefficients");
    let mut bad = doc.clone();
    bad["sensors"][1]["camera"]["dist"] = json!([0.1, 0.2, 0.0, 0.0]);
    assert_invalid(
        "rig.schema.json",
        &bad,
        "an opencv camera with 4 coefficients",
    );
    let mut bad = doc;
    bad["sensors"][0]["T_body_sensor"] = json!([1.0, 0.0, 0.0]);
    assert_invalid("rig.schema.json", &bad, "a 3-element transform");
}

#[test]
fn placement_fixture_validates_and_round_trips() {
    let doc = fixture("placement.json");
    assert_valid("placement.schema.json", &doc);
    round_trip::<Placement>(&doc);
    let mut bad = doc;
    bad["scale"] = json!(0.0);
    assert_invalid("placement.schema.json", &bad, "zero scale");
}

#[test]
fn artifact_fixture_validates_and_round_trips() {
    let doc = fixture("artifact.json");
    assert_valid("artifact.schema.json", &doc);
    round_trip::<ArtifactDescriptor>(&doc);
    for format in ["ply", "spz", "glb", "copc", "laz", "esdf"] {
        let mut d = doc.clone();
        d["format"] = json!(format);
        assert_valid("artifact.schema.json", &d);
    }
    let mut bad = doc;
    bad["sha256"] = json!("ABC");
    assert_invalid("artifact.schema.json", &bad, "a non-hex digest");
}

#[test]
fn transforms_fixture_validates_and_round_trips() {
    let doc = fixture("transforms.json");
    assert_valid("transforms_ext.schema.json", &doc);
    round_trip::<TransformsJson>(&doc);
    let mut bad = doc;
    bad["frames"][0].as_object_mut().unwrap().remove("t_ns");
    assert_invalid("transforms_ext.schema.json", &bad, "a frame without t_ns");
}

#[test]
fn every_schema_file_is_listed_and_has_a_fixture_test() {
    let mut on_disk: Vec<String> = std::fs::read_dir(contract_dir().join("schema"))
        .unwrap()
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .collect();
    on_disk.sort();
    let listed: Vec<String> = SCHEMA_FILES.iter().map(|(n, _)| n.to_string()).collect();
    assert_eq!(on_disk, listed, "SCHEMA_FILES drifted from contract/schema");
}
