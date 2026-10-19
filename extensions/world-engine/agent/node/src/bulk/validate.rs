//! Commit-time document checks: `manifest.json` and `rig.json` against the
//! bundled contract schemas, plus id rules for every id that later becomes a
//! file or directory name.

use std::sync::LazyLock;

use ados_capture_contract::{SCHEMA_BASE_URI, SCHEMA_FILES};
use jsonschema::Validator;
use serde_json::{json, Value};

/// Longest accepted id.
const MAX_SAFE_ID_LEN: usize = 64;
/// Most schema errors quoted in one report.
const MAX_REPORTED: usize = 8;

/// An id that is safe as a single path segment and as a tool argument:
/// 1 to 64 bytes of `[a-z0-9_-]`, starting with `[a-z0-9]`. Lowercase only, so
/// two ids never name the same folder on a case-insensitive filesystem; no
/// leading `-`, so an id never reads as a command-line flag.
pub fn is_safe_id(id: &str) -> bool {
    let bytes = id.as_bytes();
    let Some(first) = bytes.first() else {
        return false;
    };
    bytes.len() <= MAX_SAFE_ID_LEN
        && (first.is_ascii_lowercase() || first.is_ascii_digit())
        && bytes
            .iter()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || matches!(b, b'_' | b'-'))
}

struct Validators {
    manifest: Validator,
    rig: Validator,
}

fn build_validators() -> Result<Validators, String> {
    let mut registry = jsonschema::Registry::new();
    for (name, body) in SCHEMA_FILES {
        let schema: Value =
            serde_json::from_str(body).map_err(|e| format!("bundled schema {name}: {e}"))?;
        registry = registry
            .add(format!("{SCHEMA_BASE_URI}{name}"), schema)
            .map_err(|e| format!("bundled schema {name}: {e}"))?;
    }
    let registry = registry
        .prepare()
        .map_err(|e| format!("bundled schemas: {e}"))?;
    let build = |file: &str| {
        jsonschema::options()
            .with_registry(&registry)
            .build(&json!({ "$ref": format!("{SCHEMA_BASE_URI}{file}") }))
            .map_err(|e| format!("bundled schema {file}: {e}"))
    };
    Ok(Validators {
        manifest: build("manifest.schema.json")?,
        rig: build("rig.schema.json")?,
    })
}

static VALIDATORS: LazyLock<Result<Validators, String>> = LazyLock::new(build_validators);

fn validators() -> Result<&'static Validators, String> {
    VALIDATORS.as_ref().map_err(Clone::clone)
}

fn check_schema(validator: &Validator, doc: &Value, file: &str) -> Result<(), String> {
    let errors: Vec<String> = validator
        .iter_errors(doc)
        .take(MAX_REPORTED)
        .map(|e| {
            let at = e.instance_path().to_string();
            if at.is_empty() {
                e.to_string()
            } else {
                format!("{at}: {e}")
            }
        })
        .collect();
    if errors.is_empty() {
        Ok(())
    } else {
        Err(format!("{file}: {}", errors.join("; ")))
    }
}

fn parse(body: &[u8], file: &str) -> Result<Value, String> {
    serde_json::from_slice(body).map_err(|e| format!("{file}: not JSON: {e}"))
}

fn check_id(doc: &Value, pointer: &str, file: &str) -> Result<(), String> {
    match doc.pointer(pointer).and_then(Value::as_str) {
        Some(id) if is_safe_id(id) => Ok(()),
        Some(id) => Err(format!("{file}: {pointer} {id:?} is not a safe id")),
        None => Err(format!("{file}: {pointer} missing")),
    }
}

/// Check a session's `manifest.json` and `rig.json`: both must validate
/// against the contract schemas, and the manifest `device_id`, the rig
/// `rig_id`, `calib_version` and every `sensors[].id` must pass
/// [`is_safe_id`]. The error names the file and the offending fields.
pub fn validate_session_docs(manifest: &[u8], rig: &[u8]) -> Result<(), String> {
    let validators = validators()?;
    let manifest = parse(manifest, "manifest.json")?;
    let rig = parse(rig, "rig.json")?;
    check_schema(&validators.manifest, &manifest, "manifest.json")?;
    check_schema(&validators.rig, &rig, "rig.json")?;

    check_id(&manifest, "/device_id", "manifest.json")?;
    check_id(&rig, "/rig_id", "rig.json")?;
    check_id(&rig, "/calib_version", "rig.json")?;
    let sensors = rig
        .get("sensors")
        .and_then(Value::as_array)
        .ok_or_else(|| "rig.json: /sensors missing".to_string())?;
    for i in 0..sensors.len() {
        check_id(&rig, &format!("/sensors/{i}/id"), "rig.json")?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn safe_ids() {
        for ok in ["cam0", "imu_0", "tiny-rig", "1", "sim-1"] {
            assert!(is_safe_id(ok), "{ok}");
        }
        let long = "a".repeat(MAX_SAFE_ID_LEN + 1);
        for bad in [
            "",
            "../x",
            "a/b",
            "a\\b",
            "cam\nzero",
            "Cam0",
            "-x",
            "_x",
            ".x",
            "a.b",
            "a b",
            &long,
        ] {
            assert!(!is_safe_id(bad), "{bad:?}");
        }
    }

    #[test]
    fn bundled_schemas_compile() {
        validators().unwrap();
    }
}
