//! Resolving the plugin config: the host stores it as flat dotted keys
//! (`navigation.pose_provider`, `navigation.lidar`, ...). A service reads its
//! keys into a map ([`read_values`]) and resolves the map with these helpers
//! (absent or null keys take their defaults), so the resolution is pure and
//! testable without a host.

use std::collections::BTreeMap;

use ados_sdk::{ClientError, PluginContext};
use serde::de::DeserializeOwned;
use serde_json::Value;

/// A key whose value does not resolve.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
#[error("invalid `{key}`: {reason}")]
pub struct ConfigError {
    pub key: &'static str,
    pub reason: String,
}

/// The value of `key`, absent when unset or null.
pub fn present<'a>(values: &'a BTreeMap<String, Value>, key: &str) -> Option<&'a Value> {
    values.get(key).filter(|v| !v.is_null())
}

/// `key` deserialized as `T`, `default` when absent.
pub fn typed<T: DeserializeOwned>(
    values: &BTreeMap<String, Value>,
    key: &'static str,
    default: T,
) -> Result<T, ConfigError> {
    match present(values, key) {
        None => Ok(default),
        Some(v) => serde_json::from_value(v.clone()).map_err(|e| ConfigError {
            key,
            reason: e.to_string(),
        }),
    }
}

/// `key` as an integer (an integral float is accepted: a JSON editor may
/// write `100.0`), `default` when absent.
pub fn integer(
    values: &BTreeMap<String, Value>,
    key: &'static str,
    default: i64,
) -> Result<i64, ConfigError> {
    let Some(v) = present(values, key) else {
        return Ok(default);
    };
    v.as_i64()
        .or_else(|| {
            v.as_f64()
                .filter(|f| f.fract() == 0.0 && f.abs() < i64::MAX as f64)
                .map(|f| f as i64)
        })
        .ok_or_else(|| ConfigError {
            key,
            reason: format!("expected an integer, got {v}"),
        })
}

/// `key` as a non-negative integer, `default` when absent.
pub fn unsigned(
    values: &BTreeMap<String, Value>,
    key: &'static str,
    default: u64,
) -> Result<u64, ConfigError> {
    let n = integer(values, key, default as i64)?;
    u64::try_from(n).map_err(|_| ConfigError {
        key,
        reason: format!("expected a non-negative integer, got {n}"),
    })
}

/// `key` as a non-negative integer, `None` when absent.
pub fn optional_unsigned(
    values: &BTreeMap<String, Value>,
    key: &'static str,
) -> Result<Option<u64>, ConfigError> {
    match present(values, key) {
        None => Ok(None),
        Some(_) => unsigned(values, key, 0).map(Some),
    }
}

/// `key` as a heading in degrees (-360 to 360), `None` when absent.
pub fn heading_deg(
    values: &BTreeMap<String, Value>,
    key: &'static str,
) -> Result<Option<f64>, ConfigError> {
    let Some(deg) = typed::<Option<f64>>(values, key, None)? else {
        return Ok(None);
    };
    if !deg.is_finite() || deg.abs() > 360.0 {
        return Err(ConfigError {
            key,
            reason: format!("expected degrees from -360 to 360, got {deg}"),
        });
    }
    Ok(Some(deg))
}

/// Read `keys` from the host. The outer error is the host's; the inner one a
/// stored value that is not JSON.
pub async fn read_values(
    ctx: &PluginContext,
    keys: &[&'static str],
) -> Result<Result<BTreeMap<String, Value>, ConfigError>, ClientError> {
    let mut values = BTreeMap::new();
    for &key in keys {
        let raw = ctx.config.get(key, rmpv::Value::Nil).await?;
        match rmpv::ext::from_value::<Value>(raw) {
            Ok(v) => {
                values.insert(key.to_string(), v);
            }
            Err(e) => {
                return Ok(Err(ConfigError {
                    key,
                    reason: e.to_string(),
                }))
            }
        }
    }
    Ok(Ok(values))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn values(pairs: &[(&str, Value)]) -> BTreeMap<String, Value> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), v.clone()))
            .collect()
    }

    #[test]
    fn absent_and_null_take_the_default_and_bad_values_name_their_key() {
        let v = values(&[
            ("a.n", json!(100.0)),
            ("a.null", Value::Null),
            ("a.neg", json!(-1)),
        ]);
        assert_eq!(integer(&v, "a.n", 5), Ok(100));
        assert_eq!(integer(&v, "a.null", 5), Ok(5));
        assert_eq!(optional_unsigned(&v, "a.missing"), Ok(None));
        let e = unsigned(&v, "a.neg", 0).unwrap_err();
        assert_eq!(e.key, "a.neg");
        let e = heading_deg(&values(&[("h", json!(400.0))]), "h").unwrap_err();
        assert!(e.reason.contains("400"), "{e}");
        assert_eq!(
            heading_deg(&values(&[("h", json!(-90))]), "h"),
            Ok(Some(-90.0))
        );
    }
}
