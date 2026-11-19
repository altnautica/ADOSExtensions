//! Reconstruct's routes on a drone, on the plugin's `http.sock` (the control
//! plane forwards `/api/plugins/com.altnautica.reconstruct/x/<rest>` here as
//! `/<rest>`, already authenticated):
//!
//! - `GET /reconstruct/status`: the offload setting and the live scan
//!   stream (whether Navigation streams the map, the node, forwarded,
//!   dropped and failed scans).
//! - `PUT /reconstruct/config`: patch the `reconstruct.offload.*` keys.
//!   Nothing restarts: the reconciler reads them on its next tick.
//! - `POST /reconstruct/node-credential`: install a node credential (the
//!   offload-stream, capture-ingest and job-submit lanes this process uses).

use std::collections::BTreeMap;
use std::sync::Arc;

use ados_mapping_transport::credentials::{self, CredentialStore};
use ados_mapping_transport::host::{config_json, Host};
use ados_mapping_transport::reply::{respond, ApiError};
use axum::extract::State;
use axum::routing::{get, put};
use axum::{Json, Router};
use serde_json::{json, Value};

use crate::stream::SharedStream;
use crate::{CAMERA_KEY, ENABLED_KEY, PIN_KEY};

#[derive(Clone)]
pub struct DroneApi {
    pub host: Arc<dyn Host>,
    pub credentials: CredentialStore,
    pub stream: SharedStream,
}

pub fn router(api: DroneApi) -> Router {
    let credentials = api.credentials.clone();
    Router::new()
        .route("/reconstruct/status", get(status))
        .route("/reconstruct/config", put(put_config))
        .with_state(api)
        .merge(credentials::router(
            "/reconstruct/node-credential",
            credentials,
        ))
}

async fn status(State(api): State<DroneApi>) -> Json<Value> {
    let stream = api.stream.lock().clone();
    Json(json!({
        "offload": {
            "enabled": config_json(api.host.as_ref(), ENABLED_KEY).await,
            "compute_node_addr": config_json(api.host.as_ref(), PIN_KEY).await,
            "camera": config_json(api.host.as_ref(), CAMERA_KEY).await,
        },
        "stream": stream,
    }))
}

/// A string, or null to clear the key.
fn is_text(v: &Value) -> bool {
    matches!(v, Value::String(_) | Value::Null)
}

/// The writes a patch makes; the refusal names the key and why.
pub fn config_writes(
    patch: &BTreeMap<String, Value>,
) -> Result<Vec<(&'static str, Value)>, String> {
    let mut writes = Vec::new();
    for (key, value) in patch {
        match key.as_str() {
            ENABLED_KEY => match value.as_str() {
                Some("auto" | "on" | "off") => writes.push((ENABLED_KEY, value.clone())),
                _ => return Err(format!("`{ENABLED_KEY}` is auto, on or off, got {value}")),
            },
            PIN_KEY if is_text(value) => writes.push((PIN_KEY, value.clone())),
            CAMERA_KEY if is_text(value) => writes.push((CAMERA_KEY, value.clone())),
            PIN_KEY | CAMERA_KEY => return Err(format!("`{key}` is a string, got {value}")),
            other if other.starts_with("reconstruct.") => {
                return Err(format!("`{other}` is not a Reconstruct setting on a drone"))
            }
            other => {
                return Err(format!(
                    "`{other}` is not a Reconstruct key; set it on its own extension"
                ))
            }
        }
    }
    Ok(writes)
}

async fn put_config(
    State(api): State<DroneApi>,
    Json(patch): Json<BTreeMap<String, Value>>,
) -> axum::response::Response {
    let writes = match config_writes(&patch) {
        Ok(w) => w,
        Err(msg) => return respond(Err(ApiError::bad_request(msg))),
    };
    for (key, value) in writes {
        let v = match rmpv::ext::to_value(&value) {
            Ok(v) => v,
            Err(e) => return respond(Err(ApiError::io(e))),
        };
        if let Err(e) = api.host.config_set(key, v).await {
            return respond(Err(ApiError::io(format!("could not write {key}: {e}"))));
        }
    }
    respond(Ok(json!({
        "status": "ok",
        "persisted": true,
        "restart": {"status": "ok", "message": "applied on the reconciler's next tick"},
    })))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn patch(pairs: &[(&str, Value)]) -> BTreeMap<String, Value> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), v.clone()))
            .collect()
    }

    #[test]
    fn only_reconstruct_keys_with_valid_values_are_written() {
        let w = config_writes(&patch(&[
            (ENABLED_KEY, json!("on")),
            (CAMERA_KEY, json!("nose")),
        ]))
        .unwrap();
        assert_eq!(w.len(), 2);
        assert!(config_writes(&patch(&[(ENABLED_KEY, json!("sometimes"))]))
            .unwrap_err()
            .contains(ENABLED_KEY));
        assert!(config_writes(&patch(&[("capture.enabled", json!(true))]))
            .unwrap_err()
            .contains("own extension"));
        assert!(
            config_writes(&patch(&[("reconstruct.serving.enabled", json!(true))]))
                .unwrap_err()
                .contains("not a Reconstruct setting on a drone")
        );
    }
}
