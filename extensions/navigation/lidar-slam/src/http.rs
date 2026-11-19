//! `navigation-lidar-slam`'s routes on the plugin's `http.sock` (the control
//! plane forwards `/api/plugins/com.altnautica.navigation/x/<rest>` here as
//! `/<rest>`, already authenticated as the operator):
//!
//! - `GET /navigation/status`: the provider, its health and pose age, the
//!   LiDAR link and the live map's memory and placement ([`StatusView`]);
//! - `GET /navigation/sweep?since=<seq>`: the live sweep cloud
//!   ([`crate::sweep`]);
//! - `PUT /navigation/config`: patch the `navigation.*` keys. The patch is
//!   resolved with the stored keys before anything is written, so an invalid
//!   value is refused whole; the lane applies it on its next config poll.

use std::collections::BTreeMap;
use std::sync::Arc;

use async_trait::async_trait;
use axum::extract::{Query, State};
use axum::http::{header, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::routing::{get, put};
use axum::{Json, Router};
use serde::Deserialize;
use serde_json::{json, Value};

use crate::settings::{NavConfig, CONFIG_KEYS};
use crate::status::{Shared, StatusView};
use crate::sweep::{encode, SweepReply};

/// The plugin config the routes read and write.
#[async_trait]
pub trait ConfigStore: Send + Sync + 'static {
    async fn get(&self, key: &str) -> Result<Value, String>;
    async fn set(&self, key: &str, value: Value) -> Result<(), String>;
}

/// The host's plugin config, drone scope.
pub struct HostConfig(pub ados_sdk::PluginContext);

#[async_trait]
impl ConfigStore for HostConfig {
    async fn get(&self, key: &str) -> Result<Value, String> {
        let raw = self
            .0
            .config
            .get(key, rmpv::Value::Nil)
            .await
            .map_err(|e| e.to_string())?;
        rmpv::ext::from_value(raw).map_err(|e| e.to_string())
    }
    async fn set(&self, key: &str, value: Value) -> Result<(), String> {
        let v = rmpv::ext::to_value(&value).map_err(|e| e.to_string())?;
        self.0
            .config
            .set(key, v, "drone")
            .await
            .map(|_| ())
            .map_err(|e| e.to_string())
    }
}

#[derive(Clone)]
struct Api {
    shared: Arc<Shared>,
    config: Arc<dyn ConfigStore>,
}

pub fn router(shared: Arc<Shared>, config: Arc<dyn ConfigStore>) -> Router {
    Router::new()
        .route("/navigation/status", get(status))
        .route("/navigation/sweep", get(sweep))
        .route("/navigation/config", put(put_config))
        .with_state(Api { shared, config })
}

fn detail(code: StatusCode, msg: impl Into<String>) -> Response {
    (code, Json(json!({ "detail": msg.into() }))).into_response()
}

async fn status(State(api): State<Api>) -> Json<StatusView> {
    Json(api.shared.view.lock().clone())
}

#[derive(Deserialize)]
struct SweepQuery {
    since: Option<u64>,
}

fn octets(body: Vec<u8>) -> Response {
    ([(header::CONTENT_TYPE, "application/octet-stream")], body).into_response()
}

async fn sweep(State(api): State<Api>, Query(q): Query<SweepQuery>) -> Response {
    let reply = api.shared.sweep.lock().reply(q.since);
    if let SweepReply::Delta {
        seq,
        voxel_m,
        points,
    } = reply
    {
        return octets(encode(seq, voxel_m, false, &points));
    }
    let Some(tx) = api.shared.whole.lock().clone() else {
        return detail(
            StatusCode::SERVICE_UNAVAILABLE,
            "no navigation run: no live map",
        );
    };
    let (reply_tx, reply_rx) = tokio::sync::oneshot::channel();
    if tx.send(reply_tx).await.is_err() {
        return detail(
            StatusCode::SERVICE_UNAVAILABLE,
            "the navigation run stopped",
        );
    }
    match reply_rx.await {
        Ok((points, voxel_m, seq)) => octets(encode(seq, voxel_m, true, &points)),
        Err(_) => detail(
            StatusCode::SERVICE_UNAVAILABLE,
            "the navigation run stopped",
        ),
    }
}

/// The writes a patch makes, after resolving it with the stored keys; the
/// refusal says which key and why.
pub async fn validate_patch(
    store: &dyn ConfigStore,
    patch: &BTreeMap<String, Value>,
) -> Result<Vec<(&'static str, Value)>, String> {
    let mut writes = Vec::new();
    for (key, value) in patch {
        let Some(&known) = CONFIG_KEYS.iter().find(|k| *k == key) else {
            return Err(if key.starts_with("navigation.") {
                format!("`{key}` is not a Navigation setting")
            } else {
                format!("`{key}` is not a Navigation key; set it on its own extension")
            });
        };
        writes.push((known, value.clone()));
    }
    let mut merged = BTreeMap::new();
    for &key in CONFIG_KEYS {
        merged.insert(key.to_string(), store.get(key).await?);
    }
    for (key, value) in &writes {
        merged.insert(key.to_string(), value.clone());
    }
    NavConfig::from_values(&merged).map_err(|e| e.to_string())?;
    Ok(writes)
}

async fn put_config(
    State(api): State<Api>,
    Json(patch): Json<BTreeMap<String, Value>>,
) -> Response {
    let writes = match validate_patch(api.config.as_ref(), &patch).await {
        Ok(w) => w,
        Err(msg) => return detail(StatusCode::BAD_REQUEST, msg),
    };
    for (key, value) in writes {
        if let Err(e) = api.config.set(key, value).await {
            return detail(
                StatusCode::INTERNAL_SERVER_ERROR,
                format!("could not write {key}: {e}"),
            );
        }
    }
    Json(json!({
        "status": "ok",
        "persisted": true,
        "restart": {"status": "ok", "message": "applied on the navigation service's next config poll"},
    }))
    .into_response()
}

#[cfg(test)]
#[path = "http_tests.rs"]
mod tests;
