//! The slice of the plugin host the link uses, as one object-safe trait.
//!
//! Production implements it over [`PluginContext`] (every call is one SDK
//! method on the plugin socket); tests implement it in memory, so the adapters,
//! the loops and the HTTP routes are exercised without a host.

use std::sync::Arc;

use ados_protocol::framebus::DetectionBatch;
use ados_protocol::node_info::NodeInfo;
use ados_sdk::{ClientError, OffloadAdvertisement, PluginContext};
use rmpv::Value;

/// A callback for one received aux datagram `(channel, payload)`.
pub type AuxCallback = Arc<dyn Fn(u8, Vec<u8>) + Send + Sync>;

/// The config scope every link write lands in (the host falls back to the
/// global scope on a node with no bound drone id).
const CONFIG_SCOPE: &str = "drone";

/// The host methods the link calls.
#[async_trait::async_trait]
pub trait Host: Send + Sync + 'static {
    /// `config.get`: the plugin config value for `key`, or `default`.
    async fn config_get(&self, key: &str, default: Value) -> Result<Value, ClientError>;
    /// `config.set` in the drone scope.
    async fn config_set(&self, key: &str, value: Value) -> Result<(), ClientError>;
    /// `telemetry.extend` (cap `telemetry.extend`).
    async fn telemetry_extend(&self, channel: &str, payload: Value) -> Result<(), ClientError>;
    /// `event.publish` (cap `event.publish`).
    async fn event_publish(&self, topic: &str, payload: Value) -> Result<(), ClientError>;
    /// `cloud.publish` (cap `cloud.publish`); the reply map as the host sent it.
    async fn cloud_publish(&self, stream: &str, payload: &[u8]) -> Result<Value, ClientError>;
    /// `radio.aux_stream.open` (cap `radio.aux_stream`).
    async fn aux_open(&self) -> Result<Value, ClientError>;
    /// `radio.aux_stream.send` (cap `radio.aux_stream`).
    async fn aux_send(&self, channel: u8, payload: &[u8]) -> Result<Value, ClientError>;
    /// `radio.aux_stream.subscribe` plus the delivery callback.
    async fn aux_subscribe(&self, callback: AuxCallback) -> Result<(), ClientError>;
    /// `vision.publish_detection` (cap `vision.detection.publish`).
    async fn publish_detection(&self, batch: &DetectionBatch) -> Result<Value, ClientError>;
    /// `offload.advertise` (cap `vision.detection.publish`).
    async fn advertise_offload(&self, advert: &OffloadAdvertisement) -> Result<Value, ClientError>;
    /// `node.info` (cap `node.info.read`): the board, ground-station role and
    /// camera facts.
    async fn node_info(&self) -> Result<NodeInfo, ClientError>;
}

#[async_trait::async_trait]
impl Host for PluginContext {
    async fn config_get(&self, key: &str, default: Value) -> Result<Value, ClientError> {
        self.config.get(key, default).await
    }

    async fn config_set(&self, key: &str, value: Value) -> Result<(), ClientError> {
        self.config.set(key, value, CONFIG_SCOPE).await.map(|_| ())
    }

    async fn telemetry_extend(&self, channel: &str, payload: Value) -> Result<(), ClientError> {
        self.telemetry.extend(channel, payload).await.map(|_| ())
    }

    async fn event_publish(&self, topic: &str, payload: Value) -> Result<(), ClientError> {
        self.events.publish(topic, payload).await.map(|_| ())
    }

    async fn cloud_publish(&self, stream: &str, payload: &[u8]) -> Result<Value, ClientError> {
        self.cloud.publish(stream, payload).await
    }

    async fn aux_open(&self) -> Result<Value, ClientError> {
        self.radio.open_aux_stream().await
    }

    async fn aux_send(&self, channel: u8, payload: &[u8]) -> Result<Value, ClientError> {
        self.radio.send_aux(channel, payload).await
    }

    async fn aux_subscribe(&self, callback: AuxCallback) -> Result<(), ClientError> {
        self.radio
            .subscribe_aux(move |channel, payload| callback(channel, payload))
            .await
    }

    async fn publish_detection(&self, batch: &DetectionBatch) -> Result<Value, ClientError> {
        self.vision.publish_detection(batch).await
    }

    async fn advertise_offload(&self, advert: &OffloadAdvertisement) -> Result<Value, ClientError> {
        self.vision.advertise_offload(advert).await
    }

    async fn node_info(&self) -> Result<NodeInfo, ClientError> {
        self.node.info().await
    }
}

/// A key of a host reply map.
pub fn reply_field<'a>(reply: &'a Value, key: &str) -> Option<&'a Value> {
    reply
        .as_map()?
        .iter()
        .find(|(k, _)| k.as_str() == Some(key))
        .map(|(_, v)| v)
}

/// Whether a host reply is `{ok: true}`.
pub fn reply_ok(reply: &Value) -> bool {
    reply_field(reply, "ok").and_then(Value::as_bool) == Some(true)
}

/// The `error` string of a graceful-degrade reply (`not_available`,
/// `not_implemented`), if the reply is one.
pub fn reply_error(reply: &Value) -> Option<&str> {
    reply_field(reply, "error").and_then(Value::as_str)
}

/// A config value as JSON (`null` when it does not convert).
pub fn to_json(value: Value) -> serde_json::Value {
    rmpv::ext::from_value(value).unwrap_or(serde_json::Value::Null)
}

/// Read a plugin config key as JSON, degrading to `null` when the host does not
/// answer: a config read never takes a loop or a route down.
pub async fn config_json(host: &dyn Host, key: &str) -> serde_json::Value {
    match host.config_get(key, Value::Nil).await {
        Ok(v) => to_json(v),
        Err(e) => {
            tracing::debug!(key, error = %e, "config read failed");
            serde_json::Value::Null
        }
    }
}

/// A boolean plugin config key, `default` when absent or not a boolean.
pub async fn config_bool(host: &dyn Host, key: &str, default: bool) -> bool {
    config_json(host, key).await.as_bool().unwrap_or(default)
}

/// A non-empty string plugin config key, `None` when absent, empty or not a string.
pub async fn config_string(host: &dyn Host, key: &str) -> Option<String> {
    config_json(host, key)
        .await
        .as_str()
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(str::to_string)
}

/// The node facts, or `None` when the host cannot answer (an older host, a
/// missing grant, or the socket is down). A caller treats `None` as "nothing
/// known", so a gate that needs a fact stays closed.
pub async fn node_info(host: &dyn Host) -> Option<NodeInfo> {
    match host.node_info().await {
        Ok(info) => Some(info),
        Err(e) => {
            tracing::debug!(error = %e, "node.info unavailable");
            None
        }
    }
}
