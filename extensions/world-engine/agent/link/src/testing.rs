//! An in-memory [`Host`] for the link's tests: records the calls the tests
//! assert on and answers with configurable replies.

use std::collections::HashMap;
use std::sync::Arc;
use std::time::Duration;

use ados_protocol::framebus::DetectionBatch;
use ados_protocol::node_info::{BoardInfo, CameraInfo, GroundStationInfo, NodeInfo};
use ados_protocol::plugin_mdns::DiscoveredService;
use ados_sdk::{ClientError, OffloadAdvertisement};
use parking_lot::Mutex;
use rmpv::Value;
use tokio::sync::Notify;

use crate::host::{AuxCallback, Host};

/// Every call the link made, plus the replies it gets.
pub struct FakeHost {
    pub config: Mutex<HashMap<String, Value>>,
    pub telemetry: Mutex<Vec<(String, Value)>>,
    pub events: Mutex<Vec<(String, Value)>>,
    pub cloud: Mutex<Vec<(String, Vec<u8>)>>,
    pub cloud_reply: Mutex<Value>,
    pub aux_open_reply: Mutex<Result<Value, String>>,
    pub aux_sent: Mutex<Vec<(u8, Vec<u8>)>>,
    pub aux_callback: Mutex<Option<AuxCallback>>,
    /// `None` answers `node.info` with an error.
    pub node_info: Mutex<Option<NodeInfo>>,
    /// The instances an `mdns.browse` answers with, by service type; a type
    /// with no entry answers with an error.
    pub mdns: Mutex<HashMap<String, Vec<DiscoveredService>>>,
    /// Every service type browsed, in order.
    pub mdns_browsed: Mutex<Vec<String>>,
    changed: Notify,
}

impl FakeHost {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            config: Mutex::default(),
            telemetry: Mutex::default(),
            events: Mutex::default(),
            cloud: Mutex::default(),
            cloud_reply: Mutex::new(Self::ok()),
            aux_open_reply: Mutex::new(Ok(Self::map(&[
                ("ok", Value::Boolean(true)),
                ("active", Value::Boolean(true)),
            ]))),
            aux_sent: Mutex::default(),
            aux_callback: Mutex::default(),
            node_info: Mutex::default(),
            mdns: Mutex::default(),
            mdns_browsed: Mutex::default(),
            changed: Notify::new(),
        })
    }

    pub fn map(entries: &[(&str, Value)]) -> Value {
        Value::Map(
            entries
                .iter()
                .map(|(k, v)| (Value::from(*k), v.clone()))
                .collect(),
        )
    }

    pub fn ok() -> Value {
        Self::map(&[("ok", Value::Boolean(true))])
    }

    pub fn not_available(method: &str) -> Value {
        Self::map(&[
            ("error", Value::from("not_available")),
            ("method", Value::from(method)),
        ])
    }

    /// Set a plugin config key from JSON.
    pub fn set_config(&self, key: &str, value: serde_json::Value) {
        self.config
            .lock()
            .insert(key.to_string(), rmpv::ext::to_value(&value).unwrap());
    }

    /// A plugin config key as JSON (`null` when unset).
    pub fn config_value(&self, key: &str) -> serde_json::Value {
        self.config
            .lock()
            .get(key)
            .cloned()
            .map(crate::host::to_json)
            .unwrap_or(serde_json::Value::Null)
    }

    /// Wait (bounded) for a cloud publish, then take the oldest one.
    pub async fn next_cloud_publish(&self) -> (String, Vec<u8>) {
        self.wait_for(|h| !h.cloud.lock().is_empty()).await;
        self.cloud.lock().remove(0)
    }

    /// Wait (bounded) until `pred` holds.
    pub async fn wait_for(&self, pred: impl Fn(&Self) -> bool) {
        tokio::time::timeout(Duration::from_secs(5), async {
            loop {
                let changed = self.changed.notified();
                if pred(self) {
                    return;
                }
                changed.await;
            }
        })
        .await
        .expect("the fake host saw the expected call");
    }

    fn touch(&self) {
        self.changed.notify_waiters();
    }

    /// Node facts: `has_npu` of `None` is an unprobed board.
    pub fn info(
        profile: &str,
        has_npu: Option<bool>,
        camera_ready: bool,
        role: Option<&str>,
    ) -> NodeInfo {
        NodeInfo {
            profile: profile.to_string(),
            board: has_npu.map(|has_npu| BoardInfo {
                id: "board".into(),
                name: "Board".into(),
                has_npu,
                accelerators: if has_npu { vec!["npu".into()] } else { vec![] },
            }),
            ground_station: GroundStationInfo {
                role: role.map(str::to_string),
            },
            camera: CameraInfo {
                ready: camera_ready,
                main: None,
            },
        }
    }
}

#[async_trait::async_trait]
impl Host for FakeHost {
    async fn config_get(&self, key: &str, default: Value) -> Result<Value, ClientError> {
        Ok(self.config.lock().get(key).cloned().unwrap_or(default))
    }

    async fn config_set(&self, key: &str, value: Value) -> Result<(), ClientError> {
        self.config.lock().insert(key.to_string(), value);
        self.touch();
        Ok(())
    }

    async fn telemetry_extend(&self, channel: &str, payload: Value) -> Result<(), ClientError> {
        self.telemetry.lock().push((channel.to_string(), payload));
        self.touch();
        Ok(())
    }

    async fn event_publish(&self, topic: &str, payload: Value) -> Result<(), ClientError> {
        self.events.lock().push((topic.to_string(), payload));
        self.touch();
        Ok(())
    }

    async fn cloud_publish(&self, stream: &str, payload: &[u8]) -> Result<Value, ClientError> {
        self.cloud
            .lock()
            .push((stream.to_string(), payload.to_vec()));
        self.touch();
        Ok(self.cloud_reply.lock().clone())
    }

    async fn aux_open(&self) -> Result<Value, ClientError> {
        self.aux_open_reply.lock().clone().map_err(ClientError::Rpc)
    }

    async fn aux_send(&self, channel: u8, payload: &[u8]) -> Result<Value, ClientError> {
        self.aux_sent.lock().push((channel, payload.to_vec()));
        self.touch();
        Ok(Self::ok())
    }

    async fn aux_subscribe(&self, callback: AuxCallback) -> Result<(), ClientError> {
        *self.aux_callback.lock() = Some(callback);
        self.touch();
        Ok(())
    }

    async fn publish_detection(&self, _batch: &DetectionBatch) -> Result<Value, ClientError> {
        Ok(Self::ok())
    }

    async fn advertise_offload(
        &self,
        _advert: &OffloadAdvertisement,
    ) -> Result<Value, ClientError> {
        Ok(Self::ok())
    }

    async fn node_info(&self) -> Result<NodeInfo, ClientError> {
        self.node_info
            .lock()
            .clone()
            .ok_or_else(|| ClientError::Rpc("not_available".into()))
    }

    async fn mdns_browse(
        &self,
        service_type: &str,
        _window: Duration,
    ) -> Result<Vec<DiscoveredService>, ClientError> {
        self.mdns_browsed.lock().push(service_type.to_string());
        self.mdns
            .lock()
            .get(service_type)
            .cloned()
            .ok_or_else(|| ClientError::Rpc("not_available".into()))
    }
}
