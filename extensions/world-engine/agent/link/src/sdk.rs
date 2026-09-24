//! Adapters from the transport crate's seams onto the plugin host:
//!
//! - [`SdkAuxLane`]: the WFB relay bearer's [`AuxLane`] over
//!   `radio.aux_stream.open` / `.send` on the application-stream channel.
//! - [`CloudPublishBearer`]: the ladder's cloud rung over `cloud.publish`.
//! - [`SdkDetectionPublisher`]: the offload return path's [`DetectionPublisher`]
//!   over `vision.publish_detection`.
//! - [`CloudDetectionTee`]: the offload [`DetectionTee`] that mirrors each
//!   returned batch onto the core cloud detection stream.

use std::sync::Arc;
use std::time::Duration;

use ados_protocol::aux_mux::AuxChannel;
use ados_protocol::cloud_publish::{MAX_PAYLOAD, VISION_DETECTIONS_STREAM};
use ados_protocol::framebus::DetectionBatch;
use ados_sdk::ClientError;
use parking_lot::Mutex;
use tokio::sync::mpsc;
use tokio::time::Instant;
use world_engine_transport::{
    AtlasBearer, AtlasEvent, AuxLane, BearerKind, DetectionPublisher, DetectionTee, TransportError,
};

use crate::host::{reply_error, reply_field, reply_ok, Host};

/// The aux channel the drone-to-ground Atlas stream rides.
const APP_STREAM: u8 = AuxChannel::AppStream as u8;

/// The shared-topic namespace stripped from an event topic to name its cloud
/// stream (`plugin.world-engine.pose` rides `atlas.pose`).
const SHARED_TOPIC_PREFIX: &str = "plugin.world-engine.";
/// The internal-topic namespace stripped the same way (`atlas.keyframe` rides
/// `atlas.keyframe`).
const INTERNAL_TOPIC_PREFIX: &str = "atlas.";
/// The longest cloud stream name the host accepts.
const MAX_STREAM_LEN: usize = 64;

/// After the cloud relay answered `not_available`, the rung reports itself
/// unavailable for this long before a send probes it again.
const CLOUD_RETRY_AFTER: Duration = Duration::from_secs(15);

/// Depth of the cloud detection tee's queue. The lane is lossy by design: a
/// full queue drops the newest batch rather than stalling the return path.
const TEE_QUEUE: usize = 8;

/// The WFB relay bearer's radio lane over the host's aux-stream methods.
pub struct SdkAuxLane {
    host: Arc<dyn Host>,
}

impl SdkAuxLane {
    pub fn new(host: Arc<dyn Host>) -> Self {
        Self { host }
    }
}

#[async_trait::async_trait]
impl AuxLane for SdkAuxLane {
    async fn open(&self) -> Result<bool, TransportError> {
        match self.host.aux_open().await {
            // The radio service answers `{ok, active}`; a disabled lane or a
            // down service is a refused open, not an error.
            Ok(reply) => {
                let active = reply_field(&reply, "active").and_then(rmpv::Value::as_bool);
                Ok(reply_ok(&reply) && active != Some(false))
            }
            // The host refused the open (another plugin owns the stream, the
            // grant is missing): the lane is simply not ours to use.
            Err(ClientError::Rpc(_) | ClientError::CapabilityDenied(_)) => Ok(false),
            Err(e) => {
                tracing::debug!(error = %e, "aux stream open failed");
                Err(TransportError::Unavailable)
            }
        }
    }

    async fn send(&self, payload: &[u8]) -> Result<(), TransportError> {
        match self.host.aux_send(APP_STREAM, payload).await {
            Ok(reply) if reply_ok(&reply) => Ok(()),
            Ok(reply) => {
                tracing::debug!(error = ?reply_error(&reply), "aux stream send refused");
                Err(TransportError::Unavailable)
            }
            Err(e) => {
                tracing::debug!(error = %e, "aux stream send failed");
                Err(TransportError::Unavailable)
            }
        }
    }
}

/// The cloud stream an Atlas event topic rides: `atlas.<leaf>`, where the leaf
/// is the topic without its `plugin.world-engine.` or `atlas.` namespace (dots
/// kept). `None` for a topic that yields no leaf or no valid stream name.
pub fn cloud_stream(topic: &str) -> Option<String> {
    let leaf = topic
        .strip_prefix(SHARED_TOPIC_PREFIX)
        .or_else(|| topic.strip_prefix(INTERNAL_TOPIC_PREFIX))
        .unwrap_or(topic);
    if leaf.is_empty() {
        return None;
    }
    let stream = format!("{INTERNAL_TOPIC_PREFIX}{leaf}");
    let valid = stream.len() <= MAX_STREAM_LEN
        && stream
            .bytes()
            .all(|b| b.is_ascii_lowercase() || b.is_ascii_digit() || b"._-".contains(&b));
    valid.then_some(stream)
}

/// The last rung of the bearer ladder: the whole framed [`AtlasEvent`] published
/// on the plugin's own cloud stream for an off-LAN operator.
///
/// The cloud lane carries descriptors, not artifacts: a framed event over the
/// host's 64 KiB publish ceiling is declined with a retriable
/// [`TransportError::PayloadTooLarge`], so the ladder offers it to a fatter lane.
/// A `not_available` reply (unpaired, or the relay is down) marks the rung
/// unavailable, so the ladder skips it; after a back-off one send probes the
/// relay again, and a publish that succeeds clears the mark.
#[derive(Clone)]
pub struct CloudPublishBearer {
    host: Arc<dyn Host>,
    down_since: Arc<Mutex<Option<Instant>>>,
}

impl CloudPublishBearer {
    pub fn new(host: Arc<dyn Host>) -> Self {
        Self {
            host,
            down_since: Arc::new(Mutex::new(None)),
        }
    }

    fn marked_down(&self) -> bool {
        self.down_since
            .lock()
            .is_some_and(|t| t.elapsed() < CLOUD_RETRY_AFTER)
    }
}

#[async_trait::async_trait]
impl AtlasBearer for CloudPublishBearer {
    fn kind(&self) -> BearerKind {
        BearerKind::Cloud
    }

    async fn is_available(&self) -> bool {
        !self.marked_down()
    }

    async fn send(&self, event: &AtlasEvent) -> Result<(), TransportError> {
        if self.marked_down() {
            return Err(TransportError::Unavailable);
        }
        // The whole envelope, the same wire the LAN receiver decodes.
        let body = event.encode()?;
        if body.len() > MAX_PAYLOAD {
            return Err(TransportError::PayloadTooLarge(body.len()));
        }
        let stream = cloud_stream(&event.topic).ok_or_else(|| {
            TransportError::Encode(format!("no cloud stream for {}", event.topic))
        })?;
        match self.host.cloud_publish(&stream, &body).await {
            Ok(reply) if reply_ok(&reply) => {
                *self.down_since.lock() = None;
                Ok(())
            }
            Ok(reply) if reply_error(&reply) == Some("not_available") => {
                *self.down_since.lock() = Some(Instant::now());
                Err(TransportError::Unavailable)
            }
            Ok(reply) => {
                tracing::debug!(error = ?reply_error(&reply), "cloud publish not accepted");
                Err(TransportError::Unavailable)
            }
            Err(e) => {
                tracing::debug!(error = %e, "cloud publish failed");
                Err(TransportError::Unavailable)
            }
        }
    }
}

/// Republishes offloaded detections onto the drone's own `vision.detection` bus.
pub struct SdkDetectionPublisher {
    host: Arc<dyn Host>,
}

impl SdkDetectionPublisher {
    pub fn new(host: Arc<dyn Host>) -> Self {
        Self { host }
    }
}

#[async_trait::async_trait]
impl DetectionPublisher for SdkDetectionPublisher {
    async fn publish(&self, batch: &DetectionBatch) -> anyhow::Result<()> {
        let reply = self.host.publish_detection(batch).await?;
        match reply_error(&reply) {
            Some(e) => Err(anyhow::anyhow!("detection publish refused: {e}")),
            None => Ok(()),
        }
    }
}

/// Mirrors each returned detection batch onto the core cloud detection stream,
/// as the `DetectionBatch` JSON the local detections WebSocket emits, so an
/// off-LAN GCS parses one shape. Never blocks the caller: batches queue on a
/// small bounded channel a task drains, and a full queue drops the batch.
pub struct CloudDetectionTee {
    tx: mpsc::Sender<Vec<u8>>,
}

impl CloudDetectionTee {
    /// Build the tee and spawn its drain task (it ends when the tee is dropped).
    pub fn spawn(host: Arc<dyn Host>) -> Self {
        let (tx, mut rx) = mpsc::channel::<Vec<u8>>(TEE_QUEUE);
        tokio::spawn(async move {
            while let Some(body) = rx.recv().await {
                match host.cloud_publish(VISION_DETECTIONS_STREAM, &body).await {
                    Ok(reply) if reply_ok(&reply) => {}
                    Ok(reply) => {
                        tracing::trace!(error = ?reply_error(&reply), "cloud detection batch dropped")
                    }
                    Err(e) => tracing::trace!(error = %e, "cloud detection batch dropped"),
                }
            }
        });
        Self { tx }
    }
}

impl DetectionTee for CloudDetectionTee {
    fn publish(&self, batch: &DetectionBatch) {
        let body = match serde_json::to_vec(batch) {
            Ok(b) => b,
            Err(e) => {
                tracing::debug!(error = %e, "encode detection batch for the cloud lane");
                return;
            }
        };
        if self.tx.try_send(body).is_err() {
            tracing::trace!("cloud detection tee busy; batch dropped");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::FakeHost;
    use ados_protocol::framebus::{BoundingBox, Detection, VISION_DETECTION_VERSION};
    use rmpv::Value;

    fn event(topic: &str, payload: Vec<u8>) -> AtlasEvent {
        AtlasEvent::new(topic, None, payload)
    }

    #[test]
    fn cloud_streams_drop_the_namespace_and_keep_the_dots() {
        assert_eq!(
            cloud_stream("plugin.world-engine.pose").as_deref(),
            Some("atlas.pose")
        );
        assert_eq!(
            cloud_stream("atlas.keyframe").as_deref(),
            Some("atlas.keyframe")
        );
        assert_eq!(
            cloud_stream("atlas.pose.offload").as_deref(),
            Some("atlas.pose.offload")
        );
        assert_eq!(
            cloud_stream("atlas.capture.state").as_deref(),
            Some("atlas.capture.state")
        );
        // A bare namespace has no leaf; a name the host would refuse is not built.
        assert_eq!(cloud_stream("atlas."), None);
        assert_eq!(cloud_stream("plugin.world-engine."), None);
        assert_eq!(cloud_stream("atlas.Bad/Topic"), None);
        assert_eq!(cloud_stream(&format!("atlas.{}", "x".repeat(64))), None);
    }

    #[tokio::test]
    async fn a_descriptor_publishes_the_whole_envelope_on_its_stream() {
        let host = FakeHost::new();
        let bearer = CloudPublishBearer::new(host.clone());
        let ev = event("plugin.world-engine.occupancy", vec![1, 2, 3]);
        bearer.send(&ev).await.unwrap();

        let pubs = host.cloud.lock();
        assert_eq!(pubs.len(), 1);
        assert_eq!(pubs[0].0, "atlas.occupancy");
        assert_eq!(AtlasEvent::decode(&pubs[0].1).unwrap(), ev);
    }

    #[tokio::test]
    async fn an_event_over_64_kib_is_declined_retriably_and_not_published() {
        let host = FakeHost::new();
        let bearer = CloudPublishBearer::new(host.clone());
        let err = bearer
            .send(&event("atlas.keyframe", vec![0u8; MAX_PAYLOAD]))
            .await
            .unwrap_err();
        assert!(matches!(err, TransportError::PayloadTooLarge(n) if n > MAX_PAYLOAD));
        assert!(err.is_retriable(), "a fatter lane carries the keyframe");
        assert!(host.cloud.lock().is_empty());
    }

    #[tokio::test(start_paused = true)]
    async fn a_not_available_relay_is_skipped_until_a_later_publish_succeeds() {
        let host = FakeHost::new();
        *host.cloud_reply.lock() = FakeHost::not_available("cloud.publish");
        let bearer = CloudPublishBearer::new(host.clone());
        assert!(bearer.is_available().await);

        let err = bearer
            .send(&event("atlas.pose", vec![1]))
            .await
            .unwrap_err();
        assert!(matches!(err, TransportError::Unavailable));
        // Down: the ladder skips the rung, and a send does not reach the host.
        assert!(!bearer.is_available().await);
        assert!(bearer.send(&event("atlas.pose", vec![2])).await.is_err());
        assert_eq!(host.cloud.lock().len(), 1);

        // After the back-off the rung is offered again; the relay is back up, so
        // the probe publish succeeds and the rung stays available.
        tokio::time::advance(CLOUD_RETRY_AFTER).await;
        *host.cloud_reply.lock() = FakeHost::ok();
        assert!(bearer.is_available().await);
        bearer.send(&event("atlas.pose", vec![3])).await.unwrap();
        assert!(bearer.is_available().await);
        assert_eq!(host.cloud.lock().len(), 2);
    }

    #[tokio::test]
    async fn an_aux_open_the_host_refuses_is_an_inactive_lane() {
        let host = FakeHost::new();
        let lane = SdkAuxLane::new(host.clone());
        *host.aux_open_reply.lock() = Err("radio aux stream is open by another plugin".into());
        assert!(!lane.open().await.unwrap());
        *host.aux_open_reply.lock() = Ok(FakeHost::map(&[
            ("ok", Value::Boolean(true)),
            ("active", Value::Boolean(false)),
        ]));
        assert!(!lane.open().await.unwrap());
        *host.aux_open_reply.lock() = Ok(FakeHost::map(&[
            ("ok", Value::Boolean(true)),
            ("active", Value::Boolean(true)),
        ]));
        assert!(lane.open().await.unwrap());

        lane.send(b"frame").await.unwrap();
        assert_eq!(host.aux_sent.lock()[0], (8, b"frame".to_vec()));
    }

    fn batch() -> DetectionBatch {
        DetectionBatch {
            v: VISION_DETECTION_VERSION,
            model_id: "offload".into(),
            camera_id: "front".into(),
            frame_id: 7,
            ts_ms: 1_700_000_000_000,
            frame_width: 1280,
            frame_height: 720,
            detections: vec![Detection {
                bbox: Some(BoundingBox {
                    x: 320.0,
                    y: 180.0,
                    width: 640.0,
                    height: 360.0,
                }),
                class_label: "person".into(),
                confidence: 0.8,
                track_id: Some(3),
                assoc_confidence: None,
                lock_state: None,
                attributes: None,
                mask: None,
                keypoints: None,
                depth: None,
                world_pos: None,
            }],
        }
    }

    #[tokio::test]
    async fn the_tee_publishes_the_batch_json_on_the_detection_stream() {
        let host = FakeHost::new();
        let tee = CloudDetectionTee::spawn(host.clone());
        tee.publish(&batch());

        let (stream, payload) = host.next_cloud_publish().await;
        assert_eq!(stream, VISION_DETECTIONS_STREAM);
        let json: serde_json::Value = serde_json::from_slice(&payload).unwrap();
        assert_eq!(json["camera_id"], "front");
        assert_eq!(json["detections"][0]["class_label"], "person");
        assert_eq!(json["detections"][0]["bbox"]["width"], 640.0);
        let back: DetectionBatch = serde_json::from_slice(&payload).unwrap();
        assert_eq!(back, batch());
    }
}
