//! Where the lane's topics go. The lane encodes each payload as a named
//! msgpack map and queues it ([`Publisher`]); one task hands the queue to
//! the host (`event.publish`, [`forward`]), so a slow host call never stalls
//! the pose lane, and a test reads the queue directly.

use ados_mapping_protocol::capture::{
    LidarStatus, LiveOccupancy, MapStatus, PolarScan, PoseDescriptor, PoseTrackBatch, ScanChunk,
    ScanDescriptor,
};
use ados_protocol::shutdown::Shutdown;
use ados_sdk::PluginContext;
use serde::Serialize;
use tokio::sync::mpsc;

use crate::topics;

/// Events queued for the host before the lane waits.
pub const QUEUE_DEPTH: usize = 256;

/// One queued event: the topic and its msgpack body.
pub type Event = (&'static str, Vec<u8>);

/// The lane's side of the queue.
#[derive(Clone)]
pub struct Publisher {
    tx: mpsc::Sender<Event>,
}

impl Publisher {
    pub fn new() -> (Self, mpsc::Receiver<Event>) {
        let (tx, rx) = mpsc::channel(QUEUE_DEPTH);
        (Self { tx }, rx)
    }

    async fn send<T: Serialize>(&self, topic: &'static str, v: &T) {
        match rmp_serde::to_vec_named(v) {
            Ok(body) => {
                // Closed only while the service stops.
                let _ = self.tx.send((topic, body)).await;
            }
            Err(e) => tracing::warn!(topic, error = %e, "navigation_publish_encode_failed"),
        }
    }

    pub async fn pose(&self, p: &PoseDescriptor) {
        self.send(topics::POSE, p).await;
    }
    pub async fn scan(&self, s: &ScanDescriptor) {
        self.send(topics::SCAN, s).await;
    }
    pub async fn map_status(&self, s: &MapStatus) {
        self.send(topics::MAP_STATUS, s).await;
    }
    pub async fn lidar(&self, s: &LidarStatus) {
        self.send(topics::LIDAR, s).await;
    }
    pub async fn occupancy(&self, o: &LiveOccupancy) {
        self.send(topics::OCCUPANCY, o).await;
    }
    pub async fn scan_polar(&self, p: &PolarScan) {
        self.send(topics::SCAN_POLAR, p).await;
    }
    pub async fn scan_raw(&self, c: &ScanChunk) {
        self.send(topics::SCAN_RAW, c).await;
    }
    pub async fn pose_track(&self, b: &PoseTrackBatch) {
        self.send(topics::POSE_TRACK, b).await;
    }
}

/// Hand every queued event to the host until `cancel`. A refused publish
/// is logged once per topic until one succeeds again.
pub async fn forward(ctx: PluginContext, mut rx: mpsc::Receiver<Event>, cancel: Shutdown) {
    let mut failing: Vec<&'static str> = Vec::new();
    loop {
        let (topic, body) = tokio::select! {
            _ = cancel.wait() => return,
            e = rx.recv() => match e { Some(e) => e, None => return },
        };
        let payload = match rmpv::decode::read_value(&mut body.as_slice()) {
            Ok(v) => v,
            Err(e) => {
                tracing::warn!(topic, error = %e, "navigation_publish_decode_failed");
                continue;
            }
        };
        match ctx.events.publish(topic, payload).await {
            Ok(_) => failing.retain(|t| *t != topic),
            Err(e) if !failing.contains(&topic) => {
                tracing::warn!(topic, error = %e, "navigation_publish_failed");
                failing.push(topic);
            }
            Err(_) => {}
        }
    }
}
