//! The live scan stream for a node-built map: while Navigation's map status
//! says the heavy map is built on a node (`placement: stream`), every placed
//! planar scan it publishes on `plugin.navigation.scan_polar` is forwarded to
//! the paired node's capture-event lane as `reconstruct.live.scan`, where the
//! node paints its full-resolution live map. Outside `stream` nothing is
//! forwarded. Counts of forwarded, dropped (queue full) and failed scans feed
//! `GET /reconstruct/status`.

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::Duration;

use ados_mapping_protocol::atlas::PolarScan;
use ados_mapping_transport::compute_node;
use ados_mapping_transport::credentials::{CredentialStore, ATLAS_INGEST_LANE};
use ados_mapping_transport::host::Host;
use ados_mapping_transport::{AtlasBearer, AtlasEvent, LanHttpBearer};
use parking_lot::Mutex;
use rmpv::Value;
use serde::Serialize;
use tokio::sync::{mpsc, watch};
use tokio::time::Instant;

/// What the node's ingest keys a forwarded scan on.
pub const LIVE_SCAN_TOPIC: &str = "reconstruct.live.scan";
/// Navigation's map status (its `placement` gates the stream).
pub const MAP_STATUS_TOPIC: &str = "plugin.navigation.map_status";
/// Navigation's placed planar scans.
pub const SCAN_POLAR_TOPIC: &str = "plugin.navigation.scan_polar";

/// Scans queued for the node before the newest are dropped.
const QUEUE: usize = 64;
/// How often a failed subscription or an unresolved node is retried.
const RETRY: Duration = Duration::from_secs(10);

/// The stream's state for `/reconstruct/status`.
#[derive(Debug, Clone, Default, Serialize, PartialEq)]
pub struct StreamView {
    /// Navigation's placement is `stream`.
    pub streaming: bool,
    /// The node scans go to, once resolved.
    pub node: Option<String>,
    pub forwarded: u64,
    /// Dropped because the queue to the node was full.
    pub dropped: u64,
    /// The node did not take them.
    pub failed: u64,
}

pub type SharedStream = Arc<Mutex<StreamView>>;

/// The placement a map status names.
fn streams(payload: &Value) -> bool {
    payload
        .as_map()
        .and_then(|m| m.iter().find(|(k, _)| k.as_str() == Some("placement")))
        .and_then(|(_, v)| v.as_str())
        == Some("stream")
}

/// A `scan_polar` payload as the binary polar record the node decodes.
/// Through JSON: `rmpv`'s deserializer takes no enum from a string.
fn polar_record(payload: Value) -> Option<Vec<u8>> {
    let json = rmpv::ext::from_value::<serde_json::Value>(payload).ok()?;
    let scan: PolarScan = serde_json::from_value(json).ok()?;
    let mut body = Vec::with_capacity(scan.encoded_len());
    scan.encode(&mut body);
    Some(body)
}

pub struct Streamer {
    pub host: Arc<dyn Host>,
    pub credentials: CredentialStore,
    pub device_id: String,
    pub view: SharedStream,
}

impl Streamer {
    /// Subscribe to Navigation's topics, then forward until `shutdown`.
    pub async fn run(self, mut shutdown: watch::Receiver<bool>) {
        let on = Arc::new(AtomicBool::new(false));
        let (tx, mut rx) = mpsc::channel::<Value>(QUEUE);
        let (mut have_status, mut have_scans) = (false, false);
        let mut bearer: Option<LanHttpBearer> = None;
        let mut next_try = Instant::now();
        // An unresolved node is looked for again at most every RETRY.
        let mut resolve_after = Instant::now();
        loop {
            if !(have_status && have_scans) && Instant::now() >= next_try {
                if !have_status {
                    let (on, view) = (on.clone(), self.view.clone());
                    let cb = Arc::new(move |p: Value| {
                        let s = streams(&p);
                        on.store(s, Ordering::Relaxed);
                        view.lock().streaming = s;
                    });
                    have_status = self
                        .host
                        .event_subscribe(MAP_STATUS_TOPIC, cb)
                        .await
                        .is_ok();
                }
                if !have_scans {
                    let (on, tx, view) = (on.clone(), tx.clone(), self.view.clone());
                    let cb = Arc::new(move |p: Value| {
                        if on.load(Ordering::Relaxed) && tx.try_send(p).is_err() {
                            view.lock().dropped += 1;
                        }
                    });
                    have_scans = self
                        .host
                        .event_subscribe(SCAN_POLAR_TOPIC, cb)
                        .await
                        .is_ok();
                }
                if !(have_status && have_scans) {
                    tracing::debug!("reconstruct stream: Navigation's topics not available yet");
                }
                next_try = Instant::now() + RETRY;
            }
            let payload = tokio::select! {
                r = shutdown.changed() => {
                    if r.is_err() || *shutdown.borrow() { return; }
                    continue;
                }
                _ = tokio::time::sleep_until(next_try), if !(have_status && have_scans) => continue,
                p = rx.recv() => match p { Some(p) => p, None => return },
            };
            let Some(body) = polar_record(payload) else {
                self.view.lock().failed += 1;
                continue;
            };
            if bearer.is_none() && Instant::now() >= resolve_after {
                bearer = self.resolve().await;
                if bearer.is_none() {
                    resolve_after = Instant::now() + RETRY;
                }
            }
            let Some(b) = &bearer else {
                self.view.lock().failed += 1;
                continue;
            };
            let event = AtlasEvent::new(LIVE_SCAN_TOPIC, Some(self.device_id.clone()), body);
            match b.send(&event).await {
                Ok(()) => self.view.lock().forwarded += 1,
                Err(e) => {
                    tracing::debug!(error = %e, "reconstruct stream: the node did not take a scan");
                    self.view.lock().failed += 1;
                    bearer = None;
                    resolve_after = Instant::now() + RETRY;
                }
            }
        }
    }

    /// The paired node's capture-event lane, or `None` (the next scan tries
    /// again).
    async fn resolve(&self) -> Option<LanHttpBearer> {
        let node = compute_node::resolve(self.host.as_ref(), &self.credentials, &ATLAS_INGEST_LANE)
            .await?;
        let credential = node.credential(&self.credentials, &ATLAS_INGEST_LANE).ok();
        self.view.lock().node = Some(node.device_id.clone().unwrap_or_else(|| node.addr()));
        Some(LanHttpBearer::new(node.base_url(), credential))
    }
}

#[cfg(test)]
#[path = "stream_tests.rs"]
mod tests;
