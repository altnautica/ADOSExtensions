//! The ground-station Atlas relay: bridge the radio aux lane onto the LAN.
//!
//! The drone radiates small Atlas events on the aux application stream. The
//! ground station receives them through its own aux subscription
//! (`radio.aux_stream.subscribe`); this relay decodes each application-stream
//! payload as a framed [`AtlasEvent`] and re-POSTs it onto the LAN into the
//! compute node's event router, so the field radio lane reaches the same
//! receiver the direct-LAN bearer uses.
//!
//! A garbled or hostile frame off the air never takes the relay down: a
//! malformed payload is dropped and counted. The received-side counters are the
//! delivery proof: the drone's send into the aux lane is fire-and-forget, so only
//! what this relay decodes proves the radio lane carried it.
//!
//! Runs only on a ground station in the `relay` role (`node.info`) with plugin
//! config `relay.enabled` set, both polled. The compute node is
//! `relay.compute_base_url` when set, else a node resolved over mDNS.

use std::sync::Arc;
use std::time::Duration;

use ados_protocol::aux_mux::AuxChannel;
use parking_lot::Mutex;
use serde::Serialize;
use tokio::sync::{mpsc, watch};
use tokio::time::Instant;
use world_engine_transport::{AtlasBearer, AtlasEvent, LanHttpBearer};

use crate::compute_node;
use crate::credentials::{CredentialStore, ATLAS_INGEST_LANE};
use crate::forward::now_ms;
use crate::host::{config_bool, config_string, node_info, Host};
use ados_protocol::node_info::NodeInfo;

/// How often the gate and the compute URL are re-read.
const GATE_POLL: Duration = Duration::from_secs(5);
/// Cadence the relay republishes its counters at (and re-reads its credential),
/// so the surface stays current while the aux lane is idle.
const SNAPSHOT_INTERVAL: Duration = Duration::from_secs(2);
/// A snapshot older than this is served as stale.
pub const SNAPSHOT_FRESH: Duration = Duration::from_secs(10);
/// One mDNS browse's window, and the wait between browses.
const RESOLVE_TIMEOUT: Duration = Duration::from_secs(5);
const RESOLVE_RETRY: Duration = Duration::from_secs(10);
/// Received payloads buffered between the host callback and the relay loop.
const RX_QUEUE: usize = 256;
/// The aux channel the drone-to-ground Atlas stream rides.
const APP_STREAM: u8 = AuxChannel::AppStream as u8;
/// The ground-station role that bridges the radio lane onto the LAN.
const RELAY_ROLE: &str = "relay";

/// Whether the node facts put this ground station in the relay role.
fn is_relay_role(info: &NodeInfo) -> bool {
    info.ground_station.role.as_deref() == Some(RELAY_ROLE)
}

/// Running counters for one relay run.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct RelayStats {
    /// Application-stream payloads received (received-side liveness proof).
    pub datagrams_seen: u64,
    /// Events decoded and accepted by the compute receiver.
    pub forwarded: u64,
    /// Payloads that did not decode to an Atlas event (dropped).
    pub malformed: u64,
    /// Events that decoded but whose forward POST failed.
    pub forward_failed: u64,
}

/// The relay's published live state: the body `GET /atlas-relay/status` serves.
#[derive(Debug, Clone, Serialize)]
pub struct RelaySnapshot {
    /// True while the relay is running; false on the teardown snapshot.
    pub up: bool,
    pub datagrams_seen: u64,
    pub forwarded: u64,
    pub malformed: u64,
    pub forward_failed: u64,
    /// The compute node base URL decoded events are forwarded to.
    pub compute_url: String,
    /// The aux lane is received through the host, not a local port, so this is
    /// always `null`; the key stays so a reader's key set is unchanged.
    pub listen_port: Option<u16>,
    /// Epoch-millis the snapshot was generated.
    pub generated_at_ms: i64,
}

impl RelaySnapshot {
    fn new(stats: &RelayStats, compute_url: &str, up: bool) -> Self {
        Self {
            up,
            datagrams_seen: stats.datagrams_seen,
            forwarded: stats.forwarded,
            malformed: stats.malformed,
            forward_failed: stats.forward_failed,
            compute_url: compute_url.to_string(),
            listen_port: None,
            generated_at_ms: now_ms(),
        }
    }
}

/// The latest snapshot and when it was taken, shared with the status route.
pub type SharedSnapshot = Arc<Mutex<Option<(RelaySnapshot, Instant)>>>;

/// The status route body: the latest snapshot stamped `stale: false` while
/// fresh; past [`SNAPSHOT_FRESH`] every key it carried nulled under
/// `stale: true` (a vanished key would read as "not reported" and be coerced to
/// 0 downstream); with no snapshot at all, just `{"stale": true}`.
pub fn status_body(snapshot: Option<&(RelaySnapshot, Instant)>) -> serde_json::Value {
    let (fresh, map) = match snapshot {
        Some((snap, at)) => (
            at.elapsed() <= SNAPSHOT_FRESH,
            match serde_json::to_value(snap) {
                Ok(serde_json::Value::Object(m)) => m,
                _ => serde_json::Map::new(),
            },
        ),
        None => (false, serde_json::Map::new()),
    };
    let mut out: serde_json::Map<String, serde_json::Value> = if fresh {
        map
    } else {
        map.into_iter()
            .map(|(k, _)| (k, serde_json::Value::Null))
            .collect()
    };
    out.insert("stale".to_string(), serde_json::Value::Bool(!fresh));
    serde_json::Value::Object(out)
}

/// Decode one received payload and forward it to the compute node. Never
/// panics: a non-decoding payload is counted as malformed and dropped.
async fn forward_payload(bearer: &dyn AtlasBearer, payload: &[u8], stats: &mut RelayStats) {
    stats.datagrams_seen += 1;
    match AtlasEvent::decode(payload) {
        Ok(event) => match bearer.send(&event).await {
            Ok(()) => stats.forwarded += 1,
            Err(e) => {
                stats.forward_failed += 1;
                tracing::debug!(error = %e, topic = %event.topic, "atlas_relay_forward_failed");
            }
        },
        Err(_) => {
            stats.malformed += 1;
            tracing::debug!(bytes = payload.len(), "atlas_relay_malformed_datagram");
        }
    }
}

/// One live relay run toward one compute node.
struct Run {
    compute_url: String,
    node_id: Option<String>,
    credential: Option<String>,
    bearer: LanHttpBearer,
    stats: RelayStats,
}

/// Everything the relay loop needs.
pub struct Relay {
    pub host: Arc<dyn Host>,
    pub credentials: CredentialStore,
    pub snapshot: SharedSnapshot,
}

impl Relay {
    /// Run until `shutdown` flips.
    pub async fn run(self, mut shutdown: watch::Receiver<bool>) {
        let (tx, mut rx) = mpsc::channel::<Vec<u8>>(RX_QUEUE);
        let mut subscribed = false;
        let mut run: Option<Run> = None;
        let mut configured: Option<String> = None;
        let mut enabled = false;
        // The last role the host reported, kept across a failed `node.info`
        // read so a transient blip does not restart the relay.
        let mut relay_role = false;
        let mut last_resolve: Option<Instant> = None;
        let mut gate = tokio::time::interval(GATE_POLL);
        gate.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        let mut publish = tokio::time::interval(SNAPSHOT_INTERVAL);
        publish.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);

        loop {
            tokio::select! {
                r = shutdown.changed() => {
                    if r.is_err() || *shutdown.borrow() { break; }
                }
                _ = gate.tick() => {
                    if let Some(info) = node_info(self.host.as_ref()).await {
                        relay_role = is_relay_role(&info);
                    }
                    let now_enabled = relay_role
                        && config_bool(self.host.as_ref(), "relay.enabled", false).await;
                    let url = config_string(self.host.as_ref(), "relay.compute_base_url").await;
                    if now_enabled != enabled || url != configured {
                        tracing::info!(enabled = now_enabled, compute_url = ?url, "atlas relay config changed");
                        self.stop(run.take());
                        enabled = now_enabled;
                        configured = url;
                        last_resolve = None;
                    }
                    if enabled && !subscribed {
                        subscribed = self.subscribe(tx.clone()).await;
                    }
                    if enabled && run.is_none() && last_resolve.is_none_or(|t| t.elapsed() >= RESOLVE_RETRY) {
                        last_resolve = Some(Instant::now());
                        run = self.start(configured.clone()).await;
                    }
                }
                Some(payload) = rx.recv() => {
                    // Received while stopped: nothing to forward to.
                    if let Some(r) = run.as_mut() {
                        forward_payload(&r.bearer, &payload, &mut r.stats).await;
                        self.publish(r, true);
                    }
                }
                _ = publish.tick() => {
                    if let Some(r) = run.as_mut() {
                        // Pick up a credential installed (or rotated) after the
                        // relay started, without a restart.
                        let fresh = self.credentials.lookup(r.node_id.as_deref(), &ATLAS_INGEST_LANE);
                        if fresh != r.credential {
                            tracing::info!("atlas_relay_credential_changed");
                            r.bearer = LanHttpBearer::new(r.compute_url.clone(), fresh.clone());
                            r.credential = fresh;
                        }
                        self.publish(r, true);
                    }
                }
            }
        }
        self.stop(run.take());
    }

    /// Register the aux callback and subscribe; the callback only queues (it
    /// runs on the plugin socket's reader task) and drops when the queue is
    /// full.
    async fn subscribe(&self, tx: mpsc::Sender<Vec<u8>>) -> bool {
        let callback = Arc::new(move |channel: u8, payload: Vec<u8>| {
            if channel == APP_STREAM && tx.try_send(payload).is_err() {
                tracing::debug!("atlas relay queue full; payload dropped");
            }
        });
        match self.host.aux_subscribe(callback).await {
            Ok(()) => {
                tracing::info!("atlas relay subscribed to the aux lane");
                true
            }
            Err(e) => {
                tracing::warn!(error = %e, "atlas relay aux subscribe failed");
                false
            }
        }
    }

    /// Start a run toward the configured URL, or a node resolved over mDNS.
    async fn start(&self, configured: Option<String>) -> Option<Run> {
        let (compute_url, node_id) = match configured {
            Some(url) => (url, None),
            None => {
                let issuers = self.credentials.issuers(&ATLAS_INGEST_LANE);
                let Some(node) =
                    compute_node::discover(self.host.as_ref(), &issuers, RESOLVE_TIMEOUT).await
                else {
                    tracing::debug!("atlas relay: no compute node on mDNS yet");
                    return None;
                };
                let url = format!("http://{}:{}", node.host, node.job_api_port);
                (url, Some(node.device_id).filter(|d| !d.is_empty()))
            }
        };
        let credential = self
            .credentials
            .lookup(node_id.as_deref(), &ATLAS_INGEST_LANE);
        tracing::info!(compute_url = %compute_url, "starting ground-station Atlas aux-lane relay");
        let run = Run {
            bearer: LanHttpBearer::new(compute_url.clone(), credential.clone()),
            compute_url,
            node_id,
            credential,
            stats: RelayStats::default(),
        };
        self.publish(&run, true);
        Some(run)
    }

    /// End a run, leaving a final `up: false` snapshot.
    fn stop(&self, run: Option<Run>) {
        if let Some(r) = run {
            self.publish(&r, false);
            tracing::info!(stats = ?r.stats, "atlas_relay_stopped");
        }
    }

    fn publish(&self, run: &Run, up: bool) {
        *self.snapshot.lock() = Some((
            RelaySnapshot::new(&run.stats, &run.compute_url, up),
            Instant::now(),
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::FakeHost;
    use world_engine_transport::atlas_event_router;

    /// Spawn the compute receiver on an ephemeral port; return its base URL and
    /// the channel events land on.
    async fn spawn_receiver() -> (String, mpsc::Receiver<AtlasEvent>) {
        let (tx, rx) = mpsc::channel(16);
        let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
        let addr = listener.local_addr().unwrap();
        tokio::spawn(async move {
            let _ = axum::serve(listener, atlas_event_router(tx)).await;
        });
        (format!("http://{addr}"), rx)
    }

    #[tokio::test]
    async fn a_decoded_payload_is_forwarded_to_the_compute_receiver() {
        let (base, mut rx) = spawn_receiver().await;
        let bearer = LanHttpBearer::new(base, None);
        let mut stats = RelayStats::default();
        let ev = AtlasEvent::new("atlas.occupancy", None, vec![1, 2, 3]);
        forward_payload(&bearer, &ev.encode().unwrap(), &mut stats).await;
        assert_eq!(rx.recv().await.unwrap(), ev);
        assert_eq!(
            stats,
            RelayStats {
                datagrams_seen: 1,
                forwarded: 1,
                ..Default::default()
            }
        );
    }

    #[tokio::test]
    async fn a_malformed_payload_is_dropped_and_counted_not_forwarded() {
        let (base, mut rx) = spawn_receiver().await;
        let bearer = LanHttpBearer::new(base, None);
        let mut stats = RelayStats::default();
        forward_payload(&bearer, b"not msgpack at all \xff\xff", &mut stats).await;
        assert_eq!(stats.malformed, 1);
        assert_eq!(stats.forwarded, 0);
        let got = tokio::time::timeout(Duration::from_millis(50), rx.recv()).await;
        assert!(got.is_err());
    }

    fn relay(host: &Arc<FakeHost>, snapshot: &SharedSnapshot) -> Relay {
        Relay {
            host: host.clone(),
            credentials: CredentialStore { path: None },
            snapshot: snapshot.clone(),
        }
    }

    /// End to end: a relay-role ground station with the relay enabled forwards
    /// an app-stream payload off the aux subscription to the configured compute
    /// URL (no mDNS) and counts it; a payload on another channel is ignored;
    /// stopping leaves an `up: false` snapshot.
    #[tokio::test]
    async fn the_relay_forwards_aux_payloads_while_enabled() {
        let (base, mut rx) = spawn_receiver().await;
        let host = FakeHost::new();
        *host.node_info.lock() = Some(FakeHost::info("ground-station", None, false, Some("relay")));
        host.set_config("relay.enabled", serde_json::json!(true));
        host.set_config("relay.compute_base_url", serde_json::json!(base));
        let snapshot: SharedSnapshot = Arc::default();
        let relay = relay(&host, &snapshot);

        let (stop, shutdown) = watch::channel(false);
        let task = tokio::spawn(relay.run(shutdown));

        host.wait_for(|h| h.aux_callback.lock().is_some()).await;
        let callback = host.aux_callback.lock().clone().unwrap();
        let ev = AtlasEvent::new("atlas.pose", None, vec![7]);
        callback(9, ev.encode().unwrap());
        callback(APP_STREAM, ev.encode().unwrap());
        assert_eq!(rx.recv().await.unwrap(), ev);

        stop.send(true).unwrap();
        task.await.unwrap();
        let body = status_body(snapshot.lock().as_ref());
        assert_eq!(body["up"], false);
        assert_eq!(body["datagrams_seen"], 1);
        assert_eq!(body["forwarded"], 1);
        assert_eq!(body["compute_url"], serde_json::json!(base));
        assert_eq!(body["stale"], false);
    }

    #[tokio::test(start_paused = true)]
    async fn only_the_relay_role_runs_the_relay() {
        let host = FakeHost::new();
        host.set_config("relay.enabled", serde_json::json!(true));
        host.set_config(
            "relay.compute_base_url",
            serde_json::json!("http://192.0.2.9:8092"),
        );
        let snapshot: SharedSnapshot = Arc::default();
        let (_stop, shutdown) = watch::channel(false);
        tokio::spawn(relay(&host, &snapshot).run(shutdown));

        // A receiver-role ground station, then no node facts at all: enabled
        // or not, nothing is subscribed or started.
        *host.node_info.lock() = Some(FakeHost::info(
            "ground-station",
            None,
            false,
            Some("receiver"),
        ));
        tokio::time::sleep(GATE_POLL * 2).await;
        *host.node_info.lock() = None;
        tokio::time::sleep(GATE_POLL * 2).await;
        assert!(host.aux_callback.lock().is_none());
        assert!(snapshot.lock().is_none());

        // Switched to the relay role: the next poll starts it.
        *host.node_info.lock() = Some(FakeHost::info("ground-station", None, false, Some("relay")));
        tokio::time::sleep(GATE_POLL * 2).await;
        assert!(host.aux_callback.lock().is_some());
        assert_eq!(status_body(snapshot.lock().as_ref())["up"], true);
    }

    #[tokio::test(start_paused = true)]
    async fn a_stale_snapshot_nulls_its_keys_and_no_snapshot_is_just_stale() {
        assert_eq!(status_body(None), serde_json::json!({"stale": true}));

        let snap = (
            RelaySnapshot::new(&RelayStats::default(), "http://compute.local:8092", true),
            Instant::now(),
        );
        let fresh = status_body(Some(&snap));
        assert_eq!(fresh["up"], true);
        assert_eq!(fresh["stale"], false);
        let mut keys: Vec<String> = fresh.as_object().unwrap().keys().cloned().collect();
        keys.sort();
        assert_eq!(
            keys,
            [
                "compute_url",
                "datagrams_seen",
                "forward_failed",
                "forwarded",
                "generated_at_ms",
                "listen_port",
                "malformed",
                "stale",
                "up",
            ]
        );

        tokio::time::advance(SNAPSHOT_FRESH + Duration::from_millis(1)).await;
        let stale = status_body(Some(&snap));
        assert_eq!(stale["stale"], true);
        assert!(stale["up"].is_null());
        assert!(stale["forwarded"].is_null());
        assert_eq!(stale.as_object().unwrap().len(), keys.len());
    }
}
