//! The drone-side Atlas forwarder.
//!
//! The capture service publishes pose-tagged keyframes, the live pose and the
//! capture state onto its private bus (`atlas.sock`, one length-prefixed framed
//! [`AtlasEvent`] per broadcast). This loop is their egress: it subscribes to
//! the bus and forwards every event over the bearer ladder, direct LAN first,
//! then the WFB relay for the field, then the plugin's cloud stream.
//!
//! The compute node is the pinned `offload.compute_node_addr` when set, else an
//! mDNS `profile=workstation` advert preferring one that issued this drone a
//! credential (see [`crate::compute_node`]); its job-API base URL backs the
//! direct-LAN bearer, which presents the credential that node issued.
//! When the LAN bearer stops carrying, the loop re-resolves and rebuilds the
//! ladder; while no LAN bearer is present it periodically re-browses so a node
//! that boots after the drone is still picked up.
//!
//! Inert while Atlas is disabled or the camera is not ready: it re-reads the
//! `atlas.enabled` plugin config and the `node.info` camera readiness
//! on a fixed interval before every bus connection and does no Atlas work while
//! it is off.
//!
//! The transport facts only this loop knows (the resolved node, the carrying
//! bearer, the last forwarded keyframe) live in the shared [`ForwardStatus`],
//! which the `status` telemetry and the readiness route read.

use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use ados_protocol::frame::PLUGIN_MAX_FRAME;
use ados_protocol::ipc::{connect_with_retry, read_length_prefixed};
use parking_lot::Mutex;
use tokio::sync::watch;
use world_engine_protocol::atlas::{
    AtlasForwardStatus, ATLAS_FORWARD_STATUS_VERSION, ATLAS_KEYFRAME_TOPIC,
};
use world_engine_transport::{
    AtlasBearer, AtlasEvent, BearerKind, BearerLadder, LanHttpBearer, WfbRelayBearer,
};

use crate::compute_node;
use crate::credentials::{CredentialStore, ATLAS_INGEST_LANE};
use crate::host::{config_bool, node_info, Host};
use crate::sdk::{CloudPublishBearer, SdkAuxLane};
use crate::sleep_or_shutdown;

/// Backoff before reconnecting the bus after an EOF / read error.
const RECONNECT_DELAY: Duration = Duration::from_secs(2);
/// While no direct-LAN bearer is present, re-browse for a compute node no more
/// often than this.
const RERESOLVE_INTERVAL: Duration = Duration::from_secs(30);
/// Consecutive sends that did NOT ride the LAN bearer before the LAN bearer is
/// treated as gone and the node is re-resolved.
const LAN_MISS_THRESHOLD: u32 = 5;
/// Connect-retry budget for the bus: the capture service may still be binding
/// the socket on a cold boot.
pub const CONNECT_RETRIES: u32 = 30;
pub const CONNECT_RETRY_DELAY: Duration = Duration::from_millis(500);
/// How often a disabled forwarder re-reads the Atlas gate.
const GATE_POLL: Duration = Duration::from_secs(5);

/// The forwarder's transport facts, shared with the status writer and the
/// readiness route.
pub type SharedForward = Arc<Mutex<ForwardStatus>>;

/// Outcome of one forward attempt.
enum Forwarded {
    /// The event decoded; `carried` is the bearer that carried it (`None` if
    /// every bearer declined), `keyframe` whether it was a keyframe.
    Decoded {
        carried: Option<BearerKind>,
        keyframe: bool,
    },
    /// The framed body was not a valid event: logged, nothing sent (not a
    /// transport miss, so it never re-resolves the node).
    DecodeError,
}

/// Epoch milliseconds.
pub fn now_ms() -> i64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_millis() as i64)
        .unwrap_or(0)
}

/// Map a bearer kind to the GCS Stream-card vocabulary.
fn bearer_label(kind: BearerKind) -> &'static str {
    match kind {
        // Loopback is the same-host LAN-direct case; the live ladder only ever
        // carries over DirectLan / WfbRelay / Cloud on real hardware.
        BearerKind::DirectLan | BearerKind::Loopback => "direct-lan",
        BearerKind::WfbRelay => "wfb-relay",
        BearerKind::Cloud => "cloud",
    }
}

/// The transport facts only the forwarder knows.
#[derive(Debug, Default)]
pub struct ForwardStatus {
    compute_node_id: Option<String>,
    bearer: Option<String>,
    last_kf_at_ms: Option<i64>,
}

impl ForwardStatus {
    /// Record the resolved compute node (or its loss). An empty advertised id
    /// counts as none; losing the node clears the bearer (nothing is carrying
    /// anymore).
    pub(crate) fn note_node(&mut self, node_id: Option<String>) {
        let node_id = node_id.filter(|s| !s.is_empty());
        if node_id.is_none() {
            self.bearer = None;
        }
        self.compute_node_id = node_id;
    }

    /// Record a forwarded event's carrying bearer (+ keyframe time). A declined
    /// send (`None`) keeps the last-known bearer so a transient decline does not
    /// flicker it.
    fn note_carried(&mut self, bearer: Option<&'static str>, keyframe: bool, now_ms: i64) {
        if let Some(b) = bearer {
            if self.bearer.as_deref() != Some(b) {
                self.bearer = Some(b.to_string());
            }
        }
        if keyframe {
            self.last_kf_at_ms = Some(now_ms);
        }
    }

    /// The resolved compute node, if any.
    pub fn compute_node_id(&self) -> Option<&str> {
        self.compute_node_id.as_deref()
    }

    /// The facts as the wire struct, stamped `generated_at_ms`.
    pub fn snapshot(&self, generated_at_ms: i64) -> AtlasForwardStatus {
        AtlasForwardStatus {
            version: ATLAS_FORWARD_STATUS_VERSION,
            compute_node_id: self.compute_node_id.clone(),
            bearer: self.bearer.clone(),
            last_kf_at_ms: self.last_kf_at_ms,
            generated_at_ms,
        }
    }
}

/// Everything the forwarder loop needs.
pub struct Forwarder {
    pub host: Arc<dyn Host>,
    /// Stamped on every egressing event (empty leaves it unstamped).
    pub device_id: String,
    pub credentials: CredentialStore,
    pub bus_socket: PathBuf,
    pub status: SharedForward,
}

impl Forwarder {
    /// Run until `shutdown` flips. Inert while Atlas is disabled or the camera
    /// is not ready (`node.info`).
    pub async fn run(self, mut shutdown: watch::Receiver<bool>) {
        tracing::info!("atlas forwarder starting");
        // One cloud rung for the whole run, so its relay-down back-off survives
        // ladder rebuilds.
        let cloud = CloudPublishBearer::new(self.host.clone());
        let mut was_enabled = false;
        loop {
            if *shutdown.borrow() {
                break;
            }
            let atlas_enabled = config_bool(self.host.as_ref(), "atlas.enabled", false).await;
            let camera_ready = atlas_enabled
                && node_info(self.host.as_ref())
                    .await
                    .is_some_and(|i| i.camera.ready);
            let enabled = atlas_enabled && camera_ready;
            if enabled != was_enabled {
                tracing::info!(atlas_enabled, camera_ready, "atlas forwarder gate changed");
                was_enabled = enabled;
            }
            if !enabled {
                // Nothing is being forwarded, so no compute node is reported.
                self.status.lock().note_node(None);
                if sleep_or_shutdown(&mut shutdown, GATE_POLL).await {
                    break;
                }
                continue;
            }
            let mut stream =
                match connect_with_retry(&self.bus_socket, CONNECT_RETRIES, CONNECT_RETRY_DELAY)
                    .await
                {
                    Ok(s) => s,
                    Err(e) => {
                        tracing::debug!(error = %e, "atlas bus not reachable; retrying");
                        if sleep_or_shutdown(&mut shutdown, RECONNECT_DELAY).await {
                            break;
                        }
                        continue;
                    }
                };
            tracing::info!(socket = %self.bus_socket.display(), "atlas forwarder subscribed to the bus");

            let (mut ladder, node_id) = self.build_ladder(&cloud).await;
            let mut have_lan = node_id.is_some();
            self.status.lock().note_node(node_id);
            let mut lan_misses: u32 = 0;
            let mut last_resolve = Instant::now();

            // The read is raced ONLY against shutdown, so a partial read is never
            // cancelled mid-frame by a timer.
            loop {
                let read = tokio::select! {
                    r = shutdown.changed() => {
                        if r.is_err() || *shutdown.borrow() { return; }
                        continue;
                    }
                    r = read_length_prefixed(&mut stream, PLUGIN_MAX_FRAME, false) => r,
                };
                match read {
                    Ok(Some(body)) => {
                        if let Forwarded::Decoded { carried, keyframe } =
                            forward_event(&ladder, &body, &self.device_id).await
                        {
                            if have_lan {
                                if carried == Some(BearerKind::DirectLan) {
                                    lan_misses = 0;
                                } else {
                                    lan_misses += 1;
                                }
                            }
                            self.status.lock().note_carried(
                                carried.map(bearer_label),
                                keyframe,
                                now_ms(),
                            );
                        }
                        // Re-resolve when the LAN bearer stopped carrying, or
                        // periodically while there is none. Done between whole
                        // frames so it never cancels a partial read.
                        let lan_lost = have_lan && lan_misses >= LAN_MISS_THRESHOLD;
                        let want_discover =
                            !have_lan && last_resolve.elapsed() >= RERESOLVE_INTERVAL;
                        if lan_lost || want_discover {
                            if lan_lost {
                                tracing::info!(
                                    "atlas LAN bearer stopped carrying; re-resolving compute node"
                                );
                            }
                            let (l, node_id) = self.build_ladder(&cloud).await;
                            let lan = node_id.is_some();
                            if lan && !have_lan {
                                tracing::info!("atlas forwarder discovered a compute node");
                            }
                            self.status.lock().note_node(node_id);
                            ladder = l;
                            have_lan = lan;
                            lan_misses = 0;
                            last_resolve = Instant::now();
                        }
                    }
                    Ok(None) => {
                        tracing::debug!("atlas bus closed; reconnecting");
                        break;
                    }
                    Err(e) => {
                        tracing::debug!(error = %e, "atlas bus read error; reconnecting");
                        break;
                    }
                }
            }
            if sleep_or_shutdown(&mut shutdown, RECONNECT_DELAY).await {
                break;
            }
        }
        tracing::info!("atlas forwarder stopped");
    }

    /// Resolve the compute node (a pinned address, else mDNS) and build the
    /// ladder. Returns the ladder and the node's id (`Some` iff a direct-LAN
    /// bearer is present): its advertised device id, or `host:port` for a pinned
    /// address. The credential stores are re-read on every rebuild, so one
    /// installed after boot is picked up on the next re-resolve.
    async fn build_ladder(&self, cloud: &CloudPublishBearer) -> (BearerLadder, Option<String>) {
        let mut bearers: Vec<Box<dyn AtlasBearer>> = Vec::new();
        let node_id = match compute_node::resolve(
            self.host.as_ref(),
            &self.credentials,
            &ATLAS_INGEST_LANE,
        )
        .await
        {
            Some(node) => {
                let base = node.base_url();
                tracing::info!(base = %base, node = ?node.device_id, "atlas forwarder resolved compute node");
                let credential = node.credential(&self.credentials, &ATLAS_INGEST_LANE);
                bearers.push(Box::new(LanHttpBearer::new(base, credential)));
                let addr = node.addr();
                Some(node.device_id.unwrap_or(addr))
            }
            None => {
                tracing::debug!("atlas forwarder: no compute node yet");
                None
            }
        };
        bearers.push(Box::new(WfbRelayBearer::new(Arc::new(SdkAuxLane::new(
            self.host.clone(),
        )))));
        bearers.push(Box::new(cloud.clone()));
        (BearerLadder::new(bearers), node_id)
    }
}

/// Decode one framed [`AtlasEvent`] body, stamp the capturing drone's device id,
/// and send it over the ladder. Never panics, never propagates: a forward
/// failure is logged and the loop continues.
///
/// The device id is stamped here, the single egress choke point every bearer
/// passes through, so the compute node can attribute the reconstruct job to the
/// drone that captured it. An empty id leaves the event unstamped.
async fn forward_event(ladder: &BearerLadder, body: &[u8], device_id: &str) -> Forwarded {
    let mut event = match AtlasEvent::decode(body) {
        Ok(ev) => ev,
        Err(e) => {
            tracing::warn!(error = %e, "atlas forwarder dropped a malformed event");
            return Forwarded::DecodeError;
        }
    };
    if !device_id.is_empty() {
        event.device_id = Some(device_id.to_string());
    }
    let keyframe = event.topic == ATLAS_KEYFRAME_TOPIC;
    match ladder.send(&event).await {
        Ok(kind) => {
            tracing::debug!(topic = %event.topic, bearer = ?kind, bytes = event.payload.len(), "atlas event forwarded");
            Forwarded::Decoded {
                carried: Some(kind),
                keyframe,
            }
        }
        Err(e) => {
            tracing::debug!(topic = %event.topic, error = %e, "atlas event not forwarded (all bearers declined)");
            Forwarded::Decoded {
                carried: None,
                keyframe,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use world_engine_transport::LoopbackBearer;

    fn carried_kind(f: &Forwarded) -> Option<BearerKind> {
        match f {
            Forwarded::Decoded { carried, .. } => *carried,
            Forwarded::DecodeError => None,
        }
    }

    #[tokio::test]
    async fn forward_event_carries_a_decoded_event_stamped_with_the_device_id() {
        let (bearer, mut rx) = LoopbackBearer::channel();
        let ladder = BearerLadder::new(vec![Box::new(bearer)]);
        let ev = AtlasEvent::new(ATLAS_KEYFRAME_TOPIC, None, vec![1, 2, 3]);
        let body = ev.encode().unwrap();

        match forward_event(&ladder, &body, "drone-7").await {
            Forwarded::Decoded {
                carried: Some(BearerKind::Loopback),
                keyframe: true,
            } => {}
            other => panic!(
                "expected a loopback-carried keyframe, got {:?}",
                carried_kind(&other)
            ),
        }
        let got = rx
            .try_recv()
            .expect("the loopback bearer carried the event");
        assert_eq!(got.topic, ev.topic);
        assert_eq!(got.payload, ev.payload);
        assert_eq!(got.device_id.as_deref(), Some("drone-7"));
    }

    #[tokio::test]
    async fn an_empty_device_id_leaves_the_event_unstamped() {
        let (bearer, mut rx) = LoopbackBearer::channel();
        let ladder = BearerLadder::new(vec![Box::new(bearer)]);
        let body = AtlasEvent::new("atlas.capture.state", None, vec![1])
            .encode()
            .unwrap();
        forward_event(&ladder, &body, "").await;
        assert_eq!(rx.try_recv().unwrap().device_id, None);
    }

    #[tokio::test]
    async fn forward_event_reports_a_decode_error_for_a_malformed_body() {
        let (bearer, mut rx) = LoopbackBearer::channel();
        let ladder = BearerLadder::new(vec![Box::new(bearer)]);
        assert!(matches!(
            forward_event(&ladder, b"not-an-event", "drone-7").await,
            Forwarded::DecodeError
        ));
        assert!(rx.try_recv().is_err(), "no event should reach the bearer");
    }

    #[tokio::test]
    async fn forward_event_reports_no_bearer_when_the_ladder_is_empty() {
        let ladder = BearerLadder::new(vec![]);
        let body = AtlasEvent::new("plugin.world-engine.pose", None, vec![9])
            .encode()
            .unwrap();
        assert!(matches!(
            forward_event(&ladder, &body, "drone-7").await,
            Forwarded::Decoded {
                carried: None,
                keyframe: false
            }
        ));
    }

    #[test]
    fn losing_the_node_clears_the_bearer() {
        let mut fwd = ForwardStatus::default();
        fwd.note_node(Some("rtx-box".into()));
        fwd.note_carried(Some("direct-lan"), false, 100);
        assert_eq!(fwd.snapshot(0).bearer.as_deref(), Some("direct-lan"));
        // Losing the node clears the (now meaningless) bearer.
        fwd.note_node(None);
        let snap = fwd.snapshot(0);
        assert_eq!(snap.compute_node_id, None);
        assert_eq!(snap.bearer, None);
        // An empty advertised id is treated as no node.
        fwd.note_node(Some(String::new()));
        assert_eq!(fwd.compute_node_id(), None);
    }

    #[test]
    fn note_carried_tracks_the_bearer_and_keyframe_time() {
        let mut fwd = ForwardStatus::default();
        fwd.note_carried(Some("wfb-relay"), true, 1_700);
        fwd.note_carried(Some("wfb-relay"), false, 1_800);
        let snap = fwd.snapshot(0);
        assert_eq!(snap.bearer.as_deref(), Some("wfb-relay"));
        // Only a keyframe advances the keyframe time.
        assert_eq!(snap.last_kf_at_ms, Some(1_700));
        fwd.note_carried(Some("cloud"), true, 1_900);
        // A declined send keeps the last-known bearer.
        fwd.note_carried(None, false, 2_000);
        let snap = fwd.snapshot(0);
        assert_eq!(snap.bearer.as_deref(), Some("cloud"));
        assert_eq!(snap.last_kf_at_ms, Some(1_900));
    }

    #[test]
    fn bearer_label_maps_to_the_gcs_vocabulary() {
        assert_eq!(bearer_label(BearerKind::DirectLan), "direct-lan");
        assert_eq!(bearer_label(BearerKind::WfbRelay), "wfb-relay");
        assert_eq!(bearer_label(BearerKind::Cloud), "cloud");
    }
}
