//! The drone-side perception offload reconciler.
//!
//! An NPU-less drone runs its detection on a workstation instead of on board.
//! Each tick, when the node facts (`node.info`) and plugin config
//! `offload.enabled` call for offload, the camera is up, and a
//! `profile=workstation` node is reachable on the LAN, this loop starts and
//! supervises the offload orchestrator: the drone streams its RTSP camera to the
//! node, the node runs the detector, and detections return onto the drone's own
//! `vision.detection` bus (transparent to every consumer). While a session is
//! live it re-advertises the offload link to the host (`offload.advertise`) so
//! the node's perception tier reads `offload`; when the session stops it
//! advertises `paired: false` once.
//!
//! `offload.enabled`: `auto` offloads only when the board has no NPU, `on`
//! always, `off` never. A board the node has not probed (`board: None`) counts
//! as NPU-less, the same reading the agent's own perception-tier decision makes
//! of an absent board fingerprint (`npu_tops` 0), so `auto` offloads there too.
//! Either way the camera must report ready: the node can only pull frames the
//! drone is actually publishing. With no node facts at all (an older host, or
//! the read failed) nothing is known about the camera, so nothing is offloaded.
//!
//! Local-first: the node is the pinned `offload.compute_node_addr` or an mDNS
//! advert (see [`crate::compute_node`]), reached by its LAN job-API address; no cloud
//! round-trip. The drone presents the credential that node issued it; discovery
//! prefers a workstation that issued one. The RTSP URL handed to the node is the
//! drone's LAN egress IP (never `localhost`: the node pulls the feed).

use std::net::{IpAddr, ToSocketAddrs, UdpSocket};
use std::sync::Arc;
use std::time::{Duration, Instant};

use ados_sdk::OffloadAdvertisement;
use tokio::sync::{watch, Notify};
use tokio::task::JoinHandle;
use world_engine_transport::{
    run_offload_orchestrator, DetectionTee, NodeEndpoint, OrchestratorConfig,
};

use crate::compute_node;
use crate::credentials::{CredentialStore, OFFLOAD_STREAM_LANE};
use crate::host::{config_json, node_info, Host};
use crate::sdk::{CloudDetectionTee, SdkDetectionPublisher};
use ados_protocol::node_info::NodeInfo;

/// The reconcile cadence, which is also the re-advertise cadence: comfortably
/// under the host's 20 s offload-link staleness window.
const TICK: Duration = Duration::from_secs(5);
/// While searching (no live session), do not re-browse more often than this.
const SEARCH_BACKOFF: Duration = Duration::from_secs(15);
/// The RTSP port + main path the drone's encoder publishes to.
const RTSP_PORT: u16 = 8554;
const RTSP_PATH: &str = "main";
/// The camera the offloaded detections are attributed to (the drone's primary).
const CAMERA_ID: &str = "front";
/// The fixed frame size the node decodes the feed at (the node scales the
/// camera's own size to it).
const FRAME_WIDTH: u32 = 1280;
const FRAME_HEIGHT: u32 = 720;
/// The freshness budget (ms) for the returned detection stream: past this with
/// no new batch, the safety gate trips the lock to Lost.
const TARGET_BUDGET_MS: i64 = 700;
/// The model id advertised for the offload session.
const OFFLOAD_MODEL_ID: &str = "offload";

/// The live offload session the reconciler owns.
struct RunningSession {
    cancel: Arc<Notify>,
    handle: JoinHandle<()>,
    /// The node address this session offloads to (`host:port`).
    target: String,
    /// The node's device id (from discovery).
    device_id: Option<String>,
}

/// A resolved node to offload to.
struct Target {
    base_url: String,
    target: String,
    node_device_id: Option<String>,
    credential: Option<String>,
    rtsp_url: String,
}

/// The operator's `offload.enabled` setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum OffloadMode {
    Auto,
    On,
    Off,
}

impl OffloadMode {
    /// `off` / `false` and `on` / `true`; anything else (absent included) is
    /// `auto`.
    fn parse(value: &serde_json::Value) -> Self {
        match value {
            serde_json::Value::Bool(false) => Self::Off,
            serde_json::Value::Bool(true) => Self::On,
            v => match v.as_str().map(str::trim) {
                Some("off") => Self::Off,
                Some("on") => Self::On,
                _ => Self::Auto,
            },
        }
    }
}

/// Whether the drone should offload (start, or keep a live session): the camera
/// is ready, and the mode is `on`, or `auto` on a board with no NPU. Unknown node
/// facts (`None`) keep it off, since camera readiness is not known; an unprobed
/// board (no board facts) counts as NPU-less.
fn wants_offload(mode: OffloadMode, info: Option<&NodeInfo>) -> bool {
    let Some(info) = info else {
        return false;
    };
    if !info.camera.ready {
        return false;
    }
    match mode {
        OffloadMode::Off => false,
        OffloadMode::On => true,
        OffloadMode::Auto => !info.board.as_ref().is_some_and(|b| b.has_npu),
    }
}

/// The drone's egress IP toward `host:port`: the source address a connection to
/// the node would use, i.e. the address the node reaches the RTSP feed on. A
/// UDP "connect" picks the source IP without sending a packet. `None` when the
/// address does not resolve or no route exists.
fn local_ip_towards(host: &str, port: u16) -> Option<IpAddr> {
    let addr = format!("{host}:{port}").to_socket_addrs().ok()?.next()?;
    let sock = UdpSocket::bind(("0.0.0.0", 0)).ok()?;
    sock.connect(addr).ok()?;
    sock.local_addr().ok().map(|a| a.ip())
}

/// Everything the reconciler needs.
pub struct Reconciler {
    pub host: Arc<dyn Host>,
    /// Names the offload session (`offload-<agent id>`).
    pub agent_id: String,
    pub credentials: CredentialStore,
}

impl Reconciler {
    /// Run until `shutdown` flips.
    pub async fn run(self, mut shutdown: watch::Receiver<bool>) {
        tracing::info!("offload reconciler armed");
        let mut ticker = tokio::time::interval(TICK);
        ticker.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Delay);
        let mut running: Option<RunningSession> = None;
        let mut last_search: Option<Instant> = None;
        // Built on the first session and reused: one drain task for the process.
        let mut tee: Option<Arc<CloudDetectionTee>> = None;

        loop {
            tokio::select! {
                r = shutdown.changed() => {
                    if r.is_err() || *shutdown.borrow() { break; }
                    continue;
                }
                _ = ticker.tick() => {}
            }
            // A finished orchestrator (the node went away, the stream ended)
            // frees the slot so the next search re-resolves.
            if running.as_ref().is_some_and(|s| s.handle.is_finished()) {
                if let Some(sess) = running.take() {
                    tracing::info!(target = %sess.target, "offload session ended");
                }
                self.advertise(None).await;
            }
            let mode =
                OffloadMode::parse(&config_json(self.host.as_ref(), "offload.enabled").await);
            let info = node_info(self.host.as_ref()).await;
            let enabled = wants_offload(mode, info.as_ref());

            if let Some(sess) = running.as_ref() {
                // A live session is kept on the cheap gate alone, never a
                // re-resolve: a transient mDNS miss must not tear it down.
                if enabled {
                    self.advertise(Some(sess)).await;
                } else if let Some(sess) = running.take() {
                    tracing::info!(target = %sess.target, "offload reconciler: stopping the session");
                    // notify_one stores a permit, so the stop is observed even
                    // while the orchestrator is still submitting / opening the
                    // WebSocket and not yet parked on the cancel.
                    sess.cancel.notify_one();
                    self.advertise(None).await;
                }
                continue;
            }
            if !enabled || !last_search.is_none_or(|t| t.elapsed() >= SEARCH_BACKOFF) {
                continue;
            }
            last_search = Some(Instant::now());
            let Some(target) = self.resolve_target().await else {
                continue;
            };
            let tee = tee
                .get_or_insert_with(|| Arc::new(CloudDetectionTee::spawn(self.host.clone())))
                .clone();
            let cfg = OrchestratorConfig::vision_only(
                format!("offload-{}", self.agent_id),
                CAMERA_ID,
                target.rtsp_url,
                FRAME_WIDTH,
                FRAME_HEIGHT,
                TARGET_BUDGET_MS,
                Arc::new(SdkDetectionPublisher::new(self.host.clone())),
            )
            .with_detection_tee(Some(tee as Arc<dyn DetectionTee>));
            let endpoint = NodeEndpoint {
                base_url: target.base_url,
                credential: target.credential,
            };
            let cancel = Arc::new(Notify::new());
            let cancel_task = cancel.clone();
            tracing::info!(
                target = %target.target,
                camera = ?info.as_ref().and_then(|i| i.camera.main),
                "offload reconciler: starting a session"
            );
            let handle = tokio::spawn(async move {
                if let Err(e) = run_offload_orchestrator(cfg, endpoint, cancel_task).await {
                    tracing::warn!(error = %e, "offload orchestrator ended");
                }
            });
            let sess = RunningSession {
                cancel,
                handle,
                target: target.target,
                device_id: target.node_device_id,
            };
            self.advertise(Some(&sess)).await;
            running = Some(sess);
        }

        if let Some(sess) = running.take() {
            sess.cancel.notify_one();
            self.advertise(None).await;
        }
    }

    /// Resolve the node to offload to (a pinned address, else mDNS) and the
    /// drone's egress IP toward it.
    async fn resolve_target(&self) -> Option<Target> {
        let node =
            compute_node::resolve(self.host.as_ref(), &self.credentials, &OFFLOAD_STREAM_LANE)
                .await?;
        let local_ip = local_ip_towards(&node.host, node.port)?;
        // The credential THIS node issued, or the sole one for a pinned address.
        let credential = node.credential(&self.credentials, &OFFLOAD_STREAM_LANE);
        if credential.is_none() {
            tracing::info!(node = %node.addr(), "offload reconciler: no credential from this workstation; a paired workstation will refuse the session");
        }
        Some(Target {
            base_url: node.base_url(),
            target: node.addr(),
            node_device_id: node.device_id,
            credential,
            rtsp_url: format!("rtsp://{local_ip}:{RTSP_PORT}/{RTSP_PATH}"),
        })
    }

    /// Advertise the live session's link, or `paired: false` for none. A
    /// resolved, reachable node is treated as an acceptable LAN bearer.
    async fn advertise(&self, session: Option<&RunningSession>) {
        let advert = match session {
            Some(s) => OffloadAdvertisement {
                paired: true,
                bearer_acceptable: true,
                target: Some(s.target.clone()),
                device_id: s.device_id.clone().filter(|d| !d.is_empty()),
                model_id: Some(OFFLOAD_MODEL_ID.to_string()),
            },
            None => OffloadAdvertisement::default(),
        };
        if let Err(e) = self.host.advertise_offload(&advert).await {
            tracing::debug!(error = %e, "offload advertise failed");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testing::FakeHost;
    use serde_json::json;

    #[test]
    fn auto_offloads_only_when_npu_less() {
        let npu = FakeHost::info("drone", Some(true), true, None);
        let npu_less = FakeHost::info("drone", Some(false), true, None);
        let unprobed = FakeHost::info("drone", None, true, None);
        // auto + an accelerator: run local.
        assert!(!wants_offload(OffloadMode::Auto, Some(&npu)));
        // auto + no accelerator (or none declared): offload.
        assert!(wants_offload(OffloadMode::Auto, Some(&npu_less)));
        assert!(wants_offload(OffloadMode::Auto, Some(&unprobed)));
        // on forces offload even with an accelerator; off never offloads.
        assert!(wants_offload(OffloadMode::On, Some(&npu)));
        assert!(!wants_offload(OffloadMode::Off, Some(&npu_less)));
    }

    #[test]
    fn offload_waits_for_a_ready_camera() {
        let not_ready = FakeHost::info("drone", Some(false), false, None);
        assert!(!wants_offload(OffloadMode::On, Some(&not_ready)));
        assert!(!wants_offload(OffloadMode::Auto, Some(&not_ready)));
        // No node facts at all: readiness is unknown, so no offload.
        assert!(!wants_offload(OffloadMode::On, None));
    }

    #[test]
    fn the_mode_reads_off_on_and_defaults_to_auto() {
        assert_eq!(OffloadMode::parse(&json!("off")), OffloadMode::Off);
        assert_eq!(OffloadMode::parse(&json!(false)), OffloadMode::Off);
        assert_eq!(OffloadMode::parse(&json!("on")), OffloadMode::On);
        assert_eq!(OffloadMode::parse(&json!("auto")), OffloadMode::Auto);
        assert_eq!(OffloadMode::parse(&json!(null)), OffloadMode::Auto);
    }

    #[test]
    fn local_ip_towards_a_loopback_node_is_loopback() {
        // No packet is sent; the call only picks the source address.
        assert_eq!(
            local_ip_towards("127.0.0.1", 8092),
            Some(IpAddr::from([127, 0, 0, 1]))
        );
        // An unresolvable host yields None (never a fabricated reach).
        assert!(local_ip_towards("not a host", 80).is_none());
    }
}
