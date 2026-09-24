//! Which compute node a drone lane talks to.
//!
//! A pinned address (plugin config `offload.compute_node_addr`, `host:port` or a
//! bare host) wins over mDNS, for segmented networks where multicast does not
//! reach the node. Otherwise the node is a `profile=workstation` advert,
//! preferring one that issued this drone a credential. The Atlas forwarder and
//! the offload reconciler share this rule.

use std::future::Future;
use std::time::Duration;

use world_engine_transport::{resolve_compute, ResolvedComputeNode};

use crate::credentials::CredentialStore;
use crate::host::{config_string, Host};
use world_engine_protocol::node_credential::NodeLane;

/// The plugin config key that pins the compute node.
pub const PIN_KEY: &str = "offload.compute_node_addr";
/// The compute node's default job-API port (used when a pinned addr omits one).
pub const DEFAULT_JOB_API_PORT: u16 = 8092;
/// One mDNS browse's timeout.
const RESOLVE_TIMEOUT: Duration = Duration::from_secs(3);

/// A compute node to reach.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ComputeNode {
    pub host: String,
    pub port: u16,
    /// The node's advertised device id; `None` for a pinned address, which
    /// carries no id.
    pub device_id: Option<String>,
}

impl ComputeNode {
    /// `host:port`.
    pub fn addr(&self) -> String {
        format!("{}:{}", self.host, self.port)
    }

    /// The job-API base URL.
    pub fn base_url(&self) -> String {
        format!("http://{}", self.addr())
    }

    /// The credential to present on `lane`: the one this node issued, or for a
    /// pinned address (no node id) the sole credential installed for the lane.
    pub fn credential(&self, stores: &CredentialStore, lane: &NodeLane) -> Option<String> {
        stores.lookup(self.device_id.as_deref(), lane)
    }
}

/// Split a `host:port` (or bare `host`) into `(host, port)`, defaulting the port.
/// An IPv6 literal in brackets keeps its colons.
pub fn split_host_port(s: &str, default_port: u16) -> Option<(String, u16)> {
    let s = s.trim();
    if s.is_empty() {
        return None;
    }
    if let Some(rest) = s.strip_prefix('[') {
        let (host, after) = rest.split_once(']')?;
        let port = after
            .strip_prefix(':')
            .and_then(|p| p.parse().ok())
            .unwrap_or(default_port);
        return Some((host.to_string(), port));
    }
    match s.rsplit_once(':') {
        // A single colon = host:port; more than one and unbracketed = a bare IPv6.
        Some((h, p)) if !h.contains(':') => {
            let port = p.parse().ok().unwrap_or(default_port);
            Some((h.to_string(), port))
        }
        _ => Some((s.to_string(), default_port)),
    }
}

/// The node to use: the pinned address when one is configured (no browse), else
/// the result of `browse`.
async fn resolve_with<F, Fut>(host: &dyn Host, browse: F) -> Option<ComputeNode>
where
    F: FnOnce() -> Fut,
    Fut: Future<Output = Option<ResolvedComputeNode>>,
{
    if let Some(pinned) = config_string(host, PIN_KEY).await {
        let (h, p) = split_host_port(&pinned, DEFAULT_JOB_API_PORT)?;
        return Some(ComputeNode {
            host: h,
            port: p,
            device_id: None,
        });
    }
    browse().await.map(|n| ComputeNode {
        host: n.host,
        port: n.job_api_port,
        device_id: Some(n.device_id),
    })
}

/// The node to use on `lane`: pinned, else discovered over mDNS preferring a
/// workstation that issued this drone a credential for the lane.
pub async fn resolve(
    host: &dyn Host,
    stores: &CredentialStore,
    lane: &NodeLane,
) -> Option<ComputeNode> {
    resolve_with(host, || async {
        resolve_compute(RESOLVE_TIMEOUT, &stores.issuers(lane)).await
    })
    .await
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::credentials::{ATLAS_INGEST_LANE, STORE_FILE};
    use crate::testing::FakeHost;
    use serde_json::json;
    use world_engine_protocol::node_credential::{InstalledCredential, WorkstationCredentials};

    fn discovered() -> Option<ResolvedComputeNode> {
        Some(ResolvedComputeNode {
            host: "192.0.2.9".into(),
            job_api_port: 8092,
            device_id: "ws-mdns".into(),
        })
    }

    #[tokio::test]
    async fn a_pinned_address_wins_over_mdns() {
        let host = FakeHost::new();
        host.set_config(PIN_KEY, json!("192.0.2.5:9000"));
        let browsed = std::sync::atomic::AtomicBool::new(false);
        let node = resolve_with(host.as_ref(), || async {
            browsed.store(true, std::sync::atomic::Ordering::SeqCst);
            discovered()
        })
        .await
        .unwrap();
        assert!(!browsed.into_inner(), "a pinned node must not browse");
        assert_eq!(node.base_url(), "http://192.0.2.5:9000");
        assert_eq!(node.device_id, None);

        // A bare host takes the default job-API port.
        host.set_config(PIN_KEY, json!("192.0.2.5"));
        let node = resolve_with(host.as_ref(), || async { discovered() })
            .await
            .unwrap();
        assert_eq!(node.addr(), "192.0.2.5:8092");
    }

    #[tokio::test]
    async fn without_a_pin_the_mdns_node_is_used() {
        let host = FakeHost::new();
        // An empty pin is no pin.
        host.set_config(PIN_KEY, json!("  "));
        let node = resolve_with(host.as_ref(), || async { discovered() })
            .await
            .unwrap();
        assert_eq!(node.base_url(), "http://192.0.2.9:8092");
        assert_eq!(node.device_id.as_deref(), Some("ws-mdns"));
        assert_eq!(resolve_with(host.as_ref(), || async { None }).await, None);
    }

    #[test]
    fn a_pinned_node_presents_the_sole_credential_for_the_lane() {
        let dir = tempfile::tempdir().unwrap();
        let ext = dir.path().join(STORE_FILE);
        let mut store = WorkstationCredentials::default();
        store.upsert(InstalledCredential {
            workstation_node_id: "ws-a".into(),
            credential: "secret-a".into(),
            lanes: vec![ATLAS_INGEST_LANE],
            installed_at_ms: 1,
        });
        store.save(&ext).unwrap();
        let stores = CredentialStore {
            path: Some(ext.clone()),
        };
        let pinned = ComputeNode {
            host: "192.0.2.5".into(),
            port: 8092,
            device_id: None,
        };
        assert_eq!(
            pinned.credential(&stores, &ATLAS_INGEST_LANE).as_deref(),
            Some("secret-a")
        );
        // With two installed there is no sole one to guess from.
        store.upsert(InstalledCredential {
            workstation_node_id: "ws-b".into(),
            credential: "secret-b".into(),
            lanes: vec![ATLAS_INGEST_LANE],
            installed_at_ms: 2,
        });
        store.save(&ext).unwrap();
        assert_eq!(pinned.credential(&stores, &ATLAS_INGEST_LANE), None);
    }

    #[test]
    fn split_host_port_defaults_and_parses() {
        assert_eq!(
            split_host_port("192.0.2.5:9000", 8092),
            Some(("192.0.2.5".into(), 9000))
        );
        assert_eq!(
            split_host_port("192.0.2.5", 8092),
            Some(("192.0.2.5".into(), 8092))
        );
        assert_eq!(split_host_port("  ", 8092), None);
        assert_eq!(
            split_host_port("fe80::1", 8092),
            Some(("fe80::1".into(), 8092))
        );
        assert_eq!(
            split_host_port("[fe80::1]:9000", 8092),
            Some(("fe80::1".into(), 9000))
        );
    }
}
