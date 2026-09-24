//! Compute-node discovery on the local network.
//!
//! The compute node advertises its job API as [`COMPUTE_SERVICE`] on its
//! declared listen port, with its node id in the [`DEVICE_ID_TXT`] TXT key, and
//! a drone or ground station browses for it. Neither end runs an mDNS
//! responder of its own: every plugin unit carries `SocketBindDeny=any`, which
//! refuses even the responder's first ephemeral bind, so both go through the
//! agent's host methods (`ctx.mdns.advertise` / `ctx.mdns.browse`). The host
//! publishes under the system hostname, the one name avahi answers for, and
//! withdraws the record when the node's connection ends.
//!
//! The GCS finds a compute node the way it finds any node: through the agent's
//! own always-on `_ados._tcp` pairing record. This service type is only the
//! job API a drone lane dials.

use std::collections::BTreeMap;
use std::net::Ipv4Addr;

use ados_protocol::plugin_mdns::DiscoveredService;

/// The DNS-SD service type a compute node's job API is advertised under.
pub const COMPUTE_SERVICE: &str = "_ados-compute._tcp";
/// The TXT key carrying the node's id: the id it names in every credential it
/// issues, so a drone picks the credential that node issued it.
pub const DEVICE_ID_TXT: &str = "deviceId";

/// The TXT record a compute node advertises its job API with.
pub fn compute_advert_txt(node_id: &str) -> BTreeMap<String, String> {
    BTreeMap::from([(DEVICE_ID_TXT.to_string(), node_id.to_string())])
}

/// A resolved compute node: where to reach its job API, and who it is.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ResolvedComputeNode {
    /// The host to dial (a concrete IPv4 when advertised, else the `.local` name).
    pub host: String,
    /// The job-API port (the advertised SRV port).
    pub job_api_port: u16,
    /// The node's id (the `deviceId` TXT), empty when the advert omits it.
    pub device_id: String,
}

/// The compute node to use from one browse of [`COMPUTE_SERVICE`]: the first
/// answering node whose id is in `preferred` (the workstations that issued
/// this drone a credential), else the first node that answered.
pub fn pick_compute_node(
    services: &[DiscoveredService],
    preferred: &[String],
) -> Option<ResolvedComputeNode> {
    let nodes: Vec<ResolvedComputeNode> = services.iter().filter_map(compute_node_of).collect();
    let preferred_node = nodes
        .iter()
        .position(|n| !n.device_id.is_empty() && preferred.contains(&n.device_id));
    let pick = preferred_node.unwrap_or(0);
    nodes.into_iter().nth(pick)
}

/// The compute node one resolved instance describes, or `None` when it carries
/// no usable port or host. An IPv4 address is preferred (the HTTP client dials
/// it directly, with no second mDNS lookup); the advertised hostname is the
/// fallback.
///
/// A node answers with every interface's address, and the first one is not
/// necessarily the LAN a drone shares with it: a workstation on a tailnet also
/// answers with its `100.64.0.0/10` address, and a drone off that overlay
/// cannot dial it. A private LAN address (RFC 1918) is taken first, then any
/// other routable one; a shared-overlay, link-local or loopback address only
/// when nothing else was offered.
fn compute_node_of(service: &DiscoveredService) -> Option<ResolvedComputeNode> {
    if service.port == 0 {
        return None;
    }
    let host = service
        .addresses
        .iter()
        .filter_map(|a| a.parse::<Ipv4Addr>().ok())
        .min_by_key(lan_reach_rank)
        .map(|ip| ip.to_string())
        .unwrap_or_else(|| service.hostname.trim_end_matches('.').to_string());
    (!host.is_empty()).then(|| ResolvedComputeNode {
        host,
        job_api_port: service.port,
        device_id: service.txt.get(DEVICE_ID_TXT).cloned().unwrap_or_default(),
    })
}

/// How likely a drone on the node's LAN is to reach `ip`: 0 for a private LAN
/// address, 1 for another routable one, 2 for the shared-overlay range
/// (`100.64.0.0/10`), link-local and loopback.
fn lan_reach_rank(ip: &Ipv4Addr) -> u8 {
    let [a, b, ..] = ip.octets();
    let shared_overlay = a == 100 && (b & 0xc0) == 64;
    if ip.is_private() {
        0
    } else if shared_overlay || ip.is_link_local() || ip.is_loopback() {
        2
    } else {
        1
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn service(
        id: Option<&str>,
        addresses: &[&str],
        hostname: &str,
        port: u16,
    ) -> DiscoveredService {
        DiscoveredService {
            fullname: format!("n-{}.{COMPUTE_SERVICE}.local.", id.unwrap_or("x")),
            hostname: hostname.to_string(),
            port,
            addresses: addresses.iter().map(|a| a.to_string()).collect(),
            txt: id.map(compute_advert_txt).unwrap_or_default(),
        }
    }

    #[test]
    fn the_advert_carries_the_node_id_the_credentials_name() {
        let txt = compute_advert_txt("compute-8270beb76258");
        assert_eq!(
            txt.get("deviceId").map(String::as_str),
            Some("compute-8270beb76258")
        );
    }

    #[test]
    fn a_node_that_issued_this_drone_a_credential_wins() {
        let services = [
            service(Some("ws-a"), &["192.0.2.10"], "a.local", 8092),
            service(Some("ws-b"), &["192.0.2.11"], "b.local", 8092),
        ];
        let pick = pick_compute_node(&services, &["ws-b".to_string()]).unwrap();
        assert_eq!(
            (pick.host.as_str(), pick.device_id.as_str()),
            ("192.0.2.11", "ws-b")
        );
        // Nothing preferred answers: the first node that did.
        let pick = pick_compute_node(&services, &["ws-z".to_string()]).unwrap();
        assert_eq!(pick.device_id, "ws-a");
        assert_eq!(pick_compute_node(&[], &[]), None);
    }

    #[test]
    fn an_ipv4_address_is_dialled_before_the_hostname() {
        let v6_first = service(Some("ws"), &["fe80::1", "192.0.2.10"], "ws.local", 8092);
        assert_eq!(
            pick_compute_node(&[v6_first], &[]).unwrap().host,
            "192.0.2.10"
        );
        let no_v4 = service(Some("ws"), &["fe80::1"], "ws.local.", 9000);
        let node = pick_compute_node(&[no_v4], &[]).unwrap();
        assert_eq!((node.host.as_str(), node.job_api_port), ("ws.local", 9000));
        // No port, or nowhere to dial: not a node.
        assert_eq!(
            pick_compute_node(&[service(Some("ws"), &["192.0.2.10"], "", 0)], &[]),
            None
        );
        assert_eq!(
            pick_compute_node(&[service(Some("ws"), &[], "", 8092)], &[]),
            None
        );
    }

    /// A workstation answers with every interface's address, sorted. The one
    /// a drone on its LAN can dial wins over an overlay or link-local one that
    /// happens to sort first.
    #[test]
    fn the_private_lan_address_wins_over_overlay_and_link_local_ones() {
        let ws = service(
            Some("ws"),
            &["100.64.0.1", "169.254.3.4", "192.168.1.50", "fd7a::1"],
            "ws.local",
            8092,
        );
        assert_eq!(
            pick_compute_node(&[ws], &[]).unwrap().host,
            "192.168.1.50"
        );
        // With no private address, a routable one beats the overlay.
        let public = service(
            Some("ws"),
            &["100.64.0.1", "203.0.113.7"],
            "ws.local",
            8092,
        );
        assert_eq!(
            pick_compute_node(&[public], &[]).unwrap().host,
            "203.0.113.7"
        );
        // An overlay address alone is still used rather than nothing.
        let overlay = service(Some("ws"), &["100.64.0.1"], "ws.local", 8092);
        assert_eq!(
            pick_compute_node(&[overlay], &[]).unwrap().host,
            "100.64.0.1"
        );
    }
}
