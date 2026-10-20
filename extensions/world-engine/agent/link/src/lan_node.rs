//! The paired compute node for the bulk lanes (session upload, job submit),
//! held to the LAN.
//!
//! Session data and reconstruction requests go straight to the node over the
//! local network, never over the radio relay or the cloud lane. The node comes
//! from [`compute_node::resolve`] (the pinned address, else mDNS). Its host is
//! resolved once here; it is accepted only when every address it resolves to
//! is a LAN address (RFC 1918, IPv4 link-local, loopback, IPv6 unique-local
//! `fc00::/7` or link-local `fe80::/10`), and the checked address is then
//! pinned into the HTTP client ([`PinnedHost`]), so the request cannot be sent
//! to an address a second lookup returned. Those clients follow no redirects.

use std::net::{IpAddr, SocketAddr};

use axum::http::StatusCode;
use world_engine_protocol::node_credential::NodeLane;
use world_engine_transport::PinnedHost;

use crate::compute_node::{self, ComputeNode};
use crate::credentials::{CredentialGap, CredentialStore};
use crate::host::Host;
use crate::reply::{codes, ApiError};

/// Whether `ip` is on the local network.
pub fn lan_ip(ip: IpAddr) -> bool {
    match ip {
        IpAddr::V4(v4) => v4.is_private() || v4.is_link_local() || v4.is_loopback(),
        IpAddr::V6(v6) => {
            if let Some(v4) = v6.to_ipv4_mapped() {
                return lan_ip(IpAddr::V4(v4));
            }
            let first = v6.segments()[0];
            v6.is_loopback() || (first & 0xfe00) == 0xfc00 || (first & 0xffc0) == 0xfe80
        }
    }
}

/// The address to pin from one lookup's answer: the first, when every
/// address is a LAN address; `None` for an empty answer or any public one.
pub fn pick_lan(addrs: &[SocketAddr]) -> Option<SocketAddr> {
    (!addrs.is_empty() && addrs.iter().all(|a| lan_ip(a.ip()))).then(|| addrs[0])
}

/// Resolve `host:port` once and pick the LAN address to pin.
pub async fn pin_lan(host: &str, port: u16) -> Option<SocketAddr> {
    if let Ok(ip) = host.parse::<IpAddr>() {
        return lan_ip(ip).then_some(SocketAddr::new(ip, port));
    }
    let addrs: Vec<SocketAddr> = tokio::net::lookup_host((host, port)).await.ok()?.collect();
    pick_lan(&addrs)
}

/// A node held to the LAN, with its pinned address.
#[derive(Debug, Clone)]
pub struct LanNode {
    pub node: ComputeNode,
    pub pin: PinnedHost,
}

impl LanNode {
    pub fn base_url(&self) -> String {
        self.node.base_url()
    }

    /// The name a node is reported by: its advertised id, else its address.
    pub fn label(&self) -> String {
        self.node
            .device_id
            .clone()
            .unwrap_or_else(|| self.node.addr())
    }

    pub fn credential(
        &self,
        stores: &CredentialStore,
        lane: &NodeLane,
    ) -> Result<String, CredentialGap> {
        self.node.credential(stores, lane)
    }
}

/// The node for `lane`, on the LAN.
pub async fn resolve_lan(
    host: &dyn Host,
    credentials: &CredentialStore,
    lane: &NodeLane,
) -> Result<LanNode, ApiError> {
    let node = compute_node::resolve(host, credentials, lane)
        .await
        .ok_or_else(|| {
            ApiError::new(
                StatusCode::SERVICE_UNAVAILABLE,
                codes::NO_NODE,
                "no compute node is pinned or answering on the LAN",
            )
        })?;
    let Some(addr) = pin_lan(&node.host, node.port).await else {
        return Err(ApiError::new(
            StatusCode::FORBIDDEN,
            codes::NOT_LAN,
            format!(
                "node address {} does not resolve to LAN addresses only",
                node.addr()
            ),
        ));
    };
    let pin = PinnedHost {
        host: node.host.clone(),
        addr,
    };
    Ok(LanNode { node, pin })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_lan_addresses_qualify() {
        for ok in [
            "10.1.2.3",
            "172.16.0.9",
            "192.168.1.50",
            "169.254.3.4",
            "127.0.0.1",
            "::1",
            "fd12:3456::1",
            "fe80::1",
            "::ffff:192.168.1.50",
        ] {
            assert!(lan_ip(ok.parse().unwrap()), "{ok}");
        }
        for bad in [
            "8.8.8.8",
            "100.64.0.1",
            "172.32.0.1",
            "2001:db8::1",
            "::ffff:8.8.8.8",
        ] {
            assert!(!lan_ip(bad.parse().unwrap()), "{bad}");
        }
    }

    #[test]
    fn a_name_resolving_to_any_public_address_is_refused() {
        let lan: SocketAddr = "192.168.1.50:8092".parse().unwrap();
        let public: SocketAddr = "203.0.113.7:8092".parse().unwrap();
        assert_eq!(pick_lan(&[lan]), Some(lan));
        assert_eq!(pick_lan(&[public]), None);
        assert_eq!(pick_lan(&[lan, public]), None);
        assert_eq!(pick_lan(&[]), None);
    }

    #[tokio::test]
    async fn a_name_is_resolved_once_and_pinned() {
        let pinned = pin_lan("localhost", 8092).await.unwrap();
        assert!(pinned.ip().is_loopback());
        assert_eq!(pinned.port(), 8092);
    }

    #[tokio::test]
    async fn a_public_pinned_node_is_refused() {
        let host = crate::testing::FakeHost::new();
        host.set_config(compute_node::PIN_KEY, serde_json::json!("203.0.113.7:8092"));
        let stores = CredentialStore { path: None };
        let lane = crate::credentials::SESSION_UPLOAD_LANE;
        let err = resolve_lan(host.as_ref(), &stores, &lane)
            .await
            .unwrap_err();
        assert_eq!(err.code, codes::NOT_LAN);
        host.set_config(
            compute_node::PIN_KEY,
            serde_json::json!("192.168.1.50:8092"),
        );
        let node = resolve_lan(host.as_ref(), &stores, &lane).await.unwrap();
        assert_eq!(node.label(), "192.168.1.50:8092");
        assert_eq!(node.pin.addr, "192.168.1.50:8092".parse().unwrap());
    }
}
