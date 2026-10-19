//! The paired compute node for the bulk lanes (session upload, job submit),
//! held to the LAN.
//!
//! Session data and reconstruction requests go straight to the node over the
//! local network, never over the radio relay or the cloud lane. The node comes
//! from [`compute_node::resolve`] (the pinned address, else mDNS) and is
//! accepted only when every address it resolves to is a LAN address:
//! RFC 1918, IPv4 link-local, loopback, IPv6 unique-local (`fc00::/7`) or
//! link-local (`fe80::/10`).

use std::net::IpAddr;

use axum::http::StatusCode;
use world_engine_protocol::node_credential::NodeLane;

use crate::compute_node::{self, ComputeNode};
use crate::credentials::CredentialStore;
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

/// Whether `host:port` resolves, and only to LAN addresses.
pub async fn lan_host(host: &str, port: u16) -> bool {
    if let Ok(ip) = host.parse::<IpAddr>() {
        return lan_ip(ip);
    }
    match tokio::net::lookup_host((host, port)).await {
        Ok(addrs) => {
            let ips: Vec<IpAddr> = addrs.map(|a| a.ip()).collect();
            !ips.is_empty() && ips.into_iter().all(lan_ip)
        }
        Err(_) => false,
    }
}

/// The name a node is reported by: its advertised id, else its address.
pub fn node_label(node: &ComputeNode) -> String {
    node.device_id.clone().unwrap_or_else(|| node.addr())
}

/// The node for `lane`, on the LAN.
pub async fn resolve_lan(
    host: &dyn Host,
    credentials: &CredentialStore,
    lane: &NodeLane,
) -> Result<ComputeNode, ApiError> {
    let node = compute_node::resolve(host, credentials, lane)
        .await
        .ok_or_else(|| {
            ApiError::new(
                StatusCode::SERVICE_UNAVAILABLE,
                codes::NO_NODE,
                "no compute node is pinned or answering on the LAN",
            )
        })?;
    if !lan_host(&node.host, node.port).await {
        return Err(ApiError::new(
            StatusCode::FORBIDDEN,
            codes::NOT_LAN,
            format!("node address {} is not a LAN address", node.addr()),
        ));
    }
    Ok(node)
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

    #[tokio::test]
    async fn a_public_pinned_node_is_refused() {
        let host = crate::testing::FakeHost::new();
        host.set_config(compute_node::PIN_KEY, serde_json::json!("203.0.113.7:8092"));
        let stores = CredentialStore { path: None };
        let err = resolve_lan(
            host.as_ref(),
            &stores,
            &crate::credentials::SESSION_UPLOAD_LANE,
        )
        .await
        .unwrap_err();
        assert_eq!(err.code, codes::NOT_LAN);
        host.set_config(
            compute_node::PIN_KEY,
            serde_json::json!("192.168.1.50:8092"),
        );
        let node = resolve_lan(
            host.as_ref(),
            &stores,
            &crate::credentials::SESSION_UPLOAD_LANE,
        )
        .await
        .unwrap();
        assert_eq!(node_label(&node), "192.168.1.50:8092");
    }
}
