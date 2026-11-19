//! The JSON file `LivoxLidarSdkInit` reads.
//!
//! Format: Livox-SDK2 `sdk_core/parse_cfg_file.cpp` (tag v1.2.5) and the
//! sample `samples/livox_lidar_quick_start/mid360_config.json`: a `MID360`
//! object with the LiDAR-side ports in `lidar_net_info` and an array
//! `host_net_info` of host endpoints. A `host_net_info` entry may restrict
//! itself to listed devices with a `lidar_ip` string array; `multicast_ip` is
//! optional. Port defaults are the protocol's (wiki "Port Description").
//! `lidar_log_enable` is left out: when present the parser also demands the
//! log cache size and path, and absent means device logging off.

use serde_json::{json, Value};

/// The five UDP ports of one side of the link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Ports {
    pub cmd_data: u16,
    pub push_msg: u16,
    pub point_data: u16,
    pub imu_data: u16,
    pub log_data: u16,
}

impl Ports {
    pub const LIDAR_DEFAULT: Self = Self {
        cmd_data: 56100,
        push_msg: 56200,
        point_data: 56300,
        imu_data: 56400,
        log_data: 56500,
    };
    pub const HOST_DEFAULT: Self = Self {
        cmd_data: 56101,
        push_msg: 56201,
        point_data: 56301,
        imu_data: 56401,
        log_data: 56501,
    };

    fn to_json(self) -> Value {
        json!({
            "cmd_data_port": self.cmd_data,
            "push_msg_port": self.push_msg,
            "point_data_port": self.point_data,
            "imu_data_port": self.imu_data,
            "log_data_port": self.log_data,
        })
    }
}

/// Network setup of a Mid-360 link.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SdkNetConfig {
    /// Host address on the LiDAR's subnet, e.g. `192.168.1.50`.
    pub host_ip: String,
    /// Only these LiDAR addresses; empty accepts any.
    pub lidar_ips: Vec<String>,
    pub multicast_ip: Option<String>,
    pub lidar_ports: Ports,
    pub host_ports: Ports,
}

impl SdkNetConfig {
    pub fn new(host_ip: impl Into<String>) -> Self {
        Self {
            host_ip: host_ip.into(),
            lidar_ips: Vec::new(),
            multicast_ip: None,
            lidar_ports: Ports::LIDAR_DEFAULT,
            host_ports: Ports::HOST_DEFAULT,
        }
    }

    pub fn to_json(&self) -> Value {
        let mut host = self.host_ports.to_json();
        host["host_ip"] = json!(self.host_ip);
        if let Some(m) = &self.multicast_ip {
            host["multicast_ip"] = json!(m);
        }
        if !self.lidar_ips.is_empty() {
            host["lidar_ip"] = json!(self.lidar_ips);
        }
        json!({
            "MID360": {
                "lidar_net_info": self.lidar_ports.to_json(),
                "host_net_info": [host],
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_the_quick_start_shape() {
        let mut c = SdkNetConfig::new("192.168.1.50");
        c.lidar_ips.push("192.168.1.12".into());
        let v = c.to_json();
        let m = &v["MID360"];
        assert_eq!(m["lidar_net_info"]["point_data_port"], 56300);
        assert_eq!(m["host_net_info"][0]["host_ip"], "192.168.1.50");
        assert_eq!(m["host_net_info"][0]["imu_data_port"], 56401);
        assert_eq!(m["host_net_info"][0]["lidar_ip"][0], "192.168.1.12");
        assert!(v.get("lidar_log_enable").is_none());
        assert!(m["host_net_info"][0].get("multicast_ip").is_none());
    }
}
