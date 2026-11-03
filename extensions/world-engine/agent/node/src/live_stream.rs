//! The compute node's live map of a streaming drone
//! (`atlas.compute_placement = stream`): every raw polar scan the drone sends
//! on `atlas.scan` is painted into a per-device [`LiveMap`] at full
//! resolution, and once a second the changed occupancy tiles and new sweep
//! points go out on that device's world stream (`/ws/atlas/<device_id>`) as
//! live occupancy and point-cloud payloads (`kind: "live"`), beside the
//! reconstruction descriptors.

use std::collections::HashMap;
use std::path::PathBuf;

use world_engine_livemap::LiveMap;
use world_engine_protocol::atlas::{
    AtlasEvent, PolarScan, PLUGIN_ATLAS_OCCUPANCY_TOPIC, PLUGIN_ATLAS_POINTCLOUD_TOPIC,
};

/// The voxel the node's live sweep cloud starts at.
pub const NODE_VOXEL_M: f32 = 0.05;
/// RAM one device's sweep cloud may hold before it coarsens.
pub const NODE_SWEEP_CAP_BYTES: usize = 1 << 30;
/// RAM one device's occupancy tiles may hold before the farthest go to disk.
pub const NODE_OCCUPANCY_CAP_BYTES: usize = 512 << 20;
/// The device key of a scan whose event carried no device id.
const UNATTRIBUTED: &str = "unknown";

/// Live maps by device.
pub struct LiveStreams {
    maps: HashMap<String, LiveMap>,
    /// Evicted tiles go under `<root>/<device>/occupancy`.
    root: PathBuf,
    /// Scans refused as malformed.
    pub rejected: u64,
}

impl LiveStreams {
    pub fn new(root: PathBuf) -> Self {
        Self {
            maps: HashMap::new(),
            root,
            rejected: 0,
        }
    }

    /// Paint one `atlas.scan` event.
    pub fn on_scan(&mut self, event: &AtlasEvent) {
        match PolarScan::decode(&event.payload) {
            Ok((scan, _)) => {
                let device = event.device_id.as_deref().unwrap_or(UNATTRIBUTED);
                self.maps
                    .entry(device.to_string())
                    .or_insert_with(|| LiveMap::new(NODE_VOXEL_M, NODE_SWEEP_CAP_BYTES))
                    .insert(&scan);
            }
            Err(e) => {
                self.rejected += 1;
                tracing::debug!(error = %e, "atlas_live_scan_rejected");
            }
        }
    }

    /// The live payloads due now, as `(device, event)` for the world stream.
    pub fn tick(&mut self) -> Vec<(String, AtlasEvent)> {
        let mut out = Vec::new();
        for (device, map) in &mut self.maps {
            let dir = self.root.join(device).join("occupancy");
            let (occ, cloud) = map.tick(NODE_OCCUPANCY_CAP_BYTES, Some(&dir));
            let mut push = |topic: &str, body: Result<Vec<u8>, rmp_serde::encode::Error>| match body {
                Ok(body) => out.push((
                    device.clone(),
                    AtlasEvent::new(topic, Some(device.clone()), body),
                )),
                Err(e) => tracing::warn!(topic, error = %e, "atlas_live_encode_failed"),
            };
            if let Some(o) = occ {
                push(PLUGIN_ATLAS_OCCUPANCY_TOPIC, o.to_msgpack());
            }
            if let Some(c) = cloud {
                push(PLUGIN_ATLAS_POINTCLOUD_TOPIC, c.to_msgpack());
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use world_engine_protocol::atlas::{
        LiveOccupancy, LivePointCloud, MapPayloadKind, VioHealth, ATLAS_SCAN_TOPIC,
    };

    fn scan_event(device: Option<&str>) -> AtlasEvent {
        let n = 720;
        let s = PolarScan {
            t_ns: 1,
            angle_start_rad: 0.0,
            angle_inc_rad: std::f32::consts::TAU / n as f32,
            t: [0.0; 3],
            q: [1.0, 0.0, 0.0, 0.0],
            health: VioHealth::Good,
            range_mm: vec![3020; n],
            quality: vec![100; n],
        };
        let mut body = Vec::new();
        s.encode(&mut body);
        AtlasEvent::new(ATLAS_SCAN_TOPIC, device.map(str::to_string), body)
    }

    #[test]
    fn streamed_scans_become_live_payloads_on_their_device_stream() {
        let dir = tempfile::tempdir().unwrap();
        let mut live = LiveStreams::new(dir.path().to_path_buf());
        for _ in 0..3 {
            live.on_scan(&scan_event(Some("drone-1")));
        }
        live.on_scan(&AtlasEvent::new(ATLAS_SCAN_TOPIC, None, vec![1, 2]));
        assert_eq!(live.rejected, 1);
        let out = live.tick();
        assert_eq!(out.len(), 2);
        assert!(out.iter().all(|(d, e)| d == "drone-1"
            && e.device_id.as_deref() == Some("drone-1")));
        let occ = out.iter().find(|(_, e)| e.topic == PLUGIN_ATLAS_OCCUPANCY_TOPIC).unwrap();
        let occ = LiveOccupancy::from_msgpack(&occ.1.payload).unwrap();
        assert_eq!(occ.kind, MapPayloadKind::Live);
        assert!(!occ.tiles.is_empty());
        let cloud = out.iter().find(|(_, e)| e.topic == PLUGIN_ATLAS_POINTCLOUD_TOPIC).unwrap();
        let cloud = LivePointCloud::from_msgpack(&cloud.1.payload).unwrap();
        assert!(cloud.count > 100);
        assert!(live.tick().is_empty(), "nothing new");
    }
}
