//! LiDAR scans for the session record, from Navigation.
//!
//! Navigation streams each scan as measured on `plugin.navigation.scan_raw`
//! while a session records, cut into parts that fit one host event
//! ([`ScanChunk`]). [`ScanRaw`] reassembles them in any arrival order and
//! hands each whole scan to the running capture loop, which records it as
//! the in-process LiDAR path did and feeds the LiDAR depth check. A scan
//! whose parts do not all arrive within [`WINDOW_NS`] of its first is
//! dropped and counted ([`ScanRaw::incomplete`], the health record's
//! `lidar_scans_incomplete`).

use std::sync::Arc;

use ados_mapping_pose::lidar_scan::LidarScan;
use ados_mapping_protocol::atlas::{unpack_points, AssembledScan, ScanAssembler, ScanChunk};
use parking_lot::Mutex;
use tokio::sync::mpsc;

/// How long a scan's parts have to arrive.
pub const WINDOW_NS: u64 = 500_000_000;

/// A whole scan as the capture loop takes it.
pub fn to_scan(a: AssembledScan) -> LidarScan {
    let (points, intensity, t_offset_ns) = unpack_points(&a.points);
    LidarScan {
        sensor_id: a.sensor_id,
        t_ns: a.t_ns,
        points,
        t_offset_ns,
        intensity,
    }
}

/// The process-lifetime `scan_raw` subscription, routed to the current run.
pub struct ScanRaw {
    asm: Mutex<ScanAssembler>,
    sink: Mutex<Option<mpsc::Sender<LidarScan>>>,
}

impl ScanRaw {
    pub fn new() -> Arc<Self> {
        Arc::new(Self {
            asm: Mutex::new(ScanAssembler::new(WINDOW_NS)),
            sink: Mutex::new(None),
        })
    }

    /// Route whole scans to `tx` (the current run), or nowhere.
    pub fn attach(&self, tx: Option<mpsc::Sender<LidarScan>>) {
        *self.sink.lock() = tx;
    }

    /// Fold one part received at host monotonic `now_ns`; the scan it
    /// completes.
    pub fn on_chunk(&self, c: ScanChunk, now_ns: u64) -> Option<LidarScan> {
        self.asm.lock().push(c, now_ns).map(to_scan)
    }

    /// Scans dropped incomplete so far, as of `now_ns`.
    pub fn incomplete(&self, now_ns: u64) -> u64 {
        let mut asm = self.asm.lock();
        asm.expire(now_ns);
        asm.incomplete()
    }

    /// The callback for `event.subscribe`: deliveries carry the part under
    /// `payload`; one that does not decode is dropped, and a whole scan the
    /// run cannot take at once is dropped (the run's queue is its budget).
    pub fn callback(self: &Arc<Self>) -> ados_sdk::EventCallback {
        let me = self.clone();
        Arc::new(move |args: rmpv::Value| {
            let Some(payload) = args.as_map().and_then(|m| {
                m.iter()
                    .find(|(k, _)| k.as_str() == Some("payload"))
                    .map(|(_, v)| v)
            }) else {
                return;
            };
            let mut bytes = Vec::new();
            if rmpv::encode::write_value(&mut bytes, payload).is_err() {
                return;
            }
            let Ok(chunk) = ScanChunk::from_msgpack(&bytes) else {
                return;
            };
            if let Some(scan) = me.on_chunk(chunk, ados_mapping_pose::clock::mono_ns()) {
                if let Some(tx) = me.sink.lock().as_ref() {
                    let _ = tx.try_send(scan);
                }
            }
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::lidar_record::scan_to_point_cloud;
    use ados_mapping_protocol::atlas::{chunk_scan, pack_points};

    fn scan(n: usize) -> LidarScan {
        LidarScan {
            sensor_id: "mid360".into(),
            t_ns: 77,
            points: (0..n).map(|i| [i as f32 * 0.01, 1.5, -0.2]).collect(),
            t_offset_ns: (0..n).map(|i| i as u32 * 5_000).collect(),
            intensity: (0..n).map(|i| (i % 200) as u8).collect(),
        }
    }

    #[test]
    fn a_scan_in_six_parts_out_of_order_is_recorded_once_as_the_in_process_path_did() {
        let original = scan(20_000);
        let packed = pack_points(&original.points, &original.intensity, &original.t_offset_ns);
        let mut parts = chunk_scan(&original.sensor_id, original.t_ns, 4, &packed);
        assert_eq!(parts.len(), 6);
        parts.swap(0, 5);
        parts.swap(2, 3);
        let raw = ScanRaw::new();
        let whole: Vec<LidarScan> = parts
            .into_iter()
            .filter_map(|c| {
                let c = ScanChunk::from_msgpack(&c.to_msgpack().unwrap()).unwrap();
                raw.on_chunk(c, 1_000)
            })
            .collect();
        assert_eq!(whole.len(), 1);
        assert_eq!(
            scan_to_point_cloud(&whole[0], 5, "mid360"),
            scan_to_point_cloud(&original, 5, "mid360")
        );
        assert_eq!(raw.incomplete(1_000 + 2 * WINDOW_NS), 0);
    }

    #[test]
    fn a_scan_missing_a_part_is_counted_incomplete_after_the_window() {
        let original = scan(8_000);
        let packed = pack_points(&original.points, &original.intensity, &original.t_offset_ns);
        let parts = chunk_scan("mid360", 1, 1, &packed);
        let raw = ScanRaw::new();
        assert!(raw.on_chunk(parts[0].clone(), 0).is_none());
        assert!(raw.on_chunk(parts[2].clone(), 10).is_none());
        assert_eq!(raw.incomplete(WINDOW_NS), 0);
        assert_eq!(raw.incomplete(WINDOW_NS + 1), 1);
    }
}
