//! What the recorder writes beside the session log, through byte-bounded
//! queues instead of the item channel so their RAM is capped by
//! `atlas.memory_budget_mb` and a backlog drops the oldest entry, never blocks
//! the capture loop:
//!
//! - keyframe previews, `keyframes/<file>` (the keyframe share, 40 %);
//! - planar scans in polar form, `lidar/<sensor>.polar.zst`: a zstd (level 3)
//!   stream of [`PolarScan`] records, 3 bytes a bin (the scan-ring share,
//!   10 %).

use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Path, PathBuf};

use world_engine_protocol::atlas::PolarScan;

use super::bounded::BoundedQueue;

/// zstd level of the polar scan streams.
pub const POLAR_ZSTD_LEVEL: i32 = 3;
/// The session subdirectory polar scan streams live in.
pub const POLAR_DIR: &str = "lidar";

/// The polar stream file of `sensor`, relative to the session directory.
pub fn polar_file(sensor: &str) -> String {
    format!("{POLAR_DIR}/{sensor}.polar.zst")
}

/// The queues the capture loop fills and the writer thread drains.
pub struct Sidecars {
    pub keyframes: BoundedQueue<(String, Vec<u8>)>,
    pub scans: BoundedQueue<(String, Vec<u8>)>,
}

impl Sidecars {
    /// Unbounded until [`Sidecars::set_caps`] (a session that never learns a
    /// budget keeps today's behaviour).
    pub fn new() -> Self {
        Self {
            keyframes: BoundedQueue::new(usize::MAX),
            scans: BoundedQueue::new(usize::MAX),
        }
    }

    pub fn set_caps(&self, keyframe_bytes: usize, scan_bytes: usize) {
        self.keyframes.set_cap(keyframe_bytes);
        self.scans.set_cap(scan_bytes);
    }

    /// Queue one polar scan of `sensor`.
    pub fn push_scan(&self, sensor: &str, scan: &PolarScan) {
        let mut rec = Vec::with_capacity(scan.encoded_len());
        scan.encode(&mut rec);
        let n = rec.len();
        self.scans.push((sensor.to_string(), rec), n);
    }

    /// Bytes waiting in both queues.
    pub fn bytes(&self) -> usize {
        self.keyframes.bytes() + self.scans.bytes()
    }
}

impl Default for Sidecars {
    fn default() -> Self {
        Self::new()
    }
}

/// The writer thread's open polar streams.
#[derive(Default)]
pub struct PolarWriter {
    open: HashMap<String, zstd::Encoder<'static, BufWriter<File>>>,
}

impl PolarWriter {
    fn stream(
        &mut self,
        dir: &Path,
        sensor: &str,
    ) -> std::io::Result<&mut zstd::Encoder<'static, BufWriter<File>>> {
        if !self.open.contains_key(sensor) {
            let path: PathBuf = dir.join(polar_file(sensor));
            if let Some(parent) = path.parent() {
                fs::create_dir_all(parent)?;
            }
            let enc = zstd::Encoder::new(BufWriter::new(File::create(path)?), POLAR_ZSTD_LEVEL)?;
            self.open.insert(sensor.to_string(), enc);
        }
        Ok(self.open.get_mut(sensor).expect("inserted above"))
    }

    /// Append one encoded record to `sensor`'s stream.
    pub fn write(&mut self, dir: &Path, sensor: &str, record: &[u8]) -> std::io::Result<()> {
        self.stream(dir, sensor)?.write_all(record)
    }

    /// Close every stream (the zstd frame end and the buffer flush), before
    /// the session's checksums are taken.
    pub fn finish(&mut self) -> std::io::Result<()> {
        for (_, enc) in self.open.drain() {
            enc.finish()?.flush()?;
        }
        Ok(())
    }
}

/// Write everything queued. A keyframe whose name is unsafe, or a write that
/// fails, is returned as `(kind, error)` per failure and skipped.
pub fn drain(
    side: &Sidecars,
    dir: &Path,
    polar: &mut PolarWriter,
) -> Vec<(&'static str, anyhow::Error)> {
    let mut bad = Vec::new();
    for (file, jpeg) in side.keyframes.drain() {
        if !crate::storage::safe_session_id(file.trim_end_matches(".jpg")) {
            bad.push((
                "keyframe",
                anyhow::anyhow!("bad keyframe file name `{file}`"),
            ));
            continue;
        }
        if let Err(e) = fs::write(dir.join("keyframes").join(&file), jpeg) {
            bad.push(("keyframe", e.into()));
        }
    }
    for (sensor, rec) in side.scans.drain() {
        if !crate::storage::safe_session_id(&sensor) {
            bad.push(("polar_scan", anyhow::anyhow!("bad sensor id `{sensor}`")));
            continue;
        }
        if let Err(e) = polar.write(dir, &sensor, &rec) {
            bad.push(("polar_scan", e.into()));
        }
    }
    bad
}

/// Every record of a session's polar stream for `sensor`.
pub fn read_polar(session_dir: &Path, sensor: &str) -> anyhow::Result<Vec<PolarScan>> {
    let raw = zstd::decode_all(File::open(session_dir.join(polar_file(sensor)))?)?;
    Ok(PolarScan::decode_all(&raw)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    use world_engine_protocol::atlas::VioHealth;

    fn scan(t: u64) -> PolarScan {
        PolarScan {
            t_ns: t,
            angle_start_rad: 0.0,
            angle_inc_rad: 0.5f32.to_radians(),
            t: [0.0; 3],
            q: [1.0, 0.0, 0.0, 0.0],
            health: VioHealth::Good,
            range_mm: (0..720).map(|k| 1000 + k as u16).collect(),
            quality: vec![100; 720],
        }
    }

    #[test]
    fn polar_scans_round_trip_through_the_compressed_stream() {
        let dir = tempfile::tempdir().unwrap();
        let side = Sidecars::new();
        for t in 0..20 {
            side.push_scan("lidar0", &scan(t));
        }
        let mut w = PolarWriter::default();
        assert!(drain(&side, dir.path(), &mut w).is_empty());
        w.finish().unwrap();
        let back = read_polar(dir.path(), "lidar0").unwrap();
        assert_eq!(back.len(), 20);
        assert_eq!(back[7], scan(7));
        let on_disk = fs::metadata(dir.path().join(polar_file("lidar0")))
            .unwrap()
            .len();
        let raw = 20 * scan(0).encoded_len() as u64;
        assert!(on_disk < raw, "{on_disk} >= {raw}");
    }

    #[test]
    fn the_scan_ring_drops_the_oldest_past_its_share() {
        let side = Sidecars::new();
        let one = scan(0).encoded_len();
        side.set_caps(usize::MAX, one * 3);
        for t in 0..5 {
            side.push_scan("lidar0", &scan(t));
        }
        assert_eq!(side.scans.dropped(), 2);
        assert_eq!(side.bytes(), one * 3);
    }
}
