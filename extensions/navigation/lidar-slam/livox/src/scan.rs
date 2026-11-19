//! Scan assembly: points from consecutive packets cut into fixed-period scans.
//!
//! The Mid-360 scans non-repetitively, and the protocol marks `frame_cnt`
//! invalid for that pattern, so scans are cut on the device clock instead: a
//! scan covers one period-aligned window `[k * period, (k + 1) * period)` of
//! device time, split per point, so a packet straddling a boundary lands in
//! both scans. A window closes when a point at or past its end arrives (or
//! before its start: a device clock step), and is then mapped to host time
//! with the current clock offset.

use crate::packet::{Packet, TimeType};

/// 10 Hz.
pub const DEFAULT_PERIOD_NS: u64 = 100_000_000;

/// One assembled scan, sensor frame.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Scan {
    /// Host monotonic ns of the first point.
    pub t_ns: u64,
    /// Device ns of the first point.
    pub device_t_ns: u64,
    pub time_type: Option<TimeType>,
    /// Metres, sensor frame (x forward, y left, z up).
    pub points: Vec<[f32; 3]>,
    /// Per-point offset from `t_ns`, ns.
    pub t_offset_ns: Vec<u32>,
    /// Per-point reflectivity.
    pub reflectivity: Vec<u8>,
    /// Per-point tag byte (protocol section 2.4 "Tag Information").
    pub tag: Vec<u8>,
}

impl Scan {
    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

#[derive(Debug)]
struct Window {
    start: u64,
    end: u64,
    first_dev: u64,
    scan: Scan,
}

#[derive(Debug)]
pub struct ScanAssembler {
    period_ns: u64,
    cur: Option<Window>,
    /// Pre-allocation hint: points in the last scan.
    last_len: usize,
}

impl ScanAssembler {
    pub fn new(period_ns: u64) -> Self {
        Self {
            period_ns: period_ns.max(1),
            cur: None,
            last_len: 0,
        }
    }

    pub fn period_ns(&self) -> u64 {
        self.period_ns
    }

    /// Add the valid points of a point-cloud packet. `offset_ns` is the
    /// current `host - device` estimate; every scan this call closes is
    /// mapped with it and pushed to `out`.
    pub fn push(&mut self, pkt: &Packet<'_>, offset_ns: i128, out: &mut Vec<Scan>) {
        let base = pkt.header.timestamp_ns;
        let time_type = pkt.header.time_type;
        for p in pkt.points().filter(|p| p.valid) {
            let dev = base.saturating_add(p.offset_ns);
            if self
                .cur
                .as_ref()
                .is_some_and(|w| dev >= w.end || dev < w.start)
            {
                self.flush(offset_ns, out);
            }
            let period = self.period_ns;
            let hint = self.last_len;
            let w = self.cur.get_or_insert_with(|| {
                let start = dev - dev % period;
                let mut scan = Scan {
                    device_t_ns: dev,
                    time_type: Some(time_type),
                    ..Scan::default()
                };
                scan.points.reserve(hint);
                scan.t_offset_ns.reserve(hint);
                scan.reflectivity.reserve(hint);
                scan.tag.reserve(hint);
                Window {
                    start,
                    end: start.saturating_add(period),
                    first_dev: dev,
                    scan,
                }
            });
            // Points inside a window are in time order; clamp anything
            // earlier than the first to it.
            let off = dev.saturating_sub(w.first_dev).min(u64::from(u32::MAX)) as u32;
            w.scan.points.push(p.xyz);
            w.scan.t_offset_ns.push(off);
            w.scan.reflectivity.push(p.reflectivity);
            w.scan.tag.push(p.tag);
        }
    }

    /// Close the open window, if any, mapping it with `offset_ns`.
    pub fn flush(&mut self, offset_ns: i128, out: &mut Vec<Scan>) {
        let Some(w) = self.cur.take() else {
            return;
        };
        let mut scan = w.scan;
        if scan.is_empty() {
            return;
        }
        scan.t_ns = (scan.device_t_ns as i128 + offset_ns).clamp(0, u64::MAX as i128) as u64;
        self.last_len = scan.len();
        out.push(scan);
    }

    /// Drop the open window without emitting it.
    pub fn clear(&mut self) {
        self.cur = None;
    }
}
