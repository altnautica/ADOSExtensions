//! Standard-scan byte stream to timed 360-degree revolutions.
//!
//! The device has no clock output, so time comes from the host: each chunk
//! handed over by `read()` is stamped with the host time it returned, and the
//! node ending `bytes_after` bytes before the chunk's end is back-dated by the
//! serial time of those bytes, `t_k = t_rx - bytes_after * 10 / baud` (8N1
//! puts 10 bits on the wire per byte). A node with the start flag closes the
//! previous revolution; that revolution's node times are least-squares fitted
//! to `a + b * k` over its node slots `k` (the device samples at a steady
//! rate), which removes the read-scheduling jitter. Slots include nodes with
//! no return, so the fit spacing stays the sampling interval.

use crate::proto::{decode_standard_node, flu_from_device_deg, node_framing_ok, NODE_LEN};

/// A revolution holding more node slots than this without a start flag is
/// discarded: the flag was lost and the stream is no longer one revolution.
pub const MAX_SLOTS: usize = 8192;

/// One revolution, sensor FLU frame.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RawScan {
    /// Host monotonic ns of the first node slot (the fitted intercept).
    pub t_ns: u64,
    /// Metres, `x = d cos θ`, `y = d sin θ`, `z = 0`, θ the FLU azimuth.
    pub points: Vec<[f32; 3]>,
    /// Per-point offset from `t_ns`, ns (same length as `points`).
    pub t_offset_ns: Vec<u32>,
    /// Per-point quality 0..63 (same length as `points`).
    pub intensity: Vec<u8>,
}

impl RawScan {
    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}

/// Counters since the assembler was created.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct AssemblerStats {
    /// Nodes with valid framing.
    pub nodes: u64,
    /// Nodes with no return (distance 0), kept as time slots only.
    pub no_return: u64,
    /// Bytes skipped to regain node framing.
    pub resync_bytes: u64,
    /// Revolutions emitted.
    pub revolutions: u64,
    /// Revolutions discarded for exceeding [`MAX_SLOTS`].
    pub overlong: u64,
}

#[derive(Debug, Default)]
struct Rev {
    /// Host ns of slot 0 (raw, before the fit), the time origin of `t_rel`.
    t0: i128,
    /// Raw back-dated time of each slot relative to `t0`, ns.
    t_rel: Vec<f64>,
    /// `(slot, point, quality)` for slots with a return.
    hits: Vec<(u32, [f32; 3], u8)>,
}

/// Turns received bytes into [`RawScan`]s; pure, no I/O.
#[derive(Debug)]
pub struct RevolutionAssembler {
    ns_per_byte: f64,
    pending: Vec<u8>,
    cur: Option<Rev>,
    stats: AssemblerStats,
}

impl RevolutionAssembler {
    /// `baud` sets the per-byte wire time used to back-date nodes.
    pub fn new(baud: u32) -> Self {
        Self {
            ns_per_byte: 10.0e9 / f64::from(baud.max(1)),
            pending: Vec::with_capacity(4 * NODE_LEN),
            cur: None,
            stats: AssemblerStats::default(),
        }
    }

    pub fn stats(&self) -> AssemblerStats {
        self.stats
    }

    /// Drop partial state (a reconnect: the next revolution starts afresh).
    pub fn reset(&mut self) {
        self.pending.clear();
        self.cur = None;
    }

    /// Feed one received chunk stamped with the host monotonic time `t_rx_ns`
    /// its read returned; completed revolutions go to `emit`.
    pub fn push(&mut self, chunk: &[u8], t_rx_ns: u64, mut emit: impl FnMut(RawScan)) {
        self.pending.extend_from_slice(chunk);
        let total = self.pending.len();
        let mut i = 0;
        while i + NODE_LEN <= total {
            let b: [u8; NODE_LEN] = self.pending[i..i + NODE_LEN].try_into().expect("5 bytes");
            if !node_framing_ok(&b) {
                i += 1;
                self.stats.resync_bytes += 1;
                continue;
            }
            i += NODE_LEN;
            let bytes_after = (total - i) as f64;
            let t_k = i128::from(t_rx_ns) - (bytes_after * self.ns_per_byte).round() as i128;
            self.node(b, t_k, &mut emit);
        }
        self.pending.drain(..i);
    }

    fn node(&mut self, b: [u8; NODE_LEN], t_k: i128, emit: &mut impl FnMut(RawScan)) {
        self.stats.nodes += 1;
        let start = b[0] & 1 == 1;
        if start {
            if let Some(rev) = self.cur.take() {
                let scan = fit(rev);
                self.stats.revolutions += 1;
                emit(scan);
            }
            self.cur = Some(Rev {
                t0: t_k,
                t_rel: Vec::with_capacity(512),
                hits: Vec::with_capacity(512),
            });
        }
        // Nodes before the first start flag belong to a partial revolution.
        let Some(rev) = self.cur.as_mut() else {
            if decode_standard_node(b).is_none() {
                self.stats.no_return += 1;
            }
            return;
        };
        let slot = rev.t_rel.len() as u32;
        rev.t_rel.push((t_k - rev.t0) as f64);
        match decode_standard_node(b) {
            Some(n) => {
                let th = flu_from_device_deg(f64::from(n.angle_deg)).to_radians();
                let d = f64::from(n.dist_m);
                let p = [(d * th.cos()) as f32, (d * th.sin()) as f32, 0.0];
                rev.hits.push((slot, p, n.quality));
            }
            None => self.stats.no_return += 1,
        }
        if rev.t_rel.len() > MAX_SLOTS {
            self.cur = None;
            self.stats.overlong += 1;
        }
    }
}

/// Least-squares `t = a + b k` over the revolution's slots; the scan time is
/// the intercept, each point's offset `b k`. A negative slope (impossible for
/// a real stream) is clamped to 0.
fn fit(rev: Rev) -> RawScan {
    let (a, b) = line_fit(&rev.t_rel);
    let b = b.max(0.0);
    let t_ns = (rev.t0 + a.round() as i128).max(0) as u64;
    let n = rev.hits.len();
    let mut scan = RawScan {
        t_ns,
        points: Vec::with_capacity(n),
        t_offset_ns: Vec::with_capacity(n),
        intensity: Vec::with_capacity(n),
    };
    for (slot, p, q) in rev.hits {
        scan.points.push(p);
        scan.t_offset_ns
            .push((b * f64::from(slot)).round().min(f64::from(u32::MAX)) as u32);
        scan.intensity.push(q);
    }
    scan
}

/// Intercept and slope of the least-squares line through `(k, t[k])`.
fn line_fit(t: &[f64]) -> (f64, f64) {
    let n = t.len();
    if n < 2 {
        return (t.first().copied().unwrap_or(0.0), 0.0);
    }
    let nf = n as f64;
    let k_mean = (nf - 1.0) / 2.0;
    let t_mean = t.iter().sum::<f64>() / nf;
    let (mut sxy, mut sxx) = (0.0, 0.0);
    for (k, &tk) in t.iter().enumerate() {
        let dk = k as f64 - k_mean;
        sxy += dk * (tk - t_mean);
        sxx += dk * dk;
    }
    let b = sxy / sxx;
    (t_mean - b * k_mean, b)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::encode_standard_node;

    const BAUD: u32 = 460_800;
    /// 5 kHz sampling.
    const SAMPLE_NS: u64 = 200_000;

    /// `revs` revolutions of `n` nodes at device angles `k * 360 / n`, the
    /// node at slot `zero` (if any) with no return.
    fn stream(revs: usize, n: usize, zero: Option<usize>) -> Vec<[u8; 5]> {
        let mut out = Vec::new();
        for _ in 0..revs {
            for k in 0..n {
                let d = if Some(k) == zero { 0.0 } else { 2.0 };
                out.push(encode_standard_node(
                    k == 0,
                    40,
                    k as f32 * 360.0 / n as f32,
                    d,
                ));
            }
        }
        out
    }

    /// Deliver nodes in chunks of `per_chunk`, as a USB serial bridge hands
    /// them over: node `j` has fully arrived at `t_start + j * SAMPLE_NS`, and
    /// each read returns when its last node arrived plus a scheduling jitter.
    fn deliver(
        asm: &mut RevolutionAssembler,
        nodes: &[[u8; 5]],
        per_chunk: usize,
        t_start: u64,
    ) -> Vec<RawScan> {
        let mut out = Vec::new();
        for (c, chunk) in nodes.chunks(per_chunk).enumerate() {
            let last = c * per_chunk + chunk.len() - 1;
            let jitter = [0u64, 300_000, 100_000, 500_000, 50_000][c % 5];
            let t_rx = t_start + last as u64 * SAMPLE_NS + jitter;
            let bytes: Vec<u8> = chunk.iter().flatten().copied().collect();
            asm.push(&bytes, t_rx, |s| out.push(s));
        }
        out
    }

    #[test]
    fn revolutions_are_fitted_to_the_sampling_rate_despite_read_jitter() {
        let mut asm = RevolutionAssembler::new(BAUD);
        let t_start = 5_000_000_000;
        let scans = deliver(&mut asm, &stream(4, 500, Some(7)), 9, t_start);
        // Four start flags close three revolutions.
        assert_eq!(scans.len(), 3);
        for (r, s) in scans.iter().enumerate() {
            assert_eq!(s.len(), 499);
            assert!(s.points.iter().all(|p| p[2] == 0.0));
            // Slot 0 of revolution r is node r * 500. Back-dating assumes the
            // chunk's bytes came back to back, so it errs late by at most a
            // chunk's idle line time plus the jitter; the fit averages both.
            let truth = t_start + r as u64 * 500 * SAMPLE_NS;
            let err = s.t_ns as i64 - truth as i64;
            assert!(err.abs() < 1_000_000, "rev {r}: t_ns off by {err} ns");
            // Last slot is 499 sample intervals after slot 0.
            let last = *s.t_offset_ns.last().unwrap() as f64;
            assert!(
                (last / (499.0 * SAMPLE_NS as f64) - 1.0).abs() < 0.02,
                "{last}"
            );
            assert!(s.t_offset_ns.windows(2).all(|w| w[0] <= w[1]));
        }
        assert!(scans.windows(2).all(|w| w[0].t_ns < w[1].t_ns));
        let st = asm.stats();
        assert_eq!(st.revolutions, 3);
        assert_eq!(st.no_return, 4);
        assert_eq!(st.resync_bytes, 0);
    }

    #[test]
    fn device_angles_land_mirrored_in_the_flu_frame() {
        let mut asm = RevolutionAssembler::new(BAUD);
        // Device 90 deg (clockwise, the right side) is FLU 270 deg: -y.
        let nodes = [
            encode_standard_node(true, 10, 0.0, 1.0),
            encode_standard_node(false, 10, 90.0, 1.0),
            encode_standard_node(true, 10, 0.0, 1.0),
        ];
        let mut out = Vec::new();
        let bytes: Vec<u8> = nodes.iter().flatten().copied().collect();
        asm.push(&bytes, 1_000_000, |s| out.push(s));
        assert_eq!(out.len(), 1);
        let p = out[0].points[1];
        assert!(p[0].abs() < 1e-6 && (p[1] + 1.0).abs() < 1e-6, "{p:?}");
        let p0 = out[0].points[0];
        assert!((p0[0] - 1.0).abs() < 1e-6 && p0[1].abs() < 1e-6);
    }

    #[test]
    fn a_stray_byte_is_skipped_and_framing_regained() {
        let mut asm = RevolutionAssembler::new(BAUD);
        let mut bytes = vec![0xFFu8];
        for n in stream(3, 100, None) {
            bytes.extend_from_slice(&n);
        }
        let mut out = Vec::new();
        // Split mid-node to exercise the carry-over.
        let (a, b) = bytes.split_at(203);
        asm.push(a, 10_000_000, |s| out.push(s));
        asm.push(b, 70_000_000, |s| out.push(s));
        assert_eq!(out.len(), 2);
        assert!(out.iter().all(|s| s.len() == 100));
        assert_eq!(asm.stats().resync_bytes, 1);
    }

    #[test]
    fn nodes_before_the_first_start_flag_are_not_a_revolution() {
        let mut asm = RevolutionAssembler::new(BAUD);
        let mut nodes = stream(1, 50, None)[10..].to_vec();
        nodes.extend(stream(2, 50, None));
        let mut out = Vec::new();
        let bytes: Vec<u8> = nodes.iter().flatten().copied().collect();
        asm.push(&bytes, 1_000_000_000, |s| out.push(s));
        assert_eq!(out.len(), 1);
        assert_eq!(out[0].len(), 50);
    }

    #[test]
    fn line_fit_recovers_an_exact_line() {
        let t: Vec<f64> = (0..10).map(|k| 3.0 + 2.5 * k as f64).collect();
        let (a, b) = line_fit(&t);
        assert!((a - 3.0).abs() < 1e-9 && (b - 2.5).abs() < 1e-9);
        assert_eq!(line_fit(&[7.0]), (7.0, 0.0));
    }
}
