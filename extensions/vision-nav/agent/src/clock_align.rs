//! Bidirectional TIMESYNC clock alignment.
//!
//! The plugin and the flight controller swap TIMESYNC once a second:
//!
//! * the plugin emits `TIMESYNC(tc1=0, ts1=monotonic_ns)`,
//! * the FC replies `TIMESYNC(tc1=fc_time_ns, ts1=plugin_ts1)`,
//! * the plugin folds `offset = tc1 - ts1` into an EMA.
//!
//! The estimated offset converts the plugin's monotonic clock into FC
//! time so the `time_usec` fields on emitted messages line up with the
//! FC's notion of "now". A sudden drift larger than the breach
//! threshold bumps a reset counter that pose emitters propagate into
//! the `reset_counter` of `VISION_POSITION_ESTIMATE` / `ODOMETRY`.
//!
//! The maths is a pure state machine over `(tc1, ts1)`; the tick loop
//! and the MAVLink subscription live in the pipeline so this module
//! stays unit-testable without a host.

use std::sync::atomic::{AtomicI64, AtomicU32, Ordering};
use std::sync::Mutex;

/// Drift breach: 50 ms in ns. A larger gap between consecutive
/// estimates is treated as a real discontinuity, not noise.
pub const DRIFT_BREACH_NS: i64 = 50_000_000;

/// EMA smoothing applied after the first sample.
pub const EMA_ALPHA: f64 = 0.1;

/// A running estimate of `fc_ns - monotonic_ns`, plus the outgoing-ts1
/// in-flight set. Cheap to clone-share across the emit and the
/// subscription callback via an `Arc`.
pub struct ClockAlign {
    offset_ns: AtomicI64,
    reset_counter: AtomicU32,
    inner: Mutex<Inner>,
}

struct Inner {
    has_sample: bool,
    /// ts1 values we have sent and not yet matched, so an echo of our
    /// own outgoing query is not mistaken for a response.
    inflight: Vec<i64>,
}

impl Default for ClockAlign {
    fn default() -> Self {
        Self {
            offset_ns: AtomicI64::new(0),
            reset_counter: AtomicU32::new(0),
            inner: Mutex::new(Inner {
                has_sample: false,
                inflight: Vec::new(),
            }),
        }
    }
}

impl ClockAlign {
    pub fn new() -> Self {
        Self::default()
    }

    /// Convert a monotonic-ns reading into FC-clock ns.
    pub fn convert_to_fc_clock(&self, monotonic_ns: i64) -> i64 {
        monotonic_ns + self.offset_ns.load(Ordering::Relaxed)
    }

    /// FC-clock timestamp in microseconds for the current monotonic now.
    pub fn fc_time_us(&self, monotonic_ns: i64) -> u64 {
        (self.convert_to_fc_clock(monotonic_ns).max(0) / 1000) as u64
    }

    /// The current reset counter (bumped on a drift breach).
    pub fn reset_counter(&self) -> u32 {
        self.reset_counter.load(Ordering::Relaxed)
    }

    /// Register an outgoing ts1 so the matching response is accepted.
    pub fn mark_outgoing(&self, ts1: i64) {
        let mut g = self.inner.lock().expect("clock align lock");
        g.inflight.push(ts1);
        // Bound the in-flight set; entries are tiny and expire on match.
        if g.inflight.len() > 64 {
            let drop = g.inflight.len() - 64;
            g.inflight.drain(0..drop);
        }
    }

    /// Handle an incoming TIMESYNC `(tc1, ts1)`. A `tc1 == 0` payload is
    /// an echo of our own query (or another node asking for time) and
    /// is ignored; a `ts1` we never sent is ignored. Otherwise the
    /// offset estimate is folded in.
    pub fn handle_response(&self, tc1: i64, ts1: i64) {
        if tc1 == 0 {
            return;
        }
        {
            let mut g = self.inner.lock().expect("clock align lock");
            match g.inflight.iter().position(|&x| x == ts1) {
                Some(idx) => {
                    g.inflight.remove(idx);
                }
                None => return,
            }
        }
        let estimate = tc1 - ts1;
        let mut g = self.inner.lock().expect("clock align lock");
        if !g.has_sample {
            self.offset_ns.store(estimate, Ordering::Relaxed);
            g.has_sample = true;
        } else {
            let current = self.offset_ns.load(Ordering::Relaxed);
            if (estimate - current).abs() > DRIFT_BREACH_NS {
                self.reset_counter.fetch_add(1, Ordering::Relaxed);
            }
            let blended =
                ((1.0 - EMA_ALPHA) * current as f64 + EMA_ALPHA * estimate as f64).round() as i64;
            self.offset_ns.store(blended, Ordering::Relaxed);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_sample_adopts_estimate_verbatim() {
        let ca = ClockAlign::new();
        ca.mark_outgoing(1000);
        ca.handle_response(5000, 1000); // offset = 4000
        assert_eq!(ca.convert_to_fc_clock(0), 4000);
        assert_eq!(ca.reset_counter(), 0);
    }

    #[test]
    fn unmatched_ts1_is_ignored() {
        let ca = ClockAlign::new();
        // No mark_outgoing for ts1=42, so this response is dropped.
        ca.handle_response(9999, 42);
        assert_eq!(ca.convert_to_fc_clock(0), 0);
    }

    #[test]
    fn echo_tc1_zero_is_ignored() {
        let ca = ClockAlign::new();
        ca.mark_outgoing(1000);
        ca.handle_response(0, 1000);
        assert_eq!(ca.convert_to_fc_clock(0), 0);
    }

    #[test]
    fn drift_breach_bumps_reset_counter() {
        let ca = ClockAlign::new();
        ca.mark_outgoing(1000);
        ca.handle_response(5000, 1000); // offset 4000
        ca.mark_outgoing(2000);
        // estimate = 4000 + 60ms -> breach.
        ca.handle_response(2000 + 4000 + 60_000_000, 2000);
        assert_eq!(ca.reset_counter(), 1);
    }

    #[test]
    fn ema_blends_subsequent_samples() {
        let ca = ClockAlign::new();
        ca.mark_outgoing(0);
        ca.handle_response(1000, 0); // offset 1000
        ca.mark_outgoing(0);
        ca.handle_response(2000, 0); // estimate 2000, drift 1000 < breach
        // blended = 0.9*1000 + 0.1*2000 = 1100.
        assert_eq!(ca.convert_to_fc_clock(0), 1100);
    }
}
