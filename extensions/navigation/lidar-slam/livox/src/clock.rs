//! Device time to host monotonic time.
//!
//! A packet leaves the LiDAR after its last sample is measured and reaches the
//! host some non-negative latency later, so for every packet
//!
//! `host_rx_ns - device_last_sample_ns = offset + latency`, `latency >= 0`.
//!
//! The smallest such difference over a sliding window is the best estimate of
//! `offset` (plus the minimum transport latency, which is the same for every
//! packet). A sliding window rather than an all-time minimum lets the
//! estimate follow the drift between the two oscillators. The window resets
//! when the device clock changes source (`time_type`) or jumps: a step of the
//! device clock, forwards or backwards, that the host clock did not take by
//! more than [`ClockConfig::max_step_ns`]. A packet whose device time is a
//! little older than the previous one (a reordered or late datagram) is an
//! ordinary observation, not a jump.
//!
//! One mapper serves one packet stream: the point and IMU streams of a device
//! travel on different UDP ports with different delays, so each gets its own.

use std::collections::VecDeque;

use crate::packet::TimeType;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClockConfig {
    /// Sliding-window length on the host clock.
    pub window_ns: u64,
    /// A disagreement between device and host progress larger than this
    /// resets the filter.
    pub max_step_ns: u64,
}

impl Default for ClockConfig {
    fn default() -> Self {
        Self {
            window_ns: 5_000_000_000,
            max_step_ns: 200_000_000,
        }
    }
}

#[derive(Debug)]
pub struct ClockMapper {
    cfg: ClockConfig,
    time_type: Option<TimeType>,
    /// Last `(host_rx_ns, device_ns)` observed.
    last: Option<(u64, u64)>,
    /// `(host_rx_ns, offset)` with strictly increasing offsets: the front is
    /// the window minimum.
    window: VecDeque<(u64, i128)>,
    resets: u64,
}

impl ClockMapper {
    pub fn new(cfg: ClockConfig) -> Self {
        Self {
            cfg,
            time_type: None,
            last: None,
            window: VecDeque::new(),
            resets: 0,
        }
    }

    /// Times the filter restarted after the first observation.
    pub fn resets(&self) -> u64 {
        self.resets
    }

    /// Feed one packet: its device time of the last sample and the host
    /// monotonic receive time. Returns true when the filter restarted.
    pub fn observe(&mut self, time_type: TimeType, device_ns: u64, host_rx_ns: u64) -> bool {
        let mut reset = self.time_type.is_some_and(|t| t != time_type);
        if let Some((h0, d0)) = self.last {
            let dh = host_rx_ns as i128 - h0 as i128;
            let dd = device_ns as i128 - d0 as i128;
            if (dd - dh).unsigned_abs() > u128::from(self.cfg.max_step_ns) {
                reset = true;
            }
        }
        if reset {
            self.window.clear();
            self.resets += 1;
        }
        self.time_type = Some(time_type);
        self.last = Some((host_rx_ns, device_ns));

        let offset = host_rx_ns as i128 - device_ns as i128;
        while self.window.back().is_some_and(|&(_, o)| o >= offset) {
            self.window.pop_back();
        }
        self.window.push_back((host_rx_ns, offset));
        let horizon = host_rx_ns.saturating_sub(self.cfg.window_ns);
        // The newest entry always stays: it is inside any window.
        while self.window.len() > 1 && self.window.front().is_some_and(|&(h, _)| h < horizon) {
            self.window.pop_front();
        }
        reset
    }

    /// Current `host - device` estimate, ns.
    pub fn offset_ns(&self) -> Option<i128> {
        self.window.front().map(|&(_, o)| o)
    }

    /// Host monotonic time of a device time, clamped at zero.
    pub fn to_host(&self, device_ns: u64) -> Option<u64> {
        let o = self.offset_ns()?;
        Some((device_ns as i128 + o).clamp(0, u64::MAX as i128) as u64)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: u64 = 1_000_000;

    #[test]
    fn minimum_latency_wins() {
        let mut c = ClockMapper::new(ClockConfig::default());
        // Device runs 1 s behind host; latencies 3, 1, 7 ms.
        for (i, lat) in [3, 1, 7].into_iter().enumerate() {
            let d = 10_000 * MS + i as u64 * 10 * MS;
            c.observe(TimeType::PowerOn, d, d + 1_000 * MS + lat * MS);
        }
        assert_eq!(c.offset_ns(), Some((1_001 * MS) as i128));
        assert_eq!(c.to_host(20_000 * MS), Some(21_001 * MS));
    }

    #[test]
    fn window_forgets_old_minimum() {
        let cfg = ClockConfig {
            window_ns: 100 * MS,
            ..ClockConfig::default()
        };
        let mut c = ClockMapper::new(cfg);
        c.observe(TimeType::PowerOn, 0, 1_000 * MS);
        for i in 1..20u64 {
            let d = i * 10 * MS;
            c.observe(TimeType::PowerOn, d, d + 1_000 * MS + 2 * MS);
        }
        // The 0-latency sample is older than the window now.
        assert_eq!(c.offset_ns(), Some((1_002 * MS) as i128));
    }

    #[test]
    fn source_change_and_jumps_reset() {
        let mut c = ClockMapper::new(ClockConfig::default());
        assert!(!c.observe(TimeType::PowerOn, 5 * MS, 1_000 * MS));
        assert!(!c.observe(TimeType::PowerOn, 15 * MS, 1_010 * MS));
        // PTP lock: device time steps to an epoch time.
        let epoch = 1_760_000_000_000_000_000u64;
        assert!(c.observe(TimeType::Ptp, epoch, 1_020 * MS));
        assert_eq!(
            c.offset_ns(),
            Some(1_020 * MS as i128 - epoch as i128),
            "old offsets must not survive a source change"
        );
        // Device clock steps backwards 1 s.
        assert!(c.observe(TimeType::Ptp, epoch - 1_000 * MS, 1_030 * MS));
        // Device clock leaps forward 1 s while the host advanced 10 ms.
        assert!(c.observe(TimeType::Ptp, epoch + 1_000 * MS, 1_040 * MS));
        assert!(!c.observe(TimeType::Ptp, epoch + 1_010 * MS, 1_050 * MS));
        assert_eq!(c.resets(), 3);
    }

    #[test]
    fn a_late_datagram_is_not_a_jump() {
        let mut c = ClockMapper::new(ClockConfig::default());
        c.observe(TimeType::PowerOn, 100 * MS, 1_100 * MS);
        // Device time 3 ms older than the previous packet, received later.
        assert!(!c.observe(TimeType::PowerOn, 97 * MS, 1_102 * MS));
        assert!(!c.observe(TimeType::PowerOn, 110 * MS, 1_110 * MS));
        assert_eq!(c.resets(), 0);
        assert_eq!(c.offset_ns(), Some((1_000 * MS) as i128));
    }
}
