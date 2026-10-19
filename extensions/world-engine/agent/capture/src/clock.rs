//! Host clock reads and the host-monotonic → session-timebase map.
//!
//! Every producer on the sensor bus and the framebus stamps host
//! `CLOCK_MONOTONIC` nanoseconds. The agent's clock service publishes
//! `ClockSample`s relating that clock to the timebase a session should use
//! (`/run/ados/clock.json` names it), so the recorder converts each stamp once
//! with [`ClockMap::to_session`].

use std::path::{Path, PathBuf};

use ados_capture_contract::v1::ClockSample;
use ados_capture_contract::Timebase;
use serde::Deserialize;

/// Host `CLOCK_MONOTONIC`, nanoseconds.
pub fn mono_ns() -> u64 {
    match nix::time::clock_gettime(nix::time::ClockId::CLOCK_MONOTONIC) {
        Ok(ts) => (ts.tv_sec() as i128 * 1_000_000_000 + ts.tv_nsec() as i128).max(0) as u64,
        Err(_) => 0,
    }
}

/// Host `CLOCK_REALTIME`, nanoseconds since the Unix epoch.
pub fn realtime_ns() -> i64 {
    match nix::time::clock_gettime(nix::time::ClockId::CLOCK_REALTIME) {
        Ok(ts) => ts.tv_sec() * 1_000_000_000 + ts.tv_nsec(),
        Err(_) => 0,
    }
}

/// Env override of the clock sidecar path.
pub const CLOCK_SIDECAR_ENV: &str = "ADOS_CLOCK_SIDECAR";

/// `/run/ados/clock.json`.
pub fn clock_sidecar_path() -> PathBuf {
    std::env::var_os(CLOCK_SIDECAR_ENV)
        .map(PathBuf::from)
        .unwrap_or_else(|| ados_protocol::paths::run_path("clock.json"))
}

/// The fields of `clock.json` the recorder reads.
#[derive(Debug, Clone, Deserialize, PartialEq)]
pub struct ClockSidecar {
    pub timebase: String,
    #[serde(default)]
    pub locked: bool,
    #[serde(default = "unmeasured")]
    pub sigma_ns: i64,
}

fn unmeasured() -> i64 {
    -1
}

impl ClockSidecar {
    pub fn read(path: &Path) -> Option<Self> {
        serde_json::from_slice(&std::fs::read(path).ok()?).ok()
    }

    /// The contract timebase this sidecar names (unknown → host monotonic).
    pub fn timebase(&self) -> Timebase {
        parse_timebase(&self.timebase)
    }
}

pub fn parse_timebase(s: &str) -> Timebase {
    match s {
        "gnss" => Timebase::Gnss,
        "hub_freerun" => Timebase::HubFreerun,
        "sim" => Timebase::Sim,
        _ => Timebase::HostMonotonic,
    }
}

pub fn timebase_str(t: Timebase) -> &'static str {
    match t {
        Timebase::Gnss => "gnss",
        Timebase::HubFreerun => "hub_freerun",
        Timebase::Sim => "sim",
        Timebase::HostMonotonic => "host_monotonic",
    }
}

/// Maps host monotonic ns onto a session timebase from the two latest clock
/// samples (offset + rate). With no sample the map is the identity, which is
/// exact for `host_monotonic`.
#[derive(Debug, Clone, Default)]
pub struct ClockMap {
    prev: Option<ClockSample>,
    last: Option<ClockSample>,
}

impl ClockMap {
    pub fn new() -> Self {
        Self::default()
    }

    /// Fold in one bus `clock` sample. A sample not newer on the host clock
    /// than the last one is ignored.
    pub fn observe(&mut self, s: ClockSample) {
        if self
            .last
            .as_ref()
            .is_some_and(|l| s.host_mono_ns <= l.host_mono_ns)
        {
            return;
        }
        self.prev = self.last.take();
        self.last = Some(s);
    }

    pub fn last(&self) -> Option<&ClockSample> {
        self.last.as_ref()
    }

    /// Session-timebase ns of host monotonic `host_ns`.
    pub fn to_session(&self, host_ns: u64) -> u64 {
        let Some(l) = &self.last else {
            return host_ns;
        };
        let rate = match &self.prev {
            Some(p) if l.host_mono_ns > p.host_mono_ns => {
                let r = (l.t_ns as f64 - p.t_ns as f64) / (l.host_mono_ns - p.host_mono_ns) as f64;
                // A rate far from 1 is a source jump (a sim reset), not drift.
                if (r - 1.0).abs() < 1e-3 {
                    r
                } else {
                    1.0
                }
            }
            _ => 1.0,
        };
        let dh = host_ns as f64 - l.host_mono_ns as f64;
        (l.t_ns as f64 + dh * rate).max(0.0).round() as u64
    }

    /// UTC ns at host monotonic `host_ns`, when the samples carry UTC.
    pub fn utc_at(&self, host_ns: u64) -> Option<i64> {
        let l = self.last.as_ref()?;
        let utc = l.utc_ns?;
        Some(utc + (host_ns as i64 - l.host_mono_ns as i64))
    }

    /// Lock state and sigma of the latest sample (`-1` when none).
    pub fn lock(&self) -> (bool, i64) {
        self.last
            .as_ref()
            .map_or((false, -1), |l| (l.locked, l.sigma_ns))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(t: u64, host: u64) -> ClockSample {
        ClockSample {
            t_ns: t,
            host_mono_ns: host,
            utc_ns: Some(1_700_000_000_000_000_000 + host as i64),
            locked: true,
            sigma_ns: 400_000,
        }
    }

    #[test]
    fn identity_without_samples_and_offset_with_one() {
        let mut m = ClockMap::new();
        assert_eq!(m.to_session(123), 123);
        m.observe(sample(5_000_000_000, 1_000_000_000));
        assert_eq!(m.to_session(1_500_000_000), 5_500_000_000);
        assert_eq!(m.lock(), (true, 400_000));
    }

    #[test]
    fn two_samples_carry_the_rate() {
        let mut m = ClockMap::new();
        m.observe(sample(0, 1_000_000_000));
        // 40 ppm fast source.
        m.observe(sample(1_000_040_000, 2_000_000_000));
        let t = m.to_session(3_000_000_000);
        assert!((t as i64 - 2_000_080_000).abs() <= 1, "{t}");
        // A stale sample does not rewind the map.
        m.observe(sample(0, 1_500_000_000));
        assert_eq!(m.last().unwrap().host_mono_ns, 2_000_000_000);
    }

    #[test]
    fn a_jump_is_not_taken_as_a_rate() {
        let mut m = ClockMap::new();
        m.observe(sample(0, 1_000_000_000));
        m.observe(sample(50_000_000_000, 2_000_000_000));
        assert_eq!(m.to_session(2_000_000_100), 50_000_000_100);
    }

    #[test]
    fn sidecar_names_the_timebase() {
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("clock.json");
        std::fs::write(
            &p,
            r#"{"timebase":"sim","locked":true,"sigma_ns":1000,"sources":[]}"#,
        )
        .unwrap();
        let s = ClockSidecar::read(&p).unwrap();
        assert_eq!(s.timebase(), Timebase::Sim);
        assert_eq!(timebase_str(s.timebase()), "sim");
        assert!(ClockSidecar::read(&dir.path().join("none")).is_none());
    }
}
