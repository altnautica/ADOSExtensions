//! Host clock reads and the host-monotonic → session-timebase map.
//!
//! Every producer on the sensor bus and the framebus stamps host
//! `CLOCK_MONOTONIC` nanoseconds. The agent's clock service publishes
//! `ClockSample`s relating that clock to the timebase a session should use
//! (`/run/ados/sensing/clock.json` names it), so the recorder converts each stamp once
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

/// `/run/ados/sensing/clock.json` (registry id `clock`).
pub fn clock_sidecar_path() -> PathBuf {
    std::env::var_os(CLOCK_SIDECAR_ENV)
        .map(PathBuf::from)
        .unwrap_or_else(|| ados_protocol::paths::run_path("clock"))
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

/// Whether a session on `timebase` can open now: a non-host timebase needs
/// at least one bus clock sample on that timebase to map onto it.
pub fn timebase_ready(timebase: Timebase, map: &ClockMap) -> bool {
    timebase == Timebase::HostMonotonic || map.is_mapped()
}

/// Session-timebase ns of host monotonic `host_ns` for a session on
/// `timebase`. A `host_monotonic` session is the identity for its whole life,
/// whatever clock samples arrive meanwhile.
pub fn session_time(timebase: Timebase, map: &ClockMap, host_ns: u64) -> u64 {
    match timebase {
        Timebase::HostMonotonic => host_ns,
        _ => map.to_session(host_ns),
    }
}

/// Largest step a session's map takes from one clock sample. A sample further
/// from the map's prediction is on another timebase (the clock service fell
/// back to the host clock or switched source) and is refused.
pub const SESSION_STEP_TOL_NS: u64 = 50_000_000;

/// Maps host monotonic ns onto a session timebase from the clock samples
/// (an anchor plus the rate between the last two samples). With no sample the
/// map is the identity, which is exact for `host_monotonic`.
///
/// The clock service publishes `t_ns == host_mono_ns` when its timebase has
/// no fit (the host fallback); such a sample is not on the source timebase
/// and never moves the map.
#[derive(Debug, Clone, Default)]
pub struct ClockMap {
    /// The newest sample of any kind (lock state, UTC, liveness).
    latest: Option<ClockSample>,
    /// The last sample taken into the map, raw (rate reference).
    last: Option<ClockSample>,
    /// `(host_ns, session_ns)` the map extrapolates from.
    anchor: Option<(u64, u64)>,
    /// Session ns per host ns; `None` is 1.
    rate: Option<f64>,
    /// A session's own map: refuses steps over [`SESSION_STEP_TOL_NS`] and
    /// never moves mapped time backwards.
    guarded: bool,
    /// Samples a guarded map refused.
    pub rejected: u64,
}

impl ClockMap {
    pub fn new() -> Self {
        Self::default()
    }

    /// A session's own copy of this map, frozen against timebase switches
    /// for the session's life.
    pub fn for_session(&self) -> Self {
        Self {
            guarded: true,
            rejected: 0,
            ..self.clone()
        }
    }

    /// Whether a sample on the source timebase has been taken.
    pub fn is_mapped(&self) -> bool {
        self.anchor.is_some()
    }

    /// Fold in one bus `clock` sample. A sample not newer on the host clock
    /// than the newest one is ignored; a host-fallback sample only updates the
    /// lock state. A guarded map also refuses a sample more than
    /// [`SESSION_STEP_TOL_NS`] from its prediction, and takes one behind it
    /// without stepping back. Returns whether the map took the sample.
    pub fn observe(&mut self, s: ClockSample) -> bool {
        if self
            .latest
            .as_ref()
            .is_some_and(|l| s.host_mono_ns <= l.host_mono_ns)
        {
            return false;
        }
        self.latest = Some(s);
        if s.t_ns == s.host_mono_ns {
            return false;
        }
        let predicted = self.anchor.map(|_| self.to_session(s.host_mono_ns));
        let behind = match predicted {
            Some(p) if self.guarded => {
                if s.t_ns.abs_diff(p) > SESSION_STEP_TOL_NS {
                    self.rejected += 1;
                    return false;
                }
                s.t_ns < p
            }
            _ => false,
        };
        self.rate = self
            .last
            .filter(|l| l.host_mono_ns < s.host_mono_ns)
            .map(|l| (s.t_ns as f64 - l.t_ns as f64) / (s.host_mono_ns - l.host_mono_ns) as f64)
            // A rate far from 1 is a source jump (a sim reset), not drift.
            .filter(|r| (r - 1.0).abs() < 1e-3);
        // Behind the prediction: keep mapped time continuous at this instant
        // and let the new rate carry the correction.
        let t = match predicted {
            Some(p) if behind => p,
            _ => s.t_ns,
        };
        self.anchor = Some((s.host_mono_ns, t));
        self.last = Some(s);
        true
    }

    /// The newest sample seen.
    pub fn last(&self) -> Option<&ClockSample> {
        self.latest.as_ref()
    }

    /// Session-timebase ns of host monotonic `host_ns`.
    pub fn to_session(&self, host_ns: u64) -> u64 {
        let Some((h, t)) = self.anchor else {
            return host_ns;
        };
        let dh = host_ns as f64 - h as f64;
        (t as f64 + dh * self.rate.unwrap_or(1.0)).max(0.0).round() as u64
    }

    /// Host monotonic ns of source-timebase `t_ns` (the inverse of
    /// [`Self::to_session`]); `None` until the map has a sample.
    pub fn to_host(&self, t_ns: u64) -> Option<u64> {
        let (h, t) = self.anchor?;
        let dt = t_ns as f64 - t as f64;
        Some((h as f64 + dt / self.rate.unwrap_or(1.0)).max(0.0).round() as u64)
    }

    /// UTC ns at host monotonic `host_ns`, when the samples carry UTC.
    pub fn utc_at(&self, host_ns: u64) -> Option<i64> {
        let l = self.latest.as_ref()?;
        let utc = l.utc_ns?;
        Some(utc + (host_ns as i64 - l.host_mono_ns as i64))
    }

    /// Lock state and sigma of the latest sample (`-1` when none).
    pub fn lock(&self) -> (bool, i64) {
        self.latest
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
    fn to_host_inverts_the_map_with_its_rate() {
        let mut m = ClockMap::new();
        assert_eq!(m.to_host(5), None, "nothing mapped yet");
        m.observe(sample(0, 1_000_000_000));
        m.observe(sample(1_000_040_000, 2_000_000_000));
        for host in [1_500_000_000u64, 2_000_000_000, 9_000_000_000] {
            let back = m.to_host(m.to_session(host)).unwrap();
            assert!(back.abs_diff(host) <= 1, "{host} -> {back}");
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
    fn a_non_host_timebase_needs_a_sample() {
        let mut m = ClockMap::new();
        assert!(timebase_ready(Timebase::HostMonotonic, &m));
        assert!(!timebase_ready(Timebase::Sim, &m));
        assert!(!timebase_ready(Timebase::Gnss, &m));
        m.observe(sample(0, 1_000));
        assert!(timebase_ready(Timebase::Sim, &m));
    }

    #[test]
    fn a_host_session_stays_identity_after_samples_arrive() {
        let mut m = ClockMap::new();
        assert_eq!(session_time(Timebase::HostMonotonic, &m, 7_000), 7_000);
        m.observe(sample(50_000_000_000, 1_000_000_000));
        m.observe(sample(51_000_000_000, 2_000_000_000));
        assert_eq!(
            session_time(Timebase::HostMonotonic, &m, 2_500_000_000),
            2_500_000_000
        );
        assert_eq!(
            session_time(Timebase::Sim, &m, 2_500_000_000),
            51_500_000_000
        );
    }

    #[test]
    fn guarded_time_never_decreases_across_a_sample() {
        let mut shared = ClockMap::new();
        shared.observe(sample(10_000_000_000, 1_000_000_000));
        let mut m = shared.for_session();
        let mut prev = 0;
        let mut host = 1_000_000_000;
        // A reset of the source (t jumps back), a forward step, and a sample
        // slightly behind the prediction.
        let arrivals = [
            (1_500_000_000, sample(2_000_000_000, 1_500_000_000)),
            (2_000_000_000, sample(11_000_000_100, 2_000_000_000)),
            (2_500_000_000, sample(11_499_990_000, 2_500_000_000)),
        ];
        for (at, s) in arrivals {
            while host < at + 400_000_000 {
                if host == at {
                    m.observe(s);
                }
                let t = m.to_session(host);
                assert!(t >= prev, "t {t} < {prev} at host {host}");
                prev = t;
                host += 100_000_000;
            }
        }
        assert_eq!(m.rejected, 1);
        // The shared (unguarded) map takes a reset.
        assert!(shared.observe(sample(0, 3_000_000_000)));
    }

    #[test]
    fn a_session_map_ignores_host_fallback_and_source_switches() {
        // Hub free-run time is hours behind the host clock.
        let off = 40_000_000_000_000u64;
        let mut shared = ClockMap::new();
        // The clock service's host fallback is no mapping.
        shared.observe(sample(900_000_000 + off, 900_000_000 + off));
        assert!(!timebase_ready(Timebase::HubFreerun, &shared));
        shared.observe(sample(1_000_000_000, 1_000_000_000 + off));
        assert!(timebase_ready(Timebase::HubFreerun, &shared));
        let mut m = shared.for_session();
        for k in 1..60u64 {
            let host = 1_000_000_000 + off + k * 1_000_000_000;
            // Every other sample is the host fallback, every fifth a jump to
            // another source.
            let s = match k % 5 {
                0 => sample(1_700_000_000_000_000_000, host),
                _ if k % 2 == 0 => sample(host, host),
                _ => sample(1_000_000_000 + k * 1_000_000_000, host),
            };
            m.observe(s);
            let t = m.to_session(host + 500_000_000);
            assert_eq!(t, 1_000_000_000 + k * 1_000_000_000 + 500_000_000);
        }
        assert!(m.rejected >= 11, "{}", m.rejected);
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
