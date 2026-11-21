//! The LiDAR lane in the simulator's lockstep barrier.
//!
//! In lockstep (`simulation.clock_mode: lockstep`) the simulator steps past a
//! grant only once every barrier participant is done with it. Outside the
//! barrier a run at `--speed max` outpaces the pose provider: scans arrive in
//! bursts, the provider's short queue drops most of them and odometry
//! starves. So on a node following the simulator's clock the lane is a
//! participant from its second scan, once the scan period is known, and
//! reports done to:
//!
//! - nothing new while a scan is with the pose provider: the simulator holds
//!   at the boundary the scan arrived by until the provider answers for it
//!   (its newest estimate is at or past the scan's stamp), or for at most
//!   [`SCAN_BUDGET_MS`] of wall time;
//! - otherwise half a period past the time the next scan reaches the lane:
//!   a scan closes when the first sample of the one after it arrives (its
//!   source depth, [`SOURCE_DEPTH_SCANS`]), so the scan stamped `t` arrives
//!   once the simulator reaches `t + (1 + depth) * period`.
//!
//! So every scan is matched before the simulator runs a scan further, at any
//! speed, and a matcher that keeps up costs no wall time beyond its own.
//! Stamps are on the lane's timebase ([`crate::timebase`]), sim time here.
//!
//! A lane held past its next scan's arrival with no scan for
//! [`SCAN_PROBE_MS`] of wall time takes one scan more depth, up to
//! [`MAX_SOURCE_DEPTH_SCANS`]; at the deepest, [`SCAN_SILENCE_MS`] more
//! without a scan (a LiDAR that failed or stopped) and it leaves the barrier,
//! under World Engine's default 2 s participant timeout. It joins again from
//! the second scan once scans return. A stamp going back (a new run) leaves
//! too. Dropping the lane (the service stops or restarts the run) closes the
//! connection, which leaves the barrier.

use std::collections::VecDeque;
use std::path::PathBuf;
use std::time::Instant;

use ados_protocol::simclock::{participants_socket_beside, Clock, Participant, CLOCK_FEED_ENV};

/// Scans a source holds back: a scan is handed over once the next one's
/// first sample arrives (the RPLIDAR's next start flag, the Livox window's
/// end).
pub const SOURCE_DEPTH_SCANS: u64 = 1;
/// The deepest source the lane learns before it leaves the barrier instead.
pub const MAX_SOURCE_DEPTH_SCANS: u64 = 3;
/// Wall time, ms, the lane is held past its next scan's arrival with no scan
/// before it takes one scan more depth.
pub const SCAN_PROBE_MS: u64 = 250;
/// Wall time, ms, a lane at the deepest waits for a scan that should have
/// come before it leaves the barrier.
pub const SCAN_SILENCE_MS: u64 = 1_000;
/// Wall time, ms, a scan may stay with the pose provider before the lane
/// stops holding the simulator for it.
pub const SCAN_BUDGET_MS: u64 = 500;
/// Weight of the newest scan interval in the learned period.
const PERIOD_ALPHA: f64 = 0.25;

/// What [`ScanPacing::check`] did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Paced {
    /// The provider did not answer for the scan stamped this within
    /// [`SCAN_BUDGET_MS`]; the lane no longer waits for it.
    Overrun(u64),
    /// The source is deeper than thought: now this many scans.
    Deeper(u64),
    /// No scan came: the lane leaves the barrier until scans return.
    Gone,
}

/// The pure pacing state of the lane: its scans' stamps, period, source
/// depth and the scans with the provider. Times are ns on the lane's
/// timebase, wall times ms.
#[derive(Debug, Clone)]
pub struct ScanPacing {
    last_ns: Option<u64>,
    period_ns: Option<u64>,
    depth: u64,
    /// `(stamp, wall ms fed)` of the scans the provider has not answered.
    inflight: VecDeque<(u64, u64)>,
    /// Wall ms since which the simulator is past the next scan's arrival.
    overdue_since_ms: Option<u64>,
}

impl Default for ScanPacing {
    fn default() -> Self {
        Self {
            last_ns: None,
            period_ns: None,
            depth: SOURCE_DEPTH_SCANS,
            inflight: VecDeque::new(),
            overdue_since_ms: None,
        }
    }
}

impl ScanPacing {
    /// The learned scan period, ns.
    pub fn period_ns(&self) -> Option<u64> {
        self.period_ns
    }

    /// The source depth, scans.
    pub fn depth(&self) -> u64 {
        self.depth
    }

    /// Scans with the provider.
    pub fn inflight(&self) -> usize {
        self.inflight.len()
    }

    /// A scan stamped `stamp_ns` went to the provider at wall `wall_ms`.
    /// Returns false for a stamp before the last one (a new run): the state
    /// starts over, keeping the learned depth.
    pub fn fed(&mut self, stamp_ns: u64, wall_ms: u64) -> bool {
        let fresh = self.last_ns.is_none_or(|l| stamp_ns >= l);
        if !fresh {
            self.restart();
        }
        if let Some(last) = self.last_ns.filter(|l| stamp_ns > *l) {
            let dt = stamp_ns - last;
            self.period_ns = Some(match self.period_ns {
                None => dt,
                // A missed scan's double interval is no period.
                Some(p) if dt * 2 < p || dt > p + p / 2 => p,
                Some(p) => (PERIOD_ALPHA * dt as f64 + (1.0 - PERIOD_ALPHA) * p as f64) as u64,
            });
        }
        self.last_ns = Some(self.last_ns.map_or(stamp_ns, |l| l.max(stamp_ns)));
        self.overdue_since_ms = None;
        self.inflight.push_back((stamp_ns, wall_ms));
        fresh
    }

    /// The provider's newest estimate is at `newest_ns`: every scan at or
    /// before it is answered. `None` (a provider with no track: starting,
    /// restarting, or none at all) answers every scan.
    pub fn answered(&mut self, newest_ns: Option<u64>) {
        match newest_ns {
            None => self.inflight.clear(),
            Some(n) => self.inflight.retain(|(t, _)| *t > n),
        }
    }

    /// When the scan after the last one reaches the lane.
    fn expected_ns(&self) -> Option<u64> {
        Some(self.last_ns? + self.period_ns? * (1 + self.depth))
    }

    /// What the lane reports done to: `None` while a scan is with the
    /// provider (the report stays where it was) or before the period is
    /// known; else half a period past the next scan's arrival.
    pub fn done_target(&self) -> Option<u64> {
        if !self.inflight.is_empty() {
            return None;
        }
        Some(self.expected_ns()? + self.period_ns? / 2)
    }

    /// At sim time `sim_now_ns` and wall time `wall_ms`: scans with the
    /// provider past [`SCAN_BUDGET_MS`] are let go; a lane past its next
    /// scan's arrival with no scan for [`SCAN_PROBE_MS`] takes one more
    /// depth, and at [`MAX_SOURCE_DEPTH_SCANS`] for [`SCAN_SILENCE_MS`] it
    /// is gone (its state starts over).
    pub fn check(&mut self, sim_now_ns: Option<u64>, wall_ms: u64) -> Vec<Paced> {
        let mut out = Vec::new();
        while let Some(&(t, at)) = self.inflight.front() {
            if wall_ms.saturating_sub(at) < SCAN_BUDGET_MS {
                break;
            }
            self.inflight.pop_front();
            out.push(Paced::Overrun(t));
        }
        let late = self.inflight.is_empty()
            && self
                .expected_ns()
                .zip(sim_now_ns)
                .is_some_and(|(e, now)| now > e);
        if !late {
            self.overdue_since_ms = None;
            return out;
        }
        let waited = wall_ms.saturating_sub(*self.overdue_since_ms.get_or_insert(wall_ms));
        if self.depth < MAX_SOURCE_DEPTH_SCANS {
            if waited >= SCAN_PROBE_MS {
                self.depth += 1;
                self.overdue_since_ms = None;
                out.push(Paced::Deeper(self.depth));
            }
        } else if waited >= SCAN_SILENCE_MS {
            self.restart();
            out.push(Paced::Gone);
        }
        out
    }

    /// Start over (gone, or a new run), keeping the learned depth.
    fn restart(&mut self) {
        *self = Self {
            depth: self.depth,
            ..Self::default()
        };
    }
}

/// The lane's participation: the pacing state and its barrier registration.
pub struct LidarLockstep {
    clock: Clock,
    socket: PathBuf,
    pacing: ScanPacing,
    part: Option<Participant>,
    origin: Instant,
}

impl LidarLockstep {
    /// The barrier's participants socket on a node following the
    /// simulator's clock: beside the relay the host named in
    /// `ADOS_CLOCK_FEED`. `None` on wall time.
    pub fn socket_from_env() -> Option<PathBuf> {
        std::env::var_os(CLOCK_FEED_ENV)
            .filter(|p| !p.is_empty())
            .map(|p| participants_socket_beside(std::path::Path::new(&p)))
    }

    /// A lane on `clock` reporting to the barrier at `socket`. It registers
    /// only from its second scan. Needs a Tokio runtime.
    pub fn new(clock: Clock, socket: PathBuf) -> Self {
        Self {
            clock,
            socket,
            pacing: ScanPacing::default(),
            part: None,
            origin: Instant::now(),
        }
    }

    /// Whether the lane is registered with the barrier.
    pub fn joined(&self) -> bool {
        self.part.is_some()
    }

    pub fn pacing(&self) -> &ScanPacing {
        &self.pacing
    }

    fn wall_ms(&self) -> u64 {
        u64::try_from(self.origin.elapsed().as_millis()).unwrap_or(u64::MAX)
    }

    /// A scan stamped `stamp_ns` went to the pose provider.
    pub fn fed(&mut self, stamp_ns: u64) {
        let wall = self.wall_ms();
        if !self.pacing.fed(stamp_ns, wall) && self.part.take().is_some() {
            tracing::info!(
                stamp_ns,
                "nav_lockstep_left: scan time went back, joining afresh"
            );
        }
    }

    /// Pace after the lane's work: the provider's newest estimate is at
    /// `newest_ns` (`None`: it has no track), or with no provider every scan
    /// is answered once the lane has handled it. Reports to the barrier.
    pub fn poll(&mut self, newest_ns: Option<u64>, provider: bool) {
        self.pacing
            .answered(if provider { newest_ns } else { None });
        let wall = self.wall_ms();
        for p in self.pacing.check(self.clock.sim_ns(), wall) {
            match p {
                Paced::Overrun(stamp_ns) => tracing::warn!(
                    stamp_ns,
                    budget_ms = SCAN_BUDGET_MS,
                    "nav_lockstep_scan_overrun: the pose provider did not answer in time"
                ),
                Paced::Deeper(depth) => {
                    tracing::info!(depth, "nav_lockstep_deeper: scans arrive one later")
                }
                Paced::Gone => {
                    if self.part.take().is_some() {
                        tracing::warn!(
                            silence_ms = SCAN_SILENCE_MS,
                            "nav_lockstep_left: no scan, leaving the barrier until scans return"
                        );
                    }
                }
            }
        }
        let Some(done) = self.pacing.done_target() else {
            return;
        };
        if self.part.is_none() {
            tracing::info!(
                period_ns = self.pacing.period_ns(),
                "nav_lockstep_joined: the simulator waits for each scan's pose"
            );
        }
        self.part
            .get_or_insert_with(|| Participant::connect(self.socket.clone()))
            .done(done);
    }
}

#[cfg(test)]
#[path = "lockstep_tests.rs"]
mod tests;
