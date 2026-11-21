//! The service's timebase: the clock every stamp the lane pairs is read on.
//!
//! On wall time it is the host's monotonic clock, the sensor bus timebase.
//! On a node following the simulator's clock ([`ados_sdk::clock`] is sim
//! time) it is sim time: the LiDAR drivers' receive stamps, the flight
//! controller's frames, the depth frames and the lane's own "now" all read
//! it, so a scan, the IMU that deskews it and the pose it gets stay paired
//! at any simulator speed. Host time would compress every gap between them
//! by the speed of a run faster than real time (a 100 ms scan period read as
//! 25 ms at 4x), and the pose interval would thin the poses by as much.
//!
//! On sim time the first read must run inside a Tokio runtime (it starts the
//! relay follower); the supervisor makes it before any driver starts.

use ados_protocol::simclock::Clock;

/// Now on `clock`'s timebase: sim time on a node following the simulator
/// (`None` before its first tick), else host monotonic time.
pub fn stamp_on(clock: &Clock) -> Option<u64> {
    match clock {
        Clock::Sim(_) => clock.sim_ns(),
        Clock::Wall(_) => Some(ados_mapping_pose::clock::mono_ns()),
    }
}

/// Now on the plugin clock's timebase ([`stamp_on`]).
pub fn stamp() -> Option<u64> {
    stamp_on(&ados_sdk::clock())
}

/// [`stamp`], 0 before the simulator's first tick: the receive clock the
/// LiDAR drivers stamp on.
pub fn now_ns() -> u64 {
    stamp().unwrap_or(0)
}

/// An age on `clock`'s timebase as wall time: divided by the simulator's
/// speed on sim time, unchanged on wall time and while the simulator runs
/// as fast as it can (no fixed ratio).
pub fn wall_age_ns(clock: &Clock, age_ns: u64) -> u64 {
    match clock.wall_to_ados() {
        Some(k) if k.is_finite() && k > 0.0 => (age_ns as f64 / k) as u64,
        _ => age_ns,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ados_protocol::simclock::{wire::Tick, SimFeed};

    fn tick(feed: &SimFeed, seq: u32, sim_ns: u64, speed: f64) {
        let t = Tick {
            flags: 0,
            seq,
            sim_ns,
            grant_ns: sim_ns,
            speed,
        };
        assert!(feed.ingest(&t, std::time::Instant::now()));
    }

    #[test]
    fn wall_time_is_the_host_monotonic_clock() {
        let clock = Clock::wall();
        let before = ados_mapping_pose::clock::mono_ns();
        let now = stamp_on(&clock).unwrap();
        assert!(now >= before && now <= ados_mapping_pose::clock::mono_ns());
        assert_eq!(wall_age_ns(&clock, 50_000_000), 50_000_000);
    }

    #[test]
    fn sim_time_is_the_feed_and_ages_scale_by_its_speed() {
        let feed = SimFeed::new();
        let clock = Clock::Sim(feed.clone());
        assert_eq!(stamp_on(&clock), None, "no stamp before the first tick");
        tick(&feed, 1, 7_000_000_000, 4.0);
        assert_eq!(stamp_on(&clock), Some(7_000_000_000));
        assert_eq!(wall_age_ns(&clock, 200_000_000), 50_000_000, "200 ms at 4x");
        tick(&feed, 2, 7_020_000_000, 0.0);
        assert_eq!(
            wall_age_ns(&clock, 200_000_000),
            200_000_000,
            "as fast as it can"
        );
    }
}
