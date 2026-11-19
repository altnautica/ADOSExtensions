//! EKF source-set switching (`MAV_CMD_SET_EKF_SOURCE_SET`, 42007).
//!
//! Two policies, picked by `external_nav.gps_primary`:
//!
//! * External nav primary (`false`): the external-nav set is active while
//!   the pose is tracked (good or degraded). Lost (or no pose) for 1 s
//!   switches to the fallback set (the flight controller's optical-flow
//!   set); tracked for 3 s switches back. A degraded pose re-engages too:
//!   a planar lidar between two parallel walls (an aisle, a drift) is
//!   degenerate along them for as long as it stays there, and its
//!   ODOMETRY carries the degraded covariance and quality, so the EKF
//!   weighs it; waiting for `good` left the flow set active for the whole
//!   corridor.
//! * GPS primary (`true`): the fallback (GPS) set is active while the GPS
//!   is usable: a 3D fix (`GPS_RAW_INT.fix_type >= 3`) whose reported
//!   horizontal accuracy (`h_acc`, when the receiver sends it) is within
//!   [`GPS_MAX_H_ACC_MM`]. A jammed or spoofed receiver often keeps its 3D
//!   fix while its accuracy collapses, so the fix type alone is not enough.
//!   No usable GPS for 2 s with a good pose switches to the external-nav
//!   set; usable GPS for 10 s switches back.
//!
//! Pure: the caller feeds time, health and whether the GPS is usable
//! ([`gps_usable`]) each tick and sends the command the machine returns.
//! Nothing is commanded before the first decision, so the flight controller
//! keeps its boot source set until the inputs say otherwise.

use crate::external_nav::ExtnavHealth;
use crate::nav_config::ExternalNavConfig;

const EXTNAV_LOST_NS: i64 = 1_000_000_000;
const EXTNAV_TRACKED_NS: i64 = 3_000_000_000;
const GPS_LOST_NS: i64 = 2_000_000_000;
const GPS_BACK_NS: i64 = 10_000_000_000;
/// `GPS_FIX_TYPE_3D_FIX`.
const FIX_3D: u8 = 3;
/// A GPS reporting a horizontal accuracy worse than this (mm) is degraded.
pub const GPS_MAX_H_ACC_MM: u32 = 5_000;

/// Whether the GPS is usable as the primary source: a 3D fix whose
/// `h_acc` (when known) is within [`GPS_MAX_H_ACC_MM`].
pub fn gps_usable(fix_type: u8, h_acc_mm: Option<u32>) -> bool {
    fix_type >= FIX_3D && h_acc_mm.is_none_or(|a| a <= GPS_MAX_H_ACC_MM)
}

/// The state machine.
pub struct SourceSwitch {
    extnav_set: u8,
    fallback_set: u8,
    gps_primary: bool,
    active: Option<u8>,
    tracked_since: Option<i64>,
    good_since: Option<i64>,
    lost_since: Option<i64>,
    fix_since: Option<i64>,
    no_fix_since: Option<i64>,
}

impl SourceSwitch {
    pub fn new(cfg: &ExternalNavConfig) -> Self {
        Self {
            extnav_set: cfg.extnav_set,
            fallback_set: cfg.fallback_set,
            gps_primary: cfg.gps_primary,
            active: None,
            tracked_since: None,
            good_since: None,
            lost_since: None,
            fix_since: None,
            no_fix_since: None,
        }
    }

    /// The set last commanded, `None` before the first switch.
    pub fn active(&self) -> Option<u8> {
        self.active
    }

    /// Advance one tick. `health` is `None` when no fresh pose exists
    /// (treated as lost); `gps_ok` is [`gps_usable`] for this tick. Returns
    /// the set to command on a change.
    pub fn update(
        &mut self,
        now_ns: i64,
        health: Option<ExtnavHealth>,
        gps_ok: bool,
    ) -> Option<u8> {
        let health = health.unwrap_or(ExtnavHealth::Lost);
        since(&mut self.good_since, health == ExtnavHealth::Good, now_ns);
        since(
            &mut self.tracked_since,
            health != ExtnavHealth::Lost,
            now_ns,
        );
        since(&mut self.lost_since, health == ExtnavHealth::Lost, now_ns);
        since(&mut self.fix_since, gps_ok, now_ns);
        since(&mut self.no_fix_since, !gps_ok, now_ns);
        let held = |s: Option<i64>, d: i64| s.is_some_and(|t| now_ns - t >= d);

        let want = if self.gps_primary {
            let on_extnav = self.active == Some(self.extnav_set);
            if on_extnav {
                held(self.fix_since, GPS_BACK_NS).then_some(self.fallback_set)
            } else if health == ExtnavHealth::Good && held(self.no_fix_since, GPS_LOST_NS) {
                Some(self.extnav_set)
            } else if self.active.is_none() && gps_ok {
                Some(self.fallback_set)
            } else {
                None
            }
        } else {
            let on_extnav = self.active == Some(self.extnav_set);
            if on_extnav {
                held(self.lost_since, EXTNAV_LOST_NS).then_some(self.fallback_set)
            } else if held(self.tracked_since, EXTNAV_TRACKED_NS) {
                Some(self.extnav_set)
            } else if self.active.is_none() && held(self.lost_since, EXTNAV_LOST_NS) {
                Some(self.fallback_set)
            } else {
                None
            }
        };
        match want {
            Some(set) if self.active != Some(set) => {
                self.active = Some(set);
                Some(set)
            }
            _ => None,
        }
    }
}

/// Track since when `cond` has held continuously.
fn since(slot: &mut Option<i64>, cond: bool, now_ns: i64) {
    match (cond, *slot) {
        (true, None) => *slot = Some(now_ns),
        (false, _) => *slot = None,
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const S: i64 = 1_000_000_000;

    fn switch(gps_primary: bool) -> SourceSwitch {
        SourceSwitch::new(&ExternalNavConfig {
            gps_primary,
            ..ExternalNavConfig::default()
        })
    }

    /// Run 100 ms ticks over `[from, to)` and collect the commands.
    fn run(
        s: &mut SourceSwitch,
        from: i64,
        to: i64,
        h: Option<ExtnavHealth>,
        fix: u8,
    ) -> Vec<(i64, u8)> {
        run_gps(s, from, to, h, fix, None)
    }

    fn run_gps(
        s: &mut SourceSwitch,
        from: i64,
        to: i64,
        h: Option<ExtnavHealth>,
        fix: u8,
        h_acc_mm: Option<u32>,
    ) -> Vec<(i64, u8)> {
        let mut out = Vec::new();
        let mut t = from;
        while t < to {
            if let Some(set) = s.update(t, h, gps_usable(fix, h_acc_mm)) {
                out.push((t, set));
            }
            t += S / 10;
        }
        out
    }

    #[test]
    fn extnav_primary_engages_after_three_tracked_seconds() {
        let mut s = switch(false);
        let cmds = run(&mut s, 0, 4 * S, Some(ExtnavHealth::Good), 0);
        assert_eq!(cmds, vec![(3 * S, 1)]);
        assert_eq!(s.active(), Some(1));
        // A degraded pose (a planar lidar in a corridor) engages as well.
        let mut s = switch(false);
        let cmds = run(&mut s, 0, 4 * S, Some(ExtnavHealth::Degraded), 0);
        assert_eq!(cmds, vec![(3 * S, 1)]);
    }

    #[test]
    fn extnav_primary_falls_back_after_one_lost_second_and_returns() {
        let mut s = switch(false);
        run(&mut s, 0, 4 * S, Some(ExtnavHealth::Good), 0);
        // Degraded holds the external-nav set.
        assert!(run(&mut s, 4 * S, 6 * S, Some(ExtnavHealth::Degraded), 0).is_empty());
        // A short loss does not switch.
        assert!(run(&mut s, 6 * S, 6 * S + S / 2, None, 0).is_empty());
        assert!(run(&mut s, 6 * S + S / 2, 7 * S, Some(ExtnavHealth::Good), 0).is_empty());
        let lost = run(&mut s, 7 * S, 9 * S, Some(ExtnavHealth::Lost), 0);
        assert_eq!(lost, vec![(8 * S, 2)]);
        let back = run(&mut s, 9 * S, 13 * S, Some(ExtnavHealth::Good), 0);
        assert_eq!(back, vec![(12 * S, 1)]);
    }

    #[test]
    fn extnav_primary_returns_from_the_startup_fallback_on_a_degraded_pose() {
        // Lost until the first submap fills, then degraded in an aisle: the
        // boot-time fallback must not become permanent.
        let mut s = switch(false);
        assert_eq!(run(&mut s, 0, 2 * S, None, 0), vec![(S, 2)]);
        let back = run(&mut s, 2 * S, 6 * S, Some(ExtnavHealth::Degraded), 0);
        assert_eq!(back, vec![(5 * S, 1)]);
    }

    #[test]
    fn extnav_primary_commands_fallback_when_no_pose_ever_arrives() {
        let mut s = switch(false);
        assert_eq!(run(&mut s, 0, 2 * S, None, 0), vec![(S, 2)]);
    }

    #[test]
    fn gps_primary_starts_on_gps_and_takes_over_on_fix_loss() {
        let mut s = switch(true);
        assert_eq!(run(&mut s, 0, S, Some(ExtnavHealth::Good), 3), vec![(0, 2)]);
        // Fix lost for under 2 s: stay on GPS.
        assert!(run(&mut s, S, 2 * S + S / 2, Some(ExtnavHealth::Good), 0).is_empty());
        let take = run(&mut s, 2 * S + S / 2, 5 * S, Some(ExtnavHealth::Good), 1);
        assert_eq!(take, vec![(3 * S, 1)]);
        // A short fix return is not enough; 10 s of 3D fix is.
        assert!(run(&mut s, 5 * S, 10 * S, Some(ExtnavHealth::Good), 3).is_empty());
        let back = run(&mut s, 10 * S, 20 * S, Some(ExtnavHealth::Good), 3);
        assert_eq!(back, vec![(15 * S, 2)]);
    }

    #[test]
    fn gps_primary_does_not_take_over_without_a_good_pose() {
        let mut s = switch(true);
        run(&mut s, 0, S, Some(ExtnavHealth::Good), 3);
        assert!(run(&mut s, S, 6 * S, Some(ExtnavHealth::Degraded), 0).is_empty());
        assert_eq!(s.active(), Some(2));
    }

    /// A receiver that keeps its 3D fix while its accuracy collapses (8 m)
    /// is degraded: after 2 s the external-nav set takes over.
    #[test]
    fn gps_primary_takes_over_when_a_3d_fix_loses_its_accuracy() {
        let mut s = switch(true);
        let good = Some(ExtnavHealth::Good);
        assert_eq!(run_gps(&mut s, 0, S, good, 3, Some(1500)), vec![(0, 2)]);
        let take = run_gps(&mut s, S, 4 * S, good, 3, Some(8000));
        assert_eq!(take, vec![(3 * S, 1)]);
        // Accuracy back within 5 m for 10 s: back to GPS.
        let back = run_gps(&mut s, 4 * S, 15 * S, good, 3, Some(5000));
        assert_eq!(back, vec![(14 * S, 2)]);
    }

    #[test]
    fn gps_usable_needs_a_3d_fix_and_a_known_accuracy_within_5_m() {
        assert!(gps_usable(3, None));
        assert!(gps_usable(6, Some(GPS_MAX_H_ACC_MM)));
        assert!(!gps_usable(3, Some(GPS_MAX_H_ACC_MM + 1)));
        assert!(!gps_usable(2, Some(100)));
    }
}
