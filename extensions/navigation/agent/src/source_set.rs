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
//!   The external-nav set takes over after no usable GPS for 2 s with a
//!   good pose, and only when the flight controller has reported an EKF
//!   origin: EKF3 sets its origin from the GPS (`readGpsData` on every
//!   fusion step, whichever set is active), so an origin first set after
//!   the takeover leaves the external-nav position in a frame of its own,
//!   and the switch back resets the EKF position by the whole offset. A
//!   receiver still starting at boot (or a fix that flickers for a tick)
//!   sets no origin, so nothing takes over then and the flight controller
//!   arms on its GPS set. Usable GPS for 10 s switches back, and so does
//!   any usable GPS once the pose has been lost for 1 s. Armed, a switch
//!   to GPS happens only when the origin already existed at the takeover,
//!   and the first command after a start (the flight controller's set is
//!   then unknown) waits for the vehicle to be disarmed. In Guided with a
//!   position target ArduPilot slews the target through an EKF position
//!   reset (`ModeGuided::move_vehicle_on_ekf_reset`), so the vehicle flies
//!   the offset: a boot takeover without an origin and a return in flight
//!   flew 21 m at up to 7.8 m/s past a trunk at 0.32 m.
//!   When a switch is withheld the machine says why ([`SourceHold`]).
//!
//! Pure: the caller feeds time, health, whether the GPS is usable
//! ([`gps_usable`]) and the flight controller's state ([`Fc`]) each tick
//! and sends the command the machine returns. Nothing is commanded before
//! the first decision, so the flight controller keeps its boot source set
//! until the inputs say otherwise. A new flight-controller boot starts the
//! machine over.

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

/// What the flight controller reports that the switch waits on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Fc {
    /// The last heartbeat reported the vehicle armed.
    pub armed: bool,
    /// The flight controller has reported an EKF origin this boot.
    pub origin: bool,
    /// The flight-controller boot the reports belong to.
    pub boot_epoch: u64,
}

impl Fc {
    pub fn of(s: &crate::fc_nav::FcNavSnapshot) -> Self {
        Self {
            armed: s.armed,
            origin: s.origin_seen,
            boot_epoch: s.boot_epoch,
        }
    }
}

/// Why the GPS-primary machine holds the set it is on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceHold {
    /// The GPS is lost and the pose is good, but the flight controller has
    /// no EKF origin: the external-nav set does not take over.
    NoOrigin,
    /// On the external-nav set with the pose lost and no usable GPS: no
    /// source to switch to; the flight controller's EKF failsafe acts.
    NoPosition,
    /// Started while armed: the flight controller's set is unknown (a
    /// previous instance may have left it on external nav), so nothing is
    /// commanded until it is disarmed, or until the pose is lost with a
    /// usable GPS and an EKF origin, when the switch to GPS is the safe one.
    UnknownSet,
}

impl SourceHold {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::NoOrigin => "no_origin",
            Self::NoPosition => "no_position",
            Self::UnknownSet => "unknown_set",
        }
    }
}

/// The state machine.
pub struct SourceSwitch {
    extnav_set: u8,
    fallback_set: u8,
    gps_primary: bool,
    epoch: Option<u64>,
    active: Option<u8>,
    /// The flight controller had an EKF origin when the external-nav set
    /// took over (GPS primary).
    takeover_origin: bool,
    hold: Option<SourceHold>,
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
            epoch: None,
            active: None,
            takeover_origin: false,
            hold: None,
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

    /// Why the current set is held although a switch was wanted.
    pub fn hold(&self) -> Option<SourceHold> {
        self.hold
    }

    /// Advance one tick. `health` is `None` when no fresh pose exists
    /// (treated as lost); `gps_ok` is [`gps_usable`] for this tick. Returns
    /// the set to command on a change.
    pub fn update(
        &mut self,
        now_ns: i64,
        health: Option<ExtnavHealth>,
        gps_ok: bool,
        fc: Fc,
    ) -> Option<u8> {
        if self.epoch != Some(fc.boot_epoch) {
            // A new boot: the flight controller is back on its boot set
            // with no origin, so what the machine knew no longer holds.
            self.epoch = Some(fc.boot_epoch);
            self.active = None;
            self.takeover_origin = false;
            self.tracked_since = None;
            self.good_since = None;
            self.lost_since = None;
            self.fix_since = None;
            self.no_fix_since = None;
        }
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
        let lost = held(self.lost_since, EXTNAV_LOST_NS);
        self.hold = None;

        let want = if self.gps_primary {
            let on_extnav = self.active == Some(self.extnav_set);
            // A switch to GPS while armed moves the vehicle by the EKF
            // reset unless the external-nav frame was aligned to an origin.
            let gps_safe = !fc.armed || (on_extnav && self.takeover_origin);
            if on_extnav {
                let back = held(self.fix_since, GPS_BACK_NS) || (lost && gps_ok);
                if lost && !gps_ok {
                    self.hold = Some(SourceHold::NoPosition);
                }
                (back && gps_safe).then_some(self.fallback_set)
            } else if health == ExtnavHealth::Good && held(self.no_fix_since, GPS_LOST_NS) {
                if fc.origin {
                    self.takeover_origin = true;
                    Some(self.extnav_set)
                } else {
                    self.hold = Some(SourceHold::NoOrigin);
                    None
                }
            } else if self.active.is_none() && gps_ok && gps_safe {
                Some(self.fallback_set)
            } else if self.active.is_none() && fc.armed {
                if lost && gps_ok && fc.origin {
                    Some(self.fallback_set)
                } else {
                    self.hold = Some(SourceHold::UnknownSet);
                    None
                }
            } else {
                None
            }
        } else {
            let on_extnav = self.active == Some(self.extnav_set);
            if on_extnav {
                lost.then_some(self.fallback_set)
            } else if held(self.tracked_since, EXTNAV_TRACKED_NS) {
                Some(self.extnav_set)
            } else if self.active.is_none() && lost {
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

    /// Disarmed, with an EKF origin.
    const GROUND: Fc = Fc {
        armed: false,
        origin: true,
        boot_epoch: 1,
    };
    /// Disarmed, no EKF origin yet.
    const BOOTING: Fc = Fc {
        origin: false,
        ..GROUND
    };
    /// Armed, with an EKF origin.
    const FLYING: Fc = Fc {
        armed: true,
        ..GROUND
    };

    fn run_gps(
        s: &mut SourceSwitch,
        from: i64,
        to: i64,
        h: Option<ExtnavHealth>,
        fix: u8,
        h_acc_mm: Option<u32>,
    ) -> Vec<(i64, u8)> {
        run_fc(s, from, to, h, fix, h_acc_mm, GROUND)
    }

    fn run_fc(
        s: &mut SourceSwitch,
        from: i64,
        to: i64,
        h: Option<ExtnavHealth>,
        fix: u8,
        h_acc_mm: Option<u32>,
        fc: Fc,
    ) -> Vec<(i64, u8)> {
        let mut out = Vec::new();
        let mut t = from;
        while t < to {
            if let Some(set) = s.update(t, h, gps_usable(fix, h_acc_mm), fc) {
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

    /// The bench's GPS-primary configuration (external nav on set 2, GPS on
    /// set 1).
    fn bench_switch() -> SourceSwitch {
        SourceSwitch::new(&ExternalNavConfig {
            gps_primary: true,
            extnav_set: 2,
            fallback_set: 1,
            ..ExternalNavConfig::default()
        })
    }

    const GOOD: Option<ExtnavHealth> = Some(ExtnavHealth::Good);

    /// The logged forest follow take: a good lidar pose from boot, the GPS
    /// receiver fixing only later (here 6 s in) and the EKF origin after
    /// that, then armed. No origin, no takeover: nothing but the GPS set is
    /// commanded. (The take switched to set 2 before the fix and back to
    /// set 1 in flight 19.6 s into the run; the flight controller then
    /// flew 21 m to its displaced guided target.)
    #[test]
    fn gps_primary_waits_for_the_receiver_to_start_before_external_nav_takes_over() {
        let mut s = bench_switch();
        assert!(run_fc(&mut s, 0, 6 * S, GOOD, 0, None, BOOTING).is_empty());
        assert_eq!(s.hold(), Some(SourceHold::NoOrigin));
        let fixed = run_fc(&mut s, 6 * S, 12 * S, GOOD, 6, None, BOOTING);
        assert_eq!(fixed, vec![(6 * S, 1)]);
        assert!(run_fc(&mut s, 12 * S, 16 * S, GOOD, 6, None, GROUND).is_empty());
        assert!(run_fc(&mut s, 16 * S, 30 * S, GOOD, 6, None, FLYING).is_empty());
        assert_eq!(s.hold(), None);
    }

    /// A 3D fix for one 100 ms tick at boot, then none: no origin, so the
    /// external-nav set never takes over.
    #[test]
    fn a_one_tick_fix_flicker_at_boot_does_not_hand_over_to_external_nav() {
        let mut s = bench_switch();
        assert_eq!(
            run_fc(&mut s, 0, S / 10, GOOD, 3, None, BOOTING),
            vec![(0, 1)]
        );
        assert!(run_fc(&mut s, S / 10, 10 * S, GOOD, 0, None, BOOTING).is_empty());
        assert_eq!(s.active(), Some(1));
        assert_eq!(s.hold(), Some(SourceHold::NoOrigin));
    }

    /// Armed without an origin (the machine never takes over then) the GPS
    /// lost keeps the GPS set and says why; once the origin is there the
    /// external-nav set takes over aligned to it, and the GPS back for 10 s
    /// returns in flight: an origin first seen after a takeover cannot
    /// happen.
    #[test]
    fn the_takeover_waits_for_the_origin_and_the_return_follows_it() {
        let mut s = bench_switch();
        let armed_no_origin = Fc {
            origin: false,
            ..FLYING
        };
        run_fc(&mut s, 0, S, GOOD, 6, None, GROUND);
        assert!(run_fc(&mut s, S, 5 * S, GOOD, 0, None, armed_no_origin).is_empty());
        assert_eq!(s.hold(), Some(SourceHold::NoOrigin));
        let take = run_fc(&mut s, 5 * S, 6 * S, GOOD, 0, None, FLYING);
        assert_eq!(take, vec![(5 * S, 2)]);
        let back = run_fc(&mut s, 6 * S, 17 * S, GOOD, 6, None, FLYING);
        assert_eq!(back, vec![(16 * S, 1)]);
    }

    /// On external nav in flight with a degraded pose: it stays until the
    /// GPS has been back for 10 s, then returns; it is never stranded.
    #[test]
    fn a_degraded_pose_in_flight_returns_to_gps_once_it_is_back() {
        let mut s = bench_switch();
        run_fc(&mut s, 0, S, GOOD, 6, None, FLYING);
        assert_eq!(s.active(), None, "a start while armed commands nothing");
        assert_eq!(
            run_fc(&mut s, S, 4 * S, GOOD, 0, None, FLYING),
            vec![(3 * S, 2)]
        );
        let degraded = Some(ExtnavHealth::Degraded);
        assert!(run_fc(&mut s, 4 * S, 30 * S, degraded, 0, None, FLYING).is_empty());
        let back = run_fc(&mut s, 30 * S, 41 * S, degraded, 6, None, FLYING);
        assert_eq!(back, vec![(40 * S, 1)]);
    }

    /// Started while armed (the flight controller's set unknown) with a
    /// usable GPS: nothing is commanded and the machine says why. The pose
    /// lost switches to GPS only with an EKF origin present.
    #[test]
    fn a_start_while_armed_says_unknown_set_and_a_lost_pose_returns_to_gps() {
        let no_origin = Fc {
            origin: false,
            ..FLYING
        };
        let mut s = bench_switch();
        assert!(run_fc(&mut s, 0, 3 * S, GOOD, 6, None, FLYING).is_empty());
        assert_eq!(s.hold(), Some(SourceHold::UnknownSet));
        assert!(run_fc(&mut s, 3 * S, 6 * S, None, 6, None, no_origin).is_empty());
        assert_eq!(s.hold(), Some(SourceHold::UnknownSet));
        let back = run_fc(&mut s, 6 * S, 7 * S, None, 6, None, FLYING);
        assert_eq!(back, vec![(6 * S, 1)]);
        assert_eq!(s.hold(), None);
    }

    /// On external nav in flight, the pose lost with no usable GPS: nothing
    /// to switch to (the flight controller's EKF failsafe acts), and the
    /// machine says so; a usable GPS then returns at once.
    #[test]
    fn a_pose_lost_in_flight_without_gps_holds_and_says_no_position() {
        let mut s = bench_switch();
        run_fc(&mut s, 0, S, GOOD, 6, None, GROUND);
        assert_eq!(
            run_fc(&mut s, S, 4 * S, GOOD, 0, None, FLYING),
            vec![(3 * S, 2)]
        );
        assert!(run_fc(&mut s, 4 * S, 8 * S, None, 0, None, FLYING).is_empty());
        assert_eq!(s.hold(), Some(SourceHold::NoPosition));
        let back = run_fc(&mut s, 8 * S, 9 * S, None, 6, None, FLYING);
        assert_eq!(back, vec![(8 * S, 1)]);
    }

    /// A new flight-controller boot (it comes back on its GPS set with no
    /// origin) starts the machine over.
    #[test]
    fn a_flight_controller_reboot_starts_the_machine_over() {
        let mut s = bench_switch();
        run_fc(&mut s, 0, S, GOOD, 6, None, GROUND);
        assert_eq!(
            run_fc(&mut s, S, 4 * S, GOOD, 0, None, GROUND),
            vec![(3 * S, 2)]
        );
        let rebooted = Fc {
            boot_epoch: 2,
            ..BOOTING
        };
        assert!(run_fc(&mut s, 4 * S, 10 * S, GOOD, 0, None, rebooted).is_empty());
        assert_eq!(s.active(), None);
        assert_eq!(s.hold(), Some(SourceHold::NoOrigin));
        let fixed = run_fc(&mut s, 10 * S, 11 * S, GOOD, 6, None, rebooted);
        assert_eq!(fixed, vec![(10 * S, 1)]);
    }

    /// The pipeline feeds the machine the flight controller's armed state,
    /// origin and boot from its sink.
    #[test]
    fn the_flight_controller_state_comes_from_its_sink() {
        let sink = crate::fc_nav::FcNavSink::default();
        sink.on_heartbeat(false, 0);
        let booting = Fc::of(&sink.snapshot(0));
        assert_eq!(
            booting,
            Fc {
                boot_epoch: 1,
                ..BOOTING
            }
        );
        sink.on_origin();
        sink.on_heartbeat(true, S);
        assert_eq!(Fc::of(&sink.snapshot(S)), FLYING);
        // Silence long enough for a reboot: a new boot, no origin.
        sink.on_heartbeat(false, 60 * S);
        let rebooted = Fc::of(&sink.snapshot(60 * S));
        assert_eq!(
            rebooted,
            Fc {
                boot_epoch: 2,
                ..BOOTING
            }
        );
    }

    #[test]
    fn gps_usable_needs_a_3d_fix_and_a_known_accuracy_within_5_m() {
        assert!(gps_usable(3, None));
        assert!(gps_usable(6, Some(GPS_MAX_H_ACC_MM)));
        assert!(!gps_usable(3, Some(GPS_MAX_H_ACC_MM + 1)));
        assert!(!gps_usable(2, Some(100)));
    }
}
