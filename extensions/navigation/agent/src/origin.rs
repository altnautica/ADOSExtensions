//! When the `external_nav` mode may set the flight controller's EKF origin.
//!
//! ArduPilot's EKF3 aligns its yaw at boot with the declination it has then
//! (`COMPASS_DEC`). An origin that arrives later lets it load the declination
//! table for that place and step the earth field; with `COMPASS_AUTODEC` on,
//! the step is absorbed as a z-gyro bias and the vehicle spins. ArduPilot
//! also accepts an origin while armed. So the plugin sends
//! `SET_GPS_GLOBAL_ORIGIN` only as a last resort, when all of these hold:
//!
//! * the flight controller has reported no origin this boot;
//! * GPS is not the primary source (the GPS sets the origin itself);
//! * the vehicle is disarmed;
//! * the current flight-controller boot is at least [`ORIGIN_WAIT_NS`] old,
//!   so a flight controller that sets its origin at boot (`AHRS_OPTIONS`
//!   bit 4 with `AHRS_ORIGIN_LAT/LON/ALT`) has had time to report it;
//! * `external_nav.origin_lla` is set.
//!
//! The `GPS_GLOBAL_ORIGIN` request goes out every [`ORIGIN_WAIT_NS`] while
//! no origin is reported. The rule also remembers whether the origin the
//! flight controller reports is one the plugin sent this boot, which the
//! `ekf_origin` pre-arm row needs: a late origin is only a hazard when it
//! came after the yaw alignment.

use crate::fc_nav::FcNavSnapshot;
use crate::nav_config::ExternalNavConfig;

/// How long the flight controller gets to report its own origin, and the
/// repeat interval of the request and of the origin itself.
pub const ORIGIN_WAIT_NS: i64 = 5_000_000_000;

/// What to send this tick.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct OriginActions {
    /// Send `MAV_CMD_REQUEST_MESSAGE` for `GPS_GLOBAL_ORIGIN`.
    pub request: bool,
    /// Send `SET_GPS_GLOBAL_ORIGIN` with this `[lat, lon, alt]`.
    pub set: Option<[f64; 3]>,
}

/// Per-boot origin state; fed one [`FcNavSnapshot`] per tick.
#[derive(Debug, Default)]
pub struct OriginRule {
    epoch: u64,
    last_request: Option<i64>,
    last_set: Option<i64>,
    sent_this_boot: bool,
    seen: bool,
    by_plugin: bool,
}

impl OriginRule {
    /// Whether the origin the flight controller reported this boot followed
    /// one the plugin sent.
    pub fn origin_by_plugin(&self) -> bool {
        self.by_plugin
    }

    pub fn tick(
        &mut self,
        now_ns: i64,
        fc: &FcNavSnapshot,
        cfg: &ExternalNavConfig,
    ) -> OriginActions {
        if fc.boot_epoch != self.epoch {
            *self = Self {
                epoch: fc.boot_epoch,
                last_request: self.last_request,
                ..Self::default()
            };
        }
        if fc.origin_seen && !self.seen {
            self.by_plugin = self.sent_this_boot;
        }
        self.seen = fc.origin_seen;
        if fc.origin_seen {
            return OriginActions::default();
        }
        let due = |t: Option<i64>| t.is_none_or(|t| now_ns - t >= ORIGIN_WAIT_NS);
        let mut out = OriginActions::default();
        if due(self.last_request) {
            self.last_request = Some(now_ns);
            out.request = true;
        }
        let allowed = !cfg.gps_primary
            && !fc.armed
            && fc.boot_ns.is_some_and(|b| now_ns - b >= ORIGIN_WAIT_NS);
        if let Some(lla) = cfg.origin_lla.filter(|_| allowed) {
            if due(self.last_set) {
                self.last_set = Some(now_ns);
                self.sent_this_boot = true;
                out.set = Some(lla);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fc_nav::FcNavSink;

    const S: i64 = 1_000_000_000;
    const LLA: [f64; 3] = [52.0, -0.6, 110.0];

    fn cfg(gps_primary: bool) -> ExternalNavConfig {
        ExternalNavConfig {
            origin_lla: Some(LLA),
            gps_primary,
            ..ExternalNavConfig::default()
        }
    }

    /// 1 Hz heartbeats over `[from, to]` and 100 ms ticks; returns the times
    /// an origin was sent.
    fn run(
        rule: &mut OriginRule,
        fc: &FcNavSink,
        cfg: &ExternalNavConfig,
        armed: bool,
        from: i64,
        to: i64,
    ) -> Vec<i64> {
        let mut sent = Vec::new();
        let mut t = from;
        while t <= to {
            if (t - from) % S == 0 {
                fc.on_heartbeat(armed, t);
            }
            if rule.tick(t, &fc.snapshot(t), cfg).set.is_some() {
                sent.push(t);
            }
            t += S / 10;
        }
        sent
    }

    #[test]
    fn disarmed_without_gps_sends_after_five_seconds_of_fc_boot() {
        let (mut rule, fc) = (OriginRule::default(), FcNavSink::default());
        // The plugin started 20 s before the flight controller booted: the
        // wait counts from the FC's boot, not from the plugin's start.
        assert!(rule.tick(0, &fc.snapshot(0), &cfg(false)).set.is_none());
        let sent = run(&mut rule, &fc, &cfg(false), false, 20 * S, 26 * S);
        assert_eq!(sent, vec![25 * S]);
        // The origin the flight controller then reports is the plugin's.
        fc.on_origin();
        rule.tick(26 * S, &fc.snapshot(26 * S), &cfg(false));
        assert!(rule.origin_by_plugin());
    }

    #[test]
    fn armed_never_sends() {
        let (mut rule, fc) = (OriginRule::default(), FcNavSink::default());
        assert!(run(&mut rule, &fc, &cfg(false), true, 0, 30 * S).is_empty());
    }

    #[test]
    fn gps_primary_never_sends_but_still_requests() {
        let (mut rule, fc) = (OriginRule::default(), FcNavSink::default());
        fc.on_heartbeat(false, 0);
        let a = rule.tick(0, &fc.snapshot(0), &cfg(true));
        assert!(a.request);
        assert!(run(&mut rule, &fc, &cfg(true), false, 0, 30 * S).is_empty());
    }

    #[test]
    fn no_origin_lla_or_no_heartbeat_never_sends() {
        let (mut rule, fc) = (OriginRule::default(), FcNavSink::default());
        let none = ExternalNavConfig::default();
        assert!(run(&mut rule, &fc, &none, false, 0, 10 * S).is_empty());
        let mut rule = OriginRule::default();
        let silent = FcNavSink::default();
        for t in 0..100 {
            assert!(rule
                .tick(t * S / 10, &silent.snapshot(t * S / 10), &cfg(false))
                .set
                .is_none());
        }
    }

    #[test]
    fn an_origin_set_at_boot_is_not_the_plugins() {
        let (mut rule, fc) = (OriginRule::default(), FcNavSink::default());
        fc.on_heartbeat(false, 0);
        fc.on_origin();
        assert!(run(&mut rule, &fc, &cfg(false), false, 0, 10 * S).is_empty());
        assert!(!rule.origin_by_plugin());
    }

    #[test]
    fn a_reboot_clears_the_origin_and_allows_a_new_send_after_five_seconds() {
        let (mut rule, fc) = (OriginRule::default(), FcNavSink::default());
        assert_eq!(
            run(&mut rule, &fc, &cfg(false), false, 0, 6 * S),
            vec![5 * S]
        );
        fc.on_origin();
        rule.tick(6 * S, &fc.snapshot(6 * S), &cfg(false));
        assert!(rule.origin_by_plugin());
        // A 3 s heartbeat gap: the flight controller rebooted.
        let t = 9 * S;
        fc.on_heartbeat(false, t);
        let snap = fc.snapshot(t);
        assert!(!snap.origin_seen);
        rule.tick(t, &snap, &cfg(false));
        assert!(!rule.origin_by_plugin());
        let sent = run(&mut rule, &fc, &cfg(false), false, t, t + 6 * S);
        assert_eq!(sent, vec![t + 5 * S]);
    }
}
