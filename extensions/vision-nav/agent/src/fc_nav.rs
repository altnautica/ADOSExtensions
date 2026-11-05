//! Flight-controller state the `external_nav` mode reads.
//!
//! The MAVLink subscription callbacks write the latest `LOCAL_POSITION_NED`,
//! `ATTITUDE`, `GPS_RAW_INT` fix and accuracy, `GPS_GLOBAL_ORIGIN`,
//! `HEARTBEAT` armed state and source-set `COMMAND_ACK` here; the
//! external-nav ticks read a [`FcNavSnapshot`] with every stale field
//! already dropped.
//!
//! A heartbeat after [`FC_SILENCE_NS`] of silence (or the first one ever)
//! starts a new flight-controller boot: the boot time and epoch move on and
//! the origin the previous boot reported is forgotten, since a rebooted
//! flight controller has none until it sets or receives one again.

use std::sync::Mutex;

/// `LOCAL_POSITION_NED` older than this is not used for `ODOMETRY` z.
pub const LOCAL_POSITION_STALE_NS: i64 = 1_000_000_000;
/// `ATTITUDE` older than this is not used for the `ODOMETRY` attitude.
pub const ATTITUDE_STALE_NS: i64 = 500_000_000;
/// No `GPS_RAW_INT` for this long reads as no fix.
pub const GPS_STALE_NS: i64 = 2_000_000_000;
/// A heartbeat after this much silence starts a new flight-controller boot.
pub const FC_SILENCE_NS: i64 = 3_000_000_000;
/// `MAV_MODE_FLAG_SAFETY_ARMED` in `HEARTBEAT.base_mode`.
pub const MODE_FLAG_ARMED: u8 = 128;

/// `MAV_CMD_SET_EKF_SOURCE_SET`.
pub const CMD_SET_EKF_SOURCE_SET: u16 = 42007;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalPosition {
    pub z: f32,
    pub vz: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Attitude {
    pub roll: f32,
    pub pitch: f32,
    pub rollspeed: f32,
    pub pitchspeed: f32,
    pub yawspeed: f32,
}

/// What the ticks read: fresh values only.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct FcNavSnapshot {
    pub local: Option<LocalPosition>,
    pub attitude: Option<Attitude>,
    /// `GPS_RAW_INT.fix_type`; 0 when no fresh report.
    pub gps_fix: u8,
    /// `GPS_RAW_INT.h_acc` in mm; `None` when no fresh report carries it
    /// (0 means unknown).
    pub gps_h_acc_mm: Option<u32>,
    /// The flight controller has reported an EKF origin this boot.
    pub origin_seen: bool,
    /// The last `COMMAND_ACK` result for the source-set command.
    pub source_set_ack: Option<u8>,
    /// The last heartbeat reported the vehicle armed.
    pub armed: bool,
    /// When the current flight-controller boot was first heard; `None`
    /// before the first heartbeat.
    pub boot_ns: Option<i64>,
    /// Incremented on each new flight-controller boot.
    pub boot_epoch: u64,
}

#[derive(Default)]
struct Inner {
    local: Option<(LocalPosition, i64)>,
    attitude: Option<(Attitude, i64)>,
    gps: Option<(u8, Option<u32>, i64)>,
    origin_seen: bool,
    source_set_ack: Option<u8>,
    armed: bool,
    last_heartbeat: Option<i64>,
    boot_ns: Option<i64>,
    boot_epoch: u64,
}

/// The shared sink.
#[derive(Default)]
pub struct FcNavSink(Mutex<Inner>);

impl FcNavSink {
    pub fn on_local_position(&self, z: f32, vz: f32, now_ns: i64) {
        self.lock().local = Some((LocalPosition { z, vz }, now_ns));
    }

    pub fn on_attitude(&self, a: Attitude, now_ns: i64) {
        self.lock().attitude = Some((a, now_ns));
    }

    /// `GPS_RAW_INT` fix type and `h_acc` (mm, 0 = unknown).
    pub fn on_gps_fix(&self, fix_type: u8, h_acc_mm: u32, now_ns: i64) {
        let h_acc = (h_acc_mm != 0).then_some(h_acc_mm);
        self.lock().gps = Some((fix_type, h_acc, now_ns));
    }

    /// The flight controller's `HEARTBEAT`. The first one, or one after
    /// [`FC_SILENCE_NS`] of silence, starts a new boot and clears the origin.
    pub fn on_heartbeat(&self, armed: bool, now_ns: i64) {
        let mut g = self.lock();
        if g.last_heartbeat.is_none_or(|t| now_ns - t >= FC_SILENCE_NS) {
            g.boot_ns = Some(now_ns);
            g.boot_epoch += 1;
            g.origin_seen = false;
        }
        g.last_heartbeat = Some(now_ns);
        g.armed = armed;
    }

    pub fn on_origin(&self) {
        self.lock().origin_seen = true;
    }

    pub fn on_command_ack(&self, command: u16, result: u8) {
        if command == CMD_SET_EKF_SOURCE_SET {
            self.lock().source_set_ack = Some(result);
        }
    }

    /// Forget the last source-set ack before a new command goes out.
    pub fn clear_source_set_ack(&self) {
        self.lock().source_set_ack = None;
    }

    pub fn snapshot(&self, now_ns: i64) -> FcNavSnapshot {
        let g = self.lock();
        let fresh = |t: i64, max: i64| now_ns - t <= max;
        let gps = g.gps.filter(|(_, _, t)| fresh(*t, GPS_STALE_NS));
        FcNavSnapshot {
            local: g
                .local
                .filter(|(_, t)| fresh(*t, LOCAL_POSITION_STALE_NS))
                .map(|(v, _)| v),
            attitude: g
                .attitude
                .filter(|(_, t)| fresh(*t, ATTITUDE_STALE_NS))
                .map(|(v, _)| v),
            gps_fix: gps.map_or(0, |(f, _, _)| f),
            gps_h_acc_mm: gps.and_then(|(_, a, _)| a),
            origin_seen: g.origin_seen,
            source_set_ack: g.source_set_ack,
            armed: g.armed,
            boot_ns: g.boot_ns,
            boot_epoch: g.boot_epoch,
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Inner> {
        self.0.lock().expect("fc nav lock")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stale_fields_drop_out_of_the_snapshot() {
        let s = FcNavSink::default();
        s.on_local_position(-2.5, 0.1, 0);
        s.on_gps_fix(3, 1200, 0);
        let fresh = s.snapshot(100_000_000);
        assert_eq!(fresh.local.unwrap().z, -2.5);
        assert_eq!(fresh.gps_fix, 3);
        assert_eq!(fresh.gps_h_acc_mm, Some(1200));
        let stale = s.snapshot(3_000_000_000);
        assert!(stale.local.is_none());
        assert_eq!(stale.gps_fix, 0);
        assert_eq!(stale.gps_h_acc_mm, None);
        // An h_acc of 0 is "unknown", not a perfect fix.
        s.on_gps_fix(3, 0, 3_000_000_000);
        assert_eq!(s.snapshot(3_000_000_000).gps_h_acc_mm, None);
    }

    #[test]
    fn heartbeats_track_arming_and_a_silence_starts_a_new_boot() {
        const S: i64 = 1_000_000_000;
        let s = FcNavSink::default();
        assert_eq!(s.snapshot(0).boot_ns, None);
        assert_eq!(s.snapshot(0).boot_epoch, 0);
        s.on_heartbeat(false, 10 * S);
        s.on_origin();
        let a = s.snapshot(10 * S);
        assert_eq!((a.boot_ns, a.boot_epoch, a.armed), (Some(10 * S), 1, false));
        assert!(a.origin_seen);
        // 1 Hz heartbeats: the same boot; arming is followed.
        s.on_heartbeat(true, 11 * S);
        s.on_heartbeat(true, 12 * S);
        let b = s.snapshot(12 * S);
        assert_eq!((b.boot_ns, b.boot_epoch, b.armed), (Some(10 * S), 1, true));
        assert!(b.origin_seen);
        // 3 s of silence: a reboot. The old origin is gone.
        s.on_heartbeat(false, 15 * S);
        let c = s.snapshot(15 * S);
        assert_eq!((c.boot_ns, c.boot_epoch, c.armed), (Some(15 * S), 2, false));
        assert!(!c.origin_seen);
    }

    #[test]
    fn only_the_source_set_ack_is_kept() {
        let s = FcNavSink::default();
        s.on_command_ack(400, 0);
        assert_eq!(s.snapshot(0).source_set_ack, None);
        s.on_command_ack(CMD_SET_EKF_SOURCE_SET, 0);
        assert_eq!(s.snapshot(0).source_set_ack, Some(0));
        s.clear_source_set_ack();
        assert_eq!(s.snapshot(0).source_set_ack, None);
    }
}
