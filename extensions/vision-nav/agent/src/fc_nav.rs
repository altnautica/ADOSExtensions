//! Flight-controller state the `external_nav` mode reads.
//!
//! The MAVLink subscription callbacks write the latest `LOCAL_POSITION_NED`,
//! `ATTITUDE`, `GPS_RAW_INT` fix, `GPS_GLOBAL_ORIGIN` and source-set
//! `COMMAND_ACK` here; the external-nav ticks read a [`FcNavSnapshot`]
//! with every stale field already dropped.

use std::sync::Mutex;

/// `LOCAL_POSITION_NED` older than this is not used for `ODOMETRY` z.
pub const LOCAL_POSITION_STALE_NS: i64 = 1_000_000_000;
/// `ATTITUDE` older than this is not used for the `ODOMETRY` attitude.
pub const ATTITUDE_STALE_NS: i64 = 500_000_000;
/// No `GPS_RAW_INT` for this long reads as no fix.
pub const GPS_STALE_NS: i64 = 2_000_000_000;

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
    /// The flight controller has reported an EKF origin.
    pub origin_seen: bool,
    /// The last `COMMAND_ACK` result for the source-set command.
    pub source_set_ack: Option<u8>,
}

#[derive(Default)]
struct Inner {
    local: Option<(LocalPosition, i64)>,
    attitude: Option<(Attitude, i64)>,
    gps: Option<(u8, i64)>,
    origin_seen: bool,
    source_set_ack: Option<u8>,
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

    pub fn on_gps_fix(&self, fix_type: u8, now_ns: i64) {
        self.lock().gps = Some((fix_type, now_ns));
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
        FcNavSnapshot {
            local: g
                .local
                .filter(|(_, t)| fresh(*t, LOCAL_POSITION_STALE_NS))
                .map(|(v, _)| v),
            attitude: g
                .attitude
                .filter(|(_, t)| fresh(*t, ATTITUDE_STALE_NS))
                .map(|(v, _)| v),
            gps_fix: g
                .gps
                .filter(|(_, t)| fresh(*t, GPS_STALE_NS))
                .map_or(0, |(f, _)| f),
            origin_seen: g.origin_seen,
            source_set_ack: g.source_set_ack,
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
        s.on_gps_fix(3, 0);
        let fresh = s.snapshot(100_000_000);
        assert_eq!(fresh.local.unwrap().z, -2.5);
        assert_eq!(fresh.gps_fix, 3);
        let stale = s.snapshot(3_000_000_000);
        assert!(stale.local.is_none());
        assert_eq!(stale.gps_fix, 0);
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
