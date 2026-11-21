//! The flight controller's ground velocity for the planar matcher: along a
//! corridor (a mine drift, a bare aisle) the scans pin the pose across the
//! walls only, and the matcher follows this velocity along them
//! ([`ados_scan2d::LocalSlam::feed_velocity`]). It is the FC's
//! `OPTICAL_FLOW` (a downward flow sensor's angular rate with the body rate
//! taken out) times the range of its downward rangefinder
//! (`DISTANCE_SENSOR`, orientation 25).
//!
//! Signs, ArduPilot's sensor convention: moving forward (+x, FRD) at `v`
//! over ground `r` away reads `comp_y = v / r`; moving right (+y, FRD) reads
//! `comp_x = −v / r`. The body velocity, FLU, is `(comp_y·r, comp_x·r)`.
//! Flow sensor and rangefinder both look along the body's down axis, so the
//! range is the distance the angular rate is taken over at any tilt; the
//! matcher levels the velocity with its own attitude.
//!
//! Time: ArduPilot puts its boot milliseconds in `OPTICAL_FLOW.time_usec`,
//! PX4 microseconds; the reading that lies near the FC's newest boot time
//! is the one taken.

use crate::mavlink_msg::{DistanceSensor, OpticalFlow};

/// `MAV_SENSOR_ROTATION_PITCH_270`: facing down.
pub const DOWNWARD: u8 = 25;
/// A flow sample is scaled by a range read no further than this from it.
pub const MAX_RANGE_SKEW_MS: u32 = 300;
/// A flow time further than this from the FC's newest boot time is neither
/// milliseconds nor microseconds of the same clock.
const MAX_CLOCK_SKEW_MS: u64 = 2_000;

/// The FC boot time of a flow sample, ms, given the FC's newest boot time.
pub fn flow_time_ms(time_usec: u64, fc_ms: u32) -> Option<u32> {
    [time_usec, time_usec / 1_000]
        .into_iter()
        .find(|t| t.abs_diff(fc_ms as u64) <= MAX_CLOCK_SKEW_MS)
        .map(|t| t as u32)
}

/// The range to the ground, metres: a downward reading strictly inside the
/// sensor's valid band (a laser reports its maximum when nothing returns).
pub fn ground_range(r: &DistanceSensor) -> Option<f64> {
    (r.orientation == DOWNWARD && r.distance_m > r.min_m && r.distance_m < r.max_m)
        .then_some(r.distance_m)
}

/// The lowest flow quality the matcher follows. A downward sensor over
/// dusty or bare ground reads a sound rate at a few tens (a recorded mine
/// drift at 18 to 68); under that it is losing the ground and its rate
/// falls toward zero. Along a corridor's axis nothing else checks the
/// flow: the scans constrain only across it.
pub const MIN_QUALITY: u8 = 16;

/// The body velocity along x and y (FLU), m/s, of a flow sample over
/// `range_m`; `None` under [`MIN_QUALITY`]. Above it a bad rate across the
/// scans' constraints is kept out by its consistency with them, and one
/// stalling along a free direction by the matcher's stall check
/// (`ados_scan2d::LocalSlam::feed_velocity`).
pub fn body_velocity(flow: &OpticalFlow, range_m: f64) -> Option<[f64; 2]> {
    let v = [flow.comp[1] * range_m, flow.comp[0] * range_m];
    (flow.quality >= MIN_QUALITY && v.iter().all(|x| x.is_finite())).then_some(v)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flow(comp: [f64; 2], quality: u8) -> OpticalFlow {
        OpticalFlow {
            time_usec: 0,
            comp,
            quality,
        }
    }

    /// Recorded from ArduPilot SITL, 1.14 m over the floor, flying forward
    /// at 6.5 m/s and drifting 1.7 m/s right: `comp` (-1.33, 5.54) rad/s.
    #[test]
    fn forward_and_rightward_motion_reads_back_in_flu() {
        let v = body_velocity(&flow([-1.33, 5.54], 255), 1.14).unwrap();
        assert!((v[0] - 6.32).abs() < 0.01, "{v:?}");
        assert!((v[1] + 1.52).abs() < 0.01, "{v:?}");
        assert_eq!(body_velocity(&flow([0.1, 0.1], 0), 1.0), None, "no quality");
    }

    /// A downward flow sensor over dusty mine floor reads quality 18 to 68
    /// at a sound rate and is used; under 16 it is losing the ground (a
    /// near-zero rate along a corridor would freeze the pose) and dropped.
    #[test]
    fn dusty_mine_flow_is_used_and_lost_ground_is_not() {
        let v = body_velocity(&flow([0.0, 2.0], 41), 1.8).unwrap();
        assert!((v[0] - 3.6).abs() < 1e-9, "{v:?}");
        assert!(body_velocity(&flow([0.1, 0.1], 18), 1.0).is_some());
        assert!(body_velocity(&flow([0.1, 0.1], 16), 1.0).is_some());
        for q in [0, 1, 15] {
            assert_eq!(body_velocity(&flow([0.0, 0.01], q), 1.0), None, "q {q}");
        }
        assert_eq!(body_velocity(&flow([f64::NAN, 0.1], 200), 1.0), None);
    }

    #[test]
    fn only_a_downward_reading_inside_its_band_is_a_range() {
        let r = DistanceSensor {
            time_boot_ms: 0,
            distance_m: 1.14,
            min_m: 0.05,
            max_m: 12.0,
            orientation: DOWNWARD,
        };
        assert_eq!(ground_range(&r), Some(1.14));
        assert_eq!(
            ground_range(&DistanceSensor {
                distance_m: 12.0,
                ..r
            }),
            None,
            "no return"
        );
        assert_eq!(
            ground_range(&DistanceSensor {
                distance_m: 0.05,
                ..r
            }),
            None
        );
        assert_eq!(
            ground_range(&DistanceSensor {
                orientation: 0,
                ..r
            }),
            None,
            "forward"
        );
    }

    #[test]
    fn flow_time_is_milliseconds_or_microseconds_of_the_fc_clock() {
        assert_eq!(flow_time_ms(37_020, 37_000), Some(37_020), "ArduPilot ms");
        assert_eq!(flow_time_ms(37_020_000, 37_000), Some(37_020), "PX4 us");
        assert_eq!(flow_time_ms(90_000, 37_000), None);
    }
}
