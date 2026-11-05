//! Pre-arm checks for `external_nav` mode.
//!
//! * `extnav_source`: the pose topic is subscribed and its payloads parse.
//!   A refused subscription (the publishing plugin is not installed, or
//!   the grant is missing) blocks as "External nav source missing".
//! * `extnav_health`: good passes, degraded is pending, lost blocks.
//! * `ekf_origin`: the flight controller reports an EKF origin, and it is
//!   not one that arrived after the yaw alignment with the declination
//!   learning on (that origin steps the earth field into a gyro bias and
//!   spins the vehicle), nor a boot-time origin far from `origin_lla`.
//! * `fc_params`: the flight controller's external-nav parameters
//!   ([`extnav_fc_params`]) are read and consistent with this mode.
//! * `viso_delay`: `VISO_DELAY_MS` matches how old poses are when sent.
//! * `obstacle_feed` (when the obstacle feed is enabled): scans arrive.

use std::collections::BTreeMap;

use crate::external_nav::ExtnavHealth;
use crate::nav_commands::extnav_fc_params;
use crate::nav_config::ExternalNavConfig;
use crate::pre_arm::PreArmCheck;

/// A boot-time origin (`AHRS_ORIGIN_*`) further than this from
/// `external_nav.origin_lla` blocks.
pub const ORIGIN_MISMATCH_M: f64 = 1000.0;
/// `VISO_DELAY_MS` further than this from the measured pose age is pending.
pub const VISO_DELAY_TOLERANCE_MS: f32 = 50.0;
/// `AHRS_OPTIONS` bit 4: set the EKF origin at boot from `AHRS_ORIGIN_*`.
const AHRS_OPTION_RECORDED_ORIGIN: i64 = 16;

/// The obstacle feed's state for its pre-arm row.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObstacleFeed {
    Denied,
    Waiting,
    Live,
}

/// What the external-nav runtime knows this tick.
#[derive(Debug, Clone, PartialEq)]
pub struct ExtnavPreArmInputs {
    /// The pose-topic subscription was refused.
    pub source_denied: bool,
    /// Why the last pose payload was rejected, when it was.
    pub pose_error: Option<String>,
    /// Health of the fresh pose; `None` when there is none.
    pub health: Option<ExtnavHealth>,
    /// The flight controller has reported an EKF origin this boot.
    pub origin_seen: bool,
    /// That origin followed a `SET_GPS_GLOBAL_ORIGIN` the plugin sent.
    pub origin_by_plugin: bool,
    /// `external_nav.gps_primary`.
    pub gps_primary: bool,
    /// `external_nav.extnav_set`.
    pub extnav_set: u8,
    /// `external_nav.fallback_set`.
    pub fallback_set: u8,
    /// `external_nav.origin_lla`.
    pub origin_lla: Option<[f64; 3]>,
    /// The flight-controller parameter values read so far, by id.
    pub params: BTreeMap<String, f32>,
    /// Median age (ms) of a pose when its `ODOMETRY` is sent.
    pub pose_age_ms: Option<f32>,
    /// `None` when the obstacle feed is disabled.
    pub obstacle: Option<ObstacleFeed>,
}

impl ExtnavPreArmInputs {
    fn param(&self, id: &str) -> Option<f32> {
        self.params.get(id).copied()
    }

    /// A parameter as the integer the flight controller stores.
    fn int(&self, id: &str) -> Option<i64> {
        self.param(id).map(|v| v.round() as i64)
    }
}

pub(crate) fn extnav_checks(inputs: Option<&ExtnavPreArmInputs>) -> Vec<PreArmCheck> {
    let Some(i) = inputs else {
        return vec![PreArmCheck::blocking(
            "extnav_source",
            "External nav source missing.",
        )];
    };
    let source = if i.source_denied {
        PreArmCheck::blocking(
            "extnav_source",
            "External nav source missing: the pose topic is not published by an \
             installed plugin, or reading it is not granted.",
        )
    } else if let Some(err) = &i.pose_error {
        PreArmCheck::blocking("extnav_source", format!("Pose rejected: {err}."))
    } else if i.health.is_none() {
        PreArmCheck::pending("extnav_source", "Awaiting a pose.")
    } else {
        PreArmCheck::ok("extnav_source", "")
    };
    let health = match i.health {
        Some(ExtnavHealth::Good) => PreArmCheck::ok("extnav_health", "Tracking good."),
        Some(ExtnavHealth::Degraded) => {
            PreArmCheck::pending("extnav_health", "Tracking degraded; position noise raised.")
        }
        Some(ExtnavHealth::Lost) => PreArmCheck::blocking("extnav_health", "Tracking lost."),
        None => PreArmCheck::pending("extnav_health", "No fresh pose."),
    };
    let mut checks = vec![
        source,
        health,
        origin_check(i),
        fc_params_check(i),
        viso_delay_check(i),
    ];
    match i.obstacle {
        None => {}
        Some(ObstacleFeed::Live) => checks.push(PreArmCheck::ok("obstacle_feed", "")),
        Some(ObstacleFeed::Waiting) => {
            checks.push(PreArmCheck::pending("obstacle_feed", "Awaiting a scan."))
        }
        Some(ObstacleFeed::Denied) => checks.push(PreArmCheck::blocking(
            "obstacle_feed",
            "Scan topic not available.",
        )),
    }
    checks
}

fn origin_check(i: &ExtnavPreArmInputs) -> PreArmCheck {
    const ID: &str = "ekf_origin";
    if !i.origin_seen {
        return if i.gps_primary {
            PreArmCheck::pending(ID, "Waiting for the GPS to set the EKF origin.")
        } else {
            PreArmCheck::pending(
                ID,
                "EKF origin not set yet. Set AHRS_OPTIONS bit 4 and AHRS_ORIGIN_LAT/LON/ALT \
                 so the flight controller sets it at boot.",
            )
        };
    }
    if i.origin_by_plugin {
        let yaw = i.int(&format!("EK3_SRC{}_YAW", i.extnav_set));
        let autodec = i.int("COMPASS_AUTODEC");
        let (Some(yaw), Some(autodec)) = (yaw, autodec) else {
            return PreArmCheck::pending(ID, "Reading flight controller parameters.");
        };
        // Compass yaw (1) or compass with GSF fallback (3) aligned at boot on
        // the boot declination; the origin the plugin sent later stepped it.
        if matches!(yaw, 1 | 3) && autodec != 0 {
            return PreArmCheck::blocking(
                ID,
                "The EKF origin arrived after the yaw alignment: the declination step can \
                 become a gyro bias and spin the vehicle. Set AHRS_OPTIONS bit 4 and \
                 AHRS_ORIGIN_LAT/LON/ALT to the take-off site (or COMPASS_AUTODEC 0 with the \
                 site's COMPASS_DEC), then reboot the flight controller.",
            );
        }
        return PreArmCheck::ok(ID, "");
    }
    if !i.gps_primary
        && i.int("AHRS_OPTIONS")
            .is_some_and(|o| o & AHRS_OPTION_RECORDED_ORIGIN != 0)
    {
        if let (Some(lat), Some(lon), Some(lla)) = (
            i.param("AHRS_ORIGIN_LAT"),
            i.param("AHRS_ORIGIN_LON"),
            i.origin_lla,
        ) {
            let d = haversine_m(f64::from(lat), f64::from(lon), lla[0], lla[1]);
            if d > ORIGIN_MISMATCH_M {
                return PreArmCheck::blocking(
                    ID,
                    format!(
                        "AHRS_ORIGIN is {:.1} km from external_nav.origin_lla; set both to the \
                         take-off site.",
                        d / 1000.0
                    ),
                );
            }
        }
    }
    PreArmCheck::ok(ID, "")
}

fn fc_params_check(i: &ExtnavPreArmInputs) -> PreArmCheck {
    const ID: &str = "fc_params";
    let wanted = extnav_fc_params(&ExternalNavConfig {
        extnav_set: i.extnav_set,
        fallback_set: i.fallback_set,
        ..ExternalNavConfig::default()
    });
    let have = wanted
        .iter()
        .filter(|id| i.params.contains_key(*id))
        .count();
    if have < wanted.len() {
        return PreArmCheck::pending(
            ID,
            format!(
                "Reading flight controller parameters ({have} of {}).",
                wanted.len()
            ),
        );
    }
    let (e, f) = (i.extnav_set, i.fallback_set);
    let v = |id: &str| i.int(id).unwrap_or_default();
    let viso = v("VISO_TYPE");
    if viso != 1 {
        return PreArmCheck::blocking(
            ID,
            format!("VISO_TYPE is {viso}; external navigation needs 1 (MAVLink)."),
        );
    }
    let posxy = v(&format!("EK3_SRC{e}_POSXY"));
    if posxy != 6 {
        return PreArmCheck::blocking(
            ID,
            format!(
                "EK3_SRC{e}_POSXY is {posxy}; the external-nav source set needs 6 (ExternalNav)."
            ),
        );
    }
    // The map's north is the yaw it was built with, and that yaw follows the
    // EKF's: taking the EKF yaw back from the map closes a loop with nothing
    // holding north, so the external-nav set keeps the compass.
    if v(&format!("EK3_SRC{e}_YAW")) == 6 {
        return PreArmCheck::blocking(
            ID,
            format!(
                "EK3_SRC{e}_YAW is 6: the EKF would take north from the SLAM map; use 1 (compass)."
            ),
        );
    }
    let fallback_posxy = v(&format!("EK3_SRC{f}_POSXY"));
    if i.gps_primary && fallback_posxy != 3 {
        return PreArmCheck::blocking(
            ID,
            format!(
                "EK3_SRC{f}_POSXY is {fallback_posxy}; with GPS primary the fallback set needs 3 (GPS)."
            ),
        );
    }
    let fallback_velxy = v(&format!("EK3_SRC{f}_VELXY"));
    if !i.gps_primary && fallback_velxy != 5 {
        return PreArmCheck::blocking(
            ID,
            format!(
                "EK3_SRC{f}_VELXY is {fallback_velxy}; the fallback set needs 5 (optical flow)."
            ),
        );
    }
    // With COMPASS_AUTODEC on, the compass library sets its declination the
    // first time the EKF has a position, after the yaw alignment. EKF3 takes
    // the declination from its own table at the origin instead while
    // EK3_MAG_EF_LIM > 0; at 0 that late step reaches the yaw and becomes a
    // gyro bias.
    if v("COMPASS_AUTODEC") != 0 && i.param("EK3_MAG_EF_LIM").is_some_and(|l| l <= 0.0) {
        return PreArmCheck::blocking(
            ID,
            "The compass declination will step when the EKF first gets a position; set \
             EK3_MAG_EF_LIM above 0 (default 50) or COMPASS_AUTODEC 0 with the site's COMPASS_DEC.",
        );
    }
    PreArmCheck::ok(ID, "")
}

fn viso_delay_check(i: &ExtnavPreArmInputs) -> PreArmCheck {
    const ID: &str = "viso_delay";
    if let (Some(p), Some(m)) = (i.param("VISO_DELAY_MS"), i.pose_age_ms) {
        if (p - m).abs() > VISO_DELAY_TOLERANCE_MS {
            let (p, m) = (p.round() as i64, m.round() as i64);
            return PreArmCheck::pending(
                ID,
                format!(
                    "VISO_DELAY_MS is {p} ms but poses are {m} ms old when sent; set \
                     VISO_DELAY_MS to {m}."
                ),
            );
        }
    }
    PreArmCheck::ok(ID, "")
}

/// Great-circle distance in metres between two latitude/longitude points.
pub fn haversine_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    const EARTH_RADIUS_M: f64 = 6_371_000.0;
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dp = p2 - p1;
    let dl = (lon2 - lon1).to_radians();
    let a = (dp / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_M * a.sqrt().min(1.0).asin()
}

#[cfg(test)]
#[path = "pre_arm_extnav_tests.rs"]
mod tests;
