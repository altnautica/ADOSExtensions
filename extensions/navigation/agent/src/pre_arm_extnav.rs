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
//!   ([`extnav_fc_params`]) are read and consistent with this mode: the
//!   external-nav set takes position but not velocity from the pose, takes
//!   its velocity from the optical-flow sensor when one is configured, and
//!   takes its yaw from the pose only when the map's north is held
//!   independently of the flight controller ([`MapNorth::holds_yaw`]).
//! * `viso_delay`: `VISO_DELAY_MS` matches how old poses are when sent.
//! * `fc_flow` (when `FLOW_TYPE` is set and a source set fuses flow
//!   velocity): the flight controller's own `OPTICAL_FLOW` arrives. It sends
//!   none while its flow sensor is unhealthy (no sample for 0.5 s,
//!   `GCS_Common.cpp` `send_opticalflow`), so a stale flow is a lost sensor.
//! * `obstacle_feed` (when the obstacle feed is enabled): scans arrive.

use std::collections::BTreeMap;

use crate::external_nav::{ExtnavHealth, ImuFeed, MapNorth};
use crate::fc_nav::FcFlow;
use crate::nav_commands::extnav_fc_params;
use crate::nav_config::ExternalNavConfig;
use crate::pre_arm::PreArmCheck;

/// A boot-time origin (`AHRS_ORIGIN_*`) further than this from
/// `external_nav.origin_lla` blocks.
pub const ORIGIN_MISMATCH_M: f64 = 1000.0;
/// `VISO_DELAY_MS` further than this from the measured pose age is pending.
pub const VISO_DELAY_TOLERANCE_MS: f32 = 50.0;
/// The oldest external-navigation sample ArduPilot's EKF3 can place in time:
/// it sizes its IMU buffer from `VISO_DELAY_MS` capped at 250 ms and fuses an
/// older sample at the buffer's oldest instant (`AP_NavEKF3_core.cpp`
/// `setup_core`, `AP_NavEKF3_Measurements.cpp` `writeExtNavData`).
pub const EKF_MAX_EXTNAV_DELAY_MS: f32 = 250.0;
/// `AHRS_OPTIONS` bit 4: set the EKF origin at boot from `AHRS_ORIGIN_*`.
const AHRS_OPTION_RECORDED_ORIGIN: i64 = 16;
/// No flight-controller `OPTICAL_FLOW` for this long blocks: the flow
/// sensor is lost, or the flight controller is not streaming it.
pub const FC_FLOW_STALE_NS: i64 = 1_000_000_000;
/// None at all this long after the flight controller was first heard
/// blocks; before that the row waits for the stream request to take.
pub const FC_FLOW_FIRST_NS: i64 = 5_000_000_000;

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
    /// Where the fresh pose's map north comes from; `None` when there is no
    /// fresh pose or it does not say.
    pub north: Option<MapNorth>,
    /// Why the publisher's FC IMU feed left the fresh pose degraded or
    /// lost; `None` when it did not or there is no fresh pose.
    pub imu_feed: Option<ImuFeed>,
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
    /// The flight controller's last `OPTICAL_FLOW` and its age; `None`
    /// when none has arrived this boot.
    pub fc_flow: Option<FcFlow>,
    /// How long this flight-controller boot has been heard; `None` before
    /// its first heartbeat.
    pub fc_heard_ns: Option<i64>,
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
    // A pose the publisher's FC IMU feed held down names that feed: its
    // heading is not measured (lost) or measured coarsely (degraded).
    let feed = i
        .imu_feed
        .map_or(String::new(), |f| format!(" ({})", f.describe()));
    let health = match i.health {
        Some(ExtnavHealth::Good) => PreArmCheck::ok("extnav_health", "Tracking good."),
        Some(ExtnavHealth::Degraded) => PreArmCheck::pending(
            "extnav_health",
            format!("Tracking degraded{feed}; position noise raised."),
        ),
        Some(ExtnavHealth::Lost) => {
            PreArmCheck::blocking("extnav_health", format!("Tracking lost{feed}."))
        }
        None => PreArmCheck::pending("extnav_health", "No fresh pose."),
    };
    let mut checks = vec![
        source,
        health,
        origin_check(i),
        fc_params_check(i),
        viso_delay_check(i),
    ];
    checks.extend(fc_flow_check(i));
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
    // ArduPilot drops the ODOMETRY velocity covariance and fuses the velocity
    // as good to VISO_VEL_M_NSE (0.1 m/s by default). This velocity is a
    // difference of poses: a planar SLAM's lags and overshoots truth by about
    // 1 m/s in fast manoeuvres, and over-trusted it tips the EKF's attitude
    // and accel bias until the position innovations fail their gates.
    let velxy = v(&format!("EK3_SRC{e}_VELXY"));
    let flow = v("FLOW_TYPE");
    if velxy == 6 {
        let fix = if flow != 0 {
            "5 (optical flow)"
        } else {
            "0 (position only)"
        };
        return PreArmCheck::blocking(
            ID,
            format!(
                "EK3_SRC{e}_VELXY is 6: the EKF would trust the pose's velocity far beyond its \
                 accuracy; set {fix}."
            ),
        );
    }
    if velxy == 5 && flow == 0 {
        return PreArmCheck::blocking(
            ID,
            format!(
                "EK3_SRC{e}_VELXY is 5 (optical flow) but FLOW_TYPE is 0: the EKF would get no \
                 velocity; set FLOW_TYPE for the flow sensor, or EK3_SRC{e}_VELXY 0."
            ),
        );
    }
    // A map whose north follows the flight controller's heading only echoes
    // the EKF's own yaw back to it: nothing would hold north.
    if v(&format!("EK3_SRC{e}_YAW")) == 6 && !MapNorth::holds_yaw(i.north) {
        if i.health.is_none() {
            return PreArmCheck::pending(
                ID,
                format!("EK3_SRC{e}_YAW is 6: awaiting a pose to learn where the map's north comes from."),
            );
        }
        return PreArmCheck::blocking(
            ID,
            format!(
                "EK3_SRC{e}_YAW is 6 but the map's north follows the flight controller's heading; \
                 set the pose provider's take-off heading, or use 1 (compass)."
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
    if !i.gps_primary && flow == 0 {
        return PreArmCheck::blocking(
            ID,
            format!(
                "EK3_SRC{f}_VELXY is 5 (optical flow) but FLOW_TYPE is 0: the fallback set \
                 would get no velocity; set FLOW_TYPE for the flow sensor."
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
    // Flow velocity is measured, not differenced from late poses: with it
    // in the external-nav set the EKF holds velocity between poses, and a
    // late pose moves the position less. SLAM-only velocity stays the
    // operator's choice (a dark floor starves the flow), so this warns.
    if flow != 0 && velxy != 5 {
        return PreArmCheck::ok(
            ID,
            format!(
                "FLOW_TYPE is {flow} but EK3_SRC{e}_VELXY is {velxy}: the external-nav set \
                 ignores the optical-flow sensor. SLAM-only velocity is allowed; \
                 EK3_SRC{e}_VELXY 5 lets the EKF take its velocity from the flow between poses."
            ),
        );
    }
    PreArmCheck::ok(ID, "")
}

/// `fc_flow`: present only once `FLOW_TYPE` is read as set and a source set
/// the EKF can run on fuses flow velocity (the external-nav set, or the
/// fallback set without GPS primary); `fc_params` covers the rest.
fn fc_flow_check(i: &ExtnavPreArmInputs) -> Option<PreArmCheck> {
    const ID: &str = "fc_flow";
    if i.int("FLOW_TYPE").unwrap_or(0) == 0 {
        return None;
    }
    let velxy = |set: u8| i.int(&format!("EK3_SRC{set}_VELXY"));
    let fuses =
        velxy(i.extnav_set) == Some(5) || (!i.gps_primary && velxy(i.fallback_set) == Some(5));
    if !fuses {
        return None;
    }
    Some(match i.fc_flow {
        None if i.fc_heard_ns.is_some_and(|t| t >= FC_FLOW_FIRST_NS) => PreArmCheck::blocking(
            ID,
            "No OPTICAL_FLOW from the flight controller: it sends none while its flow sensor is \
             unhealthy. Check the flow sensor, its serial port and FLOW_TYPE.",
        ),
        None => PreArmCheck::pending(ID, "Awaiting OPTICAL_FLOW from the flight controller."),
        Some(f) if f.age_ns > FC_FLOW_STALE_NS => PreArmCheck::blocking(
            ID,
            format!(
                "No OPTICAL_FLOW from the flight controller for {:.1} s: the flow sensor is lost \
                 and the EKF has no flow velocity. Check the flow sensor and its serial port.",
                f.age_ns as f64 / 1e9
            ),
        ),
        Some(f) if f.quality == 0 => PreArmCheck::pending(
            ID,
            "Flow quality 0: the EKF fuses no flow. The sensor needs light, texture under it and \
             a height inside its working range.",
        ),
        Some(f) => PreArmCheck::ok(ID, format!("Quality {}/255.", f.quality)),
    })
}

fn viso_delay_check(i: &ExtnavPreArmInputs) -> PreArmCheck {
    const ID: &str = "viso_delay";
    if let Some(m) = i.pose_age_ms.filter(|m| *m > EKF_MAX_EXTNAV_DELAY_MS) {
        let (m, cap) = (m.round() as i64, EKF_MAX_EXTNAV_DELAY_MS as i64);
        return PreArmCheck::pending(
            ID,
            format!(
                "Poses are {m} ms old when sent, older than the {cap} ms the flight \
                 controller's EKF can place in time: it fuses them late and the vehicle \
                 sways. VISO_DELAY_MS cannot cover the difference; cut the companion's load \
                 or LiDAR processing time."
            ),
        );
    }
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
