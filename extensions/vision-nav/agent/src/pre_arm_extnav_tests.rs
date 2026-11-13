use super::*;
use crate::config::Mode;
use crate::pre_arm::{CompanionState, PreArmGate, PreArmInputs, PreArmReport, Severity};

const ORIGIN: [f64; 3] = [52.0731234, -0.6276543, 110.25];

fn inputs(extnav: Option<ExtnavPreArmInputs>) -> PreArmInputs {
    PreArmInputs {
        mode: Mode::ExternalNav,
        companion_state: CompanionState::Inactive,
        flow_quality: None,
        flow_scale_source: None,
        rangefinder_topology: None,
        rangefinder_reading_m: None,
        rangefinder_min_m: 0.0,
        rangefinder_max_m: 0.0,
        extnav,
    }
}

/// A GNSS-denied flight controller set up as the manifest asks: the origin
/// set at boot from AHRS_ORIGIN, position-only external nav, compass yaw,
/// flow fallback.
fn good_params() -> BTreeMap<String, f32> {
    [
        ("VISO_TYPE", 1.0),
        ("VISO_DELAY_MS", 120.0),
        ("COMPASS_AUTODEC", 1.0),
        ("AHRS_OPTIONS", 16.0),
        ("AHRS_ORIGIN_LAT", ORIGIN[0] as f32),
        ("AHRS_ORIGIN_LON", ORIGIN[1] as f32),
        ("EK3_SRC1_POSXY", 6.0),
        ("EK3_SRC1_VELXY", 0.0),
        ("EK3_SRC1_YAW", 1.0),
        ("EK3_SRC2_POSXY", 0.0),
        ("EK3_SRC2_VELXY", 5.0),
        ("EK3_MAG_EF_LIM", 50.0),
    ]
    .into_iter()
    .map(|(k, v)| (k.to_string(), v))
    .collect()
}

fn ready() -> ExtnavPreArmInputs {
    ExtnavPreArmInputs {
        source_denied: false,
        pose_error: None,
        health: Some(ExtnavHealth::Good),
        north: Some(MapNorth::FcHeading),
        origin_seen: true,
        origin_by_plugin: false,
        gps_primary: false,
        extnav_set: 1,
        fallback_set: 2,
        origin_lla: Some(ORIGIN),
        params: good_params(),
        pose_age_ms: Some(118.0),
        obstacle: Some(ObstacleFeed::Live),
    }
}

fn with_param(id: &str, v: f32) -> ExtnavPreArmInputs {
    let mut i = ready();
    i.params.insert(id.to_string(), v);
    i
}

fn without_param(id: &str) -> ExtnavPreArmInputs {
    let mut i = ready();
    i.params.remove(id);
    i
}

fn eval(i: ExtnavPreArmInputs) -> PreArmReport {
    PreArmGate::default().evaluate(&inputs(Some(i)))
}

fn check<'a>(r: &'a PreArmReport, id: &str) -> (Severity, &'a str) {
    let c = r.checks.iter().find(|c| c.id == id).unwrap();
    (c.severity, c.detail.as_str())
}

#[test]
fn ready_external_nav_is_armable() {
    let r = eval(ready());
    assert!(r.armable, "{:?}", r.checks);
    let ids: Vec<&str> = r.checks.iter().map(|c| c.id).collect();
    assert_eq!(
        ids,
        [
            "extnav_source",
            "extnav_health",
            "ekf_origin",
            "fc_params",
            "viso_delay",
            "obstacle_feed"
        ]
    );
}

#[test]
fn a_missing_source_blocks() {
    let r = eval(ExtnavPreArmInputs {
        source_denied: true,
        health: None,
        ..ready()
    });
    assert!(!r.armable);
    let (sev, detail) = check(&r, "extnav_source");
    assert_eq!(sev, Severity::Blocking);
    assert!(
        detail.starts_with("External nav source missing"),
        "{detail}"
    );
    let none = PreArmGate::default().evaluate(&inputs(None));
    assert_eq!(check(&none, "extnav_source").0, Severity::Blocking);
}

#[test]
fn lost_blocks_degraded_and_origin_wait() {
    let lost = eval(ExtnavPreArmInputs {
        health: Some(ExtnavHealth::Lost),
        ..ready()
    });
    assert_eq!(check(&lost, "extnav_health").0, Severity::Blocking);
    let degraded = eval(ExtnavPreArmInputs {
        health: Some(ExtnavHealth::Degraded),
        origin_seen: false,
        obstacle: None,
        ..ready()
    });
    assert_eq!(check(&degraded, "extnav_health").0, Severity::Pending);
    assert_eq!(check(&degraded, "ekf_origin").0, Severity::Pending);
    assert_eq!(degraded.checks.len(), 5);
    assert!(!degraded.armable);
}

#[test]
fn ekf_origin_not_seen_names_who_sets_it() {
    let r = eval(ExtnavPreArmInputs {
        origin_seen: false,
        ..ready()
    });
    let (sev, detail) = check(&r, "ekf_origin");
    assert_eq!(sev, Severity::Pending);
    assert!(detail.contains("AHRS_OPTIONS bit 4"), "{detail}");
    let gps = eval(ExtnavPreArmInputs {
        origin_seen: false,
        gps_primary: true,
        ..ready()
    });
    assert_eq!(
        check(&gps, "ekf_origin"),
        (
            Severity::Pending,
            "Waiting for the GPS to set the EKF origin."
        )
    );
}

/// The late-origin spin: the plugin's origin arrived after a compass-yaw
/// alignment with declination learning on.
#[test]
fn ekf_origin_late_plugin_origin_with_autodec_blocks() {
    let late = ExtnavPreArmInputs {
        origin_by_plugin: true,
        ..ready()
    };
    let r = eval(late.clone());
    let (sev, detail) = check(&r, "ekf_origin");
    assert_eq!(sev, Severity::Blocking);
    assert!(
        detail.starts_with("The EKF origin arrived after the yaw alignment"),
        "{detail}"
    );
    assert!(!r.armable);
    // EK3_SRC1_YAW 3 (compass with GSF fallback) aligns the same way.
    let mut gsf = late.clone();
    gsf.params.insert("EK3_SRC1_YAW".into(), 3.0);
    assert_eq!(check(&eval(gsf), "ekf_origin").0, Severity::Blocking);
    // COMPASS_AUTODEC 0: the declination never steps.
    let mut fixed = late.clone();
    fixed.params.insert("COMPASS_AUTODEC".into(), 0.0);
    assert_eq!(check(&eval(fixed), "ekf_origin"), (Severity::Ok, ""));
    // Until both are read, the row waits.
    let mut unread = late;
    unread.params.remove("COMPASS_AUTODEC");
    assert_eq!(
        check(&eval(unread), "ekf_origin"),
        (Severity::Pending, "Reading flight controller parameters.")
    );
}

#[test]
fn ekf_origin_boot_origin_far_from_origin_lla_blocks() {
    let far = ExtnavPreArmInputs {
        origin_lla: Some([ORIGIN[0] + 0.1, ORIGIN[1], ORIGIN[2]]),
        ..ready()
    };
    let r = eval(far.clone());
    let (sev, detail) = check(&r, "ekf_origin");
    assert_eq!(sev, Severity::Blocking);
    assert_eq!(
        detail,
        "AHRS_ORIGIN is 11.1 km from external_nav.origin_lla; set both to the take-off site."
    );
    // Without AHRS_OPTIONS bit 4 the recorded origin is not used.
    let mut off = far;
    off.params.insert("AHRS_OPTIONS".into(), 0.0);
    assert_eq!(check(&eval(off), "ekf_origin").0, Severity::Ok);
}

#[test]
fn fc_params_wait_for_every_value() {
    let r = eval(without_param("VISO_TYPE"));
    assert_eq!(
        check(&r, "fc_params"),
        (
            Severity::Pending,
            "Reading flight controller parameters (11 of 12)."
        )
    );
    assert!(!r.armable);
}

#[test]
fn fc_params_ekf_yaw_from_a_map_that_follows_the_fc_blocks() {
    let r = eval(with_param("EK3_SRC1_YAW", 6.0));
    let (sev, detail) = check(&r, "fc_params");
    assert_eq!(sev, Severity::Blocking);
    assert_eq!(
        detail,
        "EK3_SRC1_YAW is 6 but the map's north follows the flight controller's heading; set the \
         pose provider's take-off heading, or use 1 (compass)."
    );
    // An older publisher that does not say where its north comes from.
    let mut silent = with_param("EK3_SRC1_YAW", 6.0);
    silent.north = None;
    assert_eq!(check(&eval(silent), "fc_params").0, Severity::Blocking);
    // No fresh pose yet: wait for one.
    let mut waiting = with_param("EK3_SRC1_YAW", 6.0);
    (waiting.north, waiting.health) = (None, None);
    assert_eq!(check(&eval(waiting), "fc_params").0, Severity::Pending);
}

#[test]
fn fc_params_ekf_yaw_from_a_map_with_its_own_north_passes() {
    for north in [MapNorth::Declared, MapNorth::Gnss] {
        let mut i = with_param("EK3_SRC1_YAW", 6.0);
        i.north = Some(north);
        assert_eq!(check(&eval(i), "fc_params"), (Severity::Ok, ""), "{north:?}");
    }
}

#[test]
fn fc_params_ext_nav_velocity_blocks() {
    let r = eval(with_param("EK3_SRC1_VELXY", 6.0));
    assert_eq!(
        check(&r, "fc_params"),
        (
            Severity::Blocking,
            "EK3_SRC1_VELXY is 6: the EKF would trust the pose's velocity far beyond its \
             accuracy; set 0 (position only)."
        )
    );
    assert!(!r.armable);
    // Velocity from another sensor in the external-nav set is not ours to judge.
    assert_eq!(
        check(&eval(with_param("EK3_SRC1_VELXY", 5.0)), "fc_params"),
        (Severity::Ok, "")
    );
}

#[test]
fn fc_params_rules_in_order() {
    assert_eq!(
        check(&eval(with_param("VISO_TYPE", 0.0)), "fc_params").1,
        "VISO_TYPE is 0; external navigation needs 1 (MAVLink)."
    );
    assert_eq!(
        check(&eval(with_param("EK3_SRC1_POSXY", 3.0)), "fc_params").1,
        "EK3_SRC1_POSXY is 3; the external-nav source set needs 6 (ExternalNav)."
    );
    assert_eq!(
        check(&eval(with_param("EK3_SRC2_VELXY", 3.0)), "fc_params").1,
        "EK3_SRC2_VELXY is 3; the fallback set needs 5 (optical flow)."
    );
    // GPS primary: the fallback set is the GPS set.
    let gps = ExtnavPreArmInputs {
        gps_primary: true,
        ..ready()
    };
    assert_eq!(
        check(&eval(gps.clone()), "fc_params").1,
        "EK3_SRC2_POSXY is 0; with GPS primary the fallback set needs 3 (GPS)."
    );
    let mut gps_ok = gps;
    gps_ok.params.insert("EK3_SRC2_POSXY".into(), 3.0);
    assert_eq!(check(&eval(gps_ok), "fc_params"), (Severity::Ok, ""));
}

#[test]
fn viso_delay_follows_the_measured_pose_age() {
    let slow = ExtnavPreArmInputs {
        pose_age_ms: Some(240.4),
        ..ready()
    };
    assert_eq!(
        check(&eval(slow), "viso_delay"),
        (
            Severity::Pending,
            "VISO_DELAY_MS is 120 ms but poses are 240 ms old when sent; set VISO_DELAY_MS to 240."
        )
    );
    let unknown = ExtnavPreArmInputs {
        pose_age_ms: None,
        ..ready()
    };
    assert_eq!(check(&eval(unknown), "viso_delay"), (Severity::Ok, ""));
    assert_eq!(check(&eval(ready()), "viso_delay"), (Severity::Ok, ""));
}

/// The compass library's late declination update reaches EKF3 only when it
/// does not use its own table field (`EK3_MAG_EF_LIM` 0).
#[test]
fn fc_params_autodec_without_the_ekf_table_field_blocks() {
    let r = eval(with_param("EK3_MAG_EF_LIM", 0.0));
    let (sev, detail) = check(&r, "fc_params");
    assert_eq!(sev, Severity::Blocking);
    assert!(
        detail.starts_with("The compass declination will step when the EKF first gets a position"),
        "{detail}"
    );
    // The default limit with declination learning on is fine.
    assert_eq!(check(&eval(ready()), "fc_params"), (Severity::Ok, ""));
    // A fixed declination with the limit at 0 is fine too.
    let mut fixed = with_param("EK3_MAG_EF_LIM", 0.0);
    fixed.params.insert("COMPASS_AUTODEC".into(), 0.0);
    assert_eq!(check(&eval(fixed), "fc_params"), (Severity::Ok, ""));
}

#[test]
fn haversine_matches_a_known_distance() {
    // 0.1° of latitude is 11.12 km.
    let d = haversine_m(52.0, -0.6, 52.1, -0.6);
    assert!((d - 11_119.5).abs() < 1.0, "{d}");
}
