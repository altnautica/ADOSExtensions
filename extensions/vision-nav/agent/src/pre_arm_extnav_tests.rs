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
/// set at boot from AHRS_ORIGIN, external-nav position with optical-flow
/// velocity, compass yaw, flow fallback.
fn good_params() -> BTreeMap<String, f32> {
    [
        ("VISO_TYPE", 1.0),
        ("VISO_DELAY_MS", 120.0),
        ("COMPASS_AUTODEC", 1.0),
        ("AHRS_OPTIONS", 16.0),
        ("AHRS_ORIGIN_LAT", ORIGIN[0] as f32),
        ("AHRS_ORIGIN_LON", ORIGIN[1] as f32),
        ("EK3_SRC1_POSXY", 6.0),
        ("EK3_SRC1_VELXY", 5.0),
        ("EK3_SRC1_YAW", 1.0),
        ("EK3_SRC2_POSXY", 0.0),
        ("EK3_SRC2_VELXY", 5.0),
        ("EK3_MAG_EF_LIM", 50.0),
        ("FLOW_TYPE", 5.0),
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
        imu_feed: None,
        origin_seen: true,
        origin_by_plugin: false,
        gps_primary: false,
        extnav_set: 1,
        fallback_set: 2,
        origin_lla: Some(ORIGIN),
        params: good_params(),
        pose_age_ms: Some(118.0),
        obstacle: Some(ObstacleFeed::Live),
        fc_flow: Some(FcFlow {
            quality: 200,
            age_ns: 30_000_000,
        }),
        fc_heard_ns: Some(10_000_000_000),
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
            "fc_flow",
            "obstacle_feed"
        ]
    );
}

#[test]
fn fc_flow_stale_blocks_while_a_source_set_fuses_flow() {
    let stale = eval(ExtnavPreArmInputs {
        fc_flow: Some(FcFlow {
            quality: 200,
            age_ns: 2_500_000_000,
        }),
        ..ready()
    });
    let (sev, detail) = check(&stale, "fc_flow");
    assert_eq!(sev, Severity::Blocking);
    assert!(detail.contains("2.5 s"), "{detail}");
    assert!(!stale.armable);
    assert_eq!(
        check(&eval(ready()), "fc_flow"),
        (Severity::Ok, "Quality 200/255.")
    );
    // Quality 0: the EKF fuses nothing, so the row waits.
    let dark = eval(ExtnavPreArmInputs {
        fc_flow: Some(FcFlow {
            quality: 0,
            age_ns: 30_000_000,
        }),
        ..ready()
    });
    assert_eq!(check(&dark, "fc_flow").0, Severity::Pending);
}

#[test]
fn fc_flow_never_heard_waits_then_blocks() {
    let early = eval(ExtnavPreArmInputs {
        fc_flow: None,
        fc_heard_ns: Some(2_000_000_000),
        ..ready()
    });
    assert_eq!(check(&early, "fc_flow").0, Severity::Pending);
    let no_fc = eval(ExtnavPreArmInputs {
        fc_flow: None,
        fc_heard_ns: None,
        ..ready()
    });
    assert_eq!(check(&no_fc, "fc_flow").0, Severity::Pending);
    let never = eval(ExtnavPreArmInputs {
        fc_flow: None,
        fc_heard_ns: Some(FC_FLOW_FIRST_NS),
        ..ready()
    });
    let (sev, detail) = check(&never, "fc_flow");
    assert_eq!(sev, Severity::Blocking);
    assert!(detail.contains("FLOW_TYPE"), "{detail}");
}

#[test]
fn fc_flow_row_only_when_a_source_set_fuses_flow() {
    let has_row = |i: ExtnavPreArmInputs| eval(i).checks.iter().any(|c| c.id == "fc_flow");
    let no_flow = |mut i: ExtnavPreArmInputs| {
        i.fc_flow = None;
        i
    };
    // No flow sensor: fc_params speaks for VELXY 5 without FLOW_TYPE.
    assert!(!has_row(no_flow(with_param("FLOW_TYPE", 0.0))));
    assert!(!has_row(no_flow(without_param("FLOW_TYPE"))));
    // GPS primary with a GPS fallback and SLAM-only velocity: flow unused.
    let mut gps = no_flow(with_param("EK3_SRC1_VELXY", 0.0));
    gps.gps_primary = true;
    gps.params.insert("EK3_SRC2_VELXY".into(), 3.0);
    assert!(!has_row(gps));
    // GPS primary: the fallback set's flow does not count, even on VELXY 5.
    let mut gps_flow_fallback = no_flow(with_param("EK3_SRC1_VELXY", 0.0));
    gps_flow_fallback.gps_primary = true;
    assert_eq!(gps_flow_fallback.int("EK3_SRC2_VELXY"), Some(5));
    assert!(!has_row(gps_flow_fallback));
    // GPS primary with flow in the external-nav set: the row stays.
    let mut gps_flow_extnav = no_flow(with_param("EK3_SRC1_VELXY", 5.0));
    gps_flow_extnav.gps_primary = true;
    assert!(has_row(gps_flow_extnav));
    // SLAM-only velocity in the external-nav set, flow in the fallback set.
    assert!(has_row(no_flow(with_param("EK3_SRC1_VELXY", 0.0))));
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
    // No obstacle row with the feed off; the flow row stays.
    assert!(degraded.checks.iter().all(|c| c.id != "obstacle_feed"));
    assert_eq!(degraded.checks.len(), 6);
    assert!(!degraded.armable);
}

/// A pose the publisher's FC IMU feed held down says which feed failed.
#[test]
fn a_pose_held_down_by_the_fc_imu_feed_names_it() {
    let lost = eval(ExtnavPreArmInputs {
        health: Some(ExtnavHealth::Lost),
        imu_feed: Some(ImuFeed::Stalled),
        ..ready()
    });
    assert_eq!(
        check(&lost, "extnav_health"),
        (
            Severity::Blocking,
            "Tracking lost (the flight controller's RAW_IMU stopped)."
        )
    );
    let degraded = eval(ExtnavPreArmInputs {
        health: Some(ExtnavHealth::Degraded),
        imu_feed: Some(ImuFeed::Starved),
        ..ready()
    });
    assert_eq!(
        check(&degraded, "extnav_health").1,
        "Tracking degraded (the flight controller's RAW_IMU is under 40 Hz); position noise raised."
    );
    let plain = eval(ExtnavPreArmInputs {
        health: Some(ExtnavHealth::Lost),
        ..ready()
    });
    assert_eq!(check(&plain, "extnav_health").1, "Tracking lost.");
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
            "Reading flight controller parameters (12 of 13)."
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
             accuracy; set 5 (optical flow)."
        )
    );
    assert!(!r.armable);
    let mut no_flow = with_param("EK3_SRC1_VELXY", 6.0);
    no_flow.params.insert("FLOW_TYPE".into(), 0.0);
    assert_eq!(
        check(&eval(no_flow), "fc_params").1,
        "EK3_SRC1_VELXY is 6: the EKF would trust the pose's velocity far beyond its \
         accuracy; set 0 (position only)."
    );
}

/// A configured flow sensor belongs in the external-nav set: its velocity
/// is measured, not differenced from late poses.
#[test]
fn fc_params_external_nav_set_fuses_the_flow_sensor() {
    assert_eq!(check(&eval(ready()), "fc_params"), (Severity::Ok, ""));
    let ignored = eval(with_param("EK3_SRC1_VELXY", 0.0));
    assert_eq!(
        check(&ignored, "fc_params"),
        (
            Severity::Ok,
            "FLOW_TYPE is 5 but EK3_SRC1_VELXY is 0: the external-nav set ignores the \
             optical-flow sensor. SLAM-only velocity is allowed; EK3_SRC1_VELXY 5 lets the EKF \
             take its velocity from the flow between poses."
        )
    );
    assert!(
        ignored.armable,
        "SLAM-only is the operator's choice: a warning, not a hold"
    );
    assert_eq!(
        check(&eval(with_param("FLOW_TYPE", 0.0)), "fc_params"),
        (
            Severity::Blocking,
            "EK3_SRC1_VELXY is 5 (optical flow) but FLOW_TYPE is 0: the EKF would get no \
             velocity; set FLOW_TYPE for the flow sensor, or EK3_SRC1_VELXY 0."
        )
    );
}

/// GNSS-denied, the fallback set flies on flow velocity: it needs a flow
/// sensor configured.
#[test]
fn fc_params_fallback_flow_velocity_needs_a_flow_sensor() {
    let mut no_sensor = with_param("EK3_SRC1_VELXY", 0.0);
    no_sensor.params.insert("FLOW_TYPE".into(), 0.0);
    let r = eval(no_sensor);
    assert_eq!(
        check(&r, "fc_params"),
        (
            Severity::Blocking,
            "EK3_SRC2_VELXY is 5 (optical flow) but FLOW_TYPE is 0: the fallback set would get \
             no velocity; set FLOW_TYPE for the flow sensor."
        )
    );
    assert!(!r.armable);
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

/// Poses older than the EKF can place get a message about the pipeline,
/// not a delay the flight controller cannot take.
#[test]
fn viso_delay_flags_poses_older_than_the_ekf_window() {
    let late = ExtnavPreArmInputs {
        pose_age_ms: Some(452.0),
        ..ready()
    };
    assert_eq!(
        check(&eval(late), "viso_delay"),
        (
            Severity::Pending,
            "Poses are 452 ms old when sent, older than the 250 ms the flight controller's EKF \
             can place in time: it fuses them late and the vehicle sways. VISO_DELAY_MS cannot \
             cover the difference; cut the companion's load or LiDAR processing time."
        )
    );
    let mut at_cap = with_param("VISO_DELAY_MS", 250.0);
    at_cap.pose_age_ms = Some(452.0);
    assert_eq!(check(&eval(at_cap), "viso_delay").0, Severity::Pending);
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
