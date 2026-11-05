use std::f64::consts::PI;

use world_engine_protocol::atlas::nav::OCCUPANCY_OCCUPIED;
use world_engine_protocol::atlas::{ComputePlacement, NavPoseMode, PoseSource, VioHealth};

use super::*;
use crate::pose::se3;

fn settings(nav_pose: NavPoseMode, placement: ComputePlacement) -> NavSettings {
    NavSettings {
        nav_pose,
        placement,
        budget: Budget::from_mb(64),
        pose_interval_ms: 33,
    }
}

fn estimate(x: f64, y: f64, yaw: f64) -> PoseEstimate {
    let (s, c) = yaw.sin_cos();
    PoseEstimate {
        t_ns: 0,
        r: [c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0],
        t: [x, y, 0.25],
        cov: [0.0; 36],
        source: PoseSource::Lidar,
        health: VioHealth::Good,
    }
}

/// A ring of returns 3.02 m around the sensor (FLU), 500 points.
fn ring() -> LidarScan {
    let points: Vec<[f32; 3]> = (0..500)
        .map(|k| {
            let a = 2.0 * std::f32::consts::PI * k as f32 / 500.0;
            [3.02 * a.cos(), 3.02 * a.sin(), 0.0]
        })
        .collect();
    LidarScan {
        sensor_id: "lidar0".into(),
        t_ns: 1,
        t_offset_ns: vec![0; points.len()],
        intensity: vec![50; points.len()],
        points,
    }
}

fn mount() -> [f64; 16] {
    se3::to_mat4(&se3::IDENTITY, [0.0, 0.0, 0.0])
}

#[test]
fn the_pose_is_published_in_map_ned_at_the_configured_interval() {
    // The FC heading equals the SLAM heading here, so map_ned = map_frd.
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Hybrid));
    let pd = nav
        .on_pose(
            &estimate(1.0, 2.0, PI / 2.0),
            Some(10.0),
            Some(-PI / 2.0),
            1_000,
            false,
        )
        .expect("always publishes with no session");
    assert_eq!(pd.pose.t[0], 1.0);
    assert_eq!(pd.pose.t[1], -2.0, "FLU left is FRD negative y");
    assert_eq!(pd.pose.t[2], 0.0, "z is the FC height above the first pose");
    // A left turn (FLU yaw +90°) reads -90° in FRD.
    assert!((pd.pose.r[3].atan2(pd.pose.r[0]) + PI / 2.0).abs() < 1e-12);
    assert_eq!(pd.source, NavPoseSource::LidarPlanar);
    assert_eq!(pd.frame, NavFrame::MapNed);

    assert!(nav
        .on_pose(&estimate(1.0, 2.0, 0.0), Some(10.0), None, 1_020, false)
        .is_none());
    let up = nav
        .on_pose(&estimate(1.0, 2.0, 0.0), Some(12.5), None, 1_034, false)
        .unwrap();
    assert_eq!(up.pose.t[2], -2.5, "climbing 2.5 m is FRD z -2.5");
}

#[test]
fn capture_mode_publishes_only_with_a_session_open() {
    let mut nav = NavMap::new(settings(NavPoseMode::Capture, ComputePlacement::Hybrid));
    assert!(nav
        .on_pose(&estimate(0.0, 0.0, 0.0), None, Some(0.0), 0, false)
        .is_none());
    assert!(nav
        .on_scan(
            &ring(),
            &mount(),
            Some(&estimate(0.0, 0.0, 0.0)),
            None,
            false
        )
        .scan
        .is_none());
    assert!(nav
        .on_pose(&estimate(0.0, 0.0, 0.0), None, None, 100, true)
        .is_some());
}

#[test]
fn scans_build_the_live_map_and_the_status_reports_it() {
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Onboard));
    let e = estimate(0.0, 0.0, 0.0);
    nav.on_pose(&e, Some(0.0), Some(0.0), 0, false);
    for _ in 0..4 {
        let out = nav.on_scan(&ring(), &mount(), Some(&e), Some(0.0), false);
        let scan = out.scan.expect("the binned scan");
        assert!(scan.ranges_m.iter().all(|r| (r - 3.02).abs() < 0.01));
        assert_eq!(out.polar.expect("a polar record").range_mm[0], 3020);
    }
    assert_eq!(nav.map.grid.value_at(3.02, 0.01), OCCUPANCY_OCCUPIED);
    let tick = nav.tick(&TickInputs::default(), false);
    let occ = tick.occupancy.expect("changed tiles");
    assert!(!occ.tiles.is_empty());
    let cloud = tick.pointcloud.expect("a sweep delta");
    assert!(cloud.count > 100);
    assert_eq!(cloud.voxel_m, 0.05, "onboard keeps the 5 cm cloud");
    let st = tick.status;
    assert_eq!(st.placement, ComputePlacement::Onboard);
    assert!(st.area_m2 > 20.0, "about a 3 m disc: {}", st.area_m2);
    assert!(st.memory_used_mb > 0.0 && st.memory_budget_mb == 64.0);
    assert_eq!(st.sweep_points, cloud.count as u64);

    let again = nav.tick(&TickInputs::default(), false);
    assert!(
        again.occupancy.is_none() && again.pointcloud.is_none(),
        "nothing new"
    );
}

#[test]
fn a_lost_pose_never_paints_the_map() {
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Hybrid));
    let mut e = estimate(0.0, 0.0, 0.0);
    e.health = VioHealth::Lost;
    nav.on_pose(&estimate(0.0, 0.0, 0.0), None, Some(0.0), 0, false);
    let out = nav.on_scan(&ring(), &mount(), Some(&e), None, false);
    assert_eq!(out.polar.unwrap().health, VioHealth::Lost);
    assert_eq!(nav.map.grid.tile_count(), 0);
    assert!(nav.map.sweep.is_empty());
}

#[test]
fn streaming_keeps_the_coarse_cloud_and_waits_for_the_link() {
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Stream));
    assert!(nav.streams());
    nav.on_pose(&estimate(0.0, 0.0, 0.0), None, Some(0.0), 0, false);
    nav.on_scan(
        &ring(),
        &mount(),
        Some(&estimate(0.0, 0.0, 0.0)),
        None,
        false,
    );
    let st = nav.tick(&TickInputs::default(), false).status;
    assert_eq!(
        st.stream.state,
        world_engine_protocol::atlas::StreamPhase::Connecting
    );
    assert_eq!(
        nav.map.sweep.voxel_m(),
        0.20,
        "the drone keeps the coarse cloud"
    );
}

#[test]
fn a_closing_session_gets_the_cloud_and_the_tiles() {
    let dir = tempfile::tempdir().unwrap();
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Onboard));
    let e = estimate(0.0, 0.0, 0.0);
    nav.on_pose(&e, None, Some(0.0), 0, false);
    nav.on_scan(&ring(), &mount(), Some(&e), None, false);
    nav.save(dir.path());
    let cloud = std::fs::metadata(dir.path().join(SESSION_MAP_DIR).join(SWEEP_FILE)).unwrap();
    assert_eq!(cloud.len() % 12, 0);
    assert!(cloud.len() > 0);
    assert!(dir
        .path()
        .join(SESSION_MAP_DIR)
        .join(TILES_DIR)
        .join("0_0.png")
        .exists());
}

#[test]
fn nothing_is_placed_before_the_fc_heading_is_known() {
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Onboard));
    let e = estimate(1.0, 0.0, 0.0);
    assert!(nav.on_pose(&e, None, None, 0, false).is_none());
    assert!(!nav.heading_known());
    let out = nav.on_scan(&ring(), &mount(), Some(&e), None, false);
    assert!(out.scan.is_some(), "the body-frame scan needs no north");
    assert!(out.polar.is_none(), "no map record without north");
    assert_eq!(nav.map.grid.tile_count(), 0);
    assert!(nav.tick(&TickInputs::default(), false).occupancy.is_none());
}

#[test]
fn the_map_is_rotated_to_north_by_the_fc_heading_at_the_first_pose() {
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Onboard));
    // The SLAM world's x is the body's forward at the first scan; the FC's
    // compass says the body faced east (heading +90°).
    let pd = nav
        .on_pose(&estimate(0.0, 0.0, 0.0), None, Some(PI / 2.0), 0, false)
        .unwrap();
    assert!(nav.heading_known());
    assert!(
        (pd.pose.r[3].atan2(pd.pose.r[0]) - PI / 2.0).abs() < 1e-12,
        "yaw reads east"
    );
    // One metre forward in SLAM is one metre east in map_ned.
    let pd = nav
        .on_pose(&estimate(1.0, 0.0, 0.0), None, Some(0.3), 1_000, false)
        .unwrap();
    assert!(
        pd.pose.t[0].abs() < 1e-12 && (pd.pose.t[1] - 1.0).abs() < 1e-12,
        "{:?}",
        pd.pose.t
    );
    // A later heading reading does not move the latched north.
    let out = nav.on_scan(
        &ring(),
        &mount(),
        Some(&estimate(1.0, 0.0, 0.0)),
        None,
        false,
    );
    let polar = out.polar.unwrap();
    assert!(
        polar.t[0].abs() < 1e-5 && (polar.t[1] - 1.0).abs() < 1e-5,
        "{:?}",
        polar.t
    );
    // The ring's forward return (3.02 m ahead) lands 3.02 m further east.
    let ahead = polar.world_points().next().unwrap();
    assert!(
        ahead[0].abs() < 1e-3 && (ahead[1] - 4.02).abs() < 1e-3,
        "{ahead:?}"
    );
}

/// ArduPilot's heading is magnetic until the EKF has an origin, then turns
/// onto true north (the bench's location has 12.8° of declination). A map
/// north latched at the first pose kept the magnetic heading, 12.8° off the
/// EKF's yaw once it settled, and the EKF fell over on the inconsistent
/// external navigation (variance failsafe, then Land).
#[test]
fn the_north_follows_the_fc_heading_on_the_spot_and_is_fixed_at_take_off() {
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Onboard));
    let truth = 180f64.to_radians();
    let decl = 12.8f64.to_radians();
    // On the ground, the SLAM body still at its first pose (small jitter), the
    // FC heading turning from magnetic to true north.
    for k in 0..=20 {
        let heading = truth - decl * (1.0 - k as f64 / 20.0);
        let jitter = 0.02 * (k % 3) as f64;
        nav.on_pose(&estimate(jitter, -jitter, 0.0), Some(0.0), Some(heading), k * 100, false)
            .unwrap();
    }
    assert!((nav.map_yaw().unwrap() - truth).abs() < 1e-9, "the settled heading");
    // Take-off: a climb past ON_SPOT_M fixes the north, and a later compass
    // swing in flight no longer moves it.
    nav.on_pose(&estimate(0.0, 0.0, 0.0), Some(ON_SPOT_M + 0.1), Some(truth), 2_100, false)
        .unwrap();
    let pd = nav
        .on_pose(&estimate(10.0, 0.0, 0.0), Some(3.0), Some(truth + 0.1), 2_200, false)
        .unwrap();
    assert!((nav.map_yaw().unwrap() - truth).abs() < 1e-9);
    // Ten metres forward in SLAM is ten metres south (heading 180°).
    assert!(
        (pd.pose.t[0] + 10.0).abs() < 1e-9 && pd.pose.t[1].abs() < 1e-9,
        "{:?}",
        pd.pose.t
    );
    // Back over the take-off spot, the north stays fixed.
    nav.on_pose(&estimate(0.0, 0.0, 0.0), Some(0.0), Some(truth - 0.2), 2_300, false)
        .unwrap();
    assert!((nav.map_yaw().unwrap() - truth).abs() < 1e-9);
}

#[test]
fn moving_off_the_spot_on_the_ground_fixes_the_north_too() {
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Onboard));
    nav.on_pose(&estimate(0.0, 0.0, 0.0), None, Some(0.5), 0, false)
        .unwrap();
    nav.on_pose(&estimate(0.3, 0.0, 0.0), None, Some(0.6), 100, false)
        .unwrap();
    assert!((nav.map_yaw().unwrap() - 0.6).abs() < 1e-12, "0.3 m is still on the spot");
    nav.on_pose(&estimate(0.3, 0.45, 0.0), None, Some(0.7), 200, false)
        .unwrap();
    assert!((nav.map_yaw().unwrap() - 0.6).abs() < 1e-12, "0.54 m away: fixed");
}

/// A forest-trail-like flight on GNSS, then GNSS loss and 137 m more on the
/// map. Truth is the EKF's NED frame, whose origin lies 7 m from the first
/// scan; the SLAM world is the body at the first pose (heading 220.8°); the
/// compass reads 3° east of the truth at the first pose; the EKF's GNSS
/// position carries white noise and a slow wander. After the loss ArduPilot
/// resets its position to the external one and follows it, so the drone's
/// position error is the published map's error against truth.
#[test]
fn a_3_degree_compass_error_stays_under_a_metre_137_m_past_gnss_loss() {
    let h0 = 220.8f64.to_radians();
    let compass_err = 3f64.to_radians();
    let origin = [6.0, -4.0];
    // (heading °, length m): 170 m on GNSS, then 160 m on the map.
    let legs = [
        (220.8, 40.0),
        (235.0, 30.0),
        (260.0, 40.0),
        (275.0, 60.0),
        (285.0, 80.0),
        (300.0, 80.0),
    ];
    let step = 0.3;
    // The truth relative to the first scan, and its heading.
    let mut path = vec![[0.0f64, 0.0f64]];
    let mut yaws = vec![h0];
    for (hdg, len) in legs {
        let (s, c) = f64::to_radians(hdg).sin_cos();
        for _ in 0..(len / step) as usize {
            let p = *path.last().unwrap();
            path.push([p[0] + c * step, p[1] + s * step]);
            yaws.push(f64::to_radians(hdg));
        }
    }
    let truth = |k: usize| [path[k][0] + origin[0], path[k][1] + origin[1]];
    let k_loss = (170.0 / step) as usize;
    let mut seed = 0x2545_f491_4f6c_dd1du64;
    let mut noise = || {
        // xorshift, uniform in [-0.5, 0.5) m: σ ≈ 0.29 m.
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1u64 << 53) as f64 - 0.5
    };
    // FLU SLAM estimate of sample k: map_frd = Rz(-h0) · (ned − first scan).
    let slam = |k: usize| {
        let frd = align::rot2(-h0, path[k]);
        estimate(frd[0], -frd[1], -(yaws[k] - h0))
    };
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Onboard));
    let mut published = Vec::with_capacity(path.len());
    for k in 0..path.len() {
        let e = slam(k);
        if k < k_loss {
            let wander = 0.3 * (k as f64 / 400.0).sin();
            let t = truth(k);
            nav.on_gnss_ref(&e, [t[0] + noise() + wander, t[1] + noise() - wander]);
        } else {
            nav.on_gnss_lost();
        }
        let pd = nav
            .on_pose(&e, None, Some(h0 + compass_err), k as i64 * 100, false)
            .expect("published every 100 ms");
        published.push([pd.pose.t[0], pd.pose.t[1]]);
    }
    let fit = nav.align.fit().expect("fitted on GNSS");
    let err = |k: usize, p: [f64; 2]| {
        let t = truth(k);
        (p[0] - t[0]).hypot(p[1] - t[1])
    };
    let k137 = (k_loss..path.len())
        .find(|&k| {
            let d = [path[k][0] - path[k_loss][0], path[k][1] - path[k_loss][1]];
            d[0].hypot(d[1]) >= 137.0
        })
        .expect("the route runs 137 m past the loss");
    // The GNSS wander at the loss (0.42 m here) is the EKF's own error too.
    let at_switch = err(k_loss, published[k_loss]);
    assert!(at_switch < 0.6, "{at_switch:.2} m jump at the switch");
    let worst = (k_loss..=k137).map(|k| err(k, published[k])).fold(0.0, f64::max);
    assert!(
        worst < 1.0,
        "{worst:.2} m within 137 m of the loss (fit {:.3}°, baseline {:.1} m)",
        wrap_pi(fit.yaw - h0).to_degrees(),
        fit.baseline_m
    );
    // The compass-latched map at the first scan alone, for scale: the fault
    // this guards.
    let compass = err(k137, align::rot2(h0 + compass_err, {
        let e = slam(k137);
        [e.t[0], -e.t[1]]
    }));
    assert!(compass > 6.0, "the compass latch alone is {compass:.2} m off");
    let fit_err = wrap_pi(fit.yaw - h0).to_degrees();
    assert!(fit_err.abs() < 0.4, "fit off by {fit_err:.3}°");
}
