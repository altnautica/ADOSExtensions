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
        takeoff_heading_rad: None,
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
        sigma_xy_m: None,
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
            &at(estimate(1.0, 2.0, PI / 2.0), 1_000),
            Some(10.0),
            Some(-PI / 2.0),
            1_000,
            0,
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
        .on_pose(
            &at(estimate(1.0, 2.0, 0.0), 1_010),
            Some(10.0),
            None,
            1_020,
            0,
            false
        )
        .is_none());
    let up = nav
        .on_pose(
            &at(estimate(1.0, 2.0, 0.0), 1_034),
            Some(12.5),
            None,
            1_034,
            0,
            false,
        )
        .unwrap();
    assert_eq!(up.pose.t[2], -2.5, "climbing 2.5 m is FRD z -2.5");
}

/// The estimator's horizontal uncertainty rides on the published pose, and
/// an estimator that reports none publishes none.
#[test]
fn the_pose_carries_the_estimators_horizontal_sigma() {
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Hybrid));
    let mut e = at(estimate(0.0, 0.0, 0.0), 0);
    e.sigma_xy_m = Some(0.65);
    let pd = nav.on_pose(&e, Some(0.0), Some(0.0), 0, 0, false).unwrap();
    assert_eq!(pd.sigma_xy_m, Some(0.65));
    let pd = nav
        .on_pose(&at(estimate(0.0, 0.0, 0.0), 1_000), Some(0.0), Some(0.0), 1_000, 0, false)
        .unwrap();
    assert_eq!(pd.sigma_xy_m, None);
}

/// The matcher's map starting over (at the first `RAW_IMU`) latches the
/// north afresh from the FC heading, once per restart.
#[test]
fn a_restarted_map_latches_its_north_afresh_once() {
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Hybrid));
    let pose = |nav: &mut NavMap, x: f64, heading: f64, t_ms: i64| {
        nav.on_pose(&at(estimate(x, 0.0, 0.0), t_ms), Some(0.0), Some(heading), t_ms, 0, false);
    };
    pose(&mut nav, 0.0, 0.0, 0);
    pose(&mut nav, 5.0, 0.0, 100);
    pose(&mut nav, 5.0, 1.0, 200);
    assert_eq!(nav.map_yaw(), Some(0.0), "fixed once the vehicle left the spot");
    nav.restart_heading_at(7);
    assert_eq!(nav.map_yaw(), None, "no north until the next pose");
    pose(&mut nav, 5.0, 1.0, 300);
    assert_eq!(nav.map_yaw(), Some(1.0));
    pose(&mut nav, 10.0, 1.0, 400);
    nav.restart_heading_at(7);
    pose(&mut nav, 10.0, 2.0, 500);
    assert_eq!(nav.map_yaw(), Some(1.0), "the same restart is taken once");
}

/// The descriptor carries the time the pose describes, not the time it is
/// published: a LiDAR pose published 150 ms after its scan's first return
/// is stamped 150 ms back.
#[test]
fn the_pose_is_stamped_with_its_measurement_time() {
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Hybrid));
    let mut e = estimate(0.0, 0.0, 0.0);
    e.t_ns = 4_000_000_000;
    let pd = nav
        .on_pose(&e, Some(0.0), Some(0.0), 1_000_000, 4_150_000_000, false)
        .unwrap();
    assert_eq!(pd.ts_ms, 999_850);
}

/// The estimate at `t_ms` of measurement time (monotonic).
fn at(mut e: PoseEstimate, t_ms: i64) -> PoseEstimate {
    e.t_ns = t_ms as u64 * 1_000_000;
    e
}

/// A 10 Hz LiDAR matcher against a 100 ms interval: the scans land a few ms
/// either side of 100 ms apart and the poll that hands them over runs on its
/// own uneven clock. A gate on the poll's clock dropped every pose that came
/// 99 ms after the last; the gate on the measurement time drops none.
#[test]
fn a_10_hz_provider_publishes_every_pose_at_a_100_ms_interval() {
    let mut nav = NavMap::new(NavSettings {
        pose_interval_ms: 100,
        ..settings(NavPoseMode::Always, ComputePlacement::Hybrid)
    });
    let jitter_ms = [0, 3, -3, 2, -2, 1, -1, 3, -3, 0];
    let poll_step_ms = [95, 105, 99, 101, 97, 103, 100, 96, 104, 98];
    let mut now_ms = 1_000_000;
    let mut published = 0;
    for k in 0..50 {
        let e = at(
            estimate(0.0, 0.0, 0.0),
            1_000 + k * 100 + jitter_ms[k as usize % 10],
        );
        now_ms += poll_step_ms[k as usize % 10];
        let now_mono_ns = e.t_ns + 120_000_000;
        if nav
            .on_pose(&e, Some(0.0), Some(0.0), now_ms, now_mono_ns, false)
            .is_some()
        {
            published += 1;
        }
    }
    assert_eq!(published, 50, "every 10 Hz pose is published");
}

/// A 30 Hz provider against a 100 ms interval publishes every third pose on
/// average, never two within half an interval of measurement time.
#[test]
fn a_30_hz_provider_is_thinned_to_the_interval() {
    let mut nav = NavMap::new(NavSettings {
        pose_interval_ms: 100,
        ..settings(NavPoseMode::Always, ComputePlacement::Hybrid)
    });
    let mut published = Vec::new();
    for k in 0..90 {
        let mut e = estimate(0.0, 0.0, 0.0);
        e.t_ns = 1_000_000_000 + k * 1_000_000_000 / 30;
        if nav
            .on_pose(
                &e,
                Some(0.0),
                Some(0.0),
                1_000 + k as i64 * 33,
                e.t_ns,
                false,
            )
            .is_some()
        {
            published.push(e.t_ns);
        }
    }
    assert!(
        (29..=31).contains(&published.len()),
        "{} of 90",
        published.len()
    );
    assert!(
        published.windows(2).all(|w| w[1] - w[0] >= 50_000_000),
        "{published:?}"
    );
}

/// A provider reset starts its clock again: the first pose after it is
/// published at once.
#[test]
fn a_measurement_time_that_goes_backwards_publishes() {
    let mut nav = NavMap::new(NavSettings {
        pose_interval_ms: 100,
        ..settings(NavPoseMode::Always, ComputePlacement::Hybrid)
    });
    let mut pose = |t_ms| {
        nav.on_pose(
            &at(estimate(0.0, 0.0, 0.0), t_ms),
            Some(0.0),
            Some(0.0),
            0,
            0,
            false,
        )
    };
    assert!(pose(5_000).is_some());
    assert!(pose(5_010).is_none());
    assert!(pose(20).is_some(), "the reset publishes");
    assert!(pose(30).is_none());
    assert!(pose(120).is_some());
}

#[test]
fn capture_mode_publishes_only_with_a_session_open() {
    let mut nav = NavMap::new(settings(NavPoseMode::Capture, ComputePlacement::Hybrid));
    assert!(nav
        .on_pose(&estimate(0.0, 0.0, 0.0), None, Some(0.0), 0, 0, false)
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
        .on_pose(&estimate(0.0, 0.0, 0.0), None, None, 100, 0, true)
        .is_some());
}

#[test]
fn scans_build_the_live_map_and_the_status_reports_it() {
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Onboard));
    let e = estimate(0.0, 0.0, 0.0);
    nav.on_pose(&e, Some(0.0), Some(0.0), 0, 0, false);
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
    assert_eq!(st.memory_used_mb, None, "no resident memory read: null");
    assert_eq!(st.memory_budget_mb, 64.0);
    assert_eq!(st.sweep_points, cloud.count as u64);

    let again = nav.tick(&TickInputs::default(), false);
    assert!(
        again.occupancy.is_none() && again.pointcloud.is_none(),
        "nothing new"
    );
}

/// `memory_used_mb` is the capture process's resident memory, read from
/// `/proc/self/status`; a status without `VmRSS` reads null, never 0.
#[test]
fn the_status_reports_the_resident_memory_or_null() {
    let status = "Name:\tworld-engine-ca\nVmPeak:\t  90000 kB\nVmRSS:\t   46080 kB\nThreads:\t9\n";
    assert_eq!(vm_rss_mb(status), Some(45.0));
    assert_eq!(vm_rss_mb("Name:\tx\nThreads:\t1\n"), None);
    assert_eq!(vm_rss_mb("VmRSS:\tnot-a-number kB\n"), None);
    let own = read_vm_rss_mb().expect("this test process has a VmRSS");
    assert!(own > 0.0, "{own}");

    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Onboard));
    let inputs = TickInputs {
        rss_mb: Some(45.0),
        ..TickInputs::default()
    };
    assert_eq!(nav.tick(&inputs, false).status.memory_used_mb, Some(45.0));
}

#[test]
fn a_lost_pose_never_paints_the_map() {
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Hybrid));
    let mut e = estimate(0.0, 0.0, 0.0);
    e.health = VioHealth::Lost;
    nav.on_pose(&estimate(0.0, 0.0, 0.0), None, Some(0.0), 0, 0, false);
    let out = nav.on_scan(&ring(), &mount(), Some(&e), None, false);
    assert_eq!(out.polar.unwrap().health, VioHealth::Lost);
    assert_eq!(nav.map.grid.tile_count(), 0);
    assert!(nav.map.sweep.is_empty());
}

#[test]
fn streaming_keeps_the_coarse_cloud_and_waits_for_the_link() {
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Stream));
    assert!(nav.streams());
    nav.on_pose(&estimate(0.0, 0.0, 0.0), None, Some(0.0), 0, 0, false);
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
    nav.on_pose(&e, None, Some(0.0), 0, 0, false);
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
    assert!(nav.on_pose(&e, None, None, 0, 0, false).is_none());
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
        .on_pose(&estimate(0.0, 0.0, 0.0), None, Some(PI / 2.0), 0, 0, false)
        .unwrap();
    assert!(nav.heading_known());
    assert!(
        (pd.pose.r[3].atan2(pd.pose.r[0]) - PI / 2.0).abs() < 1e-12,
        "yaw reads east"
    );
    // One metre forward in SLAM is one metre east in map_ned.
    let pd = nav
        .on_pose(
            &at(estimate(1.0, 0.0, 0.0), 1_000),
            None,
            Some(0.3),
            1_000,
            0,
            false,
        )
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
        nav.on_pose(
            &at(estimate(jitter, -jitter, 0.0), k * 100),
            Some(0.0),
            Some(heading),
            k * 100,
            0,
            false,
        )
        .unwrap();
    }
    assert!(
        (nav.map_yaw().unwrap() - truth).abs() < 1e-9,
        "the settled heading"
    );
    // Take-off: a climb past ON_SPOT_M fixes the north, and a later compass
    // swing in flight no longer moves it.
    nav.on_pose(
        &at(estimate(0.0, 0.0, 0.0), 2_100),
        Some(ON_SPOT_M + 0.1),
        Some(truth),
        2_100,
        0,
        false,
    )
    .unwrap();
    let pd = nav
        .on_pose(
            &at(estimate(10.0, 0.0, 0.0), 2_200),
            Some(3.0),
            Some(truth + 0.1),
            2_200,
            0,
            false,
        )
        .unwrap();
    assert!((nav.map_yaw().unwrap() - truth).abs() < 1e-9);
    // Ten metres forward in SLAM is ten metres south (heading 180°).
    assert!(
        (pd.pose.t[0] + 10.0).abs() < 1e-9 && pd.pose.t[1].abs() < 1e-9,
        "{:?}",
        pd.pose.t
    );
    // Back over the take-off spot, the north stays fixed.
    nav.on_pose(
        &at(estimate(0.0, 0.0, 0.0), 2_300),
        Some(0.0),
        Some(truth - 0.2),
        2_300,
        0,
        false,
    )
    .unwrap();
    assert!((nav.map_yaw().unwrap() - truth).abs() < 1e-9);
}

#[test]
fn moving_off_the_spot_on_the_ground_fixes_the_north_too() {
    let mut nav = NavMap::new(settings(NavPoseMode::Always, ComputePlacement::Onboard));
    nav.on_pose(&estimate(0.0, 0.0, 0.0), None, Some(0.5), 0, 0, false)
        .unwrap();
    nav.on_pose(
        &at(estimate(0.3, 0.0, 0.0), 100),
        None,
        Some(0.6),
        100,
        0,
        false,
    )
    .unwrap();
    assert!(
        (nav.map_yaw().unwrap() - 0.6).abs() < 1e-12,
        "0.3 m is still on the spot"
    );
    nav.on_pose(
        &at(estimate(0.3, 0.45, 0.0), 200),
        None,
        Some(0.7),
        200,
        0,
        false,
    )
    .unwrap();
    assert!(
        (nav.map_yaw().unwrap() - 0.6).abs() < 1e-12,
        "0.54 m away: fixed"
    );
}

/// With the flight controller's yaw taken from the map (ArduPilot
/// `EK3_SRC1_YAW` 6) its heading on the ground is the map's own yaw plus the
/// gyro drift integrated since the last pose. A north that follows that
/// heading feeds the drift back as the next measurement, so it walks without
/// bound (19° by take-off on the bench). A declared north is latched once
/// and the FC heading never moves it.
#[test]
fn a_declared_north_is_latched_once_and_never_follows_the_fc_heading() {
    let truth = 180f64.to_radians();
    let drift_per_pose = 0.001; // rad: gyro drift between two poses
                                // The loop the FC-heading north closes: the FC yaw is the last published
                                // map yaw plus drift.
    let walk = |s: NavSettings| {
        let mut nav = NavMap::new(s);
        let mut fc_yaw = truth;
        let mut pd = None;
        for k in 0..2_000 {
            let jitter = 0.02 * (k % 3) as f64;
            pd = nav.on_pose(
                &at(estimate(jitter, -jitter, 0.0), k * 100),
                Some(0.0),
                Some(fc_yaw),
                k * 100,
                0,
                false,
            );
            let r = pd.as_ref().unwrap().pose.r;
            fc_yaw = frd_yaw(&r) + drift_per_pose;
        }
        (nav, pd.unwrap())
    };
    let (followed, pd) = walk(settings(NavPoseMode::Always, ComputePlacement::Onboard));
    assert!(
        wrap_pi(followed.map_yaw().unwrap() - truth).abs() > 1.0,
        "the loop walks"
    );
    assert_eq!(pd.north, Some(MapNorth::FcHeading));

    let declared = NavSettings {
        takeoff_heading_rad: Some(truth),
        ..settings(NavPoseMode::Always, ComputePlacement::Onboard)
    };
    let (latched, pd) = walk(declared);
    assert!(
        wrap_pi(latched.map_yaw().unwrap() - truth).abs() < 1e-9,
        "the north holds"
    );
    assert_eq!(pd.north, Some(MapNorth::Declared));
    assert!((wrap_pi(frd_yaw(&pd.pose.r) - truth)).abs() < 1e-9);
}

#[test]
fn a_declared_north_needs_no_fc_heading_and_places_the_first_pose() {
    let mut nav = NavMap::new(NavSettings {
        takeoff_heading_rad: Some(PI / 2.0),
        ..settings(NavPoseMode::Always, ComputePlacement::Onboard)
    });
    // The SLAM body is turned 0.2 rad at the first pose: the declared heading
    // is the body's, so the SLAM x axis points 0.2 rad left of it.
    let pd = nav
        .on_pose(&estimate(0.0, 0.0, -0.2), None, None, 0, 0, false)
        .expect("published without any FC heading");
    assert!(wrap_pi(frd_yaw(&pd.pose.r) - PI / 2.0).abs() < 1e-9);
    assert!(wrap_pi(nav.map_yaw().unwrap() - (PI / 2.0 - 0.2)).abs() < 1e-9);
    // Ten metres along the SLAM x axis.
    let pd = nav
        .on_pose(
            &at(estimate(10.0, 0.0, 0.0), 1_000),
            None,
            Some(0.0),
            1_000,
            0,
            false,
        )
        .unwrap();
    let yaw = PI / 2.0 - 0.2;
    assert!((pd.pose.t[0] - 10.0 * yaw.cos()).abs() < 1e-9);
    assert!((pd.pose.t[1] - 10.0 * yaw.sin()).abs() < 1e-9);
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
        let e = at(slam(k), k as i64 * 100);
        if k < k_loss {
            let wander = 0.3 * (k as f64 / 400.0).sin();
            let t = truth(k);
            nav.on_gnss_ref(&e, [t[0] + noise() + wander, t[1] + noise() - wander]);
        } else {
            nav.on_gnss_lost();
        }
        let pd = nav
            .on_pose(&e, None, Some(h0 + compass_err), k as i64 * 100, 0, false)
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
    let worst = (k_loss..=k137)
        .map(|k| err(k, published[k]))
        .fold(0.0, f64::max);
    assert!(
        worst < 1.0,
        "{worst:.2} m within 137 m of the loss (fit {:.3}°, baseline {:.1} m)",
        wrap_pi(fit.yaw - h0).to_degrees(),
        fit.baseline_m
    );
    // The compass-latched map at the first scan alone, for scale: the fault
    // this guards.
    let compass = err(
        k137,
        align::rot2(h0 + compass_err, {
            let e = slam(k137);
            [e.t[0], -e.t[1]]
        }),
    );
    assert!(
        compass > 6.0,
        "the compass latch alone is {compass:.2} m off"
    );
    let fit_err = wrap_pi(fit.yaw - h0).to_degrees();
    assert!(fit_err.abs() < 0.4, "fit off by {fit_err:.3}°");
}
