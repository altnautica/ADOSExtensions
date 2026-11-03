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
        .on_pose(&estimate(1.0, 2.0, PI / 2.0), Some(10.0), Some(-PI / 2.0), 1_000, false)
        .expect("always publishes with no session");
    assert_eq!(pd.pose.t[0], 1.0);
    assert_eq!(pd.pose.t[1], -2.0, "FLU left is FRD negative y");
    assert_eq!(pd.pose.t[2], 0.0, "z is the FC height above the first pose");
    // A left turn (FLU yaw +90°) reads -90° in FRD.
    assert!((pd.pose.r[3].atan2(pd.pose.r[0]) + PI / 2.0).abs() < 1e-12);
    assert_eq!(pd.source, NavPoseSource::LidarPlanar);
    assert_eq!(pd.frame, NavFrame::MapNed);

    assert!(nav.on_pose(&estimate(1.0, 2.0, 0.0), Some(10.0), None, 1_020, false).is_none());
    let up = nav
        .on_pose(&estimate(1.0, 2.0, 0.0), Some(12.5), None, 1_034, false)
        .unwrap();
    assert_eq!(up.pose.t[2], -2.5, "climbing 2.5 m is FRD z -2.5");
}

#[test]
fn capture_mode_publishes_only_with_a_session_open() {
    let mut nav = NavMap::new(settings(NavPoseMode::Capture, ComputePlacement::Hybrid));
    assert!(nav.on_pose(&estimate(0.0, 0.0, 0.0), None, Some(0.0), 0, false).is_none());
    assert!(nav.on_scan(&ring(), &mount(), Some(&estimate(0.0, 0.0, 0.0)), None, false).scan.is_none());
    assert!(nav.on_pose(&estimate(0.0, 0.0, 0.0), None, None, 100, true).is_some());
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
    assert!(again.occupancy.is_none() && again.pointcloud.is_none(), "nothing new");
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
    nav.on_scan(&ring(), &mount(), Some(&estimate(0.0, 0.0, 0.0)), None, false);
    let st = nav.tick(&TickInputs::default(), false).status;
    assert_eq!(st.stream.state, world_engine_protocol::atlas::StreamPhase::Connecting);
    assert_eq!(nav.map.sweep.voxel_m(), 0.20, "the drone keeps the coarse cloud");
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
    assert!(dir.path().join(SESSION_MAP_DIR).join(TILES_DIR).join("0_0.png").exists());
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
    assert!((pd.pose.r[3].atan2(pd.pose.r[0]) - PI / 2.0).abs() < 1e-12, "yaw reads east");
    // One metre forward in SLAM is one metre east in map_ned.
    let pd = nav
        .on_pose(&estimate(1.0, 0.0, 0.0), None, Some(0.3), 1_000, false)
        .unwrap();
    assert!(pd.pose.t[0].abs() < 1e-12 && (pd.pose.t[1] - 1.0).abs() < 1e-12, "{:?}", pd.pose.t);
    // A later heading reading does not move the latched north.
    let out = nav.on_scan(&ring(), &mount(), Some(&estimate(1.0, 0.0, 0.0)), None, false);
    let polar = out.polar.unwrap();
    assert!(polar.t[0].abs() < 1e-5 && (polar.t[1] - 1.0).abs() < 1e-5, "{:?}", polar.t);
    // The ring's forward return (3.02 m ahead) lands 3.02 m further east.
    let ahead = polar.world_points().next().unwrap();
    assert!(ahead[0].abs() < 1e-3 && (ahead[1] - 4.02).abs() < 1e-3, "{ahead:?}");
}
