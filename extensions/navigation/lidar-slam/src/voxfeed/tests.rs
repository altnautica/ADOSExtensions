use std::collections::VecDeque;
use std::sync::Arc;
use std::time::{Duration, Instant};

use ados_mapping_pose::lidar_scan::LidarScan;
use ados_mapping_pose::se3;
use ados_mapping_protocol::capture::{PolarScan, VioHealth};
use ados_protocol::voxmap::{ReadStatus, VoxSnapshot, VoxmapReader, VoxmapWriter, VOX_OCCUPIED};
use ados_voxmap::{Pose, VoxMapConfig, VoxTier, VoxelState};

use super::config::{Sensors, VoxmapSettings};
use super::convert::{body_from_sensor, mount_pose};
use super::feeder::{Feeder, Input};
use super::queue::Queue;
use super::{FeedOptions, TierName, VoxFeed};
use crate::depth_source::{DepthFrame, DepthSettings, Extrinsic};

const SEC: u64 = 1_000_000_000;
/// The sensor and the body, 2 m up at the map origin.
const AT: [f32; 3] = [0.0, 0.0, -2.0];

fn scan(points: Vec<[f32; 3]>) -> LidarScan {
    let n = points.len();
    LidarScan {
        sensor_id: "lidar0".into(),
        t_ns: 0,
        points,
        t_offset_ns: vec![0; n],
        intensity: vec![0; n],
    }
}

fn identity_mat() -> [f64; 16] {
    se3::to_mat4(&se3::IDENTITY, [0.0; 3])
}

fn identity_mount() -> Pose {
    mount_pose(&identity_mat())
}

fn qmul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    let ([aw, ax, ay, az], [bw, bx, by, bz]) = (a, b);
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

/// A 3D LiDAR's view (forward-left-up) of a wall 5.1 m ahead (mid-voxel), 8 m wide, from
/// the ground (2 m below the sensor) to 3 m over it, on a rosette-like
/// pattern: -7..30 degrees of elevation, ±35 degrees of azimuth.
fn wall_cloud() -> Vec<[f32; 3]> {
    let mut out = Vec::new();
    for az in -35..=35 {
        for el in -14..=60 {
            let (az, el) = ((az as f32).to_radians(), (el as f32 * 0.5).to_radians());
            let d = [el.cos() * az.cos(), el.cos() * az.sin(), el.sin()];
            let t = 5.1 / d[0];
            let p = d.map(|v| v * t);
            if p[1].abs() <= 4.0 && (-2.0..=3.0).contains(&p[2]) {
                out.push(p);
            }
        }
    }
    out
}

fn lidar(points: Vec<[f32; 3]>, sensor: Pose, planar: bool) -> Input {
    Input::Lidar {
        scan: scan(points),
        sensor,
        mount: identity_mount(),
        blocked: Vec::new(),
        planar,
    }
}

fn feeder() -> Feeder {
    Feeder::new(VoxMapConfig::default(), 10.0).unwrap()
}

/// In the voxel just past `n` metres north of the sensor, on its level.
fn ahead(n: f32) -> [f32; 3] {
    [n + 0.1, 0.1, AT[2] + 0.1]
}

#[test]
fn a_3d_lidar_wall_is_occupied_at_the_wall_and_free_before_it() {
    let mut f = feeder();
    assert!(f
        .ingest(SEC, lidar(wall_cloud(), Pose::at(AT), false))
        .is_none());
    let map = f.map();
    assert_eq!(map.state_at(ahead(5.0)), VoxelState::Occupied);
    for n in [1.0, 2.5, 4.0] {
        assert_eq!(map.state_at(ahead(n)), VoxelState::Free, "{n} m ahead");
    }
    assert_eq!(map.state_at(ahead(7.0)), VoxelState::Unknown);
    assert_eq!(f.tier(SEC), VoxTier::Lidar3d);
}

#[test]
fn returns_on_the_airframe_and_in_blocked_sectors_are_left_out() {
    let mut f = feeder();
    let input = Input::Lidar {
        // A leg 0.3 m below, a prop 0.2 m ahead, a mast 2 m behind in the
        // blocked sector.
        scan: scan(vec![[0.0, 0.0, -0.3], [0.2, 0.0, 0.0], [-2.0, 0.0, 0.0]]),
        sensor: Pose::at(AT),
        mount: identity_mount(),
        blocked: vec![[170.0, 190.0]],
        planar: false,
    };
    f.ingest(SEC, input);
    for p in [[0.0, 0.0, -1.7], [0.2, 0.0, -2.0], [-2.0, 0.0, -2.0]] {
        assert_ne!(f.map().state_at(p), VoxelState::Occupied, "{p:?}");
    }
}

#[test]
fn a_depth_frame_of_a_wall_is_occupied_at_the_wall_and_free_before_it() {
    let (w, h) = (160u32, 120u32);
    let frame = DepthFrame {
        stamp_ns: 0,
        source_ns: 0,
        seq: 1,
        w,
        h,
        fx: 80.0,
        fy: 80.0,
        cx: 79.5,
        cy: 59.5,
        depth_mm: vec![4100; (w * h) as usize],
        extrinsic: Extrinsic::identity(),
    };
    let mut points = Vec::new();
    frame.to_points(super::depth::stride_for(w, h), 10.0, &mut points);
    let mut f = feeder();
    let back = f.ingest(
        SEC,
        Input::Depth {
            body: Pose::at(AT),
            origin_body: frame.extrinsic.t,
            points,
        },
    );
    assert!(back.is_some(), "the point buffer comes back for reuse");
    let map = f.map();
    assert_eq!(map.state_at(ahead(4.0)), VoxelState::Occupied);
    for n in [1.0, 2.0, 3.0] {
        assert_eq!(map.state_at(ahead(n)), VoxelState::Free, "{n} m ahead");
    }
    assert_eq!(f.tier(SEC), VoxTier::Depth);
}

/// Pitched 20 degrees nose-up, a planar scan's forward beam climbs 2 m over
/// 6 m into a crown: out of the band, it marks nothing and clears nothing.
/// Its level beam to the right still marks the wall it meets.
#[test]
fn planar_scans_trust_only_returns_in_the_flight_band() {
    let half = 10f32.to_radians();
    // Mid-voxel, so the level beam stays inside one row of voxels.
    let at = [0.1, 0.1, -1.9];
    let sensor = Pose {
        t: at,
        q: [half.cos(), 0.0, half.sin(), 0.0],
    };
    let mut f = feeder();
    f.ingest(
        SEC,
        lidar(vec![[6.0, 0.0, 0.0], [0.0, -3.0, 0.0]], sensor, true),
    );
    let map = f.map();
    assert_eq!(map.state_at([0.1, 3.1, -1.9]), VoxelState::Occupied);
    assert_eq!(map.state_at([0.1, 1.5, -1.9]), VoxelState::Free);
    let c = 20f32.to_radians();
    for r in [3.0f32, 6.0] {
        let p = [at[0] + r * c.cos(), at[1], at[2] - r * c.sin()];
        assert_eq!(map.state_at(p), VoxelState::Unknown, "{r} m along the beam");
    }
    assert_eq!(f.tier(SEC), VoxTier::Planar);
}

#[test]
fn the_tier_is_the_best_sensor_heard_within_a_second() {
    let mut f = feeder();
    assert_eq!(f.tier(0), VoxTier::Unknown);
    f.ingest(0, lidar(wall_cloud(), Pose::at(AT), false));
    f.ingest(
        SEC / 2,
        Input::Depth {
            body: Pose::at(AT),
            origin_body: [0.0; 3],
            points: vec![[3.0, 0.0, 0.0]],
        },
    );
    assert_eq!(f.tier(SEC / 2), VoxTier::Lidar3d);
    assert_eq!(f.tier(SEC + SEC / 4), VoxTier::Depth);
    assert_eq!(f.tier(3 * SEC), VoxTier::Unknown);
}

#[test]
fn the_body_pose_comes_from_the_sensor_pose_and_the_mount() {
    // Sensor 0.1 m forward of and 0.2 m over the body origin.
    let mount = mount_pose(&se3::to_mat4(&se3::IDENTITY, [0.1, 0.0, 0.2]));
    let body = body_from_sensor(&Pose::at([0.0, 0.0, -2.2]), &mount);
    for (a, want) in body.t.iter().zip([-0.1, 0.0, -2.0]) {
        assert!((a - want).abs() < 1e-6, "{:?}", body.t);
    }
    assert!((body.q[0] - 1.0).abs() < 1e-6, "{:?}", body.q);

    // A sensor turned 90 degrees left on its mount, on a body yawed 30
    // degrees east of north: placing the sensor from the body and back
    // gives the body.
    let left = std::f64::consts::FRAC_PI_2;
    let yaw_left = [
        left.cos(),
        -left.sin(),
        0.0,
        left.sin(),
        left.cos(),
        0.0,
        0.0,
        0.0,
        1.0,
    ];
    let mount = mount_pose(&se3::to_mat4(&yaw_left, [0.1, 0.05, 0.2]));
    let h = 15f32.to_radians();
    let truth = Pose {
        t: [1.0, 2.0, -3.0],
        q: [h.cos(), 0.0, 0.0, h.sin()],
    };
    let off = truth.rotate(mount.t);
    let sensor = Pose {
        t: [0, 1, 2].map(|a| truth.t[a] + off[a]),
        q: qmul(truth.q, mount.q),
    };
    // FRD yaw of the sensor: 30 - 90 degrees.
    let r = se3::matrix_from_quat(sensor.q.map(f64::from));
    assert!((r[3].atan2(r[0]).to_degrees() + 60.0).abs() < 1e-3);
    let body = body_from_sensor(&sensor, &mount);
    for a in 0..3 {
        assert!((body.t[a] - truth.t[a]).abs() < 1e-5, "{body:?}");
    }
    let dot: f32 = body.q.iter().zip(truth.q).map(|(a, b)| a * b).sum();
    assert!(dot.abs() > 1.0 - 1e-6, "{body:?}");
}

#[test]
fn the_published_map_reads_back_through_the_seam() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("voxmap");
    let mut f = feeder();
    let mut writer = VoxmapWriter::create(&path, f.map().dims(), None).unwrap();
    assert!(
        !f.frame(0, Some(&mut writer)).unwrap(),
        "no pose placed the map yet"
    );
    f.ingest(SEC, lidar(wall_cloud(), Pose::at(AT), false));
    assert!(f.frame(SEC, Some(&mut writer)).unwrap());

    let mut reader = VoxmapReader::open(&path).unwrap();
    let mut snap = VoxSnapshot::default();
    assert_eq!(reader.read_into(&mut snap).unwrap(), ReadStatus::Fresh);
    assert_eq!(snap.meta.tier, VoxTier::Lidar3d);
    assert_eq!(snap.meta.stamp_ns, SEC);
    assert_eq!(&snap.meta.map_from_body[..3], &AT);
    let occupied = snap
        .occupancy
        .iter()
        .filter(|&&c| c == VOX_OCCUPIED)
        .count();
    assert!(occupied > 100, "{occupied} occupied voxels");
    let wall = snap.index_of_point(ahead(5.0)).unwrap();
    assert_eq!(snap.occupancy[wall], VOX_OCCUPIED);
    assert_eq!(snap.esdf_dm[wall], 0);
    let front = snap.index_of_point(ahead(2.5)).unwrap();
    assert_eq!(snap.occupancy[front], ados_protocol::voxmap::VOX_FREE);
    assert!(snap.esdf_dm[front] >= 20, "{} dm", snap.esdf_dm[front]);
}

fn marked(i: usize) -> Input {
    Input::Depth {
        body: Pose::at(AT),
        origin_body: [0.0; 3],
        points: vec![[i as f32, 0.0, 0.0]],
    }
}

#[test]
fn a_full_queue_drops_its_oldest_input_and_never_waits() {
    let q = Queue::new(2);
    let t0 = Instant::now();
    for i in 0..100 {
        q.push(marked(i));
    }
    assert!(
        t0.elapsed() < Duration::from_millis(50),
        "{:?}",
        t0.elapsed()
    );
    assert_eq!(q.drops(), 98);
    let mut out = VecDeque::new();
    assert!(q.take(&mut out, Duration::ZERO));
    let kept: Vec<f32> = out
        .iter()
        .map(|i| match i {
            Input::Depth { points, .. } => points[0][0],
            Input::Lidar { .. } => -1.0,
        })
        .collect();
    assert_eq!(kept, [98.0, 99.0]);
    q.stop();
    assert!(
        !q.take(&mut out, Duration::from_secs(5)),
        "stop wakes the taker"
    );
}

fn polar(t: [f32; 3]) -> PolarScan {
    PolarScan {
        t_ns: 0,
        angle_start_rad: 0.0,
        angle_inc_rad: 0.0,
        t,
        q: [1.0, 0.0, 0.0, 0.0],
        health: VioHealth::Good,
        range_mm: Vec::new(),
        quality: Vec::new(),
    }
}

/// The running feed publishes the map at 10 Hz of its clock, and a burst of
/// scans while it works costs the sender nothing but drops.
#[test]
fn the_feed_thread_publishes_and_never_blocks_the_lidar_path() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("voxmap");
    let start = Instant::now();
    let feed = VoxFeed::start(FeedOptions {
        settings: VoxmapSettings::default(),
        planar: false,
        depth: DepthSettings::default(),
        path: path.clone(),
        group: None,
        now: Arc::new(move || start.elapsed().as_nanos() as u64),
    })
    .unwrap();
    let tx = feed.tx();
    let cloud = wall_cloud();
    let t0 = Instant::now();
    for _ in 0..50 {
        tx.lidar(
            scan(cloud.clone()),
            &polar(AT),
            identity_mat(),
            Vec::new(),
            false,
        );
    }
    let pushed = t0.elapsed();
    assert!(
        pushed < Duration::from_millis(200),
        "50 hand-offs took {pushed:?}"
    );
    // A planar scan is refused: this feed takes none.
    tx.lidar(
        scan(cloud.clone()),
        &polar(AT),
        identity_mat(),
        Vec::new(),
        true,
    );

    let mut reader = None;
    let mut snap = VoxSnapshot::default();
    let deadline = Instant::now() + Duration::from_secs(20);
    while Instant::now() < deadline {
        let r = match reader.as_mut() {
            Some(r) => r,
            None => match VoxmapReader::open(&path) {
                Ok(r) => reader.insert(r),
                Err(_) => {
                    std::thread::sleep(Duration::from_millis(20));
                    continue;
                }
            },
        };
        if r.read_into(&mut snap).unwrap() == ReadStatus::Fresh
            && snap.meta.tier == VoxTier::Lidar3d
        {
            break;
        }
        std::thread::sleep(Duration::from_millis(20));
    }
    assert!(snap.valid, "no map published");
    assert_eq!(snap.meta.tier, VoxTier::Lidar3d);
    let wall = snap.index_of_point(ahead(5.0)).unwrap();
    assert_eq!(snap.occupancy[wall], VOX_OCCUPIED);
    // The health block follows within a frame.
    std::thread::sleep(Duration::from_millis(250));
    let h = tx.health();
    assert!(h.enabled && h.published >= 1, "{h:?}");
    assert_eq!(h.tier, TierName::Lidar3d);
    assert!(h.update_ms_p95 > 0.0, "{h:?}");
    assert!(h.error.is_none(), "{h:?}");
    assert!(h.drops <= 46, "{h:?}");
    feed.stop();
}

#[test]
fn the_map_runs_by_default_only_on_a_3d_or_depth_sensor() {
    let d = VoxmapSettings::default();
    let s = |lidar3d, planar, depth| Sensors {
        lidar3d,
        planar,
        depth,
    };
    assert!(d.active(s(true, false, false)));
    assert!(d.active(s(false, false, true)));
    assert!(!d.active(s(false, true, false)));
    assert!(!d.active(s(false, false, false)));
    let planar = VoxmapSettings {
        planar: true,
        ..d.clone()
    };
    assert!(planar.active(s(false, true, false)));
    let off = VoxmapSettings {
        enabled: Some(false),
        ..d.clone()
    };
    assert!(!off.active(s(true, false, true)));
    let on = VoxmapSettings {
        enabled: Some(true),
        ..d
    };
    assert!(on.active(s(false, false, false)));
}

#[test]
fn the_config_key_resolves_and_refuses_bad_values() {
    use std::collections::BTreeMap;
    let key = crate::settings::KEY_VOXMAP;
    let empty = BTreeMap::new();
    assert_eq!(
        super::config::resolve(&empty, key).unwrap(),
        VoxmapSettings::default()
    );
    let mut v = BTreeMap::new();
    v.insert(
        key.to_string(),
        serde_json::json!({"planar": true, "window": "fine", "esdf_every": 2}),
    );
    let s = super::config::resolve(&v, key).unwrap();
    assert!(s.planar);
    assert_eq!(s.map_config().dims, ados_voxmap::FINE_DIMS);
    assert_eq!(s.map_config().esdf_every, 2);
    v.insert(key.to_string(), serde_json::json!({"esdf_every": 0}));
    assert!(super::config::resolve(&v, key).is_err());
    v.insert(
        key.to_string(),
        serde_json::json!({"depth_max_range_m": 50.0}),
    );
    assert!(super::config::resolve(&v, key).is_err());
    v.insert(key.to_string(), serde_json::json!({"bogus": 1}));
    assert!(super::config::resolve(&v, key).is_err());
}
