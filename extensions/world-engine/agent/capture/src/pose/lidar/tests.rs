use std::path::Path;
use std::time::{Duration, Instant};

use ados_capture_contract::Rig;
use serde_json::json;
use world_engine_protocol::atlas::VioHealth;

use super::supervise::{apply_line, health_of, world_from_body};
use super::wire::{encode_scan, parse_line, ChildPose};
use super::*;
use crate::pose::{se3, PoseProvider, PoseTrack};

/// `T_body_lidar`: LiDAR yawed +90 deg about body z, 0.1 m forward, 0.2 m up.
const T_BODY_LIDAR: [f64; 16] = [
    0.0, -1.0, 0.0, 0.1, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.2, 0.0, 0.0, 0.0, 1.0,
];

fn rig() -> Rig {
    serde_json::from_value(json!({
        "contract": "ados.capture/1",
        "rig_id": "test",
        "calib_version": "1",
        "sensors": [
            { "id": "mid360", "kind": "lidar", "role": "up", "T_body_sensor": T_BODY_LIDAR, "time_offset_ns": 0 },
            { "id": "cam", "kind": "camera", "role": "primary",
              "T_body_sensor": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1], "time_offset_ns": 0 }
        ]
    }))
    .expect("rig")
}

fn translate(x: f64, y: f64, z: f64) -> [f64; 16] {
    se3::to_mat4(&se3::IDENTITY, [x, y, z])
}

fn pose_line(t_ns: u64, t_w_l: [f64; 16], eig: [f64; 3], n: u32) -> String {
    json!({ "t_ns": t_ns, "T_w_l": t_w_l, "eig": eig, "n": n }).to_string()
}

fn scan(t_ns: u64, n: usize) -> LidarScan {
    LidarScan {
        sensor_id: "mid360".into(),
        t_ns,
        points: (0..n).map(|i| [i as f32, 0.5, -1.0]).collect(),
        t_offset_ns: (0..n as u32).map(|i| i * 1_000).collect(),
        intensity: vec![9; n],
    }
}

#[test]
fn scan_frame_layout() {
    let mut out = Vec::new();
    assert!(encode_scan(&scan(77, 2), &mut out));
    assert_eq!(out.len(), 4 + 12 + 16 * 2);
    assert_eq!(&out[0..4], &44u32.to_le_bytes());
    assert_eq!(&out[4..12], &77u64.to_le_bytes());
    assert_eq!(&out[12..16], &2u32.to_le_bytes());
    assert_eq!(&out[16..20], &0.0f32.to_le_bytes());
    assert_eq!(&out[28..32], &1.0f32.to_le_bytes()); // point 1 x
    assert_eq!(&out[44..48], &1_000u32.to_le_bytes()); // offset 1
    let mut bad = scan(1, 3);
    bad.t_offset_ns.pop();
    let mut none = Vec::new();
    assert!(!encode_scan(&bad, &mut none));
    assert!(none.is_empty());
}

#[test]
fn result_lines_parse() {
    let p = parse_line(&pose_line(
        5,
        translate(1.0, 0.0, 0.0),
        [0.5, 2.0, 4.0],
        900,
    ))
    .unwrap();
    assert_eq!(p.t_ns, 5);
    assert_eq!(p.degeneracy_ratio(), Some(0.125));
    let none = ChildPose { n: 0, ..p.clone() };
    assert_eq!(none.degeneracy_ratio(), None);
    assert!(parse_line("{\"t_ns\":1}").is_none());
    assert!(parse_line("kiss_icp_live: hello").is_none());
}

#[test]
fn world_is_the_body_frame_at_the_first_scan() {
    let w = world_from_body(&se3::to_mat4(&se3::IDENTITY, [0.0; 3]), &T_BODY_LIDAR);
    let (r, t) = se3::from_mat4(&w);
    assert!(se3::rotation_angle_between(&r, &se3::IDENTITY) < 1e-12);
    assert!(t.iter().all(|v| v.abs() < 1e-12), "{t:?}");
    // 1 m along the LiDAR's x axis is 1 m along body y.
    let (r, t) = se3::from_mat4(&world_from_body(&translate(1.0, 0.0, 0.0), &T_BODY_LIDAR));
    assert!(se3::rotation_angle_between(&r, &se3::IDENTITY) < 1e-12);
    assert!((t[0]).abs() < 1e-12 && (t[1] - 1.0).abs() < 1e-12 && t[2].abs() < 1e-12);
}

#[test]
fn health_follows_init_and_degeneracy() {
    let good = parse_line(&pose_line(
        1,
        translate(0.0, 0.0, 0.0),
        [1.0, 2.0, 4.0],
        500,
    ))
    .unwrap();
    let corridor = ChildPose {
        eig: [0.01, 3.0, 4.0],
        ..good.clone()
    };
    let empty = ChildPose {
        n: 0,
        ..good.clone()
    };
    assert_eq!(health_of(&good, 1), VioHealth::Degraded);
    assert_eq!(health_of(&good, 4), VioHealth::Good);
    assert_eq!(health_of(&corridor, 10), VioHealth::Degraded);
    assert_eq!(health_of(&empty, 10), VioHealth::Degraded);

    let track = PoseTrack::new();
    let line = pose_line(1_000, translate(2.0, 0.0, 0.0), [0.01, 3.0, 4.0], 800);
    assert!(apply_line(&track, &T_BODY_LIDAR, 10, &line));
    assert!(!apply_line(&track, &T_BODY_LIDAR, 11, "not json"));
    let est = track.latest().unwrap();
    assert_eq!(est.health, VioHealth::Degraded);
    assert_eq!(est.source, PoseSource::Lidar);
    assert!((est.t[1] - 2.0).abs() < 1e-12);
    assert_eq!(track.quality().degeneracy_ratio, Some(0.0025));
}

fn config(dir: &Path, lidar_id: &str) -> LidarPoseConfig {
    LidarPoseConfig {
        launcher: Launcher::Python("python3".into()),
        project_dir: dir.to_path_buf(),
        rig: rig(),
        lidar_id: lidar_id.into(),
        work_dir: dir.join("work"),
        kiss: default_kiss_config(),
    }
}

#[tokio::test]
async fn spawn_rejects_unknown_or_non_lidar_sensors() {
    let dir = tempfile::tempdir().unwrap();
    for id in ["nope", "cam"] {
        match LidarPose::spawn(config(dir.path(), id)) {
            Err(LidarPoseError::NoLidar(got)) => assert_eq!(got, id),
            other => panic!("{id}: {:?}", other.err()),
        }
    }
}

/// A stand-in `kiss_icp_live`: answers each scan with `T_w_l` = 0.1 m per
/// scan along LiDAR x and fixed eigenvalues, then exits after five scans.
const FAKE_MAIN: &str = r#"
import json, socket, struct, sys
a = sys.argv[1:]
sock_path, cfg = a[a.index("--socket") + 1], a[a.index("--config") + 1]
json.load(open(cfg))
with open("runs.txt", "a") as f:
    f.write("run\n")
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect(sock_path)
r, w = s.makefile("rb"), s.makefile("wb")
def exact(n):
    b = r.read(n)
    if len(b) < n:
        sys.exit(0)
    return b
k = 0
while True:
    (ln,) = struct.unpack("<I", exact(4))
    body = exact(ln)
    t, n = struct.unpack_from("<QI", body)
    offs = struct.unpack_from("<%dI" % n, body, 12 + 12 * n)
    k += 1
    T = [1, 0, 0, 0.1 * k, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    w.write((json.dumps({"t_ns": t + max(offs), "T_w_l": T, "eig": [1.0, 2.0, 4.0], "n": n}) + "\n").encode())
    w.flush()
    if k == 5:
        sys.exit(3)
"#;

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn supervises_and_restarts_a_fake_child() {
    if std::process::Command::new("python3")
        .arg("-V")
        .output()
        .is_err()
    {
        eprintln!("python3 not found; skipped");
        return;
    }
    let dir = tempfile::tempdir().unwrap();
    let pkg = dir.path().join("kiss_icp_live");
    std::fs::create_dir_all(&pkg).unwrap();
    std::fs::write(pkg.join("__init__.py"), "").unwrap();
    std::fs::write(pkg.join("__main__.py"), FAKE_MAIN).unwrap();
    let pose = LidarPose::spawn(config(dir.path(), "mid360")).unwrap();
    assert!(dir.path().join("work").join(CONFIG_FILE).is_file());
    assert_eq!(pose.health(), VioHealth::Lost);

    let runs = || {
        std::fs::read_to_string(dir.path().join("runs.txt"))
            .map(|s| s.lines().count())
            .unwrap_or(0)
    };
    let deadline = Instant::now() + Duration::from_secs(20);
    let mut t = 1_000_000_000u64;
    let mut saw_good = false;
    let mut saw_degraded = false;
    while Instant::now() < deadline && !(saw_good && runs() >= 2) {
        pose.feed_lidar(&scan(t, 4));
        // Another sensor's scans are ignored.
        pose.feed_lidar(&LidarScan {
            sensor_id: "other".into(),
            ..scan(t, 4)
        });
        t += 100_000_000;
        tokio::time::sleep(Duration::from_millis(30)).await;
        if let Some(est) = pose.latest() {
            match est.health {
                VioHealth::Degraded => saw_degraded = true,
                VioHealth::Good => {
                    saw_good = true;
                    // Pose k: 0.1 k m along LiDAR x = body y; t_ns at the
                    // scan's last point (offset 3 us).
                    let k = (est.t[1] / 0.1).round();
                    assert!(k >= 4.0, "{est:?}");
                    assert!(est.t[0].abs() < 1e-9 && est.t[2].abs() < 1e-9, "{est:?}");
                    assert_eq!(est.t_ns % 100_000_000, 3_000);
                    assert_eq!(pose.quality().degeneracy_ratio, Some(0.25));
                }
                VioHealth::Lost => {}
            }
        }
    }
    assert!(saw_degraded, "first poses of a run are degraded");
    assert!(saw_good, "later poses are good");
    assert!(runs() >= 2, "child restarted after it exited");
    assert!(pose.stats().restarts >= 1);
}
