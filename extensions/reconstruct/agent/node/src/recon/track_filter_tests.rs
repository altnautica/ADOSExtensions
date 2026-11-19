use super::*;
use crate::products::cloud::{write_laz, PointCloud};

/// An `images.txt` entry: the image at world-from-camera `t_wc` with its
/// 2D points `(x, y, point3d id or -1)`.
fn image(id: u32, t_wc: &Mat4, name: &str, pts: &[(f64, f64, i64)]) -> String {
    let t_cw = geom::inverse_rigid(t_wc);
    let (q, t) = (geom::quat_of(&t_cw), geom::translation(&t_cw));
    let p: Vec<String> = pts.iter().map(|(x, y, i)| format!("{x} {y} {i}")).collect();
    format!(
        "{id} {} {} {} {} {} {} {} 1 {name}\n{}\n",
        q[0],
        q[1],
        q[2],
        q[3],
        t[0],
        t[1],
        t[2],
        p.join(" ")
    )
}

fn at(x: f64, y: f64, z: f64) -> Mat4 {
    geom::from_quat_trans([1.0, 0.0, 0.0, 0.0], [x, y, z])
}

/// A `points3D.txt` line seen by images `track` (2D index = point id - 1).
fn point(id: u64, xyz: [f64; 3], track: &[u32]) -> String {
    let t: Vec<String> = track.iter().map(|i| format!("{i} {}", id - 1)).collect();
    format!(
        "{id} {} {} {} 100 100 100 0.5 {}\n",
        xyz[0],
        xyz[1],
        xyz[2],
        t.join(" ")
    )
}

fn write_model(dir: &Path, images: &str, points: &str) {
    fs::write(dir.join("cameras.txt"), "1 PINHOLE 64 48 40 40 32 24\n").unwrap();
    fs::write(dir.join("images.txt"), format!("# header\n{images}")).unwrap();
    fs::write(dir.join("points3D.txt"), format!("# header\n{points}")).unwrap();
}

fn ids(dir: &Path) -> Vec<u64> {
    parse_points(&fs::read_to_string(dir.join("points3D.txt")).unwrap())
        .iter()
        .map(|p| p.id)
        .collect()
}

#[test]
fn short_tracks_are_dropped_and_their_observations_cleared() {
    let dir = tempfile::tempdir().unwrap();
    let obs = [(1.0, 2.0, 1), (3.0, 4.0, 2), (5.0, 6.0, 3)];
    let images: String = (1..=3)
        .map(|i| image(i, &at(i as f64, 0.0, 0.0), &format!("cam0/{i}.png"), &obs))
        .collect();
    let points = [
        point(1, [0.0, 0.0, 5.0], &[1, 2, 3]),
        point(2, [0.0, 0.0, 5.0], &[1, 2]),
        point(3, [0.0, 0.0, 5.0], &[3]),
    ]
    .concat();
    write_model(dir.path(), &images, &points);
    let dropped = run(dir.path(), dir.path(), true).unwrap();
    assert_eq!(
        dropped,
        Dropped {
            short_track: 2,
            depth: 0
        }
    );
    assert_eq!(ids(dir.path()), vec![1]);
    let back = parse_images(&fs::read_to_string(dir.path().join("images.txt")).unwrap());
    assert_eq!(back.len(), 3);
    let body = fs::read_to_string(dir.path().join("images.txt")).unwrap();
    assert!(body.contains("1 2 1 3 4 -1 5 6 -1"), "{body}");
    assert!(body.starts_with("# header\n"));
    // Nothing more to drop: the files stay as they are.
    assert_eq!(
        run(dir.path(), dir.path(), true).unwrap(),
        Dropped::default()
    );
}

#[test]
fn points_off_the_lidar_surface_are_dropped() {
    let dir = tempfile::tempdir().unwrap();
    // A wall at x = 5 (y in [-2, 2], z in [0, 2]), cameras near the origin.
    let mut wall = Vec::new();
    for iy in -40..=40 {
        for iz in 0..=40 {
            wall.push([5.0, iy as f64 * 0.05, iz as f64 * 0.05]);
        }
    }
    write_laz(
        &dir.path().join("lidar_points.laz"),
        &PointCloud {
            points: wall,
            colors: None,
        },
    )
    .unwrap();
    let images: String = (1..=3)
        .map(|i| image(i, &at(0.0, i as f64 * 0.1, 1.0), &format!("c/{i}.png"), &[]))
        .collect();
    let points = [
        // On the wall: kept.
        point(1, [5.0, 0.5, 1.0], &[1, 2, 3]),
        // A floater 2 m in front of the wall: the ray reaches it first.
        point(2, [3.0, 0.3, 1.0], &[1, 2, 3]),
        // Behind the wall: the ray meets the wall 2 m short of it.
        point(3, [7.0, 0.7, 1.0], &[1, 2, 3]),
        // Where the LiDAR saw nothing (beyond the wall's edge): kept.
        point(4, [5.0, 6.0, 1.0], &[1, 2, 3]),
    ]
    .concat();
    write_model(dir.path(), &images, &points);
    // A model outside the world frame cannot be judged against the cloud.
    assert_eq!(
        run(dir.path(), dir.path(), false).unwrap(),
        Dropped::default()
    );
    let dropped = run(dir.path(), dir.path(), true).unwrap();
    assert_eq!(
        dropped,
        Dropped {
            short_track: 0,
            depth: 2
        }
    );
    assert_eq!(ids(dir.path()), vec![1, 4]);
}

/// A `.npy` file of a constant float32 `h` x `w` map.
fn write_npy(path: &Path, h: usize, w: usize, v: f32) {
    let mut header = format!("{{'descr': '<f4', 'fortran_order': False, 'shape': ({h}, {w}), }}");
    while (10 + header.len() + 1) % 64 != 0 {
        header.push(' ');
    }
    header.push('\n');
    let mut b = b"\x93NUMPY\x01\x00".to_vec();
    b.extend_from_slice(&(header.len() as u16).to_le_bytes());
    b.extend_from_slice(header.as_bytes());
    for _ in 0..h * w {
        b.extend_from_slice(&v.to_le_bytes());
    }
    fs::create_dir_all(path.parent().unwrap()).unwrap();
    fs::write(path, b).unwrap();
}

#[test]
fn points_off_the_scaled_depth_prior_are_dropped() {
    let dir = tempfile::tempdir().unwrap();
    // The depth prior says 3 m everywhere (at half the image resolution); the
    // model is at twice metric scale, so its points at z = 6 agree and the
    // one at z = 8 (4 m) is 1 m off.
    write_npy(&dir.path().join("depths/cam0/1.png.npy"), 24, 32, 3.0);
    let n = 12u64;
    let mut obs = Vec::new();
    let mut points = String::new();
    for k in 1..=n {
        let (x, y) = (4.0 * k as f64, 3.0 * k as f64);
        let z = if k == n { 8.0 } else { 6.0 };
        obs.push((x, y, k as i64));
        // Back-projected through fx = fy = 40, cx = 32, cy = 24.
        let xyz = [(x - 32.0) / 40.0 * z, (y - 24.0) / 40.0 * z, z];
        points.push_str(&point(k, xyz, &[1, 2, 3]));
    }
    // Images 2 and 3 are far behind, so image 1 is every point's nearest view.
    let images = [
        image(1, &at(0.0, 0.0, 0.0), "cam0/1.png", &obs),
        image(2, &at(0.0, 0.0, -20.0), "cam0/2.png", &obs),
        image(3, &at(0.5, 0.0, -20.0), "cam0/3.png", &obs),
    ]
    .concat();
    write_model(dir.path(), &images, &points);
    let dropped = run(dir.path(), dir.path(), false).unwrap();
    assert_eq!(
        dropped,
        Dropped {
            short_track: 0,
            depth: 1
        }
    );
    assert_eq!(ids(dir.path()), (1..n).collect::<Vec<_>>());
}

#[test]
fn unreadable_depth_maps_are_ignored() {
    let dir = tempfile::tempdir().unwrap();
    fs::create_dir_all(dir.path().join("depths/cam0")).unwrap();
    fs::write(dir.path().join("depths/cam0/1.png.npy"), b"\x93NUMPY").unwrap();
    assert!(read_npy_f32(&dir.path().join("depths/cam0/1.png.npy")).is_none());
    let images: String = (1..=3)
        .map(|i| {
            image(
                i,
                &at(0.0, 0.0, -(i as f64)),
                &format!("cam0/{i}.png"),
                &[(1.0, 1.0, 1)],
            )
        })
        .collect();
    write_model(dir.path(), &images, &point(1, [0.0, 0.0, 5.0], &[1, 2, 3]));
    assert_eq!(
        run(dir.path(), dir.path(), false).unwrap(),
        Dropped::default()
    );
}
