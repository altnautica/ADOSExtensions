use std::fs;

use super::super::cloud::{read_laz, read_ply_mesh, read_ply_points, voxel_downsample, write_laz};
use super::super::{PointCloud, ProductError};

#[test]
fn ascii_ply_with_rgb() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("a.ply");
    fs::write(
        &path,
        "ply\nformat ascii 1.0\ncomment test\nelement vertex 2\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\nelement face 1\nproperty list uchar int vertex_indices\nend_header\n1.5 -2 3 255 0 10\n0 0.25 -1 1 2 3\n3 0 1 1\n",
    )
    .unwrap();
    let c = read_ply_points(&path).unwrap();
    assert_eq!(c.points, vec![[1.5, -2.0, 3.0], [0.0, 0.25, -1.0]]);
    assert_eq!(c.colors.unwrap(), vec![[65535, 0, 2570], [257, 514, 771]]);
}

fn le_ply(header: &str, body: &[u8]) -> Vec<u8> {
    let mut b = header.as_bytes().to_vec();
    b.extend_from_slice(body);
    b
}

#[test]
fn binary_ply_points_and_mesh() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("m.ply");
    let header = "ply\r\nformat binary_little_endian 1.0\r\nelement vertex 4\r\nproperty double x\r\nproperty double y\r\nproperty double z\r\nproperty float nx\r\nproperty uchar red\r\nproperty uchar green\r\nproperty uchar blue\r\nelement face 1\r\nproperty uchar flags\r\nproperty list uchar uint vertex_indices\r\nend_header\r\n";
    let mut body = Vec::new();
    let verts = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.125],
    ];
    for (i, v) in verts.iter().enumerate() {
        for x in v {
            body.extend_from_slice(&f64::to_le_bytes(*x));
        }
        body.extend_from_slice(&0.5f32.to_le_bytes());
        body.extend_from_slice(&[i as u8, 10, 20]);
    }
    body.push(7);
    body.push(4);
    for i in [0u32, 1, 2, 3] {
        body.extend_from_slice(&i.to_le_bytes());
    }
    fs::write(&path, le_ply(header, &body)).unwrap();

    let c = read_ply_points(&path).unwrap();
    assert_eq!(c.points, verts.to_vec());
    assert_eq!(c.colors.as_ref().unwrap()[3], [3 * 257, 10 * 257, 20 * 257]);
    let m = read_ply_mesh(&path).unwrap();
    assert_eq!(m.vertices, verts.to_vec());
    assert_eq!(m.triangles, vec![[0, 1, 2], [0, 2, 3]]);

    // Truncated body is a format error, not a panic.
    fs::write(&path, le_ply(header, &body[..body.len() - 3])).unwrap();
    assert!(matches!(read_ply_mesh(&path), Err(ProductError::Format(_))));
}

#[test]
fn splat_ply_colour_from_sh0() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("splat.ply");
    let mut header = String::from("ply\nformat binary_little_endian 1.0\nelement vertex 2\n");
    let mut props: Vec<String> = [
        "x", "y", "z", "nx", "ny", "nz", "f_dc_0", "f_dc_1", "f_dc_2",
    ]
    .iter()
    .map(|s| s.to_string())
    .collect();
    props.extend((0..45).map(|i| format!("f_rest_{i}")));
    props.extend(
        [
            "opacity", "scale_0", "scale_1", "scale_2", "rot_0", "rot_1", "rot_2", "rot_3",
        ]
        .map(String::from),
    );
    for p in &props {
        header.push_str(&format!("property float {p}\n"));
    }
    header.push_str("end_header\n");
    let c0 = 0.282_094_8_f32;
    let mut body = Vec::new();
    for (pos, dc) in [
        ([1.0f32, 2.0, 3.0], [0.0f32, 0.5 / c0, -0.5 / c0]),
        ([-1.0, 0.0, 0.5], [10.0, -10.0, 0.25 / c0]),
    ] {
        for (k, _) in props.iter().enumerate() {
            let v = match k {
                0..=2 => pos[k],
                6..=8 => dc[k - 6],
                _ => 0.1,
            };
            body.extend_from_slice(&v.to_le_bytes());
        }
    }
    fs::write(&path, le_ply(&header, &body)).unwrap();
    let c = read_ply_points(&path).unwrap();
    assert_eq!(c.points[1], [-1.0, 0.0, 0.5]);
    let col = c.colors.unwrap();
    assert_eq!(col[0][0], 32768);
    assert!(col[0][1] >= 65534 && col[0][2] <= 1, "{:?}", col[0]);
    assert_eq!(&col[1][..2], &[65535, 0]);
    assert!((col[1][2] as i32 - 49151).abs() <= 2, "{:?}", col[1]);
}

#[test]
fn laz_round_trip() {
    let dir = tempfile::tempdir().unwrap();
    let cloud = PointCloud {
        points: (0..500)
            .map(|i| {
                let t = i as f64;
                [
                    1000.0 + t * 0.0137,
                    -250.5 + (t * 0.31).sin(),
                    12.0 + t * 0.0021,
                ]
            })
            .collect(),
        colors: Some(
            (0..500)
                .map(|i| [i as u16 * 100, 65535 - i as u16, 7])
                .collect(),
        ),
    };
    let path = dir.path().join("c.laz");
    write_laz(&path, &cloud).unwrap();
    let back = read_laz(&path).unwrap();
    assert_eq!(back.points.len(), cloud.points.len());
    for (a, b) in cloud.points.iter().zip(&back.points) {
        for k in 0..3 {
            assert!((a[k] - b[k]).abs() <= 0.001, "{a:?} vs {b:?}");
        }
    }
    assert_eq!(back.colors, cloud.colors);

    let plain = PointCloud {
        points: cloud.points.clone(),
        colors: None,
    };
    let path = dir.path().join("p.laz");
    write_laz(&path, &plain).unwrap();
    let back = read_laz(&path).unwrap();
    assert!(back.colors.is_none());
    assert_eq!(back.points.len(), 500);
}

#[test]
fn voxel_downsample_averages() {
    let cloud = PointCloud {
        points: vec![[0.01, 0.01, 0.01], [0.03, 0.03, 0.03], [0.51, 0.0, 0.0]],
        colors: Some(vec![[0, 0, 0], [100, 200, 300], [5, 5, 5]]),
    };
    let d = voxel_downsample(&cloud, 0.1);
    assert_eq!(d.points.len(), 2);
    assert!((d.points[0][0] - 0.02).abs() < 1e-12);
    assert_eq!(d.colors.unwrap()[0], [50, 100, 150]);
}
