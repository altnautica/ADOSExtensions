use std::path::Path;

use super::*;
use crate::recon::chunks_merge::{merge_models, merge_splats, SplatCount};

/// Named camera centres along x at `xs`, y = 0.
fn line(xs: impl IntoIterator<Item = f64>) -> Vec<(String, [f64; 2])> {
    xs.into_iter()
        .enumerate()
        .map(|(i, x)| (format!("cam0/{i}.png"), [x, 0.0]))
        .collect()
}

/// A serpentine of `n` frames over a 30 x 20 m floor.
fn serpentine(n: usize) -> Vec<(String, [f64; 2])> {
    (0..n)
        .map(|i| {
            let row = i / 20;
            let k = (i % 20) as f64 * 1.5;
            let x = if row % 2 == 0 { k } else { 28.5 - k };
            (format!("cam0/{i}.png"), [x, row as f64 * 4.0])
        })
        .collect()
}

#[test]
fn a_session_that_fits_one_chunk_is_not_chunked() {
    assert!(plan(&line((0..20).map(f64::from)), 20, 5.0).is_none());
    // The cap never drops under the floor.
    assert!(plan(&line((0..10).map(f64::from)), 2, 5.0).is_none());
}

#[test]
fn the_grid_keeps_every_core_under_the_cap() {
    let frames = serpentine(120);
    let p = plan(&frames, 20, 5.0).expect("chunked");
    assert!(p.chunks.len() >= 6, "{} chunks", p.chunks.len());
    assert!(
        p.chunks.iter().all(|c| c.core_frames <= 20),
        "{:?}",
        p.detail()
    );
    assert_eq!(p.chunks.iter().map(|c| c.core_frames).sum::<usize>(), 120);
    // Ids are positions; every cell has an owner; points off the grid too.
    assert!(p.chunks.iter().enumerate().all(|(i, c)| c.id == i));
    assert!(p.owner.iter().all(|&o| o < p.chunks.len()));
    assert!(p.owner_of(-100.0, 500.0) < p.chunks.len());
    for (name, c) in &frames {
        let owner = p.owner_of(c[0], c[1]);
        assert!(
            p.chunks[owner].frames.contains(name),
            "{name} not in its own chunk"
        );
    }
}

#[test]
fn a_chunk_trains_on_every_frame_within_the_overlap() {
    let frames = line((0..100).map(f64::from));
    let p = plan(&frames, 20, 5.0).expect("chunked");
    assert!(p.chunks.len() >= 5);
    let mut multi = 0;
    for (name, c) in &frames {
        let mut n = 0;
        for chunk in &p.chunks {
            // Along a line each core is one run of cells, so its rectangle.
            let near = rect_dist(&chunk.core, *c) <= 5.0;
            assert_eq!(
                chunk.frames.contains(name),
                near,
                "{name} vs chunk {}",
                chunk.id
            );
            n += near as usize;
        }
        multi += (n > 1) as usize;
    }
    assert!(multi > 0);
    assert_eq!(p.shared_frames, multi);
    // A frame 4 m past a core boundary is shared; one 6 m past is not.
    let edge = p.chunks[0].core[2];
    let shared = |x: f64| {
        p.chunks[0]
            .frames
            .iter()
            .any(|n| frames.iter().any(|(m, c)| m == n && (c[0] - x).abs() < 0.5))
    };
    assert!(shared(edge + 4.0) && !shared(edge + 6.5));
}

#[test]
fn sparse_cells_merge_into_their_nearest_chunk() {
    // 60 frames in a 30 m run, then two strays 70 m further on.
    let mut xs: Vec<f64> = (0..60).map(|i| i as f64 * 0.5).collect();
    xs.extend([100.0, 100.5]);
    let frames = line(xs);
    let p = plan(&frames, 20, 5.0).expect("chunked");
    assert!(
        p.chunks.iter().all(|c| c.core_frames >= 5),
        "{:?}",
        p.detail()
    );
    let stray = p.owner_of(100.0, 0.0);
    assert!(p.chunks[stray].core_frames > 2);
    assert!(p.chunks[stray].frames.iter().any(|n| n == "cam0/60.png"));
}

/// A splat PLY with gaussians at `xs` (y = 0, z = 1) and one extra property.
fn write_splat(path: &Path, xs: &[f64]) {
    let mut bytes = format!(
        "ply\nformat binary_little_endian 1.0\ncomment test\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nproperty float opacity\nend_header\n",
        xs.len()
    )
    .into_bytes();
    for x in xs {
        for v in [*x as f32, 0.0, 1.0, 0.5] {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
    }
    std::fs::write(path, bytes).unwrap();
}

fn splat_xs(path: &Path) -> Vec<f32> {
    let bytes = std::fs::read(path).unwrap();
    let text = String::from_utf8_lossy(&bytes);
    let body = text.find("end_header\n").unwrap() + "end_header\n".len();
    let n: usize = text
        .lines()
        .find_map(|l| l.strip_prefix("element vertex "))
        .unwrap()
        .parse()
        .unwrap();
    assert_eq!(bytes.len(), body + n * 16);
    (0..n)
        .map(|i| f32::from_le_bytes(bytes[body + i * 16..body + i * 16 + 4].try_into().unwrap()))
        .collect()
}

#[test]
fn the_merged_splat_keeps_each_gaussian_from_its_core_chunk_only() {
    // Two chunks split at x = 19.5 (cells of 19.5 m; the stray last frame
    // merges into the second).
    let p = plan(&line((0..40).map(f64::from)), 20, 5.0).expect("chunked");
    assert_eq!(p.chunks.len(), 2);
    assert_eq!(p.owner_of(19.0, 0.0), 0);
    assert_eq!(p.owner_of(20.0, 0.0), 1);
    let dir = tempfile::tempdir().unwrap();
    let (a, b) = (dir.path().join("a.ply"), dir.path().join("b.ply"));
    // 19 and 20 lie in both chunks' overlaps and were trained by both.
    write_splat(&a, &[10.0, 19.0, 20.0, 24.0]);
    write_splat(&b, &[15.0, 19.0, 20.0, 30.0, f64::NAN]);
    let out = dir.path().join("merged.ply");
    let counts = merge_splats(&[(0, a), (1, b)], &p, &out).unwrap();
    assert_eq!(
        counts,
        [
            SplatCount { total: 4, core: 2 },
            SplatCount { total: 5, core: 2 }
        ]
    );
    let mut xs = splat_xs(&out);
    xs.sort_by(f32::total_cmp);
    assert_eq!(xs, [10.0, 19.0, 20.0, 30.0]);
    let merged = std::fs::read(&out).unwrap();
    assert!(String::from_utf8_lossy(&merged).contains("comment test\nelement vertex 4\n"));
}

#[test]
fn chunk_models_merge_with_shared_images_once() {
    let p = plan(&line((0..40).map(f64::from)), 20, 5.0).expect("chunked");
    let dir = tempfile::tempdir().unwrap();
    let write = |name: &str, cams: &str, imgs: &str, pts: &str| {
        let d = dir.path().join(name);
        std::fs::create_dir_all(&d).unwrap();
        std::fs::write(d.join("cameras.txt"), format!("# cameras\n{cams}")).unwrap();
        std::fs::write(d.join("images.txt"), format!("# images\n# 2d\n{imgs}")).unwrap();
        std::fs::write(d.join("points3D.txt"), format!("# points\n{pts}")).unwrap();
        d
    };
    let a = write(
        "a",
        "1 PINHOLE 64 48 50 50 32 24\n",
        "1 1 0 0 0 -10 0 0 1 cam0/a.png\n\n2 1 0 0 0 -19 0 0 1 cam0/s.png\n5 5 1 6 6 2\n",
        "1 10 0 1 1 2 3 0.5 2 0\n2 25 0 1 1 2 3 0.5 2 1\n",
    );
    let b = write(
        "b",
        "1 PINHOLE 64 48 51 51 32 24\n",
        "1 1 0 0 0 -19.03 0 0 1 cam0/s.png\n7 7 1\n2 1 0 0 0 -30 0 0 1 cam0/b.png\n8 8 1\n",
        "1 22 0 1 4 5 6 0.25 1 0 2 0\n",
    );
    let out = dir.path().join("txt");
    let m = merge_models(&[(0, a), (1, b)], &p, &out).unwrap();
    assert_eq!((m.images, m.points), (3, 2));
    assert!((m.max_shared_pose_delta_m - 0.03).abs() < 1e-9, "{m:?}");
    let cams = std::fs::read_to_string(out.join("cameras.txt")).unwrap();
    assert!(cams.contains("\n1 PINHOLE 64 48 50 50 32 24\n2 PINHOLE 64 48 51 51 32 24\n"));
    let imgs = std::fs::read_to_string(out.join("images.txt")).unwrap();
    // The shared image keeps the first chunk's pose; the second chunk's
    // observation of a kept point joins it.
    assert!(
        imgs.contains("2 1 0 0 0 -19 0 0 1 cam0/s.png\n5 5 1 6 6 -1 7 7 2\n"),
        "{imgs}"
    );
    assert!(
        imgs.contains("3 1 0 0 0 -30 0 0 2 cam0/b.png\n8 8 2\n"),
        "{imgs}"
    );
    assert!(!imgs.contains("-19.03"));
    let pts = std::fs::read_to_string(out.join("points3D.txt")).unwrap();
    assert!(
        pts.contains("\n1 10 0 1 1 2 3 0.5 2 0\n2 22 0 1 4 5 6 0.25 2 2 3 0\n"),
        "{pts}"
    );
    // The poses read back as the shared model's.
    let poses = crate::recon::colmap::read_image_poses(&out).unwrap();
    assert_eq!(poses.len(), 3);
}

#[test]
fn the_merged_model_carries_one_rig_per_camera_and_one_frame_per_image() {
    let p = plan(&line((0..40).map(f64::from)), 20, 5.0).expect("chunked");
    let dir = tempfile::tempdir().unwrap();
    let mut models = Vec::new();
    for (id, (fx, x)) in [(50, -10.0), (51, -30.0)].into_iter().enumerate() {
        let d = dir.path().join(id.to_string());
        std::fs::create_dir_all(&d).unwrap();
        let cams = format!("1 PINHOLE 64 48 {fx} {fx} 32 24\n");
        std::fs::write(d.join("cameras.txt"), cams).unwrap();
        let imgs =
            format!("1 1 0 0 0 {x} 0 0 1 cam0/{id}a.png\n\n2 0 0 0 1 {x} 1 0 1 cam0/{id}b.png\n\n");
        std::fs::write(d.join("images.txt"), imgs).unwrap();
        std::fs::write(d.join("points3D.txt"), "").unwrap();
        models.push((id, d));
    }
    let out = dir.path().join("txt");
    merge_models(&models, &p, &out).unwrap();
    let data = |f: &str| -> Vec<Vec<String>> {
        std::fs::read_to_string(out.join(f))
            .unwrap()
            .lines()
            .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
            .map(|l| l.split_whitespace().map(str::to_string).collect())
            .collect()
    };
    let cams: Vec<String> = data("cameras.txt")
        .into_iter()
        .map(|v| v[0].clone())
        .collect();
    let rigs = data("rigs.txt");
    assert_eq!(rigs.len(), cams.len());
    for (r, c) in rigs.iter().zip(&cams) {
        assert_eq!(r, &[c.as_str(), "1", "CAMERA", c.as_str()]);
    }
    // images.txt alternates header and 2D-point lines.
    let images: Vec<Vec<String>> = std::fs::read_to_string(out.join("images.txt"))
        .unwrap()
        .lines()
        .filter(|l| !l.starts_with('#'))
        .step_by(2)
        .map(|l| l.split_whitespace().map(str::to_string).collect())
        .collect();
    let frames = data("frames.txt");
    assert_eq!((images.len(), frames.len()), (4, 4));
    for (f, img) in frames.iter().zip(&images) {
        // FRAME_ID RIG_ID pose[7] 1 CAMERA CAMERA_ID IMAGE_ID
        assert_eq!(f.len(), 13, "{f:?}");
        assert_eq!(f[0], img[0]);
        assert_eq!(f[1], img[8]);
        assert_eq!(f[2..9], img[1..8], "the rig pose is the camera pose");
        assert_eq!(
            f[9..],
            [
                String::from("1"),
                "CAMERA".into(),
                img[8].clone(),
                img[0].clone()
            ]
        );
    }
}
