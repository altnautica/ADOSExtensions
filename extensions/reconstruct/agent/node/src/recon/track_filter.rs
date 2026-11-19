//! Sparse-point filter run on the SfM text model before training.
//!
//! Points seen by fewer than three images are dropped: a two-view point has
//! no redundancy, and a wrong match between two look-alike views becomes a
//! floater the trainer densifies. Where the dataset has metric depth, a point
//! whose depth along the ray from its nearest observing camera disagrees by
//! more than 0.30 m is dropped too: the world-frame LiDAR cloud of a posed
//! model, else the monocular depth maps (scaled per image onto the model by
//! the median ratio, so an unposed model of arbitrary scale is judged in
//! metres as well).

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use kiddo::{ImmutableKdTree, SquaredEuclidean};

use super::geom::{self, Mat4};
use super::StageError;
use crate::products::cloud::read_laz;

/// Shortest track kept.
const MIN_TRACK: usize = 3;
/// Largest depth disagreement kept, metres.
const MAX_DEPTH_ERR_M: f64 = 0.30;
/// LiDAR hit radius around the ray, metres (well under the tolerance).
const HIT_RADIUS_M: f64 = 0.10;
/// How far past the point the ray looks for the LiDAR surface, metres.
const SEARCH_PAST_M: f64 = 10.0;
/// Observations an image needs before its monocular depth scale is trusted.
const MIN_SCALE_OBS: usize = 10;

/// What the filter dropped.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Dropped {
    pub short_track: usize,
    pub depth: usize,
}

struct Image {
    /// World-from-camera.
    t_wc: Mat4,
    camera_id: u32,
    name: String,
    /// Each 2D point's pixel.
    xy: Vec<[f64; 2]>,
}

struct Point {
    id: u64,
    xyz: [f64; 3],
    /// (image id, 2D point index).
    track: Vec<(u32, usize)>,
}

fn parse_images(body: &str) -> BTreeMap<u32, Image> {
    let mut out = BTreeMap::new();
    let mut lines = body.lines().filter(|l| !l.starts_with('#'));
    while let Some(head) = lines.next() {
        let pts = lines.next().unwrap_or("");
        let v: Vec<&str> = head.split_whitespace().collect();
        if v.len() < 10 {
            continue;
        }
        let f = |i: usize| v[i].parse::<f64>().unwrap_or(0.0);
        let t_cw = geom::from_quat_trans([f(1), f(2), f(3), f(4)], [f(5), f(6), f(7)]);
        let p: Vec<f64> = pts
            .split_whitespace()
            .map(|s| s.parse().unwrap_or(f64::NAN))
            .collect();
        out.insert(
            v[0].parse().unwrap_or(0),
            Image {
                t_wc: geom::inverse_rigid(&t_cw),
                camera_id: v[8].parse().unwrap_or(0),
                name: v[9..].join(" "),
                xy: p.as_chunks::<3>().0.iter().map(|c| [c[0], c[1]]).collect(),
            },
        );
    }
    out
}

fn parse_points(body: &str) -> Vec<Point> {
    body.lines()
        .filter(|l| !l.starts_with('#') && !l.trim().is_empty())
        .filter_map(|l| {
            let v: Vec<&str> = l.split_whitespace().collect();
            if v.len() < 8 {
                return None;
            }
            let f = |i: usize| v[i].parse::<f64>().ok();
            Some(Point {
                id: v[0].parse().ok()?,
                xyz: [f(1)?, f(2)?, f(3)?],
                track: v[8..]
                    .as_chunks::<2>()
                    .0
                    .iter()
                    .filter_map(|c| Some((c[0].parse().ok()?, c[1].parse().ok()?)))
                    .collect(),
            })
        })
        .collect()
}

/// `(width, height)` per camera from `cameras.txt`.
fn parse_camera_sizes(body: &str) -> BTreeMap<u32, (f64, f64)> {
    body.lines()
        .filter(|l| !l.starts_with('#'))
        .filter_map(|l| {
            let v: Vec<&str> = l.split_whitespace().collect();
            Some((
                v.first()?.parse().ok()?,
                (v.get(2)?.parse().ok()?, v.get(3)?.parse().ok()?),
            ))
        })
        .collect()
}

/// The observing image nearest the point: (image id, 2D index, distance).
fn nearest_view(p: &Point, images: &BTreeMap<u32, Image>) -> Option<(u32, usize, f64)> {
    p.track
        .iter()
        .filter_map(|&(id, k)| {
            let c = geom::translation(&images.get(&id)?.t_wc);
            Some((id, k, geom::dist(c, p.xyz)))
        })
        .min_by(|a, b| a.2.total_cmp(&b.2))
}

/// The distance along the ray from `c` through `x` at which it first passes
/// within [`HIT_RADIUS_M`] of a LiDAR point, searched up to
/// [`SEARCH_PAST_M`] beyond `x`; `None` when it meets none (no LiDAR there).
/// The march is sphere-traced: with no LiDAR point within `d` of the ray
/// position, none comes within the hit radius before `d - radius` further on.
fn lidar_depth(tree: &ImmutableKdTree<f64, 3>, c: [f64; 3], x: [f64; 3]) -> Option<f64> {
    let range = geom::dist(c, x);
    if range < 1e-6 {
        return None;
    }
    let d = [
        (x[0] - c[0]) / range,
        (x[1] - c[1]) / range,
        (x[2] - c[2]) / range,
    ];
    let mut s = HIT_RADIUS_M;
    while s <= range + SEARCH_PAST_M {
        let q = [c[0] + s * d[0], c[1] + s * d[1], c[2] + s * d[2]];
        let nn = tree
            .query(&q)
            .nearest_one::<SquaredEuclidean<f64>>()
            .execute();
        let gap = nn.distance.sqrt();
        if gap <= HIT_RADIUS_M {
            return Some(s);
        }
        s += (gap - HIT_RADIUS_M).max(0.5 * HIT_RADIUS_M);
    }
    None
}

/// A `.npy` float32 (C order) 2D array: `(height, width, values)`; `None`
/// for anything else.
fn read_npy_f32(path: &Path) -> Option<(usize, usize, Vec<f32>)> {
    let b = fs::read(path).ok()?;
    if b.len() < 10 || &b[..6] != b"\x93NUMPY" {
        return None;
    }
    let (hlen, start) = match b[6] {
        1 => (u16::from_le_bytes([b[8], b[9]]) as usize, 10),
        _ if b.len() >= 12 => (u32::from_le_bytes([b[8], b[9], b[10], b[11]]) as usize, 12),
        _ => return None,
    };
    let header = std::str::from_utf8(b.get(start..start + hlen)?).ok()?;
    if !header.contains("'<f4'") || header.contains("'fortran_order': True") {
        return None;
    }
    let shape = header.split("'shape':").nth(1)?;
    let dims: Vec<usize> = shape[shape.find('(')? + 1..shape.find(')')?]
        .split(',')
        .filter_map(|s| s.trim().parse().ok())
        .collect();
    let [h, w] = dims[..] else {
        return None;
    };
    let data = b.get(start + hlen..)?;
    if data.len() < h * w * 4 {
        return None;
    }
    let v = data[..h * w * 4]
        .as_chunks::<4>()
        .0
        .iter()
        .map(|c| f32::from_le_bytes(*c))
        .collect();
    Some((h, w, v))
}

/// Points (by index into `points`) whose nearest-view depth disagrees with
/// the LiDAR cloud.
fn lidar_outliers(
    cloud: &Path,
    points: &[Point],
    images: &BTreeMap<u32, Image>,
) -> Result<BTreeSet<usize>, StageError> {
    let cloud = read_laz(cloud).map_err(StageError::io)?;
    if cloud.points.is_empty() {
        return Ok(BTreeSet::new());
    }
    let tree = ImmutableKdTree::new_from_slice(&cloud.points)
        .expect("u32 item index covers the cloud size");
    let mut out = BTreeSet::new();
    for (i, p) in points.iter().enumerate() {
        let Some((id, _, range)) = nearest_view(p, images) else {
            continue;
        };
        let c = geom::translation(&images[&id].t_wc);
        if let Some(hit) = lidar_depth(&tree, c, p.xyz) {
            if (hit - range).abs() > MAX_DEPTH_ERR_M {
                out.insert(i);
            }
        }
    }
    Ok(out)
}

/// Points whose nearest-view depth disagrees with that image's monocular
/// depth map, after scaling the map onto the model. One map is in memory at
/// a time.
fn mono_outliers(
    depths: &Path,
    points: &[Point],
    images: &BTreeMap<u32, Image>,
    sizes: &BTreeMap<u32, (f64, f64)>,
) -> BTreeSet<usize> {
    let nearest: Vec<Option<(u32, usize)>> = points
        .iter()
        .map(|p| nearest_view(p, images).map(|(id, k, _)| (id, k)))
        .collect();
    let mut per_image: BTreeMap<u32, Vec<(usize, usize)>> = BTreeMap::new();
    for (i, p) in points.iter().enumerate() {
        for &(id, k) in &p.track {
            per_image.entry(id).or_default().push((i, k));
        }
    }
    let mut out = BTreeSet::new();
    for (id, list) in per_image {
        let Some(img) = images.get(&id) else {
            continue;
        };
        let (Some((h, w, map)), Some(&(cw, ch))) = (
            read_npy_f32(&depths.join(format!("{}.npy", img.name))),
            sizes.get(&img.camera_id),
        ) else {
            continue;
        };
        let t_cw = geom::inverse_rigid(&img.t_wc);
        // (point index, 2D index, model depth, map depth) of each observation.
        let obs: Vec<(usize, usize, f64, f64)> = list
            .into_iter()
            .filter_map(|(i, k)| {
                let xy = img.xy.get(k)?;
                let z = geom::apply(&t_cw, points[i].xyz)[2];
                let (u, r) = (xy[0] * w as f64 / cw, xy[1] * h as f64 / ch);
                if z <= 0.0 || !(0.0..w as f64).contains(&u) || !(0.0..h as f64).contains(&r) {
                    return None;
                }
                let m = f64::from(map[r as usize * w + u as usize]);
                (m.is_finite() && m > 0.0).then_some((i, k, z, m))
            })
            .collect();
        if obs.len() < MIN_SCALE_OBS {
            continue;
        }
        let mut ratios: Vec<f64> = obs.iter().map(|(_, _, z, m)| z / m).collect();
        ratios.sort_by(f64::total_cmp);
        let s = ratios[ratios.len() / 2];
        for (i, k, z, m) in obs {
            if nearest[i] == Some((id, k)) && (z / s - m).abs() > MAX_DEPTH_ERR_M {
                out.insert(i);
            }
        }
    }
    out
}

/// Filter the text model in `model_txt` in place (`points3D.txt`, and the
/// dropped points' references in `images.txt` set to -1). `dataset` is where
/// `lidar_points.laz` and `depths/` are looked for; the LiDAR cloud is used
/// only for a model in the session's world frame (`world_frame`).
pub fn run(model_txt: &Path, dataset: &Path, world_frame: bool) -> Result<Dropped, StageError> {
    let images_body = fs::read_to_string(model_txt.join("images.txt"))?;
    let points_body = fs::read_to_string(model_txt.join("points3D.txt"))?;
    let images = parse_images(&images_body);
    let all = parse_points(&points_body);
    let (kept, short): (Vec<Point>, Vec<Point>) =
        all.into_iter().partition(|p| p.track.len() >= MIN_TRACK);
    let lidar = dataset.join("lidar_points.laz");
    let depths = dataset.join("depths");
    let outliers = if world_frame && lidar.is_file() {
        lidar_outliers(&lidar, &kept, &images)?
    } else if depths.is_dir() {
        let sizes = parse_camera_sizes(&fs::read_to_string(model_txt.join("cameras.txt"))?);
        mono_outliers(&depths, &kept, &images, &sizes)
    } else {
        BTreeSet::new()
    };
    let dropped = Dropped {
        short_track: short.len(),
        depth: outliers.len(),
    };
    if dropped == Dropped::default() {
        return Ok(dropped);
    }
    let mut gone: BTreeSet<u64> = short.iter().map(|p| p.id).collect();
    gone.extend(outliers.iter().map(|&i| kept[i].id));

    let mut points = String::with_capacity(points_body.len());
    for line in points_body.lines() {
        let id = line
            .split_whitespace()
            .next()
            .and_then(|s| s.parse::<u64>().ok());
        if !line.starts_with('#') && id.is_some_and(|id| gone.contains(&id)) {
            continue;
        }
        points.push_str(line);
        points.push('\n');
    }
    // `images.txt` alternates a header line and a 2D-point line (X Y POINT3D_ID ...).
    let mut out = String::with_capacity(images_body.len());
    let mut header = true;
    for line in images_body.lines() {
        if line.starts_with('#') {
            let _ = writeln!(out, "{line}");
            continue;
        }
        if header {
            let _ = writeln!(out, "{line}");
        } else {
            let fields: Vec<&str> = line.split_whitespace().collect();
            let mut row = Vec::with_capacity(fields.len());
            for c in fields.chunks(3) {
                let id = c.get(2).and_then(|s| s.parse::<u64>().ok());
                row.extend_from_slice(&c[..c.len().min(2)]);
                match (c.get(2), id.is_some_and(|id| gone.contains(&id))) {
                    (Some(_), true) => row.push("-1"),
                    (Some(s), false) => row.push(s),
                    (None, _) => {}
                }
            }
            let _ = writeln!(out, "{}", row.join(" "));
        }
        header = !header;
    }
    fs::write(model_txt.join("points3D.txt"), points)?;
    fs::write(model_txt.join("images.txt"), out)?;
    Ok(dropped)
}

#[cfg(test)]
#[path = "track_filter_tests.rs"]
mod tests;
