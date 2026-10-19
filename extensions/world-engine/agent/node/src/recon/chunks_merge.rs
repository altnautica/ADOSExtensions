//! Merging per-chunk results into one: splat PLYs cropped to each chunk's
//! core, and the chunk COLMAP text models into one model.

use std::collections::HashMap;
use std::fmt::Write as _;
use std::fs;
use std::io::Write as _;
use std::path::{Path, PathBuf};

use super::chunks::ChunkPlan;
use super::geom;
use super::StageError;

/// One chunk's gaussians: all of them, and those inside its core.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SplatCount {
    pub total: u64,
    pub core: u64,
}

/// A binary little-endian splat PLY: its vertex layout and raw records.
struct Splat {
    comments: Vec<String>,
    props: Vec<String>,
    count: usize,
    stride: usize,
    /// Byte offset and width (4 or 8) of `x`, `y`, `z`.
    xyz: [(usize, usize); 3],
    bytes: Vec<u8>,
    body: usize,
}

fn scalar_size(t: &str) -> Option<usize> {
    match t {
        "char" | "uchar" | "int8" | "uint8" => Some(1),
        "short" | "ushort" | "int16" | "uint16" => Some(2),
        "int" | "uint" | "float" | "int32" | "uint32" | "float32" => Some(4),
        "double" | "float64" => Some(8),
        _ => None,
    }
}

fn bad(path: &Path, why: &str) -> StageError {
    StageError::io(format!("{}: {why}", path.display()))
}

fn read_splat(path: &Path) -> Result<Splat, StageError> {
    let bytes = fs::read(path)?;
    let marker = b"end_header";
    let at = bytes
        .windows(marker.len())
        .position(|w| w == marker)
        .ok_or_else(|| bad(path, "no PLY header"))?;
    let body = at
        + bytes[at..]
            .iter()
            .position(|&b| b == b'\n')
            .ok_or_else(|| bad(path, "truncated header"))?
        + 1;
    let header = std::str::from_utf8(&bytes[..at]).map_err(|_| bad(path, "header is not text"))?;
    let mut s = Splat {
        comments: Vec::new(),
        props: Vec::new(),
        count: 0,
        stride: 0,
        xyz: [(usize::MAX, 0); 3],
        bytes: Vec::new(),
        body,
    };
    // 0: before the vertex element, 1: in it, 2: past it.
    let mut state = 0;
    for line in header.lines().map(str::trim) {
        let v: Vec<&str> = line.split_whitespace().collect();
        match v.as_slice() {
            ["ply"] | [] => {}
            ["format", f, ..] if *f != "binary_little_endian" => {
                return Err(bad(path, "only binary little-endian splats are merged"));
            }
            ["format", ..] => {}
            ["comment", ..] | ["obj_info", ..] => s.comments.push(line.to_string()),
            ["element", "vertex", n] if state == 0 => {
                s.count = n.parse().map_err(|_| bad(path, "bad vertex count"))?;
                state = 1;
            }
            ["element", ..] if state == 0 => {
                return Err(bad(path, "vertex is not the first element"))
            }
            ["element", ..] => state = 2,
            ["property", "list", ..] if state == 1 => {
                return Err(bad(path, "list property in vertex"))
            }
            ["property", t, name] if state == 1 => {
                let size = scalar_size(t).ok_or_else(|| bad(path, "unknown property type"))?;
                if let Some(k) = ["x", "y", "z"].iter().position(|a| a == name) {
                    if size < 4 {
                        return Err(bad(path, "vertex position is not floating point"));
                    }
                    s.xyz[k] = (s.stride, size);
                }
                s.props.push(line.to_string());
                s.stride += size;
            }
            _ => {}
        }
    }
    if s.xyz.iter().any(|a| a.0 == usize::MAX) {
        return Err(bad(path, "no x/y/z properties"));
    }
    if bytes.len() < body + s.count * s.stride {
        return Err(bad(path, "body shorter than the header says"));
    }
    s.bytes = bytes;
    Ok(s)
}

impl Splat {
    fn record(&self, i: usize) -> &[u8] {
        let at = self.body + i * self.stride;
        &self.bytes[at..at + self.stride]
    }

    fn centre(&self, i: usize) -> [f64; 3] {
        let r = self.record(i);
        self.xyz.map(|(off, size)| {
            if size == 8 {
                f64::from_le_bytes(r[off..off + 8].try_into().expect("8 bytes"))
            } else {
                f32::from_le_bytes(r[off..off + 4].try_into().expect("4 bytes")) as f64
            }
        })
    }
}

/// Records of `s` whose centre is finite and inside chunk `id`'s core.
fn core_records<'a>(
    s: &'a Splat,
    plan: &'a ChunkPlan,
    id: usize,
) -> impl Iterator<Item = usize> + 'a {
    (0..s.count).filter(move |&i| {
        let c = s.centre(i);
        c.iter().all(|v| v.is_finite()) && plan.owner_of(c[0], c[1]) == id
    })
}

/// Concatenate each chunk's gaussians that lie in its core into `out`.
/// Every chunk PLY must share one vertex layout.
pub fn merge_splats(
    parts: &[(usize, PathBuf)],
    plan: &ChunkPlan,
    out: &Path,
) -> Result<Vec<SplatCount>, StageError> {
    let mut counts = Vec::with_capacity(parts.len());
    let mut layout: Option<(Vec<String>, Vec<String>)> = None;
    for (id, path) in parts {
        let s = read_splat(path)?;
        match &layout {
            Some((_, props)) if *props != s.props => {
                return Err(bad(path, "vertex layout differs from the other chunks"));
            }
            Some(_) => {}
            None => layout = Some((s.comments.clone(), s.props.clone())),
        }
        counts.push(SplatCount {
            total: s.count as u64,
            core: core_records(&s, plan, *id).count() as u64,
        });
    }
    let Some((comments, props)) = layout else {
        return Err(StageError::input("no chunk splats to merge"));
    };
    let mut header = String::from("ply\nformat binary_little_endian 1.0\n");
    for c in &comments {
        let _ = writeln!(header, "{c}");
    }
    let total: u64 = counts.iter().map(|c| c.core).sum();
    let _ = writeln!(header, "element vertex {total}");
    for p in &props {
        let _ = writeln!(header, "{p}");
    }
    header.push_str("end_header\n");
    if let Some(parent) = out.parent() {
        fs::create_dir_all(parent)?;
    }
    let mut w = std::io::BufWriter::new(fs::File::create(out)?);
    w.write_all(header.as_bytes())?;
    for (id, path) in parts {
        let s = read_splat(path)?;
        for i in core_records(&s, plan, *id) {
            w.write_all(s.record(i))?;
        }
    }
    w.flush()?;
    Ok(counts)
}

/// What [`merge_models`] wrote.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MergedModel {
    pub images: usize,
    pub points: usize,
    /// The largest camera-centre disagreement of a frame two chunks posed.
    pub max_shared_pose_delta_m: f64,
}

struct Image {
    id: u32,
    q: [f64; 4],
    t: [f64; 3],
    camera: u32,
    name: String,
    /// `(x, y, point3D_id)`, `-1` for none.
    points2d: Vec<(f64, f64, i64)>,
}

impl Image {
    fn centre(&self) -> [f64; 3] {
        geom::translation(&geom::inverse_rigid(&geom::from_quat_trans(self.q, self.t)))
    }
}

struct Point {
    xyz: [f64; 3],
    rest: String,
    track: Vec<(u32, usize)>,
}

fn data_lines(path: &Path) -> Result<Vec<String>, StageError> {
    Ok(fs::read_to_string(path)?
        .lines()
        .filter(|l| !l.starts_with('#'))
        .map(str::to_string)
        .collect())
}

fn read_cameras(txt: &Path) -> Result<HashMap<u32, String>, StageError> {
    let mut out = HashMap::new();
    for line in data_lines(&txt.join("cameras.txt"))? {
        if let Some((id, rest)) = line.trim().split_once(' ') {
            if let Ok(id) = id.parse() {
                out.insert(id, rest.to_string());
            }
        }
    }
    Ok(out)
}

fn read_images(txt: &Path) -> Result<Vec<Image>, StageError> {
    let lines = data_lines(&txt.join("images.txt"))?;
    let mut out = Vec::new();
    for pair in lines.chunks(2) {
        let v: Vec<&str> = pair[0].split_whitespace().collect();
        if v.len() < 10 {
            continue;
        }
        let f = |i: usize| v[i].parse::<f64>().unwrap_or(f64::NAN);
        let p: Vec<&str> = pair
            .get(1)
            .map(|l| l.split_whitespace().collect())
            .unwrap_or_default();
        out.push(Image {
            id: v[0].parse().map_err(StageError::io)?,
            q: [f(1), f(2), f(3), f(4)],
            t: [f(5), f(6), f(7)],
            camera: v[8].parse().map_err(StageError::io)?,
            name: v[9..].join(" "),
            points2d: p
                .as_chunks::<3>()
                .0
                .iter()
                .map(|[x, y, id]| {
                    (
                        x.parse().unwrap_or(f64::NAN),
                        y.parse().unwrap_or(f64::NAN),
                        id.parse().unwrap_or(-1),
                    )
                })
                .collect(),
        });
    }
    Ok(out)
}

fn read_points3d(txt: &Path) -> Result<Vec<Point>, StageError> {
    let mut out = Vec::new();
    for line in data_lines(&txt.join("points3D.txt"))? {
        let v: Vec<&str> = line.split_whitespace().collect();
        if v.len() < 8 {
            continue;
        }
        let f = |i: usize| v[i].parse::<f64>().unwrap_or(f64::NAN);
        out.push(Point {
            xyz: [f(1), f(2), f(3)],
            rest: v[4..8].join(" "),
            track: v[8..]
                .as_chunks::<2>()
                .0
                .iter()
                .filter_map(|[img, idx]| Some((img.parse().ok()?, idx.parse().ok()?)))
                .collect(),
        });
    }
    Ok(out)
}

/// Merge the chunk text models `(chunk id, txt dir)` into `out_txt`: cameras
/// renumbered, images deduplicated by name (the first chunk's pose wins; the
/// later copy's observations of kept points join it as extra 2D points),
/// and from each chunk only the points inside its core. The COLMAP 4 rig
/// files are written as well: one single-camera rig per camera (rig id =
/// camera id) and one frame per image (frame id = image id).
pub fn merge_models(
    models: &[(usize, PathBuf)],
    plan: &ChunkPlan,
    out_txt: &Path,
) -> Result<MergedModel, StageError> {
    let mut cameras = String::from("# CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n");
    let mut n_cameras = 0u32;
    let mut images: Vec<Image> = Vec::new();
    let mut by_name: HashMap<String, usize> = HashMap::new();
    let mut points: Vec<Point> = Vec::new();
    let mut max_delta = 0.0f64;
    for (chunk, txt) in models {
        let chunk_cams = read_cameras(txt)?;
        let mut cam_ids: HashMap<u32, u32> = HashMap::new();
        // Chunk image id -> (merged index, the merged image is this copy).
        let mut map: HashMap<u32, (usize, bool)> = HashMap::new();
        let mut dups: HashMap<u32, Vec<(f64, f64, i64)>> = HashMap::new();
        for mut img in read_images(txt)? {
            if let Some(&k) = by_name.get(&img.name) {
                max_delta = max_delta.max(geom::dist(img.centre(), images[k].centre()));
                map.insert(img.id, (k, false));
                dups.insert(img.id, img.points2d);
                continue;
            }
            let camera = match cam_ids.get(&img.camera) {
                Some(&c) => c,
                None => {
                    let line = chunk_cams.get(&img.camera).ok_or_else(|| {
                        StageError::tool_failed("colmap", format!("camera {} missing", img.camera))
                    })?;
                    n_cameras += 1;
                    let _ = writeln!(cameras, "{n_cameras} {line}");
                    cam_ids.insert(img.camera, n_cameras);
                    n_cameras
                }
            };
            map.insert(img.id, (images.len(), true));
            by_name.insert(img.name.clone(), images.len());
            img.id = images.len() as u32 + 1;
            img.camera = camera;
            img.points2d.iter_mut().for_each(|p| p.2 = -1);
            images.push(img);
        }
        for mut p in read_points3d(txt)? {
            if !p.xyz.iter().all(|v| v.is_finite()) || plan.owner_of(p.xyz[0], p.xyz[1]) != *chunk {
                continue;
            }
            let pid = points.len() as i64 + 1;
            let mut track = Vec::with_capacity(p.track.len());
            for (image_id, idx) in p.track {
                let Some(&(k, own)) = map.get(&image_id) else {
                    continue;
                };
                let kept = &mut images[k];
                if own {
                    if let Some(e) = kept.points2d.get_mut(idx) {
                        e.2 = pid;
                        track.push((kept.id, idx));
                    }
                } else if let Some(&(x, y, _)) = dups.get(&image_id).and_then(|d| d.get(idx)) {
                    kept.points2d.push((x, y, pid));
                    track.push((kept.id, kept.points2d.len() - 1));
                }
            }
            p.track = track;
            points.push(p);
        }
    }

    fs::create_dir_all(out_txt)?;
    fs::write(out_txt.join("cameras.txt"), cameras)?;
    let mut s = String::from("# RIG_ID, NUM_SENSORS, REF_SENSOR_TYPE, REF_SENSOR_ID, SENSORS[]\n");
    for cam in 1..=n_cameras {
        let _ = writeln!(s, "{cam} 1 CAMERA {cam}");
    }
    fs::write(out_txt.join("rigs.txt"), s)?;
    let mut s = String::from(
        "# FRAME_ID, RIG_ID, RIG_FROM_WORLD[QW, QX, QY, QZ, TX, TY, TZ], NUM_DATA_IDS, DATA_IDS[]\n",
    );
    for img in &images {
        let (q, t) = (img.q, img.t);
        let _ = writeln!(
            s,
            "{} {} {} {} {} {} {} {} {} 1 CAMERA {} {}",
            img.id, img.camera, q[0], q[1], q[2], q[3], t[0], t[1], t[2], img.camera, img.id
        );
    }
    fs::write(out_txt.join("frames.txt"), s)?;
    let mut s = String::from(
        "# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n# POINTS2D[] as (X, Y, POINT3D_ID)\n",
    );
    for img in &images {
        let (q, t) = (img.q, img.t);
        let _ = writeln!(
            s,
            "{} {} {} {} {} {} {} {} {} {}",
            img.id, q[0], q[1], q[2], q[3], t[0], t[1], t[2], img.camera, img.name
        );
        let p2: Vec<String> = img
            .points2d
            .iter()
            .map(|(x, y, id)| format!("{x} {y} {id}"))
            .collect();
        let _ = writeln!(s, "{}", p2.join(" "));
    }
    fs::write(out_txt.join("images.txt"), s)?;
    let mut s = String::from("# POINT3D_ID X Y Z R G B ERROR TRACK[] as (IMAGE_ID, POINT2D_IDX)\n");
    for (i, p) in points.iter().enumerate() {
        let _ = write!(
            s,
            "{} {} {} {} {}",
            i + 1,
            p.xyz[0],
            p.xyz[1],
            p.xyz[2],
            p.rest
        );
        for (img, idx) in &p.track {
            let _ = write!(s, " {img} {idx}");
        }
        s.push('\n');
    }
    fs::write(out_txt.join("points3D.txt"), s)?;
    Ok(MergedModel {
        images: images.len(),
        points: points.len(),
        max_shared_pose_delta_m: max_delta,
    })
}
