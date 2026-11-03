//! The `planar-sweep` preset's products, from a session's polar scans alone
//! (see [`super::polar`]): a planar LiDAR flown through several heights
//! sweeps 3D structure, so the posed returns become
//!
//! - `cloud.laz`, one point per [`CLOUD_VOXEL_M`] voxel;
//! - `occupancy_<lo>-<hi>m.png` + `.yaml` per [`HEIGHT_BANDS`] band, in the
//!   ROS `map_server` layout (0 occupied, 254 free, 205 unknown; image +x is
//!   world +x, row 0 the largest y); a cell is occupied when a return in the
//!   band lands in it and free when a beam fired from inside the band crossed
//!   it;
//! - the floorplan (JSON + SVG per floor).
//!
//! Heights are metres above the first scan's sensor height, so a flight that
//! takes off from the ground reads its bands from the ground. The world is the
//! drone's `map_frd` turned z-up (`x`, `-y`, `-z`).

use std::collections::HashSet;
use std::fs;
use std::io::Cursor;
use std::path::Path;

use image::codecs::png::PngEncoder;
use image::{ExtendedColorType, ImageEncoder};
use world_engine_protocol::atlas::{PolarScan, VioHealth};

use ados_capture_contract::{
    ArtifactDescriptorFormat as Fmt, ArtifactDescriptorKind as Kind, Placement,
};
use ados_scan2d::{offline, ImuSample, Scan2dConfig};

use super::polar::{origin_flu, scan_input, world_points_flu};
use super::products_stage::{art, write_sidecar, ArtifactOut};
use super::StageError;
use crate::products::cloud::PointCloud;

/// Voxel the product cloud keeps one point per.
pub const CLOUD_VOXEL_M: f64 = 0.02;
/// Occupancy cell edge.
pub const OCC_RES_M: f64 = 0.05;
/// The occupancy height bands, metres above the first scan's sensor.
pub const HEIGHT_BANDS: [(f64, f64); 3] = [(0.0, 3.0), (3.0, 8.0), (8.0, 15.0)];
/// The largest band image side; a bigger site is drawn at a coarser cell.
const MAX_SIDE_CELLS: f64 = 8192.0;

const OCCUPIED: u8 = 0;
const FREE: u8 = 254;
const UNKNOWN: u8 = 205;

/// The swept returns of the trusted scans, z-up.
pub struct Swept {
    pub cloud: PointCloud,
    /// Each trusted scan's sensor origin and its returns.
    pub rays: Vec<([f64; 3], Vec<[f64; 3]>)>,
    /// The first scan's sensor height, the band reference.
    pub z0: f64,
}

/// Register every scan not marked lost.
pub fn sweep(scans: &[PolarScan]) -> Swept {
    let mut voxels = HashSet::new();
    let mut points = Vec::new();
    let mut rays = Vec::new();
    let z0 = scans.first().map_or(0.0, |s| origin_flu(s)[2]);
    for s in scans.iter().filter(|s| s.health != VioHealth::Lost) {
        let hits: Vec<[f64; 3]> = world_points_flu(s).collect();
        for p in &hits {
            let key = p.map(|v| (v / CLOUD_VOXEL_M).floor() as i64);
            if voxels.insert(key) {
                points.push(*p);
            }
        }
        rays.push((origin_flu(s), hits));
    }
    Swept {
        cloud: PointCloud {
            points,
            colors: None,
        },
        rays,
        z0,
    }
}

/// One band's grid.
pub struct BandGrid {
    pub width: usize,
    pub height: usize,
    pub res_m: f64,
    /// World x, y of the lower-left cell's corner.
    pub origin: [f64; 2],
    /// Row-major, row 0 the largest y.
    pub cells: Vec<u8>,
}

impl BandGrid {
    pub fn occupied(&self) -> usize {
        self.cells.iter().filter(|c| **c == OCCUPIED).count()
    }

    pub fn free(&self) -> usize {
        self.cells.iter().filter(|c| **c == FREE).count()
    }

    fn index(&self, x: f64, y: f64) -> Option<(i64, i64)> {
        let cx = ((x - self.origin[0]) / self.res_m).floor() as i64;
        let cy = ((y - self.origin[1]) / self.res_m).floor() as i64;
        (cx >= 0 && cy >= 0 && (cx as usize) < self.width && (cy as usize) < self.height)
            .then_some((cx, cy))
    }

    fn set(&mut self, cx: i64, cy: i64, v: u8) {
        if cx < 0 || cy < 0 || cx as usize >= self.width || cy as usize >= self.height {
            return;
        }
        let row = self.height - 1 - cy as usize;
        let c = &mut self.cells[row * self.width + cx as usize];
        // An occupied cell stays occupied; free only paints the unknown.
        if v == OCCUPIED || *c == UNKNOWN {
            *c = v;
        }
    }

    pub fn png(&self) -> Vec<u8> {
        let mut out = Cursor::new(Vec::new());
        PngEncoder::new(&mut out)
            .write_image(
                &self.cells,
                self.width as u32,
                self.height as u32,
                ExtendedColorType::L8,
            )
            .expect("an in-memory PNG encode cannot fail");
        out.into_inner()
    }

    /// The `map_server` YAML beside `image`.
    pub fn yaml(&self, image: &str) -> String {
        format!(
            "image: {image}\nresolution: {}\norigin: [{}, {}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
            self.res_m, self.origin[0], self.origin[1]
        )
    }
}

/// The band `[lo, hi)` above `z0` over the cloud's footprint, or `None`
/// when the cloud is empty.
pub fn band(swept: &Swept, lo: f64, hi: f64) -> Option<BandGrid> {
    let pts = &swept.cloud.points;
    let first = pts.first()?;
    let (mut min, mut max) = ([first[0], first[1]], [first[0], first[1]]);
    for p in pts {
        min = [min[0].min(p[0]), min[1].min(p[1])];
        max = [max[0].max(p[0]), max[1].max(p[1])];
    }
    for (o, _) in &swept.rays {
        min = [min[0].min(o[0]), min[1].min(o[1])];
        max = [max[0].max(o[0]), max[1].max(o[1])];
    }
    let span = (max[0] - min[0]).max(max[1] - min[1]);
    let res = OCC_RES_M.max(span / MAX_SIDE_CELLS);
    let mut g = BandGrid {
        width: ((max[0] - min[0]) / res).floor() as usize + 1,
        height: ((max[1] - min[1]) / res).floor() as usize + 1,
        res_m: res,
        origin: min,
        cells: Vec::new(),
    };
    g.cells = vec![UNKNOWN; g.width * g.height];
    let in_band = |z: f64| (lo..hi).contains(&(z - swept.z0));
    for (o, hits) in &swept.rays {
        if !in_band(o[2]) {
            continue;
        }
        let Some((x0, y0)) = g.index(o[0], o[1]) else { continue };
        for h in hits {
            let Some((x1, y1)) = g.index(h[0], h[1]) else { continue };
            trace(&mut g, (x0, y0), (x1, y1));
        }
    }
    for p in pts.iter().filter(|p| in_band(p[2])) {
        if let Some((cx, cy)) = g.index(p[0], p[1]) {
            g.set(cx, cy, OCCUPIED);
        }
    }
    Some(g)
}

/// Mark free the cells from `a` up to (not including) `b`.
fn trace(g: &mut BandGrid, a: (i64, i64), b: (i64, i64)) {
    let (dx, dy) = ((b.0 - a.0).abs(), -(b.1 - a.1).abs());
    let (sx, sy) = (if a.0 < b.0 { 1 } else { -1 }, if a.1 < b.1 { 1 } else { -1 });
    let (mut x, mut y, mut err) = (a.0, a.1, dx + dy);
    while (x, y) != b {
        g.set(x, y, FREE);
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x += sx;
        }
        if e2 <= dx {
            err += dx;
            y += sy;
        }
    }
}

/// The file stem of a band's image.
pub fn band_name(lo: f64, hi: f64) -> String {
    format!("occupancy_{lo:.0}-{hi:.0}m")
}

/// Write one band's PNG and YAML into `out`; returns the PNG name.
pub fn write_band(out: &Path, lo: f64, hi: f64, g: &BandGrid) -> Result<String, StageError> {
    let stem = band_name(lo, hi);
    let png = format!("{stem}.png");
    fs::write(out.join(&png), g.png())?;
    fs::write(out.join(format!("{stem}.yaml")), g.yaml(&png))?;
    Ok(png)
}

/// What the offline loop closure did to the recorded poses.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize)]
pub struct LoopSummary {
    pub scans: usize,
    /// Scans re-posed from the optimised trajectory.
    pub reposed: usize,
    pub submaps: usize,
    pub loops_accepted: usize,
    pub loops_rejected: usize,
}

/// `(roll, pitch, yaw)` of an FLU rotation `Rz·Ry·Rx` (row-major).
fn rpy_of(r: &[f64; 9]) -> (f64, f64, f64) {
    (r[7].atan2(r[8]), (-r[6]).clamp(-1.0, 1.0).asin(), r[3].atan2(r[0]))
}

fn rot_rpy(roll: f64, pitch: f64, yaw: f64) -> [f64; 9] {
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    [
        cy * cp,
        cy * sp * sr - sy * cr,
        cy * sp * cr + sy * sr,
        sy * cp,
        sy * sp * sr + cy * cr,
        sy * sp * cr - cy * sr,
        -sp,
        cp * sr,
        cp * cr,
    ]
}

fn quat_to_rot(q: [f32; 4]) -> [f64; 9] {
    let [w, x, y, z] = q.map(f64::from);
    [
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - w * z),
        2.0 * (x * z + w * y),
        2.0 * (x * y + w * z),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z - w * x),
        2.0 * (x * z - w * y),
        2.0 * (y * z + w * x),
        1.0 - 2.0 * (x * x + y * y),
    ]
}

fn rot_to_quat(r: &[f64; 9]) -> [f32; 4] {
    let tr = r[0] + r[4] + r[8];
    let q = if tr > 0.0 {
        let s = (tr + 1.0).sqrt() * 2.0;
        [0.25 * s, (r[7] - r[5]) / s, (r[2] - r[6]) / s, (r[3] - r[1]) / s]
    } else if r[0] > r[4] && r[0] > r[8] {
        let s = (1.0 + r[0] - r[4] - r[8]).sqrt() * 2.0;
        [(r[7] - r[5]) / s, 0.25 * s, (r[1] + r[3]) / s, (r[2] + r[6]) / s]
    } else if r[4] > r[8] {
        let s = (1.0 + r[4] - r[0] - r[8]).sqrt() * 2.0;
        [(r[2] - r[6]) / s, (r[1] + r[3]) / s, 0.25 * s, (r[5] + r[7]) / s]
    } else {
        let s = (1.0 + r[8] - r[0] - r[4]).sqrt() * 2.0;
        [(r[3] - r[1]) / s, (r[2] + r[6]) / s, (r[5] + r[7]) / s, 0.25 * s]
    };
    let n = q.iter().map(|v| v * v).sum::<f64>().sqrt();
    q.map(|v| (v / n) as f32)
}

/// `F·R·F` with `F = diag(1, -1, -1)` (FRD ⇄ FLU; its own inverse).
fn flip(r: &[f64; 9]) -> [f64; 9] {
    [r[0], -r[1], -r[2], -r[3], r[4], r[5], -r[6], r[7], r[8]]
}

/// Re-pose the recorded scans with the offline scan matcher's loop-closed
/// trajectory (`ados_scan2d::offline`): x, y and the attitude come from the
/// optimised graph, rigidly aligned onto the first recorded pose; the height
/// stays the flight controller's. The IMU the matcher levels with is
/// gravity under each recorded attitude.
pub fn close_loops(scans: &mut [PolarScan]) -> LoopSummary {
    let inputs: Vec<_> = scans.iter().map(|s| scan_input(s, s.t_ns)).collect();
    let imu: Vec<ImuSample> = scans
        .iter()
        .map(|s| {
            let r = flip(&quat_to_rot(s.q));
            ImuSample {
                t_ns: s.t_ns,
                gyro: [0.0; 3],
                accel: [r[6] * 9.81, r[7] * 9.81, r[8] * 9.81],
            }
        })
        .collect();
    let traj = offline::run(&inputs, &imu, &Scan2dConfig::for_sensor(0.05, 12.0, &[]));
    let mut summary = LoopSummary {
        scans: scans.len(),
        reposed: 0,
        submaps: traj.submaps,
        loops_accepted: traj.loops_accepted,
        loops_rejected: traj.loops_rejected,
    };
    let by_t: std::collections::HashMap<u64, &ados_scan2d::PoseOut> =
        traj.poses.iter().map(|p| (p.t_ns, p)).collect();
    // The rigid 2D alignment from the first scan both have.
    let Some((rec0, off0)) = scans
        .iter()
        .find_map(|s| by_t.get(&s.t_ns).map(|p| (s.clone(), **p)))
    else {
        return summary;
    };
    let (_, _, yaw_rec0) = rpy_of(&flip(&quat_to_rot(rec0.q)));
    let dyaw = yaw_rec0 - off0.yaw;
    let (sd, cd) = dyaw.sin_cos();
    let rot = |x: f64, y: f64| (cd * x - sd * y, sd * x + cd * y);
    let (ox, oy) = rot(off0.x, off0.y);
    let shift = (rec0.t[0] as f64 - ox, -(rec0.t[1] as f64) - oy);
    for s in scans.iter_mut() {
        let Some(p) = by_t.get(&s.t_ns) else { continue };
        let (x, y) = rot(p.x, p.y);
        let r_flu = rot_rpy(p.roll, p.pitch, p.yaw + dyaw);
        s.q = rot_to_quat(&flip(&r_flu));
        s.t[0] = (x + shift.0) as f32;
        s.t[1] = -(y + shift.1) as f32;
        summary.reposed += 1;
    }
    summary
}

/// Write the cloud, the band occupancy and the floorplan into `out`.
pub fn write_products(
    out: &Path,
    swept: &Swept,
    generation: u32,
    session_id: &str,
) -> Result<Vec<ArtifactOut>, StageError> {
    let placement = Placement::identity(None);
    let mut arts = Vec::new();
    if swept.cloud.points.is_empty() {
        return Ok(arts);
    }
    crate::products::cloud::write_laz(&out.join("cloud.laz"), &swept.cloud)
        .map_err(StageError::io)?;
    let sidecar = write_sidecar(
        out,
        "cloud.laz",
        Kind::Pointcloud,
        Fmt::Laz,
        &placement,
        generation,
        session_id,
    )?;
    arts.push(art("pointcloud", "cloud.laz", "laz", Some(sidecar)));
    for (lo, hi) in HEIGHT_BANDS {
        let Some(g) = band(swept, lo, hi) else { continue };
        let png = write_band(out, lo, hi, &g)?;
        let mut a = art("occupancy", &png, "png", None);
        a.meta = serde_json::json!({
            "band_m": [lo, hi],
            "resolution_m": g.res_m,
            "yaml": format!("{}.yaml", band_name(lo, hi)),
            "occupied_cells": g.occupied(),
        });
        arts.push(a);
    }
    let origins: Vec<[f64; 3]> = swept.rays.iter().map(|(o, _)| *o).collect();
    let plan = crate::products::floorplan::build(
        &swept.cloud,
        &[],
        &origins,
        &crate::products::floorplan::FloorplanOptions::default(),
    );
    let value = serde_json::to_value(&plan)?;
    fs::write(out.join("floorplan.json"), serde_json::to_vec_pretty(&value)?)?;
    arts.push(art("floorplan", "floorplan.json", "json", None));
    for fl in value["floors"].as_array().cloned().unwrap_or_default() {
        let floor = fl["floor"].as_i64().unwrap_or(0) as i32;
        if let Some(svg) = crate::products::floorplan::to_svg(&plan, floor) {
            let name = format!("floorplan_{floor}.svg");
            fs::write(out.join(&name), svg)?;
            arts.push(art("floorplan_svg", &name, "svg", None));
        }
    }
    for a in &mut arts {
        let size = fs::metadata(out.join(&a.file)).map(|m| m.len()).unwrap_or(0);
        match &mut a.meta {
            serde_json::Value::Object(m) => {
                m.insert("size_bytes".into(), size.into());
            }
            meta => *meta = serde_json::json!({ "size_bytes": size }),
        }
    }
    Ok(arts)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A sensor at height `z` (FRD -z) in the middle of a 4 m square room
    /// seen as 360 bins.
    fn scan_at(z: f32) -> PolarScan {
        let n = 360;
        let inc = std::f32::consts::TAU / n as f32;
        let range_mm = (0..n)
            .map(|k| {
                let a = k as f32 * inc;
                let r = 2.0 / a.cos().abs().max(a.sin().abs());
                (r * 1000.0) as u16
            })
            .collect();
        PolarScan {
            t_ns: 0,
            angle_start_rad: 0.0,
            angle_inc_rad: inc,
            t: [0.0, 0.0, -z],
            q: [1.0, 0.0, 0.0, 0.0],
            health: VioHealth::Good,
            range_mm,
            quality: vec![100; n],
        }
    }

    #[test]
    fn a_swept_room_fills_only_the_bands_it_was_flown_through() {
        let mut scans = vec![scan_at(0.5), scan_at(1.5), scan_at(5.0)];
        let mut lost = scan_at(12.0);
        lost.health = VioHealth::Lost;
        scans.push(lost);
        let s = sweep(&scans);
        assert_eq!(s.rays.len(), 3, "the lost scan is not trusted");
        assert!((s.z0 - 0.5).abs() < 1e-6);

        let low = band(&s, 0.0, 3.0).unwrap();
        assert!(low.occupied() > 100, "the walls");
        assert!(low.free() > 1000, "the swept floor area");
        let mid = band(&s, 3.0, 8.0).unwrap();
        assert!(mid.occupied() > 100);
        let high = band(&s, 8.0, 15.0).unwrap();
        assert_eq!(high.occupied() + high.free(), 0, "nothing flew there");

        let dir = tempfile::tempdir().unwrap();
        let png = write_band(dir.path(), 0.0, 3.0, &low).unwrap();
        assert_eq!(png, "occupancy_0-3m.png");
        let img = image::open(dir.path().join(&png)).unwrap().to_luma8();
        assert_eq!(img.dimensions(), (low.width as u32, low.height as u32));
        let yaml = fs::read_to_string(dir.path().join("occupancy_0-3m.yaml")).unwrap();
        assert!(yaml.contains("image: occupancy_0-3m.png") && yaml.contains("resolution: 0.05"));

        let arts = write_products(dir.path(), &s, 1, "sess").unwrap();
        let kinds: Vec<&str> = arts.iter().map(|a| a.kind.as_str()).collect();
        assert!(kinds.contains(&"pointcloud") && kinds.contains(&"floorplan"));
        assert_eq!(kinds.iter().filter(|k| **k == "occupancy").count(), 3);
    }

    #[test]
    fn the_quaternion_round_trips_through_rotation() {
        let r = rot_rpy(0.1, -0.2, 2.5);
        let back = quat_to_rot(rot_to_quat(&r));
        assert!(r.iter().zip(&back).all(|(a, b)| (a - b).abs() < 1e-6));
        let (ro, pi, ya) = rpy_of(&r);
        assert!((ro - 0.1).abs() < 1e-9 && (pi + 0.2).abs() < 1e-9 && (ya - 2.5).abs() < 1e-9);
    }

    #[test]
    fn loop_closure_keeps_a_still_room_where_it_was_recorded() {
        let mut scans: Vec<PolarScan> = (0..30)
            .map(|i| {
                let mut s = scan_at(1.0);
                s.t_ns = i * 100_000_000;
                s
            })
            .collect();
        let sum = close_loops(&mut scans);
        assert_eq!(sum.scans, 30);
        assert!(sum.reposed > 0);
        for s in &scans {
            assert!(s.t[0].abs() < 0.05 && s.t[1].abs() < 0.05, "{:?}", s.t);
            assert!((s.t[2] + 1.0).abs() < 1e-6, "height stays recorded");
        }
    }
}
