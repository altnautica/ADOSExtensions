//! Point cloud and triangle mesh types, LAZ 1.4 I/O and voxel downsampling.

use std::collections::HashMap;
use std::fs::File;
use std::io::BufWriter;
use std::path::Path;

use las::point::Format;
use las::{Builder, Color, Point, Reader, Transform, Vector, Writer};

use super::ProductError;

pub use super::ply::{read_ply_mesh, read_ply_points};

/// Points in the local ENU metre frame, with optional 16-bit RGB per point.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct PointCloud {
    pub points: Vec<[f64; 3]>,
    pub colors: Option<Vec<[u16; 3]>>,
}

/// Indexed triangle mesh in the local ENU metre frame.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct TriMesh {
    pub vertices: Vec<[f64; 3]>,
    pub triangles: Vec<[u32; 3]>,
}

impl PointCloud {
    /// Axis-aligned bounds `(min, max)`; `None` for an empty cloud.
    pub fn bounds(&self) -> Option<([f64; 3], [f64; 3])> {
        let first = *self.points.first()?;
        Some(self.points.iter().fold((first, first), |(lo, hi), p| {
            (
                [lo[0].min(p[0]), lo[1].min(p[1]), lo[2].min(p[2])],
                [hi[0].max(p[0]), hi[1].max(p[1]), hi[2].max(p[2])],
            )
        }))
    }
}

fn las_err(e: las::Error) -> ProductError {
    ProductError::Format(format!("las: {e}"))
}

/// Coordinate quantum of the written LAZ files (1 mm).
const LAZ_SCALE: f64 = 0.001;

/// Writes `cloud` as compressed LAS 1.4: point format 7 (with RGB) when the
/// cloud has colours, else format 6. Scale 1 mm, offset at the minimum corner.
pub fn write_laz(path: &Path, cloud: &PointCloud) -> Result<(), ProductError> {
    if let Some(c) = &cloud.colors {
        if c.len() != cloud.points.len() {
            return Err(ProductError::Format(format!(
                "{} colours for {} points",
                c.len(),
                cloud.points.len()
            )));
        }
    }
    let (min, _) = cloud.bounds().unwrap_or(([0.0; 3], [0.0; 3]));
    let mut builder = Builder::from((1, 4));
    builder.point_format =
        Format::new(if cloud.colors.is_some() { 7 } else { 6 }).map_err(las_err)?;
    builder.point_format.is_compressed = true;
    builder.generating_software = "world-engine-node".to_string();
    let axis = |offset: f64| Transform {
        scale: LAZ_SCALE,
        offset: (offset / LAZ_SCALE).floor() * LAZ_SCALE,
    };
    builder.transforms = Vector {
        x: axis(min[0]),
        y: axis(min[1]),
        z: axis(min[2]),
    };
    let header = builder.into_header().map_err(las_err)?;
    let mut writer = Writer::new(BufWriter::new(File::create(path)?), header).map_err(las_err)?;
    for (i, p) in cloud.points.iter().enumerate() {
        let color = cloud.colors.as_ref().map(|c| Color {
            red: c[i][0],
            green: c[i][1],
            blue: c[i][2],
        });
        writer
            .write_point(Point {
                x: p[0],
                y: p[1],
                z: p[2],
                return_number: 1,
                number_of_returns: 1,
                gps_time: Some(0.0),
                color,
                ..Default::default()
            })
            .map_err(las_err)?;
    }
    writer.close().map_err(las_err)
}

/// Reads a LAS/LAZ file; colours are kept when the point format carries RGB.
pub fn read_laz(path: &Path) -> Result<PointCloud, ProductError> {
    let mut reader = Reader::from_path(path).map_err(las_err)?;
    let data = reader.read_all().map_err(las_err)?;
    let points = data
        .x()
        .zip(data.y())
        .zip(data.z())
        .map(|((x, y), z)| [x, y, z])
        .collect();
    let colors = data.rgb().map(|it| it.map(|(r, g, b)| [r, g, b]).collect());
    Ok(PointCloud { points, colors })
}

/// One point per occupied `voxel_m` cube: the centroid of its points, with the
/// mean colour. Output is ordered by voxel index for determinism.
pub fn voxel_downsample(cloud: &PointCloud, voxel_m: f64) -> PointCloud {
    if voxel_m.is_nan() || voxel_m <= 0.0 || cloud.points.is_empty() {
        return cloud.clone();
    }
    let mut acc: HashMap<[i64; 3], ([f64; 3], [u64; 3], u64)> = HashMap::new();
    for (i, p) in cloud.points.iter().enumerate() {
        let key = p.map(|v| (v / voxel_m).floor() as i64);
        let e = acc.entry(key).or_insert(([0.0; 3], [0; 3], 0));
        for (s, v) in e.0.iter_mut().zip(p) {
            *s += v;
        }
        if let Some(c) = &cloud.colors {
            for (s, v) in e.1.iter_mut().zip(c[i]) {
                *s += v as u64;
            }
        }
        e.2 += 1;
    }
    let mut cells: Vec<_> = acc.into_iter().collect();
    cells.sort_unstable_by_key(|(k, _)| *k);
    let n = |c: u64| c as f64;
    let points = cells
        .iter()
        .map(|(_, (s, _, c))| [s[0] / n(*c), s[1] / n(*c), s[2] / n(*c)])
        .collect();
    let colors = cloud.colors.as_ref().map(|_| {
        cells
            .iter()
            .map(|(_, (_, rgb, c))| rgb.map(|v| ((v + c / 2) / c) as u16))
            .collect()
    });
    PointCloud { points, colors }
}
