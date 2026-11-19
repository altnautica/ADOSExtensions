//! The point-cloud editor's job contract: the `edit` job's params, the
//! operations it runs, and the provenance file (`edits.json`) every edited
//! artifact carries.
//!
//! An `edit` job rewrites a cloud product on its node. Each operation that
//! rewrites points writes the next version of the artifact (`cloud.v1.laz`,
//! `cloud.v2.laz`, …) and never touches an earlier one; the provenance of a
//! version lists every operation that led to it, in order, and is never
//! rewritten. Selection happens in the browser; what reaches the node is the
//! selection's footprint on the ground plane (an XY polygon, with an optional
//! height band), which the node applies to the full-resolution cloud.

use serde::{Deserialize, Serialize};

/// Most vertices a selection polygon may have.
pub const MAX_POLYGON_VERTICES: usize = 256;

/// The `params` of an `edit` job.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EditParams {
    /// The job whose `out/` holds the product; the session's newest
    /// reconstruction when absent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub job: Option<String>,
    /// The artifact version the operations start from, relative to that job's
    /// directory (`out/cloud.laz`, `out/cloud.v2.laz`).
    pub source: String,
    pub ops: Vec<EditOp>,
}

/// One editor operation.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "op", rename_all = "snake_case")]
pub enum EditOp {
    /// Keep the points inside (or outside) a ground polygon.
    Crop {
        keep: Keep,
        polygon: Vec<[f64; 2]>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        z: Option<[f64; 2]>,
    },
    /// Remove the points inside a ground polygon.
    Delete {
        polygon: Vec<[f64; 2]>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        z: Option<[f64; 2]>,
    },
    /// Copy the points inside a source box to a target placement as a new
    /// layer.
    Patch {
        layer: String,
        source: PatchSource,
        target: Transform2,
    },
    /// Remove a patch layer and all of its points.
    DeleteLayer { layer: String },
    /// Reduce the point count to a target.
    Decimate(DecimateTarget),
    /// Write the current version in a delivery format, with its manifest.
    Export {
        format: ExportFormat,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        crs: Option<String>,
        #[serde(default, skip_serializing_if = "Option::is_none")]
        decimate: Option<DecimateTarget>,
    },
}

impl EditOp {
    /// The operation's name, as its stage row and provenance entry show it.
    pub fn name(&self) -> &'static str {
        match self {
            Self::Crop { .. } => "crop",
            Self::Delete { .. } => "delete",
            Self::Patch { .. } => "patch",
            Self::DeleteLayer { .. } => "delete_layer",
            Self::Decimate(_) => "decimate",
            Self::Export { .. } => "export",
        }
    }

    /// Whether the operation writes a new version of the cloud (an export
    /// writes a delivery file beside it instead).
    pub fn rewrites(&self) -> bool {
        !matches!(self, Self::Export { .. })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Keep {
    Inside,
    Outside,
}

/// The source box of a patch: two opposite XY corners.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct PatchSource {
    #[serde(rename = "box")]
    pub bbox: [[f64; 2]; 2],
}

impl PatchSource {
    /// `(min, max)` corners whatever order the box was dragged in.
    pub fn corners(&self) -> ([f64; 2], [f64; 2]) {
        let [a, b] = self.bbox;
        (
            [a[0].min(b[0]), a[1].min(b[1])],
            [a[0].max(b[0]), a[1].max(b[1])],
        )
    }

    pub fn contains(&self, p: [f64; 2]) -> bool {
        let (lo, hi) = self.corners();
        p[0] >= lo[0] && p[0] <= hi[0] && p[1] >= lo[1] && p[1] <= hi[1]
    }
}

/// A patch's placement: a rotation about the source box centre, then an XY
/// translation, in metres and degrees.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Transform2 {
    pub translate: [f64; 2],
    #[serde(default)]
    pub rotate_deg: f64,
}

/// Exactly one of a point count or a percentage of the current count.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct DecimateTarget {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target_points: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub target_percent: Option<f32>,
}

impl DecimateTarget {
    /// The point count this target asks of a cloud of `count` points.
    pub fn points_for(&self, count: u64) -> u64 {
        match (self.target_points, self.target_percent) {
            (Some(n), _) => n.min(count),
            (None, Some(p)) => ((count as f64) * f64::from(p) / 100.0).round() as u64,
            (None, None) => count,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExportFormat {
    Laz,
    Ply,
}

impl ExportFormat {
    pub fn extension(self) -> &'static str {
        match self {
            Self::Laz => "laz",
            Self::Ply => "ply",
        }
    }
}

/// The cloud-to-cloud deviation a decimation introduced, in metres.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Deviation {
    pub mean: f64,
    pub p95: f64,
}

/// One operation as provenance records it: the operation, what it changed and
/// the version it produced.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EditRecord {
    #[serde(flatten)]
    pub op: EditOp,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub removed: Option<u64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub added: Option<u64>,
    /// A patch whose target overlaps its own source box.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub self_overlap: Option<bool>,
    /// The layers a patch copied points from.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub source_layers: Vec<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub deviation_m: Option<Deviation>,
    /// The version (or, for an export, the delivery file) the operation wrote,
    /// relative to the product job's directory.
    pub result: String,
    /// RFC 3339 UTC.
    pub ts: String,
}

/// One layer of a version: the base reconstruction (id 0) or a patch. A
/// layer's points are contiguous in the file, in layer order, and carry the
/// layer id as their LAS point source id.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LayerCount {
    pub name: String,
    pub id: u16,
    pub points: u64,
}

/// The base layer's name.
pub const BASE_LAYER: &str = "base";

/// A version's provenance (`out/<stem>.v<N>.edits.json`, and `out/edits.json`
/// for the newest version written).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct EditsFile {
    pub version: u32,
    /// The version this file describes.
    pub product: String,
    /// The unedited artifact the chain starts from.
    pub source: String,
    /// `true` iff `ops` is non-empty.
    pub edited: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub crs: Option<String>,
    /// The product's layers, in file order.
    pub layers: Vec<LayerCount>,
    pub ops: Vec<EditRecord>,
}

/// Why an edit request is refused; each message is the one sentence the GCS
/// shows.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum EditError {
    #[error("The source must be a file under out/ in the product's job.")]
    BadSource,
    #[error("An edit needs at least one operation.")]
    NoOps,
    #[error("A selection polygon needs at least three corners.")]
    TooFewVertices,
    #[error("A selection polygon may have at most 256 corners.")]
    TooManyVertices,
    #[error("The selection polygon crosses itself; draw it without crossing lines.")]
    SelfIntersecting,
    #[error("A coordinate in the operation is not a finite number.")]
    NotFinite,
    #[error("The height band's lower limit must be below its upper limit.")]
    BadBand,
    #[error("The patch source box has no area.")]
    EmptyBox,
    #[error("A layer name is letters, digits and dashes, and the base layer cannot be named.")]
    BadLayer,
    #[error("Decimate takes either a point count or a percentage, not both and not neither.")]
    DecimateTarget,
    #[error("A decimation percentage is above 0 and below 100.")]
    DecimatePercent,
    #[error("A decimation point count is above zero.")]
    DecimatePoints,
}

impl EditParams {
    /// Check every field before the job is accepted.
    pub fn validate(&self) -> Result<(), EditError> {
        valid_source(&self.source)?;
        if self.ops.is_empty() {
            return Err(EditError::NoOps);
        }
        self.ops.iter().try_for_each(EditOp::validate)
    }
}

impl EditOp {
    pub fn validate(&self) -> Result<(), EditError> {
        match self {
            Self::Crop { polygon, z, .. } | Self::Delete { polygon, z } => {
                valid_polygon(polygon)?;
                valid_band(z)
            }
            Self::Patch {
                layer,
                source,
                target,
            } => {
                valid_layer(layer)?;
                let (lo, hi) = source.corners();
                if !lo
                    .iter()
                    .chain(&hi)
                    .chain(&target.translate)
                    .all(|v| v.is_finite())
                    || !target.rotate_deg.is_finite()
                {
                    return Err(EditError::NotFinite);
                }
                if hi[0] <= lo[0] || hi[1] <= lo[1] {
                    return Err(EditError::EmptyBox);
                }
                Ok(())
            }
            Self::DeleteLayer { layer } => valid_layer(layer),
            Self::Decimate(t) => t.validate(),
            Self::Export { decimate, .. } => {
                decimate.as_ref().map_or(Ok(()), DecimateTarget::validate)
            }
        }
    }
}

impl DecimateTarget {
    pub fn validate(&self) -> Result<(), EditError> {
        match (self.target_points, self.target_percent) {
            (Some(0), None) => Err(EditError::DecimatePoints),
            (Some(_), None) => Ok(()),
            (None, Some(p)) if p.is_finite() && p > 0.0 && p < 100.0 => Ok(()),
            (None, Some(_)) => Err(EditError::DecimatePercent),
            _ => Err(EditError::DecimateTarget),
        }
    }
}

/// A product path an edit may read: a file under `out/` with no `..`.
pub fn valid_source(source: &str) -> Result<(), EditError> {
    let rest = source.strip_prefix("out/").ok_or(EditError::BadSource)?;
    let ok = !rest.is_empty()
        && rest
            .split('/')
            .all(|c| !c.is_empty() && c != "." && c != ".." && !c.contains('\\'));
    ok.then_some(()).ok_or(EditError::BadSource)
}

fn valid_layer(layer: &str) -> Result<(), EditError> {
    let ok = layer != BASE_LAYER
        && !layer.is_empty()
        && layer.len() <= 64
        && layer.chars().all(|c| c.is_ascii_alphanumeric() || c == '-');
    ok.then_some(()).ok_or(EditError::BadLayer)
}

fn valid_band(z: &Option<[f64; 2]>) -> Result<(), EditError> {
    match z {
        Some([lo, hi]) if !lo.is_finite() || !hi.is_finite() => Err(EditError::NotFinite),
        Some([lo, hi]) if lo >= hi => Err(EditError::BadBand),
        _ => Ok(()),
    }
}

/// A polygon the node can apply: 3 to 256 finite vertices that never cross.
pub fn valid_polygon(polygon: &[[f64; 2]]) -> Result<(), EditError> {
    if polygon.len() < 3 {
        return Err(EditError::TooFewVertices);
    }
    if polygon.len() > MAX_POLYGON_VERTICES {
        return Err(EditError::TooManyVertices);
    }
    if !polygon.iter().flatten().all(|v| v.is_finite()) {
        return Err(EditError::NotFinite);
    }
    if self_intersects(polygon) {
        return Err(EditError::SelfIntersecting);
    }
    Ok(())
}

/// Whether two non-adjacent edges of the closed polygon cross or touch.
pub fn self_intersects(polygon: &[[f64; 2]]) -> bool {
    let n = polygon.len();
    let edge = |i: usize| (polygon[i], polygon[(i + 1) % n]);
    for i in 0..n {
        for j in i + 1..n {
            // Edges sharing a vertex are adjacent: i and i+1, and the last
            // edge with the first.
            if j == i + 1 || (i == 0 && j == n - 1) {
                continue;
            }
            let (a, b) = edge(i);
            let (c, d) = edge(j);
            if segments_touch(a, b, c, d) {
                return true;
            }
        }
    }
    false
}

fn orient(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

fn on_segment(a: [f64; 2], b: [f64; 2], p: [f64; 2]) -> bool {
    p[0] >= a[0].min(b[0])
        && p[0] <= a[0].max(b[0])
        && p[1] >= a[1].min(b[1])
        && p[1] <= a[1].max(b[1])
}

fn segments_touch(a: [f64; 2], b: [f64; 2], c: [f64; 2], d: [f64; 2]) -> bool {
    let (d1, d2) = (orient(c, d, a), orient(c, d, b));
    let (d3, d4) = (orient(a, b, c), orient(a, b, d));
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }
    (d1 == 0.0 && on_segment(c, d, a))
        || (d2 == 0.0 && on_segment(c, d, b))
        || (d3 == 0.0 && on_segment(a, b, c))
        || (d4 == 0.0 && on_segment(a, b, d))
}

/// Whether `p` is inside `polygon` (ray casting; the polygon closes
/// implicitly).
pub fn point_in_polygon(polygon: &[[f64; 2]], p: [f64; 2]) -> bool {
    let mut inside = false;
    let n = polygon.len();
    let mut j = n.wrapping_sub(1);
    for i in 0..n {
        let (a, b) = (polygon[i], polygon[j]);
        if (a[1] > p[1]) != (b[1] > p[1])
            && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

#[cfg(test)]
#[path = "edit_tests.rs"]
mod tests;
