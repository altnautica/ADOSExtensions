//! The staged `edit` job: editor operations applied to a cloud product on its
//! node, one stage per operation.
//!
//! The product's job directory holds every version: the reconstruction's
//! `out/cloud.laz`, then `out/cloud.v1.laz`, `out/cloud.v2.laz`, … one per
//! operation that rewrites points, each with its provenance beside it
//! (`out/cloud.v<N>.edits.json`, written once) and `out/edits.json` naming the
//! newest. A version is never rewritten, so undo is opening an older one. The
//! edit job's own `out/outputs.json` lists the versions and exports it wrote.

use std::fs;
use std::io::Write;
use std::path::{Path, PathBuf};

use ados_mapping_protocol::edit::{
    EditOp, EditParams, EditRecord, EditsFile, LayerCount, BASE_LAYER,
};

use super::edit_export::{self, frame_label};
use super::edit_ops::{self, Cancelled, Layered};
use super::pipeline::{RunCtx, Stages};
use super::products_stage::{ArtifactOut, OUTPUTS_FILE};
use super::{job_dir, StageError};
use crate::products::cloud;

/// Largest cloud the editor loads in one piece.
pub const MAX_EDIT_POINTS: u64 = 150_000_000;
/// The newest version's provenance.
pub const EDITS_FILE: &str = "edits.json";

/// A version loaded for editing.
pub(super) struct Loaded {
    /// The version's path in the product job (`out/cloud.v2.laz`).
    pub rel: String,
    pub data: Layered,
    pub edits: EditsFile,
}

fn cancelled_err(_: Cancelled) -> StageError {
    StageError::new("E_CANCELLED", "the job was cancelled")
}

/// `cloud` for `out/cloud.laz` and `out/cloud.v3.laz`.
pub fn stem_of(rel: &str) -> String {
    let name = Path::new(rel)
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    match name.rsplit_once(".v") {
        Some((stem, n)) if !n.is_empty() && n.chars().all(|c| c.is_ascii_digit()) => stem.into(),
        _ => name,
    }
}

/// The provenance file of a version (`out/cloud.v2.laz` →
/// `out/cloud.v2.edits.json`).
pub fn provenance_of(rel: &str) -> String {
    let p = Path::new(rel);
    let stem = p
        .file_stem()
        .map(|s| s.to_string_lossy())
        .unwrap_or_default();
    let parent = p.parent().map(|d| d.to_string_lossy()).unwrap_or_default();
    format!("{parent}/{stem}.edits.json")
}

/// The next unused version number of `stem` in `out`.
fn next_version(out: &Path, stem: &str) -> u32 {
    let prefix = format!("{stem}.v");
    let highest = fs::read_dir(out)
        .into_iter()
        .flatten()
        .flatten()
        .filter_map(|e| {
            let name = e.file_name().to_string_lossy().into_owned();
            let n = name
                .strip_prefix(&prefix)?
                .strip_suffix(".laz")?
                .to_string();
            n.parse::<u32>().ok()
        })
        .max()
        .unwrap_or(0);
    highest + 1
}

/// Write `bytes` to `path` through a temp file, synced, then renamed.
pub(super) fn write_atomic(path: &Path, bytes: &[u8]) -> Result<(), StageError> {
    let tmp = path.with_extension("tmp");
    let mut f = fs::File::create(&tmp)?;
    f.write_all(bytes)?;
    f.sync_all()?;
    fs::rename(&tmp, path)?;
    Ok(())
}

/// Load `rel` from the product job with its provenance (an unedited artifact
/// starts an empty one).
pub(super) fn load(product: &Path, rel: &str) -> Result<Loaded, StageError> {
    let path = product.join(rel);
    if !path.is_file() {
        return Err(StageError::new(
            "E_SOURCE_MISSING",
            "The source artifact is missing on this node.",
        ));
    }
    let count = las::Reader::from_path(&path)
        .map(|r| r.header().number_of_points())
        .map_err(StageError::io)?;
    if count > MAX_EDIT_POINTS {
        return Err(StageError::new(
            "E_TOO_LARGE",
            "This cloud is too large to edit in one piece; crop it in a smaller job first.",
        ));
    }
    let (cloud, layers) = cloud::read_laz_layers(&path).map_err(StageError::io)?;
    let data = Layered { cloud, layers };
    let prov = product.join(provenance_of(rel));
    let edits = match fs::read(&prov) {
        Ok(b) => serde_json::from_slice(&b)?,
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => EditsFile {
            version: 1,
            product: rel.into(),
            source: rel.into(),
            edited: false,
            crs: frame_label(product, rel),
            layers: data
                .counts()
                .into_iter()
                .map(|(id, points)| LayerCount {
                    name: if id == 0 {
                        BASE_LAYER.into()
                    } else {
                        format!("layer-{id}")
                    },
                    id,
                    points,
                })
                .collect(),
            ops: Vec::new(),
        },
        Err(e) => return Err(e.into()),
    };
    Ok(Loaded {
        rel: rel.into(),
        data,
        edits,
    })
}

/// Now as RFC 3339 UTC (`2026-11-18T09:41:12Z`).
pub(super) fn now_rfc3339() -> String {
    let s = crate::calib::utc_stamp(std::time::SystemTime::now());
    format!(
        "{}-{}-{}T{}:{}:{}Z",
        &s[0..4],
        &s[4..6],
        &s[6..8],
        &s[9..11],
        &s[11..13],
        &s[13..15]
    )
}

/// The layer table after an operation: counts re-read from the cloud, names
/// kept, a new layer added by name.
fn relayer(before: &[LayerCount], data: &Layered, new: Option<(&str, u16)>) -> Vec<LayerCount> {
    data.counts()
        .into_iter()
        .map(|(id, points)| {
            let name = before
                .iter()
                .find(|l| l.id == id)
                .map(|l| l.name.clone())
                .or_else(|| new.filter(|n| n.1 == id).map(|n| n.0.to_string()))
                .unwrap_or_else(|| format!("layer-{id}"));
            LayerCount { name, id, points }
        })
        .collect()
}

/// Apply one rewriting operation to `cur`, write the next version and its
/// provenance, and return the record.
fn apply(
    product: &Path,
    cur: &mut Loaded,
    op: &EditOp,
    cancelled: &dyn Fn() -> bool,
) -> Result<EditRecord, StageError> {
    let mut rec = EditRecord {
        op: op.clone(),
        removed: None,
        added: None,
        self_overlap: None,
        source_layers: Vec::new(),
        deviation_m: None,
        result: String::new(),
        ts: String::new(),
    };
    let mut new_layer = None;
    match op {
        EditOp::Crop { keep, polygon, z } => {
            rec.removed = Some(
                edit_ops::crop(&mut cur.data, *keep, polygon, *z, cancelled)
                    .map_err(cancelled_err)?,
            );
        }
        EditOp::Delete { polygon, z } => {
            rec.removed = Some(
                edit_ops::delete(&mut cur.data, polygon, *z, cancelled).map_err(cancelled_err)?,
            );
        }
        EditOp::Patch {
            layer,
            source,
            target,
        } => {
            if cur.edits.layers.iter().any(|l| &l.name == layer) {
                return Err(StageError::input(format!(
                    "A layer named {layer} already exists."
                )));
            }
            let id = cur.edits.layers.iter().map(|l| l.id).max().unwrap_or(0) + 1;
            let p = edit_ops::patch(&mut cur.data, id, source, target);
            if p.added == 0 {
                return Err(StageError::input("The patch source box holds no points."));
            }
            rec.added = Some(p.added);
            rec.self_overlap = Some(p.self_overlap);
            rec.source_layers = p
                .source_layers
                .iter()
                .filter_map(|id| {
                    cur.edits
                        .layers
                        .iter()
                        .find(|l| l.id == *id)
                        .map(|l| l.name.clone())
                })
                .collect();
            new_layer = Some((layer.as_str(), id));
        }
        EditOp::DeleteLayer { layer } => {
            let Some(id) = cur
                .edits
                .layers
                .iter()
                .find(|l| &l.name == layer)
                .map(|l| l.id)
            else {
                return Err(StageError::input(format!(
                    "This version has no layer named {layer}."
                )));
            };
            let live = |name: &str| cur.edits.layers.iter().any(|l| l.name == name);
            let dependent = cur.edits.ops.iter().find_map(|r| match &r.op {
                EditOp::Patch { layer: made, .. }
                    if r.source_layers.iter().any(|s| s == layer) && live(made) =>
                {
                    Some(made.clone())
                }
                _ => None,
            });
            if let Some(d) = dependent {
                return Err(StageError::input(format!(
                    "{d} was made from this layer; delete {d} first."
                )));
            }
            rec.removed =
                Some(edit_ops::delete_layer(&mut cur.data, id, cancelled).map_err(cancelled_err)?);
        }
        EditOp::Decimate(target) => {
            let (out, _) =
                edit_ops::decimate(&cur.data, target, cancelled).map_err(cancelled_err)?;
            rec.deviation_m = edit_ops::deviation(&cur.data.cloud, &out.cloud);
            rec.removed = Some((cur.data.len() - out.len()) as u64);
            cur.data = out;
        }
        EditOp::Export { .. } => unreachable!("an export writes no version"),
    }
    if cancelled() {
        return Err(cancelled_err(Cancelled));
    }
    let out = product.join("out");
    let stem = stem_of(&cur.rel);
    let name = format!("{stem}.v{}.laz", next_version(&out, &stem));
    let rel = format!("out/{name}");
    let tmp = out.join(format!("{name}.tmp"));
    let written = cloud::write_laz_layers(&tmp, &cur.data.cloud, Some(&cur.data.layers));
    if let Err(e) = written {
        let _ = fs::remove_file(&tmp);
        return Err(StageError::io(e));
    }
    fs::rename(&tmp, out.join(&name))?;
    // A version sits in the same frame as the artifact it came from.
    let place = |r: &str| product.join(format!("{r}.place.json"));
    if place(&cur.rel).is_file() {
        fs::copy(place(&cur.rel), place(&rel))?;
    }
    rec.result = rel.clone();
    rec.ts = now_rfc3339();
    cur.edits.layers = relayer(&cur.edits.layers, &cur.data, new_layer);
    cur.edits.ops.push(rec.clone());
    cur.edits.product = rel.clone();
    cur.edits.edited = true;
    let body = serde_json::to_vec_pretty(&cur.edits)?;
    write_atomic(&product.join(provenance_of(&rel)), &body)?;
    write_atomic(&out.join(EDITS_FILE), &body)?;
    cur.rel = rel;
    Ok(rec)
}

/// Stage ids: each operation's name, numbered from the second of a kind.
fn stage_ids(ops: &[EditOp]) -> Vec<String> {
    let mut seen = std::collections::HashMap::new();
    ops.iter()
        .map(|op| {
            let n = seen.entry(op.name()).or_insert(0);
            *n += 1;
            if *n == 1 {
                op.name().to_string()
            } else {
                format!("{}-{n}", op.name())
            }
        })
        .collect()
}

pub(super) fn edit_job(
    dir: &Path,
    ctx: &RunCtx<'_>,
    product_job: &str,
    params: &EditParams,
) -> Result<Vec<ArtifactOut>, StageError> {
    let product: PathBuf = job_dir(ctx.work_root, product_job)?;
    let ids = stage_ids(&params.ops);
    let id_refs: Vec<&str> = ids.iter().map(String::as_str).collect();
    let stages = Stages::new(ctx.sink, &id_refs);
    let cancelled = || ctx.sink.cancelled();
    let mut rel = params.source.clone();
    let mut cur: Option<Loaded> = None;
    let mut arts = Vec::new();
    for (op, id) in params.ops.iter().zip(&ids) {
        let detail = stages.run(id, |progress| {
            if cur.as_ref().is_none_or(|c| c.rel != rel) {
                cur = Some(load(&product, &rel)?);
            }
            let Some(c) = cur.as_mut() else {
                unreachable!("loaded above")
            };
            progress(0.1);
            let rec = match op {
                EditOp::Export {
                    format,
                    crs,
                    decimate,
                } => edit_export::export(
                    &product,
                    c,
                    *format,
                    crs.as_deref(),
                    decimate.as_ref(),
                    &cancelled,
                )?,
                _ => apply(&product, c, op, &cancelled)?,
            };
            Ok(serde_json::json!({ "record": rec, "points": c.data.len() }))
        })?;
        let rec: EditRecord = serde_json::from_value(detail["record"].clone())?;
        let file = rec
            .result
            .strip_prefix("out/")
            .unwrap_or(&rec.result)
            .to_string();
        let mut art = ArtifactOut {
            kind: if op.rewrites() {
                "pointcloud".into()
            } else {
                "export".into()
            },
            file,
            format: Path::new(&rec.result)
                .extension()
                .map(|e| e.to_string_lossy().into_owned())
                .unwrap_or_default(),
            place_json: None,
            meta: serde_json::json!({
                "points": detail["points"],
                "edited": true,
                "op": op.name(),
                "size_bytes": fs::metadata(product.join(&rec.result)).map(|m| m.len()).unwrap_or(0),
            }),
            job: Some(product_job.to_string()),
        };
        if op.rewrites() {
            rel = rec.result.clone();
            if product.join(format!("{rel}.place.json")).is_file() {
                art.place_json = Some(format!("{}.place.json", art.file));
            }
        }
        arts.push(art);
    }
    let out = dir.join("out");
    fs::create_dir_all(&out)?;
    fs::write(out.join(OUTPUTS_FILE), serde_json::to_vec_pretty(&arts)?)?;
    Ok(arts)
}

#[cfg(test)]
#[path = "edit_tests.rs"]
mod tests;
