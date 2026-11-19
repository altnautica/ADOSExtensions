//! Walkthrough tour bundle: the static viewer, the scene files and
//! `tour.json`, staged under `<out>/tour/` and zipped to `<out>/tour.zip`.

use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::{Component, Path, PathBuf};

use ados_capture_contract::{GeoAnchor, Placement};
use serde::{Deserialize, Serialize};
use zip::write::SimpleFileOptions;
use zip::{CompressionMethod, ZipWriter};

use super::ProductError;

/// `tour.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TourManifest {
    pub contract: String,
    pub title: String,
    pub splat: String,
    pub lod: Option<String>,
    pub mesh: Option<String>,
    pub floorplan: Option<String>,
    /// The occupancy ESDF (`f32` little-endian, x fastest) walking collides
    /// with; its header (origin, resolution, dims) is the `.json` of the same
    /// stem beside it.
    #[serde(default)]
    pub esdf: Option<String>,
    pub anchor: Option<GeoAnchor>,
    pub placement: Option<Placement>,
    pub rooms: Vec<TourRoom>,
    /// Places along the walked path the viewer offers as jump targets.
    #[serde(default)]
    pub nodes: Vec<TourNode>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TourRoom {
    pub name: String,
    pub floor: i32,
    pub hotspot: [f64; 3],
    pub still: Option<String>,
}

/// A navigation node: a keyframe camera position (local ENU metres) and the
/// heading it looked along (degrees counter-clockwise from east).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct TourNode {
    pub id: String,
    pub position: [f64; 3],
    pub yaw_deg: f64,
}

fn safe_rel(rel: &str) -> Result<PathBuf, ProductError> {
    let p = Path::new(rel);
    let ok = !rel.is_empty() && p.components().all(|c| matches!(c, Component::Normal(_)));
    if ok {
        Ok(p.to_path_buf())
    } else {
        Err(ProductError::Format(format!(
            "bundle path {rel:?} is not a plain relative path"
        )))
    }
}

fn copy_tree(src: &Path, dst: &Path) -> Result<(), ProductError> {
    fs::create_dir_all(dst)?;
    for entry in fs::read_dir(src)? {
        let entry = entry?;
        let to = dst.join(entry.file_name());
        if entry.file_type()?.is_dir() {
            copy_tree(&entry.path(), &to)?;
        } else {
            fs::copy(entry.path(), to)?;
        }
    }
    Ok(())
}

/// Every file under `dir`, as sorted `/`-separated paths relative to `dir`.
fn list_files(
    dir: &Path,
    prefix: &str,
    out: &mut Vec<(String, PathBuf)>,
) -> Result<(), ProductError> {
    let mut entries: Vec<_> = fs::read_dir(dir)?.collect::<Result<_, _>>()?;
    entries.sort_by_key(|e| e.file_name());
    for e in entries {
        let name = format!("{prefix}{}", e.file_name().to_string_lossy());
        if e.file_type()?.is_dir() {
            list_files(&e.path(), &format!("{name}/"), out)?;
        } else {
            out.push((name, e.path()));
        }
    }
    Ok(())
}

/// Stages `out_dir/tour/` (a fresh copy of the viewer dist, `files` copied to
/// their relative names, `tour.json`) and writes `out_dir/tour.zip` with paths
/// relative to `tour/`. Returns the zip path.
pub fn build_bundle(
    out_dir: &Path,
    viewer_dist: &Path,
    manifest: &TourManifest,
    files: &[(PathBuf, String)],
) -> Result<PathBuf, ProductError> {
    if !viewer_dist.join("index.html").is_file() {
        return Err(ProductError::ViewerMissing(viewer_dist.to_path_buf()));
    }
    let tour = out_dir.join("tour");
    if tour.exists() {
        fs::remove_dir_all(&tour)?;
    }
    copy_tree(viewer_dist, &tour)?;
    for (src, rel) in files {
        let dst = tour.join(safe_rel(rel)?);
        if let Some(parent) = dst.parent() {
            fs::create_dir_all(parent)?;
        }
        fs::copy(src, &dst)?;
    }
    let json = serde_json::to_vec_pretty(manifest)
        .map_err(|e| ProductError::Format(format!("tour.json: {e}")))?;
    fs::write(tour.join("tour.json"), json)?;

    let mut entries = Vec::new();
    list_files(&tour, "", &mut entries)?;
    let zip_path = out_dir.join("tour.zip");
    let mut zip = ZipWriter::new(BufWriter::new(File::create(&zip_path)?));
    let opts = SimpleFileOptions::default()
        .compression_method(CompressionMethod::Deflated)
        .large_file(true);
    let zip_err = |e: zip::result::ZipError| ProductError::Format(format!("tour.zip: {e}"));
    for (name, path) in entries {
        zip.start_file(name, opts).map_err(zip_err)?;
        std::io::copy(&mut File::open(path)?, &mut zip)?;
    }
    zip.finish().map_err(zip_err)?.flush()?;
    Ok(zip_path)
}
