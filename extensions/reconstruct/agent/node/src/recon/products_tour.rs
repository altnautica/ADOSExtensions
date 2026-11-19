//! The tour part of the `products` stage: the reconstructed keyframe poses,
//! the navigation nodes thinned from them, hero stills at each room hotspot,
//! and the static tour bundle with its `tour.json`.

use std::fs;
use std::path::{Path, PathBuf};

use ados_capture_contract::Placement;

use super::geom::Mat4;
use super::prepare::read_frames;
use super::products_stage::{art, ArtifactOut, ProductsInput};
use super::{colmap, train, StageError, ToolRunner};
use crate::products::tour;

/// Spacing of the tour's navigation nodes along the walked path, metres.
pub const NODE_SPACING_M: f64 = 1.5;

/// One reconstructed keyframe camera.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Keyframe {
    /// Camera centre, local ENU metres.
    pub centre: [f64; 3],
    /// Heading of the optical axis, degrees counter-clockwise from east.
    pub yaw_deg: f64,
}

impl Keyframe {
    pub fn from_pose(t_world_cam: &Mat4) -> Self {
        let m = t_world_cam;
        Self {
            centre: [m[3], m[7], m[11]],
            yaw_deg: m[6].atan2(m[2]).to_degrees(),
        }
    }
}

/// The registered keyframes of the job in capture order (one per capture
/// instant: the first camera of a rig); empty when the model has none.
pub fn keyframes(job_dir: &Path) -> Vec<Keyframe> {
    let Ok(poses) = colmap::read_image_poses(&job_dir.join("sfm").join("txt")) else {
        return Vec::new();
    };
    let mut frames = read_frames(&job_dir.join("dataset")).unwrap_or_default();
    frames.sort_by(|a, b| a.t_ns.cmp(&b.t_ns).then_with(|| a.name.cmp(&b.name)));
    let mut last_t = None;
    let mut out = Vec::new();
    for f in &frames {
        if last_t == Some(f.t_ns) {
            continue;
        }
        if let Some(p) = poses.get(&f.name) {
            last_t = Some(f.t_ns);
            out.push(Keyframe::from_pose(p));
        }
    }
    out
}

/// Navigation nodes: the keyframes in order, each kept when it is at least
/// `spacing` metres from every node kept so far.
pub fn nav_nodes(keyframes: &[Keyframe], spacing: f64) -> Vec<tour::TourNode> {
    let mut kept: Vec<Keyframe> = Vec::new();
    for k in keyframes {
        let far = kept.iter().all(|n| {
            let d = [0, 1, 2].map(|i| n.centre[i] - k.centre[i]);
            (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() >= spacing
        });
        if far {
            kept.push(*k);
        }
    }
    let mm = |v: f64| (v * 1000.0).round() / 1000.0 + 0.0;
    kept.iter()
        .enumerate()
        .map(|(i, k)| tour::TourNode {
            id: format!("n{i}"),
            position: k.centre.map(mm),
            yaw_deg: (k.yaw_deg * 10.0).round() / 10.0 + 0.0,
        })
        .collect()
}

/// A camera at `eye` looking horizontally at `target` (OpenCV axes: x right,
/// y down, z forward) as row-major `T_world_cam`.
pub fn look_at(eye: [f64; 3], target: [f64; 3]) -> [f64; 16] {
    let mut f = [target[0] - eye[0], target[1] - eye[1], 0.0];
    let n = (f[0] * f[0] + f[1] * f[1]).sqrt();
    if n < 1e-9 {
        f = [1.0, 0.0, 0.0];
    } else {
        f = [f[0] / n, f[1] / n, 0.0];
    }
    let y = [0.0, 0.0, -1.0];
    // x = y × z
    let x = [
        y[1] * f[2] - y[2] * f[1],
        y[2] * f[0] - y[0] * f[2],
        y[0] * f[1] - y[1] * f[0],
    ];
    [
        x[0], y[0], f[0], eye[0], x[1], y[1], f[1], eye[1], x[2], y[2], f[2], eye[2], 0.0, 0.0,
        0.0, 1.0,
    ]
}

/// Hero stills at each room hotspot, then the static tour bundle.
pub fn build_tour(
    runner: &dyn ToolRunner,
    input: &ProductsInput<'_>,
    out: &Path,
    plan: Option<&serde_json::Value>,
    keyframes: &[Keyframe],
    placement: &Placement,
    progress: &mut dyn FnMut(f32),
) -> Result<ArtifactOut, StageError> {
    let mut rooms = Vec::new();
    let mut poses = Vec::new();
    for fl in plan
        .and_then(|p| p["floors"].as_array())
        .cloned()
        .unwrap_or_default()
    {
        for r in fl["rooms"].as_array().cloned().unwrap_or_default() {
            let h: Vec<f64> = r["hotspot"]
                .as_array()
                .map(|a| a.iter().filter_map(|v| v.as_f64()).collect())
                .unwrap_or_default();
            if h.len() != 3 {
                continue;
            }
            let poly: Vec<[f64; 2]> =
                serde_json::from_value(r["polygon"].clone()).unwrap_or_default();
            let n = poly.len().max(1) as f64;
            let c = poly
                .iter()
                .fold([0.0, 0.0], |a, p| [a[0] + p[0] / n, a[1] + p[1] / n]);
            // Stand off the hotspot toward the room's far side for a wide view.
            let eye = [h[0] - 0.5 * (c[0] - h[0]), h[1] - 0.5 * (c[1] - h[1]), h[2]];
            let name = format!("room_{}", rooms.len());
            poses.push(serde_json::json!({
                "name": name, "T_world_cam": look_at(eye, [c[0], c[1], h[2]]),
                "width": 1600, "height": 1000, "fx": 800.0, "fy": 800.0, "cx": 800.0, "cy": 500.0,
            }));
            rooms.push(serde_json::json!({
                "name": r["name"], "floor": fl["floor"], "hotspot": h,
                "still": format!("stills/{name}.jpg"),
            }));
        }
    }
    let stills_dir = input.job_dir.join("stills");
    let _ = fs::remove_dir_all(&stills_dir);
    // Hero stills are rendered from the splat by gsplat. Brush renders only in
    // its viewer, so on a node without the gsplat kernels (or when a render
    // fails) the rooms carry `still: null` and the tour still ships.
    if !poses.is_empty() {
        let poses_json = input.job_dir.join("stills.json");
        fs::write(&poses_json, serde_json::to_vec_pretty(&poses)?)?;
        let rendered = train::gsplat_available(runner, input.job_dir).and_then(|()| {
            train::stills(
                runner,
                &input.job_dir.join("train").join("splat.ply"),
                &poses_json,
                &stills_dir,
                &mut |p| progress(0.7 + 0.2 * p),
            )
        });
        if let Err(e) = rendered {
            tracing::warn!(error = %e, "tour stills skipped");
        }
    }
    let mut files: Vec<(PathBuf, String)> = vec![(out.join("scene.spz"), "scene.spz".into())];
    // The ESDF travels with its header (`occupancy.json` beside it), which
    // the viewer reads for the grid's origin, resolution and size.
    let esdf = out.join("occupancy.esdf").is_file() && out.join("occupancy.json").is_file();
    for f in [
        "scene.rad",
        "collision.glb",
        "floorplan.json",
        "occupancy.esdf",
        "occupancy.json",
    ] {
        if out.join(f).is_file() && (esdf || !f.starts_with("occupancy")) {
            files.push((out.join(f), f.into()));
        }
    }
    for r in &mut rooms {
        let still = r["still"].as_str().unwrap_or_default().to_string();
        let src = stills_dir.join(still.trim_start_matches("stills/"));
        if src.is_file() {
            files.push((src, still));
        } else {
            r["still"] = serde_json::Value::Null;
        }
    }
    let manifest: tour::TourManifest = serde_json::from_value(serde_json::json!({
        "contract": ados_capture_contract::CONTRACT_ID,
        "title": input.title,
        "splat": "scene.spz",
        "lod": out.join("scene.rad").is_file().then_some("scene.rad"),
        "mesh": out.join("collision.glb").is_file().then_some("collision.glb"),
        "floorplan": out.join("floorplan.json").is_file().then_some("floorplan.json"),
        "esdf": esdf.then_some("occupancy.esdf"),
        "anchor": input.anchor,
        "placement": placement,
        "rooms": rooms,
        "nodes": nav_nodes(keyframes, NODE_SPACING_M),
    }))?;
    let zip =
        tour::build_bundle(out, input.tour_viewer, &manifest, &files).map_err(|e| match e {
            crate::products::ProductError::ViewerMissing(p) => StageError::tool_missing(
                "tour-viewer",
                format!("the tour viewer build is missing at {}", p.display()),
            ),
            other => StageError::io(other),
        })?;
    let mut a = art(
        "tour",
        &zip.file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_else(|| "tour.zip".into()),
        "zip",
        None,
    );
    a.meta = serde_json::json!({
        "rooms": manifest.rooms.len(),
        "nodes": manifest.nodes.len(),
    });
    Ok(a)
}

#[cfg(test)]
#[path = "products_tour_tests.rs"]
mod tests;
