//! Stage `products`: the deliverables of a session reconstruction in `out/`
//! — the splat as SPZ (and `.rad` LoD), the collision mesh, the LAZ point
//! cloud, the ESDF, the indoor products the preset asks for, a placement
//! sidecar per georeferenced artifact, and `outputs.json` listing them.

use std::fs;
use std::path::{Path, PathBuf};

use ados_capture_contract::{
    ArtifactDescriptor, ArtifactDescriptorFormat as Fmt, ArtifactDescriptorKind as Kind, GeoAnchor,
    Placement,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use super::prepare::read_rooms;
use super::preset::PresetSpec;
use super::{colmap, train, StageError, ToolInvocation, ToolRunner};
use crate::products::cloud::{read_laz, write_laz, PointCloud};
use crate::products::{aisles, floorplan, tour};

/// One deliverable listed in `out/outputs.json`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ArtifactOut {
    /// `splat`, `splat_lod`, `mesh`, `pointcloud`, `occupancy`, `floorplan`,
    /// `floorplan_svg`, `aisles`, `tour`, `rig`, `calib_report`, `change`,
    /// `change_summary`.
    pub kind: String,
    /// Path under `out/`.
    pub file: String,
    pub format: String,
    /// The placement sidecar beside it, when it is a placed artifact.
    pub place_json: Option<String>,
    #[serde(default)]
    pub meta: serde_json::Value,
}

/// Inputs of the stage.
pub struct ProductsInput<'a> {
    pub job_dir: &'a Path,
    pub session_id: &'a str,
    pub title: &'a str,
    /// The session anchor when the model is in its ENU frame.
    pub anchor: Option<GeoAnchor>,
    pub generation: u32,
    pub spec: &'a PresetSpec,
    pub tour_viewer: &'a Path,
}

/// The file name of the deliverables list.
pub const OUTPUTS_FILE: &str = "outputs.json";

pub fn sha256_file(path: &Path) -> Result<String, StageError> {
    let mut h = Sha256::new();
    let mut f = fs::File::open(path)?;
    std::io::copy(&mut f, &mut h)?;
    Ok(hex::encode(h.finalize()))
}

/// Write `<file>.place.json` for an artifact in `out/`; returns its name.
pub fn write_sidecar(
    out: &Path,
    file: &str,
    kind: Kind,
    format: Fmt,
    placement: &Placement,
    generation: u32,
    session_id: &str,
) -> Result<String, StageError> {
    let d = ArtifactDescriptor::new(
        kind,
        file,
        format,
        sha256_file(&out.join(file))?,
        placement.clone(),
        generation,
        session_id,
    );
    let name = format!("{file}.place.json");
    fs::write(out.join(&name), serde_json::to_vec_pretty(&d)?)?;
    Ok(name)
}

/// The merged world cloud: COLMAP points plus registered LiDAR points.
fn world_cloud(job_dir: &Path) -> Result<PointCloud, StageError> {
    let mut cloud = PointCloud {
        points: Vec::new(),
        colors: Some(Vec::new()),
    };
    let txt = job_dir.join("sfm").join("txt");
    for p in colmap::read_points(&txt)? {
        cloud.points.push(p.xyz);
        if let Some(c) = cloud.colors.as_mut() {
            c.push(p.rgb.map(|v| v as u16 * 257));
        }
    }
    let lidar = job_dir.join("dataset").join("lidar_points.laz");
    if lidar.is_file() {
        let l = read_laz(&lidar).map_err(StageError::io)?;
        let n = l.points.len();
        cloud.points.extend(l.points);
        if let Some(c) = cloud.colors.as_mut() {
            c.extend(std::iter::repeat_n([40000u16, 40000, 40000], n));
        }
    }
    Ok(cloud)
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

/// Run the stage.
pub fn run(
    runner: &dyn ToolRunner,
    input: &ProductsInput<'_>,
    progress: &mut dyn FnMut(f32),
) -> Result<Vec<ArtifactOut>, StageError> {
    let job = input.job_dir;
    let out = job.join("out");
    if out.exists() {
        fs::remove_dir_all(&out)?;
    }
    fs::create_dir_all(&out)?;
    let placement = Placement::identity(input.anchor.clone());
    let (sid, generation) = (input.session_id, input.generation);
    let mut arts = Vec::new();
    let place = |file: &str, kind: Kind, fmt: Fmt| {
        write_sidecar(&out, file, kind, fmt, &placement, generation, sid)
    };

    // Splat: the trained PLY, its SPZ, and the LoD tree when asked for.
    let splat_ply = job.join("train").join("splat.ply");
    fs::copy(&splat_ply, out.join("scene.ply"))?;
    let sidecar = place("scene.ply", Kind::Splat, Fmt::Ply)?;
    arts.push(art("splat", "scene.ply", "ply", Some(sidecar)));
    // PLY -> SPZ, dropping non-finite splats.
    runner.run(
        &ToolInvocation::new("splat-transform", &out)
            .args(["-w", "--no-tty"])
            .arg(&splat_ply)
            .arg("-N")
            .arg(out.join("scene.spz")),
        &mut |_| {},
    )?;
    let sidecar = place("scene.spz", Kind::Splat, Fmt::Spz)?;
    arts.push(art("splat", "scene.spz", "spz", Some(sidecar)));
    progress(0.2);
    if input.spec.lod {
        // build-lod has no output flag: it writes `<stem>-lod.rad` beside its
        // input, here `out/scene-lod.rad`.
        runner.run(
            &ToolInvocation::new("spark-build-lod", &out)
                .arg("--rad")
                .arg(out.join("scene.ply")),
            &mut |_| {},
        )?;
        fs::rename(out.join("scene-lod.rad"), out.join("scene.rad"))?;
        arts.push(art("splat_lod", "scene.rad", "rad", None));
    }

    // Collision mesh.
    let mesh_glb = job.join("mesh").join("mesh.glb");
    if mesh_glb.is_file() {
        fs::copy(&mesh_glb, out.join("collision.glb"))?;
        let sidecar = place("collision.glb", Kind::Mesh, Fmt::Glb)?;
        arts.push(art("mesh", "collision.glb", "glb", Some(sidecar)));
    }
    progress(0.35);

    // Point cloud + ESDF.
    let cloud = world_cloud(job)?;
    if !cloud.points.is_empty() {
        write_laz(&out.join("cloud.laz"), &cloud).map_err(StageError::io)?;
        let sidecar = place("cloud.laz", Kind::Pointcloud, Fmt::Laz)?;
        arts.push(art("pointcloud", "cloud.laz", "laz", Some(sidecar)));
        let pts: Vec<[f32; 3]> = cloud
            .points
            .iter()
            .map(|p| [p[0] as f32, p[1] as f32, p[2] as f32])
            .collect();
        match crate::esdf_from_points(
            &pts,
            crate::DEFAULT_ESDF_RESOLUTION_M,
            crate::DEFAULT_ESDF_TRUNCATION_M,
        ) {
            Ok(grid) => {
                fs::write(out.join("occupancy.esdf"), grid.to_le_bytes())?;
                let header = serde_json::json!({
                    "file": "occupancy.esdf", "layout": "f32le, x fastest",
                    "origin": grid.origin, "resolution_m": grid.resolution_m,
                    "dims": grid.dims, "truncation_m": grid.truncation_m,
                });
                fs::write(
                    out.join("occupancy.json"),
                    serde_json::to_vec_pretty(&header)?,
                )?;
                let sidecar = place("occupancy.esdf", Kind::Occupancy, Fmt::Esdf)?;
                let mut a = art("occupancy", "occupancy.json", "json", Some(sidecar));
                a.meta = serde_json::json!({ "grid": "occupancy.esdf" });
                arts.push(a);
            }
            Err(e) => tracing::warn!(error = %e, "esdf skipped"),
        }
    }
    progress(0.5);

    // Indoor products.
    let rooms = read_rooms(&job.join("dataset"))?;
    let mut plan_json: Option<serde_json::Value> = None;
    if input.spec.floorplan && !cloud.points.is_empty() {
        let tracks: Vec<floorplan::RoomTrack> = rooms
            .iter()
            .map(|r| floorplan::RoomTrack {
                name: r.name.clone(),
                floor: r.floor,
                positions: r.positions.clone(),
            })
            .collect();
        let plan = floorplan::build(&cloud, &tracks, &floorplan::FloorplanOptions::default());
        let value = serde_json::to_value(&plan)?;
        fs::write(
            out.join("floorplan.json"),
            serde_json::to_vec_pretty(&value)?,
        )?;
        arts.push(art("floorplan", "floorplan.json", "json", None));
        for fl in value["floors"].as_array().cloned().unwrap_or_default() {
            let floor = fl["floor"].as_i64().unwrap_or(0) as i32;
            if let Some(svg) = floorplan::to_svg(&plan, floor) {
                let name = format!("floorplan_{floor}.svg");
                fs::write(out.join(&name), svg)?;
                arts.push(art("floorplan_svg", &name, "svg", None));
            }
        }
        plan_json = Some(value);
    }
    if input.spec.aisles && !cloud.points.is_empty() {
        let floor_z = plan_json
            .as_ref()
            .and_then(|p| p["floors"][0]["z_m"].as_f64())
            .unwrap_or_else(|| low_percentile_z(&cloud));
        let found = aisles::detect(&cloud, floor_z);
        fs::write(
            out.join("aisles.geojson"),
            serde_json::to_vec_pretty(&aisles::to_geojson(&found))?,
        )?;
        let mut a = art("aisles", "aisles.geojson", "geojson", None);
        a.meta = serde_json::json!({ "aisles": found.len() });
        arts.push(a);
    }
    progress(0.7);
    if input.spec.tour {
        arts.push(build_tour(
            runner,
            input,
            &out,
            plan_json.as_ref(),
            &placement,
            progress,
        )?);
    }

    // The dataset export: COLMAP sparse model + contract transforms.json.
    super::export::write_export(job, input.anchor.as_ref(), &out)?;
    arts.push(art("export", "export.zip", "zip", None));

    // The session rig, for clients that place sensors or push it back.
    let rig = job.join("session-rig.json");
    if rig.is_file() {
        fs::copy(&rig, out.join("rig.json"))?;
        arts.push(art("rig", "rig.json", "json", None));
    }
    for a in &mut arts {
        let size = fs::metadata(out.join(&a.file))
            .map(|m| m.len())
            .unwrap_or(0);
        if let serde_json::Value::Object(m) = &mut a.meta {
            m.insert("size_bytes".into(), size.into());
        } else {
            a.meta = serde_json::json!({ "size_bytes": size });
        }
    }
    fs::write(out.join(OUTPUTS_FILE), serde_json::to_vec_pretty(&arts)?)?;
    progress(1.0);
    Ok(arts)
}

fn art(kind: &str, file: &str, format: &str, place_json: Option<String>) -> ArtifactOut {
    ArtifactOut {
        kind: kind.into(),
        file: file.into(),
        format: format.into(),
        place_json,
        meta: serde_json::Value::Null,
    }
}

/// The 2nd-percentile height, a floor estimate when no floorplan ran.
fn low_percentile_z(cloud: &PointCloud) -> f64 {
    let mut z: Vec<f64> = cloud.points.iter().map(|p| p[2]).collect();
    z.sort_by(|a, b| a.total_cmp(b));
    z[z.len() / 50]
}

/// Hero stills at each room hotspot, then the static tour bundle.
fn build_tour(
    runner: &dyn ToolRunner,
    input: &ProductsInput<'_>,
    out: &Path,
    plan: Option<&serde_json::Value>,
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
    for (f, opt) in [
        ("scene.rad", true),
        ("collision.glb", true),
        ("floorplan.json", true),
    ] {
        if opt && out.join(f).is_file() {
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
        "anchor": input.anchor,
        "placement": placement,
        "rooms": rooms,
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
    a.meta = serde_json::json!({ "rooms": manifest.rooms.len() });
    Ok(a)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn look_at_points_the_optical_axis_at_the_target_with_y_down() {
        let t = look_at([0.0, 0.0, 1.6], [3.0, 0.0, 1.6]);
        // z (third column) is the forward axis.
        assert_eq!([t[2], t[6], t[10]], [1.0, 0.0, 0.0]);
        // y (second column) points down.
        assert_eq!([t[1], t[5], t[9]], [0.0, 0.0, -1.0]);
        // x = y × z is right-handed: looking +x with z up, right is -y.
        assert_eq!([t[0], t[4], t[8]], [0.0, -1.0, 0.0]);
        assert_eq!([t[3], t[7], t[11]], [0.0, 0.0, 1.6]);
    }
}
