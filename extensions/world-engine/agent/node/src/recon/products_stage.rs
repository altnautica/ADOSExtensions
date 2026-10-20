//! Stage `products`: the deliverables of a session reconstruction in `out/`
//! — the splat filtered and as SPZ (and `.rad` LoD), the collision mesh, the
//! LAZ point cloud, the ESDF, the indoor products the preset asks for, the
//! tour (`products_tour`), a placement sidecar per georeferenced artifact,
//! and `outputs.json` listing them.

use std::fs;
use std::path::Path;

use ados_capture_contract::{
    ArtifactDescriptor, ArtifactDescriptorFormat as Fmt, ArtifactDescriptorKind as Kind, GeoAnchor,
    Placement,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

use super::prepare::{read_rooms, LIDAR_ORIGINS_FILE};
use super::preset::PresetSpec;
use super::products_tour::{self, Keyframe};
use super::{colmap, StageError, ToolInvocation, ToolRunner};
use crate::products::cloud::{read_laz, read_ply_mesh, sample_surface, write_laz, PointCloud};
use crate::products::{aisles, floorplan};

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

/// The splat box kept by the export filter: the keyframe camera centres'
/// bounds widened by 3 m across and from 1 m below to 3 m above, as
/// splat-transform's `min,max` corner list. `None` without keyframes.
pub fn splat_box(keyframes: &[Keyframe]) -> Option<String> {
    let first = keyframes.first()?.centre;
    let (lo, hi) = keyframes.iter().fold((first, first), |(lo, hi), k| {
        (
            [0, 1, 2].map(|i| lo[i].min(k.centre[i])),
            [0, 1, 2].map(|i| hi[i].max(k.centre[i])),
        )
    });
    let r = |v: f64| (v * 1000.0).round() / 1000.0 + 0.0;
    Some(format!(
        "{},{},{},{},{},{}",
        r(lo[0] - 3.0),
        r(lo[1] - 3.0),
        r(lo[2] - 1.0),
        r(hi[0] + 3.0),
        r(hi[1] + 3.0),
        r(hi[2] + 3.0)
    ))
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
    let keyframes = products_tour::keyframes(job);

    // Splat: the trained PLY filtered, its SPZ, and the LoD tree when asked
    // for. The filter drops non-finite splats, everything outside the walked
    // box, and floaters that add to no solid 5 cm voxel at 0.1 opacity.
    let splat_ply = job.join("train").join("splat.ply");
    let mut filter = ToolInvocation::new("splat-transform", &out)
        .args(["-w", "--no-tty"])
        .arg(&splat_ply)
        .arg("-N");
    if let Some(b) = splat_box(&keyframes) {
        filter = filter.arg("-B").arg(b);
    }
    runner.run(
        &filter.args(["-F", "0.05,0.1"]).arg(out.join("scene.ply")),
        &mut |_| {},
    )?;
    let sidecar = place("scene.ply", Kind::Splat, Fmt::Ply)?;
    arts.push(art("splat", "scene.ply", "ply", Some(sidecar)));
    // PLY -> SPZ. Version 4 (ZSTD streams) is named, not left to the
    // converter's default: the viewer (Spark >= 2.4) and the world-engine
    // tiler read it.
    runner.run(
        &ToolInvocation::new("splat-transform", &out)
            .args(["-w", "--no-tty", "--spz-version", "4"])
            .arg(out.join("scene.ply"))
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
        let plan_cloud = floorplan_cloud(job, &cloud);
        let mut origins: Vec<[f64; 3]> = keyframes.iter().map(|k| k.centre).collect();
        origins.extend(read_lidar_origins(&job.join("dataset"))?);
        let plan = floorplan::build(
            &plan_cloud,
            &tracks,
            &origins,
            &floorplan::FloorplanOptions::default(),
        );
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
        let cross = found
            .iter()
            .filter(|a| a.kind == aisles::AisleKind::CrossAisle)
            .count();
        let mut a = art("aisles", "aisles.geojson", "geojson", None);
        a.meta = serde_json::json!({ "aisles": found.len() - cross, "cross_aisles": cross });
        arts.push(a);
    }
    progress(0.7);
    if input.spec.tour {
        arts.push(products_tour::build_tour(
            runner,
            input,
            &out,
            plan_json.as_ref(),
            &keyframes,
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

pub(super) fn art(kind: &str, file: &str, format: &str, place_json: Option<String>) -> ArtifactOut {
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

/// Surface samples of the extracted mesh per square metre for the floor
/// plan (one per 4 cm²): a camera-only capture of plain walls has few sparse
/// points on them but a meshed surface.
const PLAN_MESH_AREA_PER_POINT_M2: f64 = 0.0004;

/// The floor plan's cloud: the world cloud plus points sampled from the mesh
/// when the job has one.
fn floorplan_cloud(job_dir: &Path, cloud: &PointCloud) -> PointCloud {
    let mut points = cloud.points.clone();
    let mesh_ply = job_dir.join("mesh").join("mesh.ply");
    if mesh_ply.is_file() {
        match read_ply_mesh(&mesh_ply) {
            Ok(mesh) => points.extend(sample_surface(&mesh, PLAN_MESH_AREA_PER_POINT_M2)),
            Err(e) => tracing::warn!(error = %e, "mesh samples skipped for the floor plan"),
        }
    }
    PointCloud {
        points,
        colors: None,
    }
}

/// LiDAR scan origins the prepare stage recorded (empty without LiDAR).
fn read_lidar_origins(dataset_dir: &Path) -> Result<Vec<[f64; 3]>, StageError> {
    match fs::read(dataset_dir.join(LIDAR_ORIGINS_FILE)) {
        Ok(b) => Ok(serde_json::from_slice(&b)?),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Vec::new()),
        Err(e) => Err(e.into()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splat_box_widens_the_walked_bounds() {
        let k = |x: f64, y: f64, z: f64| Keyframe {
            centre: [x, y, z],
            yaw_deg: 0.0,
        };
        assert_eq!(splat_box(&[]), None);
        assert_eq!(
            splat_box(&[k(0.0, 0.0, 1.5), k(10.0, -4.0, 1.7)]).unwrap(),
            "-3,-7,0.5,13,3,4.7"
        );
    }
}
