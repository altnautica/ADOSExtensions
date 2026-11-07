//! Stage `products`: the deliverables of a session reconstruction in `out/`
//! — the splat filtered and as SPZ (and `.rad` LoD), the collision mesh, the
//! LAZ point cloud, the ESDF, the indoor products the preset asks for, the
//! tour (`products_tour`), a placement sidecar per georeferenced artifact,
//! and `outputs.json` listing them.

use std::ffi::OsString;
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

/// The keyframe camera centres' bounds widened by 3 m across and from 1 m
/// below to 3 m above. `None` without keyframes.
fn walked_box(keyframes: &[Keyframe]) -> Option<([f64; 3], [f64; 3])> {
    let first = keyframes.first()?.centre;
    let (lo, hi) = keyframes.iter().fold((first, first), |(lo, hi), k| {
        (
            [0, 1, 2].map(|i| lo[i].min(k.centre[i])),
            [0, 1, 2].map(|i| hi[i].max(k.centre[i])),
        )
    });
    Some((
        [lo[0] - 3.0, lo[1] - 3.0, lo[2] - 1.0],
        [hi[0] + 3.0, hi[1] + 3.0, hi[2] + 3.0],
    ))
}

/// The share of the reconstructed cloud left outside the floater box at each
/// end of each axis.
const SPLAT_BOX_CLOUD_TAIL: f64 = 0.01;

/// The box the floater filter runs in, in the model frame: the walked box
/// grown to the reconstructed cloud's 1st to 99th percentile on each axis, so
/// the ground below a drone and the scenery its cameras saw are filtered. The
/// far tail (Gaussians where no point was triangulated: distant scenery and
/// sky) stays out of it, or it would stretch the filter's voxel grid past what
/// it can finish. `None` without keyframes.
fn splat_bounds(keyframes: &[Keyframe], cloud: &[[f64; 3]]) -> Option<([f64; 3], [f64; 3])> {
    let (mut lo, mut hi) = walked_box(keyframes)?;
    let mut axis = Vec::with_capacity(cloud.len());
    for i in 0..3 {
        axis.clear();
        axis.extend(cloud.iter().map(|p| p[i]).filter(|v| v.is_finite()));
        let Some(last) = axis.len().checked_sub(1) else {
            continue;
        };
        let k = |q: f64| (last as f64 * q).round() as usize;
        let (k_lo, k_hi) = (k(SPLAT_BOX_CLOUD_TAIL), k(1.0 - SPLAT_BOX_CLOUD_TAIL));
        lo[i] = lo[i].min(*axis.select_nth_unstable_by(k_lo, f64::total_cmp).1);
        hi[i] = hi[i].max(*axis.select_nth_unstable_by(k_hi, f64::total_cmp).1);
    }
    Some((lo, hi))
}

/// [`splat_bounds`] in splat-transform 3.10.1's engine frame, where both
/// `-B` and `-V` read a PLY's (x, y, z) as (-x, -y, z): the x and y bounds
/// are negated and swapped (or the filter works on the mirror image of the
/// site), rounded to the millimetre.
fn engine_box(keyframes: &[Keyframe], cloud: &[[f64; 3]]) -> Option<([f64; 3], [f64; 3])> {
    let (lo, hi) = splat_bounds(keyframes, cloud)?;
    let r = |v: f64| (v * 1000.0).round() / 1000.0 + 0.0;
    Some((
        [r(-hi[0]), r(-hi[1]), r(lo[2])],
        [r(-lo[0]), r(-lo[1]), r(hi[2])],
    ))
}

/// splat-transform's inputs for the exported splat (everything before the
/// output path). Non-finite splats go everywhere. With a floater box the
/// trained PLY is read seven times: once cut to the box and floater-filtered
/// (`-F` at `voxel_m`, 0.1 opacity), and once per slab of the space outside
/// it, unfiltered. The six slabs are disjoint and cover all the box leaves:
/// x below and above it; within its x, y below and above; within its x and
/// y, z below and above. Cutting the scene to the box instead cost 4.4 dB of
/// held-out PSNR on a 270 m outdoor walk (18.0 to 13.6 dB): the distant
/// scenery and sky the views see lie outside it. Without keyframes the whole
/// PLY is filtered.
fn splat_filter_args(
    splat_ply: &Path,
    keyframes: &[Keyframe],
    cloud: &[[f64; 3]],
    voxel_m: f64,
) -> Vec<OsString> {
    let ply = || OsString::from(splat_ply);
    let floater = OsString::from(format!("{voxel_m},0.1"));
    let Some((lo, hi)) = engine_box(keyframes, cloud) else {
        return vec![ply(), "-N".into(), "-F".into(), floater];
    };
    let corners = format!(
        "{},{},{},{},{},{}",
        lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]
    );
    let mut a: Vec<OsString> = vec![
        ply(),
        "-N".into(),
        "-B".into(),
        corners.into(),
        "-F".into(),
        floater,
    ];
    const AXES: [&str; 3] = ["x", "y", "z"];
    for (i, axis) in AXES.iter().enumerate() {
        for (cmp, v) in [("lt", lo[i]), ("gt", hi[i])] {
            a.extend([ply(), "-N".into()]);
            for (j, within) in AXES[..i].iter().enumerate() {
                a.extend([
                    "-V".into(),
                    format!("{within},gte,{}", lo[j]).into(),
                    "-V".into(),
                    format!("{within},lte,{}", hi[j]).into(),
                ]);
            }
            a.extend(["-V".into(), format!("{axis},{cmp},{v}").into()]);
        }
    }
    a
}

/// The floater filter's finest voxel, metres.
const FLOATER_VOXEL_M: f64 = 0.05;
/// Voxels across the walked box's diagonal at most.
const FLOATER_VOXELS_PER_DIAGONAL: f64 = 1000.0;

/// The floater filter's voxel: 5 cm, coarsened over a large site to a
/// thousandth of the walked box's diagonal. The filter's run time grows with
/// the cube of the scene extent over the voxel: a 5 cm grid over a
/// 270 x 240 m forest walk took over an hour where a room takes seconds.
pub fn floater_voxel_m(keyframes: &[Keyframe]) -> f64 {
    let Some((lo, hi)) = walked_box(keyframes) else {
        return FLOATER_VOXEL_M;
    };
    let diag = (0..3).map(|i| (hi[i] - lo[i]).powi(2)).sum::<f64>().sqrt();
    let v = (diag / FLOATER_VOXELS_PER_DIAGONAL).max(FLOATER_VOXEL_M);
    (v * 1000.0).round() / 1000.0
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
    let cloud = world_cloud(job)?;

    // Splat: the trained PLY filtered, its SPZ, and the LoD tree when asked
    // for. The filter drops non-finite splats, and inside the floater box
    // (`splat_filter_args`) the floaters that add to no solid voxel (5 cm,
    // coarser over a large site: `floater_voxel_m`) at 0.1 opacity.
    let splat_ply = job.join("train").join("splat.ply");
    runner.run(
        &ToolInvocation::new("splat-transform", &out)
            .args(["-w", "--no-tty"])
            .args(splat_filter_args(
                &splat_ply,
                &keyframes,
                &cloud.points,
                floater_voxel_m(&keyframes),
            ))
            .arg(out.join("scene.ply")),
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

    /// The Gaussian at `p` (PLY frame) as splat-transform 3.10.1 tests it.
    fn engine(p: [f64; 3]) -> [f64; 3] {
        [-p[0], -p[1], p[2]]
    }

    /// The inputs of a splat-transform argument list (split at each read of
    /// `ply`), and whether one keeps the Gaussian at `p`: `-B` keeps it inside
    /// the box, `-V name,cmp,value` where the comparison holds, both in the
    /// engine frame. `-N` and `-F` drop no well-formed Gaussian here.
    fn inputs(args: &[OsString], ply: &str) -> Vec<Vec<String>> {
        let mut out: Vec<Vec<String>> = Vec::new();
        for a in args.iter().map(|a| a.to_str().unwrap().to_string()) {
            if a == ply {
                out.push(Vec::new());
            } else {
                out.last_mut().unwrap().push(a);
            }
        }
        out
    }

    fn keeps(input: &[String], p: [f64; 3]) -> bool {
        let q = engine(p);
        let mut it = input.iter();
        while let Some(flag) = it.next() {
            match flag.as_str() {
                "-B" => {
                    let v: Vec<f64> = it
                        .next()
                        .unwrap()
                        .split(',')
                        .map(|s| s.parse().unwrap())
                        .collect();
                    if !(0..3).all(|i| v[i] <= q[i] && q[i] <= v[i + 3]) {
                        return false;
                    }
                }
                "-V" => {
                    let f: Vec<&str> = it.next().unwrap().split(',').collect();
                    let x = q[["x", "y", "z"].iter().position(|n| *n == f[0]).unwrap()];
                    let v: f64 = f[2].parse().unwrap();
                    let ok = match f[1] {
                        "lt" => x < v,
                        "lte" => x <= v,
                        "gt" => x > v,
                        "gte" => x >= v,
                        other => panic!("unexpected comparison {other}"),
                    };
                    if !ok {
                        return false;
                    }
                }
                "-F" => {
                    it.next();
                }
                "-N" => {}
                other => panic!("unexpected flag {other}"),
            }
        }
        true
    }

    fn k(x: f64, y: f64, z: f64) -> Keyframe {
        Keyframe {
            centre: [x, y, z],
            yaw_deg: 0.0,
        }
    }

    #[test]
    fn the_floater_box_is_the_walked_bounds_in_splat_transforms_frame() {
        let ply = Path::new("/j/train/splat.ply");
        let plain = splat_filter_args(ply, &[], &[], 0.05);
        assert_eq!(plain, ["/j/train/splat.ply", "-N", "-F", "0.05,0.1"]);
        // Walked bounds -3,-7,0.5 .. 13,3,4.7 with x and y mirrored.
        let a = splat_filter_args(ply, &[k(0.0, 0.0, 1.5), k(10.0, -4.0, 1.7)], &[], 0.05);
        let filtered = &inputs(&a, "/j/train/splat.ply")[0];
        assert_eq!(
            filtered,
            &["-N", "-B", "-13,-3,0.5,3,7,4.7", "-F", "0.05,0.1"]
        );
        assert!(keeps(filtered, [12.0, -6.0, 1.0]));
        assert!(!keeps(filtered, [-12.0, 6.0, 1.0]));
    }

    #[test]
    fn every_gaussian_is_read_once_and_only_those_in_the_box_are_floater_filtered() {
        // A drone flying 4 to 14 m over ground at z -0.5 to 0 that the
        // cameras saw out to 99 m by 54 m, plus 0.5 % of points triangulated
        // far off (under the 1 % tail).
        let walk = [k(10.0, 10.0, 4.0), k(50.0, 30.0, 14.0)];
        let mut cloud: Vec<[f64; 3]> = (0..995)
            .map(|i| {
                let (x, y) = ((i % 100) as f64, (i / 100) as f64 * 6.0);
                [x, y, -0.5 + (i % 7) as f64 / 14.0]
            })
            .collect();
        cloud.extend([[2000.0, 0.0, 500.0]; 5]);
        let (lo, hi) = splat_bounds(&walk, &cloud).unwrap();
        assert!(
            lo[2] <= -0.4,
            "the ground under the walk is in the box: {lo:?}"
        );
        assert_eq!((hi[0], hi[1], hi[2]), (99.0, 54.0, 17.0));
        let a = splat_filter_args(Path::new("s.ply"), &walk, &cloud, 0.356);
        let ins = inputs(&a, "s.ply");
        assert_eq!(ins.len(), 7);
        assert!(ins[0].contains(&"-F".to_string()));
        assert!(ins[1..].iter().all(|i| !i.contains(&"-F".to_string())));
        // Points inside, on each face of, and beyond the box on every side.
        let (lo, hi) = engine_box(&walk, &cloud).unwrap();
        let mut probes = vec![[95.0, 50.0, -0.3], [50.0, 30.0, 14.0], [2000.0, 0.0, 500.0]];
        for x in [
            lo[0] - 5.0,
            lo[0],
            (lo[0] + hi[0]) / 2.0,
            hi[0],
            hi[0] + 5.0,
        ] {
            for y in [
                lo[1] - 5.0,
                lo[1],
                (lo[1] + hi[1]) / 2.0,
                hi[1],
                hi[1] + 5.0,
            ] {
                for z in [
                    lo[2] - 5.0,
                    lo[2],
                    (lo[2] + hi[2]) / 2.0,
                    hi[2],
                    hi[2] + 5.0,
                ] {
                    probes.push(engine([x, y, z]));
                }
            }
        }
        for p in probes {
            let readers: Vec<usize> = (0..ins.len()).filter(|&i| keeps(&ins[i], p)).collect();
            assert_eq!(readers.len(), 1, "{p:?} is read by inputs {readers:?}");
            let q = engine(p);
            let inside = (0..3).all(|i| lo[i] <= q[i] && q[i] <= hi[i]);
            assert_eq!(readers[0] == 0, inside, "{p:?}");
        }
        assert!(
            keeps(&ins[0], [95.0, 50.0, -0.3]),
            "the seen ground is filtered"
        );
        assert!(!keeps(&ins[0], [2000.0, 0.0, 500.0]), "the far tail is not");
    }

    #[test]
    fn the_floater_voxel_coarsens_only_over_a_large_site() {
        let k = |x: f64, y: f64, z: f64| Keyframe {
            centre: [x, y, z],
            yaw_deg: 0.0,
        };
        assert_eq!(floater_voxel_m(&[]), 0.05);
        // A room walk: 5 cm.
        assert_eq!(floater_voxel_m(&[k(0.0, 0.0, 1.5), k(8.0, 6.0, 1.5)]), 0.05);
        // A 230 x 260 m forest walk (box 236 x 266 x 4 m): a thousandth of
        // its 355.6 m diagonal.
        assert_eq!(
            floater_voxel_m(&[k(0.0, 0.0, 2.0), k(230.0, 260.0, 2.0)]),
            0.356
        );
    }
}
