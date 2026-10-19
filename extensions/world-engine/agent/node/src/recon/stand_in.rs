//! A stand-in [`ToolRunner`] for the pipeline tests (feature `mock`): every
//! tool "runs" by writing small deterministic versions of the files the real
//! tool writes, so the stages, their persistence and every product step run
//! end to end on a host with no COLMAP, trainer or converter installed.

use std::ffi::OsString;
use std::fmt::Write as _;
use std::fs;
use std::path::{Path, PathBuf};

use super::{ResolvedTool, StageError, ToolInvocation, ToolRunner};

/// The stand-in runner. `missing` lists tools it reports as not installed
/// (`gsplat_kernels` makes the gsplat probe fail while the project resolves);
/// `log` records every invocation it ran.
#[derive(Debug, Default, Clone)]
pub struct StandInRunner {
    pub missing: Vec<String>,
    pub log: std::sync::Arc<std::sync::Mutex<Vec<ToolInvocation>>>,
}

impl StandInRunner {
    /// The invocations of `tool` so far.
    pub fn runs_of(&self, tool: &str) -> Vec<ToolInvocation> {
        self.log
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .iter()
            .filter(|i| i.tool == tool)
            .cloned()
            .collect()
    }
}

/// Every file under `dir`, as `/`-separated paths relative to it.
fn files_under(dir: &Path) -> Vec<String> {
    let mut out = Vec::new();
    let mut stack = vec![dir.to_path_buf()];
    while let Some(d) = stack.pop() {
        for e in fs::read_dir(&d).into_iter().flatten().flatten() {
            let p = e.path();
            if p.is_dir() {
                stack.push(p);
            } else if let Ok(rel) = p.strip_prefix(dir) {
                out.push(rel.to_string_lossy().replace('\\', "/"));
            }
        }
    }
    out
}

fn arg_after(inv: &ToolInvocation, flag: &str) -> Option<PathBuf> {
    let i = inv.args.iter().position(|a| a == flag)?;
    inv.args.get(i + 1).map(PathBuf::from)
}

fn first(inv: &ToolInvocation) -> String {
    inv.args
        .first()
        .map(|a| a.to_string_lossy().into_owned())
        .unwrap_or_default()
}

fn io(e: impl std::fmt::Display) -> StageError {
    StageError::io(e)
}

/// A box room of points 6 x 4 x 2.5 m with a seeded jitter, for the stand-in
/// sparse model.
fn box_points() -> Vec<[f64; 3]> {
    let mut out = Vec::new();
    let mut seed = 0x1234_5678u64;
    let mut jitter = || {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed % 1000) as f64 / 1000.0 * 0.01
    };
    let mut x = 0.0;
    while x <= 6.0 {
        let mut z = 0.0;
        while z <= 2.5 {
            for (px, py) in [(x, 0.0), (x, 4.0)] {
                out.push([px + jitter(), py + jitter(), z]);
            }
            z += 0.1;
        }
        x += 0.1;
    }
    let mut y = 0.0;
    while y <= 4.0 {
        let mut z = 0.0;
        while z <= 2.5 {
            for px in [0.0, 6.0] {
                out.push([px + jitter(), y + jitter(), z]);
            }
            z += 0.1;
        }
        y += 0.1;
    }
    out
}

/// The box room plus `extra` points.
fn write_points_txt(dir: &Path, extra: &[[f64; 3]]) -> Result<(), StageError> {
    let mut s = String::from("# POINT3D_ID X Y Z R G B ERROR TRACK[]\n");
    for (i, p) in box_points().iter().chain(extra).enumerate() {
        let _ = writeln!(
            s,
            "{} {} {} {} 180 170 160 0.5 1 1",
            i + 1,
            p[0],
            p[1],
            p[2]
        );
    }
    fs::write(dir.join("points3D.txt"), s).map_err(io)
}

/// The stand-in trainer's gaussians: one per sparse point of the dataset it
/// trains on (the box room when it has no model).
fn splat_points(dataset: &Path) -> Vec<[f64; 3]> {
    match super::colmap::read_points(&dataset.join("sparse").join("0")) {
        Ok(pts) => pts.into_iter().map(|p| p.xyz).collect(),
        Err(_) => box_points(),
    }
}

/// A minimal 3DGS-style binary PLY over `pts`.
fn write_splat_ply(path: &Path, pts: &[[f64; 3]]) -> Result<(), StageError> {
    let props = [
        "x", "y", "z", "f_dc_0", "f_dc_1", "f_dc_2", "opacity", "scale_0", "scale_1", "scale_2",
        "rot_0", "rot_1", "rot_2", "rot_3",
    ];
    let mut out = format!(
        "ply\nformat binary_little_endian 1.0\nelement vertex {}\n",
        pts.len()
    );
    for p in props {
        let _ = writeln!(out, "property float {p}");
    }
    out.push_str("end_header\n");
    let mut bytes = out.into_bytes();
    for p in pts {
        for v in [
            p[0] as f32,
            p[1] as f32,
            p[2] as f32,
            0.5,
            0.4,
            0.3,
            2.0,
            -4.0,
            -4.0,
            -4.0,
            1.0,
            0.0,
            0.0,
            0.0,
        ] {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
    }
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(io)?;
    }
    fs::write(path, bytes).map_err(io)
}

/// An ASCII triangle mesh of the stand-in room's four walls.
fn write_mesh_ply(path: &Path) -> Result<(), StageError> {
    let v = [
        [0.0, 0.0, 0.0],
        [6.0, 0.0, 0.0],
        [6.0, 4.0, 0.0],
        [0.0, 4.0, 0.0],
        [0.0, 0.0, 2.5],
        [6.0, 0.0, 2.5],
        [6.0, 4.0, 2.5],
        [0.0, 4.0, 2.5],
    ];
    let f = [
        [0, 1, 5],
        [0, 5, 4],
        [1, 2, 6],
        [1, 6, 5],
        [2, 3, 7],
        [2, 7, 6],
        [3, 0, 4],
        [3, 4, 7],
    ];
    let mut s = format!(
        "ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nelement face {}\nproperty list uchar int vertex_indices\nend_header\n",
        v.len(),
        f.len()
    );
    for p in v {
        let _ = writeln!(s, "{} {} {}", p[0], p[1], p[2]);
    }
    for t in f {
        let _ = writeln!(s, "3 {} {} {}", t[0], t[1], t[2]);
    }
    fs::write(path, s).map_err(io)
}

/// Record images listed for a stand-in `feature_extractor` in a COLMAP-shaped
/// database (only the `images` table the pipeline reads back).
fn extract(inv: &ToolInvocation) -> Result<(), StageError> {
    let db = arg_after(inv, "--database_path").ok_or_else(|| StageError::input("no db"))?;
    let list = arg_after(inv, "--image_list_path").ok_or_else(|| StageError::input("no list"))?;
    let conn = rusqlite::Connection::open(&db).map_err(io)?;
    conn.execute_batch(
        "CREATE TABLE IF NOT EXISTS cameras (camera_id INTEGER PRIMARY KEY);
         CREATE TABLE IF NOT EXISTS images (image_id INTEGER PRIMARY KEY, name TEXT, camera_id INTEGER);",
    )
    .map_err(io)?;
    conn.execute("INSERT INTO cameras DEFAULT VALUES", [])
        .map_err(io)?;
    let cam = conn.last_insert_rowid();
    for name in fs::read_to_string(list).map_err(io)?.lines() {
        conn.execute(
            "INSERT INTO images (name, camera_id) VALUES (?1, ?2)",
            rusqlite::params![name, cam],
        )
        .map_err(io)?;
    }
    Ok(())
}

/// A stand-in mapper model: every database image on a line along +x.
fn map(inv: &ToolInvocation) -> Result<(), StageError> {
    let db = arg_after(inv, "--database_path").ok_or_else(|| StageError::input("no db"))?;
    let out = arg_after(inv, "--output_path")
        .ok_or_else(|| StageError::input("no output"))?
        .join("0");
    fs::create_dir_all(&out).map_err(io)?;
    let ids = super::colmap::read_db_ids(&db)?;
    let mut cams = String::new();
    let mut imgs = String::new();
    let mut seen = std::collections::BTreeSet::new();
    for (i, (name, (image_id, camera_id))) in ids.images.iter().enumerate() {
        if seen.insert(*camera_id) {
            let _ = writeln!(cams, "{camera_id} PINHOLE 64 48 50 50 32 24");
        }
        let _ = writeln!(
            imgs,
            "{image_id} 1 0 0 0 {} -2 -1.5 {camera_id} {name}\n",
            -(i as f64) * 0.5
        );
    }
    fs::write(out.join("cameras.txt"), cams).map_err(io)?;
    fs::write(out.join("images.txt"), imgs).map_err(io)?;
    fs::write(out.join("images.bin"), [1u8]).map_err(io)?;
    write_points_txt(&out, &[])
}

impl StandInRunner {
    fn colmap(&self, inv: &ToolInvocation) -> Result<(), StageError> {
        match first(inv).as_str() {
            "feature_extractor" => extract(inv),
            "mapper" | "hierarchical_mapper" => map(inv),
            "point_triangulator" => {
                let input =
                    arg_after(inv, "--input_path").ok_or_else(|| StageError::input("in"))?;
                let out =
                    arg_after(inv, "--output_path").ok_or_else(|| StageError::input("out"))?;
                fs::create_dir_all(&out).map_err(io)?;
                for f in ["cameras.txt", "images.txt"] {
                    fs::copy(input.join(f), out.join(f)).map_err(io)?;
                }
                // One point a metre in front of each posed camera, so a
                // model's points follow its cameras.
                let ahead: Vec<[f64; 3]> = super::colmap::read_image_poses(&input)?
                    .values()
                    .map(|t| [t[3] + t[2], t[7] + t[6], t[11] + t[10]])
                    .collect();
                write_points_txt(&out, &ahead)
            }
            "model_converter" => {
                let input =
                    arg_after(inv, "--input_path").ok_or_else(|| StageError::input("in"))?;
                let out =
                    arg_after(inv, "--output_path").ok_or_else(|| StageError::input("out"))?;
                fs::create_dir_all(&out).map_err(io)?;
                for f in ["cameras.txt", "images.txt", "points3D.txt"] {
                    fs::copy(input.join(f), out.join(f)).map_err(io)?;
                }
                // COLMAP 4 rig files, when the input model has them.
                for f in ["rigs.txt", "frames.txt"] {
                    if input.join(f).is_file() {
                        fs::copy(input.join(f), out.join(f)).map_err(io)?;
                    }
                }
                Ok(())
            }
            _ => Ok(()),
        }
    }

    fn script(
        &self,
        inv: &ToolInvocation,
        on_line: &mut dyn FnMut(&str),
    ) -> Result<(), StageError> {
        let script = Path::new(&first(inv))
            .file_name()
            .map(|n| n.to_string_lossy().into_owned())
            .unwrap_or_default();
        let out = arg_after(inv, "--out").unwrap_or_else(|| inv.cwd.clone());
        on_line(r#"{"event":"progress","progress":0.5}"#);
        match script.as_str() {
            "train.py" => {
                let colmap = arg_after(inv, "--colmap").unwrap_or_default();
                let pts = splat_points(&colmap);
                write_splat_ply(&out.join("splat.ply"), &pts)?;
                on_line(&format!(
                    r#"{{"event":"done","ply":"{}","gaussians":{},"psnr":31.5}}"#,
                    out.join("splat.ply")
                        .display()
                        .to_string()
                        .replace('\\', "/"),
                    pts.len()
                ));
            }
            "mesh.py" => {
                write_mesh_ply(&out.join("mesh.ply"))?;
                fs::write(out.join("mesh.glb"), b"glTF").map_err(io)?;
                on_line(r#"{"event":"done","triangles":8}"#);
            }
            "stills.py" => {
                let poses = arg_after(inv, "--poses").ok_or_else(|| StageError::input("poses"))?;
                let list: Vec<serde_json::Value> =
                    serde_json::from_slice(&fs::read(poses).map_err(io)?).map_err(io)?;
                for p in list {
                    let name = p["name"].as_str().unwrap_or("still");
                    image::save_buffer(
                        out.join(format!("{name}.jpg")),
                        &[120u8; 16 * 12 * 3],
                        16,
                        12,
                        image::ExtendedColorType::Rgb8,
                    )
                    .map_err(io)?;
                }
                on_line(r#"{"event":"done"}"#);
            }
            "masks.py" => {
                // One keep-everything mask per image, at `<image rel>.png`.
                let images =
                    arg_after(inv, "--images").ok_or_else(|| StageError::input("images"))?;
                for rel in files_under(&images) {
                    let path = out.join(format!("{rel}.png"));
                    if let Some(parent) = path.parent() {
                        fs::create_dir_all(parent).map_err(io)?;
                    }
                    image::save_buffer(&path, &[255u8; 4], 2, 2, image::ExtendedColorType::L8)
                        .map_err(io)?;
                }
                on_line(r#"{"event":"done"}"#);
            }
            "depth.py" => {
                let images =
                    arg_after(inv, "--images").ok_or_else(|| StageError::input("images"))?;
                for rel in files_under(&images) {
                    let path = out.join(format!("{rel}.npy"));
                    if let Some(parent) = path.parent() {
                        fs::create_dir_all(parent).map_err(io)?;
                    }
                    fs::write(&path, b"\x93NUMPY").map_err(io)?;
                }
                on_line(r#"{"event":"done"}"#);
            }
            "common.py" if self.missing.iter().any(|m| m == "gsplat_kernels") => {
                return Err(StageError::tool_failed(
                    "gsplat_train",
                    "stand-in: no gsplat kernels",
                ));
            }
            _ => on_line(r#"{"event":"done"}"#),
        }
        Ok(())
    }
}

impl ToolRunner for StandInRunner {
    fn resolve(&self, tool: &str) -> Result<ResolvedTool, StageError> {
        if self.missing.iter().any(|m| m == tool) {
            return Err(StageError::tool_missing(tool, "stand-in: reported missing"));
        }
        Ok(ResolvedTool {
            name: tool.to_string(),
            version: crate::tools::pins()
                .get(tool)
                .map(|p| p.version.clone())
                .unwrap_or_else(|| "stand-in".into()),
            program: PathBuf::from(tool),
            prefix_args: Vec::<OsString>::new(),
            home: PathBuf::from("stand-in"),
            wsl: false,
        })
    }

    fn run(&self, inv: &ToolInvocation, on_line: &mut dyn FnMut(&str)) -> Result<(), StageError> {
        self.resolve(&inv.tool)?;
        self.log
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .push(inv.clone());
        match inv.tool.as_str() {
            "colmap" => self.colmap(inv),
            "brush" => {
                let dir = arg_after(inv, "--export-path").unwrap_or_else(|| inv.cwd.clone());
                let name = arg_after(inv, "--export-name").unwrap_or_else(|| "splat.ply".into());
                let pts = splat_points(Path::new(&first(inv)));
                on_line(&format!("step 7000 gaussians: {}", pts.len()));
                write_splat_ply(&dir.join(name), &pts)
            }
            "gsplat_train" => self.script(inv, on_line),
            "splat-transform" => {
                // The output is the last argument; the input the first file
                // argument that exists.
                let out = inv.args.last().map(PathBuf::from).unwrap_or_default();
                let input = inv
                    .args
                    .iter()
                    .map(PathBuf::from)
                    .find(|p| p.is_file())
                    .ok_or_else(|| StageError::input("stand-in converter: no input"))?;
                fs::copy(input, out).map_err(io).map(|_| ())
            }
            "spark-build-lod" => {
                // `<stem>-lod.rad` beside the input, as the real builder does.
                let input = inv.args.last().map(PathBuf::from).unwrap_or_default();
                let stem = input
                    .file_stem()
                    .map(|s| s.to_string_lossy().into_owned())
                    .unwrap_or_default();
                let out = input.with_file_name(format!("{stem}-lod.rad"));
                fs::copy(&input, out).map_err(io).map(|_| ())
            }
            "docker" | "kalibr" | "dvlc" | "glim" => {
                // `docker info`: the runtimes, with `nvidia` unless the
                // NVIDIA runtime is reported missing.
                if first(inv) == "info" {
                    let missing = self
                        .missing
                        .iter()
                        .any(|m| m == super::glim::NVIDIA_RUNTIME);
                    on_line(if missing {
                        r#"{"runc":{"path":"runc"}}"#
                    } else {
                        r#"{"nvidia":{"path":"nvidia-container-runtime"},"runc":{"path":"runc"}}"#
                    });
                    return Ok(());
                }
                if !super::glim::stand_in(inv).map_err(io)? {
                    crate::calib::stand_in(inv).map_err(io)?;
                }
                Ok(())
            }
            _ => Ok(()),
        }
    }
}
