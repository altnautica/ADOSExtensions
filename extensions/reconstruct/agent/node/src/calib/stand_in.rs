//! Deterministic outputs of the calibration container runs, for the stand-in
//! tool runner (no container runtime on the host). Each run writes the files
//! the real tool would, with plausible values: camera chains from the
//! committed fixtures, the initial LiDAR-camera guess as the refined result.

use std::fs;
use std::io;
use std::path::{Path, PathBuf};

use serde_json::{json, Value};
use serde_yaml_ng::{Mapping, Value as Yaml};

use crate::recon::ToolInvocation;

const CAMCHAIN_IMUCAM: &str = include_str!("../../tests/fixtures/calib/camchain-imucam.yaml");

/// When `inv` is one of the calibration container runs, write what it would
/// produce and return `true`; `false` for any other invocation.
pub fn stand_in(inv: &ToolInvocation) -> io::Result<bool> {
    if inv.tool != "docker" {
        return Ok(false);
    }
    let args = inv.args_lossy();
    let root = inv.cwd.as_path();
    if let Some(cmd) = args.last().and_then(|a| a.split("rosrun kalibr ").nth(1)) {
        let words: Vec<&str> = cmd.split_whitespace().collect();
        return match words.first().copied() {
            Some("kalibr_bagcreater") => {
                let bag = host(root, flag(&words, "--output-bag")?)?;
                fs::write(bag, b"")?;
                Ok(true)
            }
            Some("kalibr_calibrate_cameras") => calibrate_cameras(root, &words).map(|_| true),
            Some("kalibr_calibrate_imu_camera") => calibrate_imu_camera(root, &words).map(|_| true),
            _ => Ok(false),
        };
    }
    if let Some(at) = args
        .iter()
        .position(|a| a == "direct_visual_lidar_calibration")
    {
        let rest: Vec<&str> = args[at + 1..].iter().map(String::as_str).collect();
        return match rest.first().copied() {
            Some("preprocess") => dvlc_preprocess(root, &rest).map(|_| true),
            Some("calibrate") => dvlc_calibrate(root, &rest).map(|_| true),
            _ => Ok(false),
        };
    }
    Ok(false)
}

fn bad(msg: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, msg.into())
}

/// Host path of a `/data/...` container path.
fn host(root: &Path, container: &str) -> io::Result<PathBuf> {
    container
        .strip_prefix("/data/")
        .map(|rel| root.join(rel))
        .ok_or_else(|| bad(format!("{container} is outside the /data mount")))
}

/// Values after `name` up to the next `--` flag.
fn values<'a>(words: &[&'a str], name: &str) -> Vec<&'a str> {
    words
        .iter()
        .position(|w| *w == name)
        .map(|i| {
            words[i + 1..]
                .iter()
                .take_while(|w| !w.starts_with("--"))
                .copied()
                .collect()
        })
        .unwrap_or_default()
}

fn flag<'a>(words: &[&'a str], name: &str) -> io::Result<&'a str> {
    values(words, name)
        .first()
        .copied()
        .ok_or_else(|| bad(format!("missing {name}")))
}

/// `<bag without .bag>-<suffix>` next to the bag.
fn output(root: &Path, words: &[&str], suffix: &str) -> io::Result<PathBuf> {
    let bag = host(root, flag(words, "--bag")?)?;
    let stem = bag
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_default();
    Ok(bag.with_file_name(format!("{stem}-{suffix}")))
}

fn to_yaml_err(e: serde_yaml_ng::Error) -> io::Error {
    bad(e.to_string())
}

fn calibrate_cameras(root: &Path, words: &[&str]) -> io::Result<()> {
    let bag = host(root, flag(words, "--bag")?)?;
    let folder = bag.with_file_name("data");
    let models = values(words, "--models");
    let mut chain = String::new();
    let mut results =
        String::from("Calibration results \n====================\nCamera-system parameters:\n");
    for (i, model) in models.iter().enumerate() {
        let (w, h) = first_image_size(&folder.join(format!("cam{i}"))).unwrap_or((752, 480));
        let (dm, dc) = if *model == "pinhole-equi" {
            ("equidistant", "[0.01, -0.002, 0.0003, -4.0e-05]")
        } else {
            ("radtan", "[-0.05, 0.01, 0.0005, -0.0003]")
        };
        chain.push_str(&format!("cam{i}:\n"));
        if i > 0 {
            chain.push_str("  T_cn_cnm1:\n  - [1.0, 0.0, 0.0, -0.1]\n  - [0.0, 1.0, 0.0, 0.0]\n  - [0.0, 0.0, 1.0, 0.0]\n  - [0.0, 0.0, 0.0, 1.0]\n");
        }
        let f = 0.9 * w as f64;
        chain.push_str(&format!(
            "  camera_model: pinhole\n  distortion_coeffs: {dc}\n  distortion_model: {dm}\n  intrinsics: [{f:.3}, {f:.3}, {:.3}, {:.3}]\n  resolution: [{w}, {h}]\n  rostopic: /cam{i}/image_raw\n",
            w as f64 / 2.0,
            h as f64 / 2.0
        ));
        results.push_str(&format!(
            "cam{i} (/cam{i}/image_raw):\n    reprojection error: [0.000000, 0.000000] +- [0.150000, 0.200000]\n\n"
        ));
    }
    fs::write(output(root, words, "camchain.yaml")?, chain)?;
    fs::write(output(root, words, "results-cam.txt")?, results)
}

fn first_image_size(dir: &Path) -> Option<(u32, u32)> {
    let mut names: Vec<PathBuf> = fs::read_dir(dir)
        .ok()?
        .filter_map(|e| Some(e.ok()?.path()))
        .collect();
    names.sort();
    image::image_dimensions(names.first()?).ok()
}

fn calibrate_imu_camera(root: &Path, words: &[&str]) -> io::Result<()> {
    let fixture: Yaml = serde_yaml_ng::from_str(CAMCHAIN_IMUCAM).map_err(to_yaml_err)?;
    let input = fs::read_to_string(host(root, flag(words, "--cams")?)?)?;
    let mut chain: Yaml = serde_yaml_ng::from_str(&input).map_err(to_yaml_err)?;
    let cams = chain
        .as_mapping_mut()
        .ok_or_else(|| bad("camera chain is not a mapping"))?;
    let mut results = String::from(
        "Calibration results\n===================\nResiduals\n----------------------------\n",
    );
    for i in 0..cams.len() {
        let key = Yaml::from(format!("cam{i}"));
        let from = fixture
            .get(format!("cam{}", i % 2))
            .ok_or_else(|| bad("fixture camera missing"))?;
        let cam: &mut Mapping = cams
            .get_mut(&key)
            .and_then(Yaml::as_mapping_mut)
            .ok_or_else(|| bad(format!("cam{i} missing from the chain")))?;
        for k in ["T_cam_imu", "timeshift_cam_imu"] {
            if let Some(v) = from.get(k) {
                cam.insert(Yaml::from(k), v.clone());
            }
        }
        results.push_str(&format!(
            "Reprojection error (cam{i}) [px]:     mean 0.18, median 0.16, std: 0.12\n"
        ));
    }
    let out = serde_yaml_ng::to_string(&chain).map_err(to_yaml_err)?;
    fs::write(output(root, words, "camchain-imucam.yaml")?, out)?;
    let imu = fs::read_to_string(host(root, flag(words, "--imu")?)?)?;
    fs::write(output(root, words, "imu.yaml")?, imu)?;
    fs::write(output(root, words, "results-imucam.txt")?, results)
}

/// Value of a `--name value` option in a dvlc argument list.
fn opt<'a>(rest: &[&'a str], name: &str) -> Option<&'a str> {
    rest.iter()
        .position(|w| *w == name)
        .and_then(|i| rest.get(i + 1).copied())
}

fn floats(s: Option<&str>) -> Vec<f64> {
    s.map(|s| s.split(',').filter_map(|v| v.parse().ok()).collect())
        .unwrap_or_default()
}

fn dvlc_preprocess(root: &Path, rest: &[&str]) -> io::Result<()> {
    let (Some(bags), Some(dst)) = (rest.get(1), rest.get(2)) else {
        return Err(bad("preprocess needs a data and a destination path"));
    };
    let bags = host(root, bags)?;
    let dst = host(root, dst)?;
    fs::create_dir_all(&dst)?;
    let mut names: Vec<String> = fs::read_dir(&bags)?
        .filter_map(|e| Some(e.ok()?.file_name().to_string_lossy().into_owned()))
        .collect();
    names.sort();
    let calib = json!({
        "camera": {
            "camera_model": opt(rest, "--camera_model").unwrap_or("plumb_bob"),
            "intrinsics": floats(opt(rest, "--camera_intrinsics")),
            "distortion_coeffs": floats(opt(rest, "--camera_distortion_coeffs")),
        },
        "meta": {
            "bag_names": names,
            "camera_info_topic": "",
            "data_path": rest[1],
            "image_topic": opt(rest, "--image_topic").unwrap_or(""),
            "intensity_channel": "intensity",
            "points_topic": opt(rest, "--points_topic").unwrap_or(""),
        },
        "results": {},
    });
    fs::write(dst.join("calib.json"), serde_json::to_vec_pretty(&calib)?)
}

fn dvlc_calibrate(root: &Path, rest: &[&str]) -> io::Result<()> {
    let dir = host(
        root,
        rest.get(1)
            .ok_or_else(|| bad("calibrate needs a data path"))?,
    )?;
    let path = dir.join("calib.json");
    let mut calib: Value = serde_json::from_slice(&fs::read(&path)?)?;
    let init = calib
        .pointer("/results/init_T_lidar_camera")
        .cloned()
        .unwrap_or_else(|| json!([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]));
    if let Some(r) = calib.pointer_mut("/results").and_then(Value::as_object_mut) {
        r.insert("T_lidar_camera".into(), init);
    }
    fs::write(&path, serde_json::to_vec_pretty(&calib)?)
}
