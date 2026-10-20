//! LiDAR-inertial trajectory of a LiDAR session, ahead of SfM.
//!
//! The session's sweeps and IMU samples are exported to a ROS 2 bag
//! ([`glim_bag`]), the pinned GLIM image runs its offline tool over it with a
//! headless configuration set to the bag's topics and the rig's LiDAR-IMU
//! extrinsic, and the optimized body trajectory it dumps (`traj_imu.txt`,
//! TUM lines; the IMU frame is the body frame because the contract records
//! IMU samples in body FLU) becomes the camera poses of `frames.json`. A
//! session with a recorded pose track gets GLIM's trajectory rigidly aligned
//! onto that track (the session's ENU world); without one, GLIM's own world
//! is used as is. A rig without an IMU runs GLIM's LiDAR-only odometry.
//!
//! Work directory `<job>/glim/` (mounted at `/data`): `bag/`,
//! `reference_traj.txt` (the session track at the sweep times),
//! `config_stock/` (the copied headless config), `config/` (the edited one),
//! `dump/` (GLIM's output), `trajectory_world.txt` (the poses used).

use std::fs;
use std::path::Path;

use ados_capture_contract::{LidarModelPattern, Rig, RigSensorKind, SessionReader};
use serde::Serialize;
use serde_json::{json, Value};

use super::geom::{self, Mat4};
use super::glim_bag::{self, read_tum, write_tum, GlimBag, IMU_TOPIC, POINTS_TOPIC};
use super::session_data;
use super::{StageError, ToolInvocation, ToolRunner};
use crate::calib::container::pinned_image;
use crate::tools::container_path;

/// Work directory under the job directory.
pub const GLIM_DIR: &str = "glim";
/// The headless GLIM configuration (viewer removed) on the docker host.
pub const CONFIG_SOURCE: &str = "/opt/ados-recon/glim-config-headless";
const STOCK: &str = "config_stock";
const REFERENCE: &str = "reference_traj.txt";
const DUMP: &str = "dump";

/// Which GLIM odometry runs (job param `glim_odometry`). A rig without an
/// IMU always runs the LiDAR-only odometry.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum GlimOdometry {
    #[default]
    Cpu,
    Gpu,
}

impl GlimOdometry {
    pub fn parse(s: &str) -> Option<Self> {
        match s {
            "cpu" => Some(Self::Cpu),
            "gpu" => Some(Self::Gpu),
            _ => None,
        }
    }

    /// `[odometry, sub mapping, global mapping]` config files.
    fn configs(self, imu: bool) -> [&'static str; 3] {
        let odom = match (imu, self) {
            (false, _) => "config_odometry_ct.json",
            (true, Self::Cpu) => "config_odometry_cpu.json",
            (true, Self::Gpu) => "config_odometry_gpu.json",
        };
        match self {
            Self::Cpu => [
                odom,
                "config_sub_mapping_cpu.json",
                "config_global_mapping_cpu.json",
            ],
            Self::Gpu => [
                odom,
                "config_sub_mapping_gpu.json",
                "config_global_mapping_gpu.json",
            ],
        }
    }
}

/// The `glim` entry of the sfm stage detail.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct GlimSummary {
    pub odometry: GlimOdometry,
    pub trajectory_poses: usize,
    /// RMS position error of the alignment onto the session pose track;
    /// `None` when the session has no track.
    pub align_rmse_m: Option<f64>,
    /// The trajectory posed the keyframes.
    pub used: bool,
    /// Why the trajectory was not used (`align_rmse`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rejected: Option<String>,
    /// Keyframes posed from the trajectory, keeping their recorded pose, and
    /// dropped (outside the trajectory with no recorded track).
    pub frames_glim: usize,
    pub frames_recorded: usize,
    pub frames_dropped: usize,
}

/// Whether the rig carries a 3D LiDAR (pattern `solid` or `spinning`): its
/// session is posed by GLIM. A planar LiDAR needs no external tool.
pub fn rig_has_3d_lidar(rig: &Rig) -> bool {
    rig.sensors.iter().any(|s| {
        s.kind == RigSensorKind::Lidar
            && s.lidar.as_ref().is_some_and(|l| {
                matches!(
                    l.pattern,
                    LidarModelPattern::Solid | LidarModelPattern::Spinning
                )
            })
    })
}

/// The runtime docker needs for `--gpus` (GPU odometry).
pub const NVIDIA_RUNTIME: &str = "nvidia-container-runtime";

/// `E_TOOL_MISSING{tool:"glim"}` unless docker resolves and the pinned image
/// is present; for GPU odometry also `E_TOOL_MISSING{tool:
/// "nvidia-container-runtime"}` unless docker lists an `nvidia` runtime.
/// `cwd` is any existing directory.
pub fn check_available(
    runner: &dyn ToolRunner,
    cwd: &Path,
    odometry: GlimOdometry,
) -> Result<(), StageError> {
    runner
        .resolve("glim")
        .map_err(|e| StageError::tool_missing("glim", e.message))?;
    let image = pinned_image("glim")?;
    let inspect =
        ToolInvocation::new("glim", cwd).args(["image", "inspect", "--format", "{{.Id}}"]);
    runner.run(&inspect.arg(&image), &mut |_| {}).map_err(|e| {
        StageError::tool_missing(
            "glim",
            format!("the pinned image {image} is not available: {}", e.message),
        )
    })?;
    if odometry == GlimOdometry::Gpu {
        let mut runtimes = String::new();
        let info =
            ToolInvocation::new("glim", cwd).args(["info", "--format", "{{json .Runtimes}}"]);
        runner
            .run(&info, &mut |l| runtimes.push_str(l))
            .map_err(|e| StageError::tool_missing(NVIDIA_RUNTIME, e.message))?;
        let has = serde_json::from_str::<Value>(&runtimes)
            .ok()
            .is_some_and(|v| v.get("nvidia").is_some());
        if !has {
            return Err(StageError::tool_missing(
                NVIDIA_RUNTIME,
                format!("docker lists no nvidia runtime ({}); GPU odometry needs the NVIDIA Container Toolkit", runtimes.trim()),
            ));
        }
    }
    Ok(())
}

fn docker_run(dir: &Path, image: &str, extra: &[&str]) -> ToolInvocation {
    let volume = format!("{}:/data", container_path(dir));
    ToolInvocation::new("glim", dir)
        .args(["run", "--rm", "-v", volume.as_str()])
        .args(extra.iter().copied())
        .arg(image)
}

/// Copy [`CONFIG_SOURCE`] into `<dir>/config_stock`.
pub fn copy_config(dir: &Path, image: &str) -> ToolInvocation {
    let mount = format!("type=bind,src={CONFIG_SOURCE},dst=/config,readonly");
    docker_run(dir, image, &["--mount", &mount, "--entrypoint", "cp"]).args([
        "-r",
        "/config",
        "/data/config_stock",
    ])
}

/// The offline run over `<dir>/bag` with `<dir>/config`, dumping to
/// `<dir>/dump`; the GPU is passed through only for GPU odometry.
pub fn glim_rosbag(dir: &Path, image: &str, odometry: GlimOdometry) -> ToolInvocation {
    let gpus: &[&str] = match odometry {
        GlimOdometry::Gpu => &["--gpus", "all"],
        GlimOdometry::Cpu => &[],
    };
    docker_run(dir, image, gpus).args([
        "ros2",
        "run",
        "glim_ros",
        "glim_rosbag",
        "/data/bag",
        "--ros-args",
        "-p",
        "config_path:=/data/config",
        "-p",
        "auto_quit:=true",
        "-p",
        "dump_path:=/data/dump",
    ])
}

/// JSON with `//` and `/* */` comments removed (string contents kept).
pub fn strip_comments(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let (mut i, mut in_str) = (0, false);
    while i < b.len() {
        let c = b[i];
        if in_str {
            out.push(c);
            if c == b'\\' && i + 1 < b.len() {
                out.push(b[i + 1]);
                i += 1;
            } else if c == b'"' {
                in_str = false;
            }
            i += 1;
        } else if c == b'"' {
            in_str = true;
            out.push(c);
            i += 1;
        } else if b[i..].starts_with(b"//") {
            while i < b.len() && b[i] != b'\n' {
                i += 1;
            }
        } else if b[i..].starts_with(b"/*") {
            i = b[i + 2..]
                .windows(2)
                .position(|w| w == b"*/")
                .map_or(b.len(), |p| i + p + 4);
        } else {
            out.push(c);
            i += 1;
        }
    }
    // Only ASCII bytes were removed, so the rest is still UTF-8.
    String::from_utf8(out).unwrap_or_default()
}

fn read_jsonc(path: &Path) -> Result<Value, StageError> {
    let text = fs::read_to_string(path)?;
    serde_json::from_str(&strip_comments(&text))
        .map_err(|e| StageError::io(format!("{}: {e}", path.display())))
}

/// Rewrite `section` of the JSON config at `path` through `f`.
fn edit(
    path: &Path,
    section: &str,
    f: impl FnOnce(&mut serde_json::Map<String, Value>),
) -> Result<(), StageError> {
    let mut v = read_jsonc(path)?;
    let sec = v
        .get_mut(section)
        .and_then(Value::as_object_mut)
        .ok_or_else(|| StageError::io(format!("{} has no {section} section", path.display())))?;
    f(sec);
    Ok(fs::write(path, serde_json::to_vec_pretty(&v)?)?)
}

/// `[x, y, z, qx, qy, qz, qw]` of a transform (GLIM's SE3 config format).
fn tum7(t: &Mat4) -> [f64; 7] {
    let [qw, qx, qy, qz] = geom::quat_of(t);
    let p = geom::translation(t);
    [p[0], p[1], p[2], qx, qy, qz, qw]
}

/// Copy the stock config to `out` and set the bag's topics, the LiDAR-IMU
/// extrinsic, the per-point time convention, and the odometry / mapping
/// modules (`odometry`, or LiDAR-only odometry with IMU factors off when the
/// bag has no IMU).
pub fn write_config(
    stock: &Path,
    out: &Path,
    bag: &GlimBag,
    odometry: GlimOdometry,
) -> Result<(), StageError> {
    if !stock.join("config.json").is_file() {
        return Err(StageError::tool_missing(
            "glim",
            format!("no GLIM config.json in {CONFIG_SOURCE}"),
        ));
    }
    fs::create_dir_all(out)?;
    for e in fs::read_dir(stock)? {
        let e = e?;
        if e.file_type()?.is_file() {
            fs::copy(e.path(), out.join(e.file_name()))?;
        }
    }
    let mut global = read_jsonc(&out.join("config.json"))?;
    let file =
        |key: &str, default: &str| out.join(global["global"][key].as_str().unwrap_or(default));
    let imu = bag.imu_id.is_some();
    edit(&file("config_ros", "config_ros.json"), "glim_ros", |r| {
        r.insert("imu_topic".into(), json!(if imu { IMU_TOPIC } else { "" }));
        r.insert("points_topic".into(), json!(POINTS_TOPIC));
        // Accelerometer in m/s^2, gyro in rad/s (contract units).
        r.insert("acc_scale".into(), json!(1.0));
        r.insert("ang_scale".into(), json!(1.0));
    })?;
    // The IMU frame is the body frame, so T_lidar_imu = T_lidar_body.
    let t_lidar_imu = tum7(&geom::inverse_rigid(&bag.t_body_lidar));
    edit(
        &file("config_sensors", "config_sensors.json"),
        "sensors",
        |s| {
            s.insert("T_lidar_imu".into(), json!(t_lidar_imu));
            s.insert("global_shutter_lidar".into(), json!(!bag.point_times));
            s.insert("autoconf_perpoint_times".into(), json!(false));
            s.insert("perpoint_relative_time".into(), json!(true));
            s.insert("perpoint_time_scale".into(), json!(1.0));
        },
    )?;
    let [odom, sub, global_mapping] = odometry.configs(imu);
    for name in [odom, sub, global_mapping] {
        if !out.join(name).is_file() {
            return Err(StageError::tool_missing(
                "glim",
                format!("no GLIM {name} in {CONFIG_SOURCE}"),
            ));
        }
    }
    if !imu {
        let no_imu = |s: &mut serde_json::Map<String, Value>| {
            s.insert("enable_imu".into(), json!(false));
        };
        edit(&out.join(sub), "sub_mapping", no_imu)?;
        edit(&out.join(global_mapping), "global_mapping", no_imu)?;
    }
    global["global"]["config_odometry"] = json!(odom);
    global["global"]["config_sub_mapping"] = json!(sub);
    global["global"]["config_global_mapping"] = json!(global_mapping);
    fs::write(out.join("config.json"), serde_json::to_vec_pretty(&global)?)?;
    Ok(())
}

/// Run GLIM for the session and re-pose `dataset/frames.json` from its
/// trajectory (see [`apply_trajectory`]). `None` when the LiDAR recorded no
/// sweeps (nothing to run on; the dataset is left as prepared).
pub fn run(
    runner: &dyn ToolRunner,
    reader: &SessionReader,
    job_dir: &Path,
    dataset: &Path,
    odometry: GlimOdometry,
    progress: &mut dyn FnMut(f32),
) -> Result<Option<GlimSummary>, StageError> {
    let image = pinned_image("glim")?;
    let dir = job_dir.join(GLIM_DIR);
    if dir.exists() {
        fs::remove_dir_all(&dir)?;
    }
    fs::create_dir_all(&dir)?;
    let bag = glim_bag::export(reader, &dir.join("bag"))?;
    if bag.cloud_times.is_empty() {
        tracing::warn!(lidar = %bag.lidar_id, "the LiDAR recorded no sweeps; GLIM skipped");
        return Ok(None);
    }
    progress(0.1);
    let track = session_data::index(reader)?.poses;
    if let Some(track) = &track {
        let mut reference: Vec<(u64, Mat4)> = bag
            .cloud_times
            .iter()
            .filter_map(|t| track.at(*t).map(|m| (*t, m)))
            .collect();
        reference.sort_by_key(|r| r.0);
        write_tum(&dir.join(REFERENCE), &reference)?;
    }
    runner
        .run(&copy_config(&dir, &image), &mut |_| {})
        .map_err(|e| {
            StageError::tool_missing(
                "glim",
                format!(
                    "the GLIM config could not be copied from {CONFIG_SOURCE}: {}",
                    e.message
                ),
            )
        })?;
    write_config(&dir.join(STOCK), &dir.join("config"), &bag, odometry)?;
    progress(0.2);
    runner.run(
        &glim_rosbag(&dir, &image, odometry),
        &mut |line| tracing::debug!(target: "glim", "{line}"),
    )?;
    progress(0.9);
    let dumped = dir.join(DUMP).join("traj_imu.txt");
    let poses = read_tum(&dumped).map_err(|e| {
        StageError::tool_failed("glim", format!("no trajectory dump: {}", e.message))
    })?;
    if poses.is_empty() {
        return Err(StageError::tool_failed(
            "glim",
            "the trajectory dump is empty",
        ));
    }
    let summary = apply_trajectory(&dir, dataset, reader.rig(), poses, track.as_ref(), odometry)?;
    progress(1.0);
    Ok(Some(summary))
}

#[path = "glim_poses.rs"]
mod poses;
pub use poses::{
    align, apply_trajectory, max_speed, rewrite_frames, Reposed, MAX_ALIGN_RMSE_M, MAX_SPEED_MPS,
};

#[cfg(any(test, feature = "mock"))]
#[path = "glim_stand_in.rs"]
mod stand_in;
#[cfg(any(test, feature = "mock"))]
pub use stand_in::stand_in;

#[cfg(test)]
#[path = "glim_tests.rs"]
mod tests;
