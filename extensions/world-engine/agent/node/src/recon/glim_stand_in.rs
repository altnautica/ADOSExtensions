//! Stand-in outputs of the GLIM container runs (feature `mock` or tests):
//! the config copy writes a minimal headless config, the offline run dumps
//! the stage's reference trajectory moved into a GLIM-like world (a fixed
//! yaw and offset), so alignment has a known transform to recover.

use std::fs;
use std::io;
use std::path::Path;

use super::super::geom::{self, Mat4};
use super::{read_tum, write_tum, DUMP, REFERENCE, STOCK};
use crate::recon::ToolInvocation;

/// GLIM's world as seen from the session world in the stand-in: 30° yaw and
/// an offset, `T_glim_session`.
pub fn stand_in_offset() -> Mat4 {
    let h = 15f64.to_radians();
    geom::from_quat_trans([h.cos(), 0.0, 0.0, h.sin()], [2.0, -1.0, 0.5])
}

const CONFIG: [(&str, &str); 10] = [
    (
        "config.json",
        r#"{ /* global */ "global": {"config_path": "", "config_ros": "config_ros.json",
            "config_sensors": "config_sensors.json", "config_odometry": "config_odometry_gpu.json",
            "config_sub_mapping": "config_sub_mapping_gpu.json",
            "config_global_mapping": "config_global_mapping_gpu.json"} }"#,
    ),
    (
        "config_ros.json",
        "{\n  // topics\n  \"glim_ros\": {\"imu_topic\": \"/os_cloud_node/imu\",\n    \"points_topic\": \"/os_cloud_node/points\", \"acc_scale\": 0.0, \"ang_scale\": 1.0,\n    \"extension_modules\": [\"libmemory_monitor.so\"]}\n}\n",
    ),
    (
        "config_sensors.json",
        r#"{"sensors": {"T_lidar_imu": [0, 0, 0, 0, 0, 0, 1], "global_shutter_lidar": false,
            "autoconf_perpoint_times": true, "perpoint_relative_time": true, "perpoint_time_scale": 1.0}}"#,
    ),
    ("config_sub_mapping_gpu.json", r#"{"sub_mapping": {"enable_imu": true}}"#),
    ("config_global_mapping_gpu.json", r#"{"global_mapping": {"enable_imu": true}}"#),
    ("config_sub_mapping_cpu.json", r#"{"sub_mapping": {"enable_imu": true}}"#),
    ("config_global_mapping_cpu.json", r#"{"global_mapping": {"enable_imu": true}}"#),
    ("config_odometry_gpu.json", r#"{"odometry_estimation": {"so_name": "libodometry_estimation_gpu.so"}}"#),
    ("config_odometry_cpu.json", r#"{"odometry_estimation": {"so_name": "libodometry_estimation_cpu.so"}}"#),
    ("config_odometry_ct.json", r#"{"odometry_estimation": {"so_name": "libodometry_estimation_ct.so"}}"#),
];

fn bad(msg: impl Into<String>) -> io::Error {
    io::Error::new(io::ErrorKind::InvalidInput, msg.into())
}

/// Poses every 100 ms along +x over the bag's time span (no reference
/// trajectory: the session has no pose track).
fn from_bag(dir: &Path) -> io::Result<Vec<(u64, Mat4)>> {
    let meta: serde_yaml_ng::Value =
        serde_yaml_ng::from_str(&fs::read_to_string(dir.join("bag/metadata.yaml"))?)
            .map_err(|e| bad(e.to_string()))?;
    let info = &meta["rosbag2_bagfile_information"];
    let start = info["starting_time"]["nanoseconds_since_epoch"]
        .as_u64()
        .unwrap_or(0);
    let dur = info["duration"]["nanoseconds"].as_u64().unwrap_or(0);
    Ok((0..=dur / 100_000_000)
        .map(|i| {
            let p = [0.1 * i as f64, 0.0, 0.0];
            (
                start + i * 100_000_000,
                geom::from_quat_trans([1.0, 0.0, 0.0, 0.0], p),
            )
        })
        .collect())
}

/// When `inv` is a GLIM container run, write what it would produce and
/// return `true`.
pub fn stand_in(inv: &ToolInvocation) -> io::Result<bool> {
    if inv.tool != "glim" {
        return Ok(false);
    }
    let args = inv.args_lossy();
    let dir = inv.cwd.as_path();
    if args.iter().any(|a| a == "cp") {
        let stock = dir.join(STOCK);
        fs::create_dir_all(&stock)?;
        for (name, body) in CONFIG {
            fs::write(stock.join(name), body)?;
        }
    } else if args.iter().any(|a| a == "glim_rosbag") {
        for need in [
            "bag/metadata.yaml",
            "config/config.json",
            "config/config_ros.json",
        ] {
            if !dir.join(need).is_file() {
                return Err(bad(format!("stand-in GLIM: {need} is missing")));
            }
        }
        let reference = dir.join(REFERENCE);
        let poses = if reference.is_file() {
            let off = stand_in_offset();
            read_tum(&reference)
                .map_err(|e| bad(e.message))?
                .into_iter()
                .map(|(t, m)| (t, geom::mul(&off, &m)))
                .collect()
        } else {
            from_bag(dir)?
        };
        let dump = dir.join(DUMP);
        fs::create_dir_all(&dump)?;
        for name in ["traj_imu.txt", "traj_lidar.txt"] {
            write_tum(&dump.join(name), &poses).map_err(|e| bad(e.message))?;
        }
    }
    Ok(true)
}
