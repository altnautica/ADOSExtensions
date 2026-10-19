//! Session → ROS 2 bag (MCAP storage) for the offline LiDAR-inertial
//! odometry run: one LiDAR's sweeps as `sensor_msgs/msg/PointCloud2`
//! (`x y z t` float32, `t` in seconds after the sweep stamp) on
//! [`POINTS_TOPIC`] and one IMU's samples as `sensor_msgs/msg/Imu` (body FLU,
//! no orientation estimate) on [`IMU_TOPIC`], CDR little-endian, stamped
//! with the session timebase plus each sensor's rig time offset. Also the
//! TUM trajectory text format (`stamp x y z qx qy qz qw`) the odometry dump
//! writes.

use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::Path;

use ados_capture_contract::foxglove::PointCloud;
use ados_capture_contract::v1::ImuBatch;
use ados_capture_contract::{RigSensorKind, SessionReader};
use mcap::records::MessageHeader;

use super::geom::{self, Mat4};
use super::session_data::decode_points;
use super::StageError;
use crate::calib::ros2_bag::{bag_metadata, cloud_def, mcap_err, Cdr, HEADER_DEF, SEP, TIME_DEF};

/// Topics and frame ids of the exported bag (plain ROS 2 names whatever the
/// rig's sensor ids are).
pub const IMU_TOPIC: &str = "/imu";
pub const POINTS_TOPIC: &str = "/points";
pub const IMU_FRAME: &str = "body";
pub const LIDAR_FRAME: &str = "lidar";

/// What the export wrote.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct GlimBag {
    pub lidar_id: String,
    /// `T_body_lidar` from the rig.
    pub t_body_lidar: Mat4,
    /// The IMU whose samples were written; `None` when the rig has no IMU or
    /// it recorded nothing.
    pub imu_id: Option<String>,
    pub imu_samples: u64,
    /// Stamp of every written sweep, in write order.
    pub cloud_times: Vec<u64>,
    /// At least one sweep carried per-point times (`t_offset_ns`).
    pub point_times: bool,
}

fn imu_def() -> String {
    format!(
        "std_msgs/Header header\ngeometry_msgs/Quaternion orientation\nfloat64[9] orientation_covariance\ngeometry_msgs/Vector3 angular_velocity\nfloat64[9] angular_velocity_covariance\ngeometry_msgs/Vector3 linear_acceleration\nfloat64[9] linear_acceleration_covariance\n{SEP}{HEADER_DEF}{SEP}{TIME_DEF}{SEP}MSG: geometry_msgs/Quaternion\nfloat64 x 0\nfloat64 y 0\nfloat64 z 0\nfloat64 w 1\n{SEP}MSG: geometry_msgs/Vector3\nfloat64 x\nfloat64 y\nfloat64 z\n"
    )
}

/// `sensor_msgs/msg/Imu` with no orientation (covariance[0] = -1).
pub fn imu_cdr(t_ns: u64, frame_id: &str, gyro: [f64; 3], accel: [f64; 3]) -> Vec<u8> {
    let mut c = Cdr::new();
    c.header(t_ns, frame_id);
    for v in [0.0, 0.0, 0.0, 1.0] {
        c.f64(v);
    }
    let cov = |c: &mut Cdr, first: f64| {
        c.f64(first);
        for _ in 0..8 {
            c.f64(0.0);
        }
    };
    cov(&mut c, -1.0);
    gyro.iter().for_each(|v| c.f64(*v));
    cov(&mut c, 0.0);
    accel.iter().for_each(|v| c.f64(*v));
    cov(&mut c, 0.0);
    c.finish()
}

/// `sensor_msgs/msg/PointCloud2` of `x y z t` float32 points.
pub fn timed_points_cdr(t_ns: u64, frame_id: &str, pts: &[[f32; 4]]) -> Vec<u8> {
    let mut c = Cdr::new();
    c.header(t_ns, frame_id);
    c.u32(1);
    c.u32(pts.len() as u32);
    c.u32(4);
    for (i, name) in ["x", "y", "z", "t"].iter().enumerate() {
        c.string(name);
        c.u32(i as u32 * 4);
        c.u8(7); // FLOAT32
        c.u32(1);
    }
    c.u8(0);
    c.u32(16);
    c.u32(16 * pts.len() as u32);
    let data: Vec<u8> = pts
        .iter()
        .flat_map(|p| p.iter().flat_map(|v| v.to_le_bytes()))
        .collect();
    c.bytes(&data);
    c.u8(1);
    c.finish()
}

/// `[x, y, z, t]` of one sweep in the LiDAR frame (the cloud's own `pose`
/// applied), `t` in seconds after the sweep stamp.
fn sweep_points(pc: &PointCloud) -> Result<Vec<[f32; 4]>, StageError> {
    let pose = pc.pose.as_ref().map(|p| {
        let pos = p.position.map(|v| [v.x, v.y, v.z]).unwrap_or([0.0; 3]);
        let q = p
            .orientation
            .map(|o| [o.w, o.x, o.y, o.z])
            .unwrap_or([1.0, 0.0, 0.0, 0.0]);
        geom::from_quat_trans(q, pos)
    });
    Ok(decode_points(pc)?
        .into_iter()
        .map(|(p, dt)| {
            let p = pose.as_ref().map_or(p, |t| geom::apply(t, p));
            [p[0] as f32, p[1] as f32, p[2] as f32, dt as f32 * 1e-9]
        })
        .collect())
}

/// Write `<bag_dir>/<name>_0.mcap` + `metadata.yaml` from the rig's first
/// LiDAR and first IMU.
pub fn export(reader: &SessionReader, bag_dir: &Path) -> Result<GlimBag, StageError> {
    let rig = reader.rig();
    let lidar = rig
        .sensors
        .iter()
        .find(|s| s.kind == RigSensorKind::Lidar)
        .ok_or_else(|| StageError::input("the rig has no LiDAR"))?;
    let imu = rig.sensors.iter().find(|s| s.kind == RigSensorKind::Imu);
    fs::create_dir_all(bag_dir)?;
    let name = bag_dir
        .file_name()
        .map(|n| n.to_string_lossy().into_owned())
        .unwrap_or_else(|| "bag".into());
    let file = format!("{name}_0.mcap");
    let out = BufWriter::new(File::create(bag_dir.join(&file))?);
    let mut w = mcap::WriteOptions::new()
        .compression(None)
        .profile("ros2")
        .library(concat!("world-engine-node/", env!("CARGO_PKG_VERSION")))
        .create(out)
        .map_err(mcap_err)?;
    let meta = BTreeMap::new();
    let s_pc = w
        .add_schema(
            "sensor_msgs/msg/PointCloud2",
            "ros2msg",
            cloud_def().as_bytes(),
        )
        .map_err(mcap_err)?;
    let s_imu = w
        .add_schema("sensor_msgs/msg/Imu", "ros2msg", imu_def().as_bytes())
        .map_err(mcap_err)?;
    let c_pc = w
        .add_channel(s_pc, POINTS_TOPIC, "cdr", &meta)
        .map_err(mcap_err)?;
    let c_imu = w
        .add_channel(s_imu, IMU_TOPIC, "cdr", &meta)
        .map_err(mcap_err)?;
    let pc_topic = ados_capture_contract::topics::lidar_points(&lidar.id);
    let imu_topic = imu.map(|s| ados_capture_contract::topics::imu(&s.id));
    let shift = |t: u64, offset: i64| (t as i64 + offset).max(0) as u64;
    let mut bag = GlimBag {
        lidar_id: lidar.id.clone(),
        t_body_lidar: lidar.t_body_sensor.0,
        ..Default::default()
    };
    let (mut t0, mut t1, mut seq) = (u64::MAX, 0u64, 0u32);
    let mut write = |channel_id: u16, t: u64, data: &[u8]| {
        let header = MessageHeader {
            channel_id,
            sequence: seq,
            log_time: t,
            publish_time: t,
        };
        seq = seq.wrapping_add(1);
        t0 = t0.min(t);
        t1 = t1.max(t);
        w.write_to_known_channel(&header, data).map_err(mcap_err)
    };
    crate::calib::visit(reader, |m| {
        if m.topic == pc_topic {
            let pc: PointCloud = m.decode()?;
            bag.point_times |= pc.fields.iter().any(|f| f.name == "t_offset_ns");
            let t = shift(m.t_ns, lidar.time_offset_ns);
            bag.cloud_times.push(t);
            write(
                c_pc,
                t,
                &timed_points_cdr(t, LIDAR_FRAME, &sweep_points(&pc)?),
            )
        } else if imu_topic.as_deref() == Some(m.topic) {
            let offset = imu.map_or(0, |s| s.time_offset_ns);
            for s in m.decode::<ImuBatch>()?.samples {
                let (Ok(g), Ok(a)) = (<[f64; 3]>::try_from(s.gyro), <[f64; 3]>::try_from(s.accel))
                else {
                    return Err(StageError::input(
                        "an IMU sample lacks three gyro/accel values",
                    ));
                };
                let t = shift(s.t_ns, offset);
                bag.imu_samples += 1;
                write(c_imu, t, &imu_cdr(t, IMU_FRAME, g, a))?;
            }
            Ok(())
        } else {
            Ok(())
        }
    })?;
    w.finish().map_err(mcap_err)?;
    drop(w);
    if bag.imu_samples > 0 {
        bag.imu_id = imu.map(|s| s.id.clone());
    }
    let clouds = bag.cloud_times.len() as u64;
    let (start, dur) = if t0 > t1 { (0, 0) } else { (t0, t1 - t0) };
    let mut topics = vec![(POINTS_TOPIC, "sensor_msgs/msg/PointCloud2", clouds)];
    if bag.imu_samples > 0 {
        topics.push((IMU_TOPIC, "sensor_msgs/msg/Imu", bag.imu_samples));
    }
    fs::write(
        bag_dir.join("metadata.yaml"),
        bag_metadata(&file, start, dur, &topics),
    )?;
    Ok(bag)
}

/// Write a TUM trajectory (`sec.nsec x y z qx qy qz qw` per line).
pub fn write_tum(path: &Path, poses: &[(u64, Mat4)]) -> Result<(), StageError> {
    let mut s = String::new();
    for (t, m) in poses {
        let [qw, qx, qy, qz] = geom::quat_of(m);
        let p = geom::translation(m);
        s.push_str(&format!(
            "{}.{:09} {:.9} {:.9} {:.9} {qx:.9} {qy:.9} {qz:.9} {qw:.9}\n",
            t / 1_000_000_000,
            t % 1_000_000_000,
            p[0],
            p[1],
            p[2]
        ));
    }
    Ok(fs::write(path, s)?)
}

/// `sec[.frac]` → nanoseconds, exactly (no float rounding; digits past the
/// ninth are dropped).
fn stamp_ns(s: &str) -> Option<u64> {
    let (sec, frac) = s.split_once('.').unwrap_or((s, ""));
    if !frac.bytes().all(|b| b.is_ascii_digit()) {
        return None;
    }
    let digits = &frac[..frac.len().min(9)];
    let nanos = match digits {
        "" => 0,
        d => d.parse::<u64>().ok()? * 10u64.pow(9 - d.len() as u32),
    };
    sec.parse::<u64>()
        .ok()?
        .checked_mul(1_000_000_000)?
        .checked_add(nanos)
}

/// Read a TUM trajectory, sorted by time; blank and `#` lines are skipped.
pub fn read_tum(path: &Path) -> Result<Vec<(u64, Mat4)>, StageError> {
    let text = fs::read_to_string(path)?;
    let mut out = Vec::new();
    for (i, line) in text.lines().enumerate() {
        let line = line.trim();
        if line.is_empty() || line.starts_with('#') {
            continue;
        }
        let bad = || StageError::io(format!("{}:{}: not a TUM pose line", path.display(), i + 1));
        let mut it = line.split_whitespace();
        let t = it.next().and_then(stamp_ns).ok_or_else(bad)?;
        let v: Vec<f64> = it
            .map(str::parse)
            .collect::<Result<_, _>>()
            .map_err(|_| bad())?;
        let [x, y, z, qx, qy, qz, qw] = <[f64; 7]>::try_from(v).map_err(|_| bad())?;
        out.push((t, geom::from_quat_trans([qw, qx, qy, qz], [x, y, z])));
    }
    out.sort_by_key(|(t, _)| *t);
    out.dedup_by_key(|(t, _)| *t);
    Ok(out)
}

#[cfg(test)]
#[path = "glim_bag_tests.rs"]
mod tests;
