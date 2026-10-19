//! Session → ROS 2 bag (MCAP storage) of one camera and one LiDAR, the input
//! the LiDAR-camera calibration preprocesses: `sensor_msgs/msg/Image` on
//! [`DVLC_IMAGE_TOPIC`] and `sensor_msgs/msg/PointCloud2` (`x y z intensity`,
//! float32) on [`DVLC_POINTS_TOPIC`], CDR little-endian, stamped with the
//! session timebase. The bag is a directory holding `metadata.yaml` and one
//! `.mcap` file.

use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::Path;

use ados_capture_contract::foxglove::packed_element_field::NumericType;
use ados_capture_contract::foxglove::PointCloud;
use ados_capture_contract::SessionReader;
use mcap::records::MessageHeader;

use super::container::{DVLC_IMAGE_TOPIC, DVLC_POINTS_TOPIC};
use super::export::{frame_from_message, Frame};
use super::visit;
use crate::recon::StageError;

pub(crate) const TIME_DEF: &str = "MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n";
pub(crate) const HEADER_DEF: &str =
    "MSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n";
pub(crate) const SEP: &str =
    "================================================================================\n";

fn image_def() -> String {
    format!(
        "std_msgs/Header header\nuint32 height\nuint32 width\nstring encoding\nuint8 is_bigendian\nuint32 step\nuint8[] data\n{SEP}{HEADER_DEF}{SEP}{TIME_DEF}"
    )
}

pub(crate) fn cloud_def() -> String {
    format!(
        "std_msgs/Header header\nuint32 height\nuint32 width\nPointField[] fields\nbool is_bigendian\nuint32 point_step\nuint32 row_step\nuint8[] data\nbool is_dense\n{SEP}{HEADER_DEF}{SEP}{TIME_DEF}{SEP}MSG: sensor_msgs/PointField\nuint8 INT8=1\nuint8 UINT8=2\nuint8 INT16=3\nuint8 UINT16=4\nuint8 INT32=5\nuint8 UINT32=6\nuint8 FLOAT32=7\nuint8 FLOAT64=8\nstring name\nuint32 offset\nuint8 datatype\nuint32 count\n"
    )
}

/// Little-endian CDR (XCDR1) encoder; alignment is relative to the byte after
/// the 4-byte encapsulation header.
pub struct Cdr(Vec<u8>);

impl Cdr {
    pub fn new() -> Self {
        Self(vec![0x00, 0x01, 0x00, 0x00])
    }

    fn align(&mut self, n: usize) {
        while !(self.0.len() - 4).is_multiple_of(n) {
            self.0.push(0);
        }
    }

    pub fn u8(&mut self, v: u8) {
        self.0.push(v);
    }

    pub fn u32(&mut self, v: u32) {
        self.align(4);
        self.0.extend_from_slice(&v.to_le_bytes());
    }

    pub fn i32(&mut self, v: i32) {
        self.align(4);
        self.0.extend_from_slice(&v.to_le_bytes());
    }

    pub fn f64(&mut self, v: f64) {
        self.align(8);
        self.0.extend_from_slice(&v.to_le_bytes());
    }

    pub fn string(&mut self, s: &str) {
        self.u32(s.len() as u32 + 1);
        self.0.extend_from_slice(s.as_bytes());
        self.0.push(0);
    }

    pub fn bytes(&mut self, b: &[u8]) {
        self.u32(b.len() as u32);
        self.0.extend_from_slice(b);
    }

    pub fn header(&mut self, t_ns: u64, frame_id: &str) {
        self.i32((t_ns / 1_000_000_000) as i32);
        self.u32((t_ns % 1_000_000_000) as u32);
        self.string(frame_id);
    }

    pub fn finish(self) -> Vec<u8> {
        self.0
    }
}

impl Default for Cdr {
    fn default() -> Self {
        Self::new()
    }
}

/// `sensor_msgs/msg/Image` (`mono8` or `rgb8`).
pub fn image_cdr(t_ns: u64, frame_id: &str, f: &Frame) -> Vec<u8> {
    let (w, h, enc, ch, data) = match f {
        Frame::Mono8(g) => (g.width(), g.height(), "mono8", 1, g.as_raw()),
        Frame::Rgb8(c) => (c.width(), c.height(), "rgb8", 3, c.as_raw()),
    };
    let mut c = Cdr::new();
    c.header(t_ns, frame_id);
    c.u32(h);
    c.u32(w);
    c.string(enc);
    c.u8(0);
    c.u32(w * ch);
    c.bytes(data);
    c.finish()
}

/// `sensor_msgs/msg/PointCloud2` of `[x, y, z, intensity]` float32 points.
pub fn points_cdr(t_ns: u64, frame_id: &str, pts: &[[f32; 4]]) -> Vec<u8> {
    let mut c = Cdr::new();
    c.header(t_ns, frame_id);
    c.u32(1);
    c.u32(pts.len() as u32);
    c.u32(4);
    for (i, name) in ["x", "y", "z", "intensity"].iter().enumerate() {
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

fn read_field(data: &[u8], at: usize, ty: i32) -> Option<f64> {
    let b = |n: usize| data.get(at..at + n);
    Some(match NumericType::try_from(ty).ok()? {
        NumericType::Uint8 => *data.get(at)? as f64,
        NumericType::Int8 => *data.get(at)? as i8 as f64,
        NumericType::Uint16 => u16::from_le_bytes(b(2)?.try_into().ok()?) as f64,
        NumericType::Int16 => i16::from_le_bytes(b(2)?.try_into().ok()?) as f64,
        NumericType::Uint32 => u32::from_le_bytes(b(4)?.try_into().ok()?) as f64,
        NumericType::Int32 => i32::from_le_bytes(b(4)?.try_into().ok()?) as f64,
        NumericType::Float32 => f32::from_le_bytes(b(4)?.try_into().ok()?) as f64,
        NumericType::Float64 => f64::from_le_bytes(b(8)?.try_into().ok()?),
        NumericType::Unknown => return None,
    })
}

/// `[x, y, z, intensity]` of a `foxglove.PointCloud` in its frame (the
/// cloud's `pose` applied); intensity 0 when the cloud has none.
pub fn cloud_points(pc: &PointCloud) -> Result<Vec<[f32; 4]>, StageError> {
    let field = |name: &str| pc.fields.iter().find(|f| f.name == name);
    let (Some(fx), Some(fy), Some(fz)) = (field("x"), field("y"), field("z")) else {
        return Err(StageError::input("point cloud lacks x/y/z fields"));
    };
    let fi = field("intensity");
    let stride = pc.point_stride as usize;
    if stride == 0 {
        return Err(StageError::input("point cloud stride is 0"));
    }
    let (p, q) = match &pc.pose {
        Some(pose) => (
            pose.position.map(|v| [v.x, v.y, v.z]).unwrap_or([0.0; 3]),
            pose.orientation
                .map(|o| [o.x, o.y, o.z, o.w])
                .unwrap_or([0.0, 0.0, 0.0, 1.0]),
        ),
        None => ([0.0; 3], [0.0, 0.0, 0.0, 1.0]),
    };
    let t = super::geom::from_xyz_quat(&[p[0], p[1], p[2], q[0], q[1], q[2], q[3]]);
    let mut out = Vec::with_capacity(pc.data.len() / stride);
    for base in (0..pc.data.len() / stride).map(|i| i * stride) {
        let get = |f: &ados_capture_contract::foxglove::PackedElementField| {
            read_field(&pc.data, base + f.offset as usize, f.r#type)
        };
        let (Some(x), Some(y), Some(z)) = (get(fx), get(fy), get(fz)) else {
            return Err(StageError::input("point cloud field outside its point"));
        };
        if !(x.is_finite() && y.is_finite() && z.is_finite()) {
            continue;
        }
        let i = fi.and_then(get).unwrap_or(0.0);
        let w = [
            t[0] * x + t[1] * y + t[2] * z + t[3],
            t[4] * x + t[5] * y + t[6] * z + t[7],
            t[8] * x + t[9] * y + t[10] * z + t[11],
        ];
        out.push([w[0] as f32, w[1] as f32, w[2] as f32, i as f32]);
    }
    Ok(out)
}

/// What a bag export wrote.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BagExport {
    pub images: u64,
    pub clouds: u64,
}

pub(crate) fn mcap_err(e: mcap::McapError) -> StageError {
    StageError::io(format!("mcap: {e}"))
}

/// Write `<bag_dir>/<name>_0.mcap` + `metadata.yaml` from `camera_id`'s
/// images and `lidar_id`'s clouds.
pub fn export_ros2_bag(
    reader: &SessionReader,
    camera_id: &str,
    lidar_id: &str,
    bag_dir: &Path,
) -> Result<BagExport, StageError> {
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
    let s_img = w
        .add_schema("sensor_msgs/msg/Image", "ros2msg", image_def().as_bytes())
        .map_err(mcap_err)?;
    let s_pc = w
        .add_schema(
            "sensor_msgs/msg/PointCloud2",
            "ros2msg",
            cloud_def().as_bytes(),
        )
        .map_err(mcap_err)?;
    let c_img = w
        .add_channel(s_img, DVLC_IMAGE_TOPIC, "cdr", &meta)
        .map_err(mcap_err)?;
    let c_pc = w
        .add_channel(s_pc, DVLC_POINTS_TOPIC, "cdr", &meta)
        .map_err(mcap_err)?;
    let img_topic = format!("/cam/{camera_id}/image");
    let pc_topic = format!("/lidar/{lidar_id}/points");
    let mut n = BagExport {
        images: 0,
        clouds: 0,
    };
    let (mut t0, mut t1, mut seq) = (u64::MAX, 0u64, 0u32);
    visit(reader, |m| {
        let (channel_id, data) = if m.topic == img_topic {
            n.images += 1;
            (
                c_img,
                image_cdr(m.t_ns, camera_id, &frame_from_message(&m)?),
            )
        } else if m.topic == pc_topic {
            n.clouds += 1;
            let pc: PointCloud = m.decode()?;
            (c_pc, points_cdr(m.t_ns, lidar_id, &cloud_points(&pc)?))
        } else {
            return Ok(());
        };
        let header = MessageHeader {
            channel_id,
            sequence: seq,
            log_time: m.t_ns,
            publish_time: m.t_ns,
        };
        seq = seq.wrapping_add(1);
        t0 = t0.min(m.t_ns);
        t1 = t1.max(m.t_ns);
        w.write_to_known_channel(&header, &data).map_err(mcap_err)
    })?;
    w.finish().map_err(mcap_err)?;
    drop(w);
    let total = n.images + n.clouds;
    let (start, dur) = if total == 0 { (0, 0) } else { (t0, t1 - t0) };
    let metadata = bag_metadata(
        &file,
        start,
        dur,
        &[
            (DVLC_IMAGE_TOPIC, "sensor_msgs/msg/Image", n.images),
            (DVLC_POINTS_TOPIC, "sensor_msgs/msg/PointCloud2", n.clouds),
        ],
    );
    fs::write(bag_dir.join("metadata.yaml"), metadata)?;
    Ok(n)
}

/// `metadata.yaml` of a one-file MCAP bag: `(topic, type, message count)` per
/// topic, the bag's first message time and duration in nanoseconds.
pub(crate) fn bag_metadata(
    file: &str,
    start: u64,
    dur: u64,
    topics: &[(&str, &str, u64)],
) -> String {
    let total: u64 = topics.iter().map(|t| t.2).sum();
    let listed: String = topics
        .iter()
        .map(|(name, ty, count)| {
            format!(
                "    - topic_metadata:\n        name: {name}\n        type: {ty}\n        serialization_format: cdr\n        offered_qos_profiles: \"\"\n      message_count: {count}\n"
            )
        })
        .collect();
    format!(
        "rosbag2_bagfile_information:\n  version: 5\n  storage_identifier: mcap\n  duration:\n    nanoseconds: {dur}\n  starting_time:\n    nanoseconds_since_epoch: {start}\n  message_count: {total}\n  topics_with_message_count:\n{listed}  compression_format: \"\"\n  compression_mode: \"\"\n  relative_file_paths:\n    - {file}\n  files:\n    - path: {file}\n      starting_time:\n        nanoseconds_since_epoch: {start}\n      duration:\n        nanoseconds: {dur}\n      message_count: {total}\n"
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use image::GrayImage;

    #[test]
    fn image_cdr_layout() {
        let f = Frame::Mono8(GrayImage::from_raw(2, 1, vec![7, 9]).unwrap());
        let b = image_cdr(1_500_000_000, "cam0", &f);
        let mut want = vec![0, 1, 0, 0];
        want.extend(1i32.to_le_bytes());
        want.extend(500_000_000u32.to_le_bytes());
        want.extend(5u32.to_le_bytes());
        want.extend(b"cam0\0");
        want.extend([0, 0, 0]); // align to 4
        want.extend(1u32.to_le_bytes()); // height
        want.extend(2u32.to_le_bytes()); // width
        want.extend(6u32.to_le_bytes());
        want.extend(b"mono8\0");
        want.push(0); // is_bigendian
        want.push(0); // align to 4
        want.extend(2u32.to_le_bytes()); // step
        want.extend(2u32.to_le_bytes());
        want.extend([7, 9]);
        assert_eq!(b, want);
    }

    #[test]
    fn point_cloud_is_converted_with_its_pose() {
        use ados_capture_contract::foxglove::{PackedElementField, Pose, Vector3};
        let field = |name: &str, offset: u32| PackedElementField {
            name: name.into(),
            offset,
            r#type: NumericType::Float32 as i32,
        };
        let data: Vec<u8> = [1.0f32, 2.0, 3.0, 0.5]
            .iter()
            .flat_map(|v| v.to_le_bytes())
            .collect();
        let pc = PointCloud {
            timestamp: None,
            frame_id: "lidar0".into(),
            pose: Some(Pose {
                position: Some(Vector3 {
                    x: 10.0,
                    y: 0.0,
                    z: 0.0,
                }),
                orientation: None,
            }),
            point_stride: 16,
            fields: vec![
                field("x", 0),
                field("y", 4),
                field("z", 8),
                field("intensity", 12),
            ],
            data,
        };
        assert_eq!(cloud_points(&pc).unwrap(), vec![[11.0, 2.0, 3.0, 0.5]]);
        let b = points_cdr(0, "lidar0", &cloud_points(&pc).unwrap());
        // Trailing bytes: the point (16 bytes) then is_dense.
        assert_eq!(b[b.len() - 17..b.len() - 13], 11.0f32.to_le_bytes());
        assert_eq!(*b.last().unwrap(), 1);
    }
}
