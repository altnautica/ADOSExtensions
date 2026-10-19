//! A [`LidarScan`] as the `/lidar/<id>/points` `foxglove.PointCloud` of the
//! capture contract.
//!
//! Points are packed with no padding, 17 bytes each, little-endian:
//!
//! | name          | offset | type    |
//! |---------------|-------:|---------|
//! | `x`           |      0 | FLOAT32 |
//! | `y`           |      4 | FLOAT32 |
//! | `z`           |      8 | FLOAT32 |
//! | `intensity`   |     12 | UINT8   |
//! | `t_offset_ns` |     13 | UINT32  |
//!
//! `t_offset_ns` is each point's time after the message timestamp; the cloud
//! is in the sensor frame (`frame_id`), so its pose is the identity.

use ados_capture_contract::foxglove::{
    packed_element_field::NumericType, PackedElementField, PointCloud, Pose, Quaternion, Vector3,
};

use crate::lidar_scan::LidarScan;

/// Bytes per packed point.
pub const POINT_STRIDE: u32 = 17;

fn field(name: &str, offset: u32, ty: NumericType) -> PackedElementField {
    PackedElementField {
        name: name.into(),
        offset,
        r#type: ty as i32,
    }
}

/// The packed cloud of `scan`, stamped `session_t_ns` (the scan's first point
/// on the session timebase). Points past the shortest of the per-point
/// vectors are dropped.
pub fn scan_to_point_cloud(scan: &LidarScan, session_t_ns: u64, frame_id: &str) -> PointCloud {
    let n = scan
        .points
        .len()
        .min(scan.intensity.len())
        .min(scan.t_offset_ns.len());
    let mut data = Vec::with_capacity(n * POINT_STRIDE as usize);
    for ((p, i), o) in scan
        .points
        .iter()
        .zip(&scan.intensity)
        .zip(&scan.t_offset_ns)
        .take(n)
    {
        data.extend_from_slice(&p[0].to_le_bytes());
        data.extend_from_slice(&p[1].to_le_bytes());
        data.extend_from_slice(&p[2].to_le_bytes());
        data.push(*i);
        data.extend_from_slice(&o.to_le_bytes());
    }
    PointCloud {
        timestamp: Some(prost_types::Timestamp {
            seconds: (session_t_ns / 1_000_000_000) as i64,
            nanos: (session_t_ns % 1_000_000_000) as i32,
        }),
        frame_id: frame_id.into(),
        pose: Some(Pose {
            position: Some(Vector3 {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            }),
            orientation: Some(Quaternion {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
            }),
        }),
        point_stride: POINT_STRIDE,
        fields: vec![
            field("x", 0, NumericType::Float32),
            field("y", 4, NumericType::Float32),
            field("z", 8, NumericType::Float32),
            field("intensity", 12, NumericType::Uint8),
            field("t_offset_ns", 13, NumericType::Uint32),
        ],
        data,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn packs_points_with_their_fields() {
        let scan = LidarScan {
            sensor_id: "mid360".into(),
            t_ns: 5,
            points: vec![[1.0, -2.0, 0.5], [0.0, 0.0, 3.25]],
            t_offset_ns: vec![0, 99_000_000],
            intensity: vec![10, 255],
        };
        let pc = scan_to_point_cloud(&scan, 12_345_678_901, "mid360");
        let ts = pc.timestamp.unwrap();
        assert_eq!((ts.seconds, ts.nanos), (12, 345_678_901));
        assert_eq!(pc.frame_id, "mid360");
        assert_eq!(pc.point_stride, 17);
        assert_eq!(pc.data.len(), 34);
        let names: Vec<_> = pc
            .fields
            .iter()
            .map(|f| (f.name.as_str(), f.offset))
            .collect();
        assert_eq!(
            names,
            [
                ("x", 0),
                ("y", 4),
                ("z", 8),
                ("intensity", 12),
                ("t_offset_ns", 13)
            ]
        );
        assert_eq!(pc.fields[3].r#type, NumericType::Uint8 as i32);
        assert_eq!(pc.fields[4].r#type, NumericType::Uint32 as i32);
        assert_eq!(pc.fields[0].r#type, NumericType::Float32 as i32);
        let p1 = &pc.data[17..34];
        assert_eq!(f32::from_le_bytes(p1[8..12].try_into().unwrap()), 3.25);
        assert_eq!(p1[12], 255);
        assert_eq!(
            u32::from_le_bytes(p1[13..17].try_into().unwrap()),
            99_000_000
        );
        assert_eq!(pc.pose.unwrap().orientation.unwrap().w, 1.0);
    }
}
