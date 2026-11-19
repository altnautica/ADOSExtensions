//! One LiDAR scan as the capture path carries it between the driver, the
//! LiDAR pose provider and the recorder.

/// A scan in the sensor's native FLU frame.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct LidarScan {
    /// Rig sensor id (`rig.json` `sensors[].id`).
    pub sensor_id: String,
    /// Host monotonic nanoseconds of the first point.
    pub t_ns: u64,
    /// Points, metres.
    pub points: Vec<[f32; 3]>,
    /// Per-point offset from `t_ns`, nanoseconds (same length as `points`).
    pub t_offset_ns: Vec<u32>,
    /// Per-point reflectivity 0..255 (same length as `points`).
    pub intensity: Vec<u8>,
}

impl LidarScan {
    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }
}
