//! One planar scan in its compact polar form: the record a capture session
//! stores (`lidar/<sensor>.polar.zst`, a zstd stream of records).
//!
//! Record layout, little-endian, no padding:
//!
//! | field             | type        | note                                    |
//! |-------------------|-------------|-----------------------------------------|
//! | `t_ns`            | `u64`       | first point, session or host clock      |
//! | `angle_start_rad` | `f32`       | bin 0, clockwise from body +x (FRD yaw) |
//! | `angle_inc_rad`   | `f32`       |                                         |
//! | `n`               | `u32`       | bins                                    |
//! | `t`               | `[f32; 3]`  | sensor origin in `map_ned`, metres      |
//! | `q`               | `[f32; 4]`  | `w, x, y, z`, world-from-sensor (FRD)   |
//! | `health`          | `u8`        | 0 good, 1 degraded, 2 lost              |
//! | bins              | `n × 3 B`   | `u16` range in mm (0 = none), `u8` quality |
//!
//! Three bytes a point against seventeen for the packed Cartesian cloud.

use serde::{Deserialize, Serialize};

use super::VioHealth;

/// Bytes before the bins.
pub const POLAR_HEADER_LEN: usize = 8 + 4 + 4 + 4 + 12 + 16 + 1;
/// Bytes per bin.
pub const POLAR_BIN_LEN: usize = 3;

/// One planar scan in polar bins (sensor FRD frame) with the sensor pose it
/// was taken at.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PolarScan {
    pub t_ns: u64,
    pub angle_start_rad: f32,
    pub angle_inc_rad: f32,
    /// Sensor origin in `map_ned`.
    pub t: [f32; 3],
    /// World-from-sensor (FRD) rotation, `w, x, y, z`.
    pub q: [f32; 4],
    pub health: VioHealth,
    /// Range per bin, millimetres; 0 = no return.
    pub range_mm: Vec<u16>,
    /// Return quality per bin (0..255), same length as `range_mm`.
    pub quality: Vec<u8>,
}

/// A record that does not parse.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum PolarError {
    #[error("polar record truncated: need {need} bytes, have {have}")]
    Truncated { need: usize, have: usize },
    #[error("polar record health byte {0} is not 0, 1 or 2")]
    Health(u8),
}

fn health_byte(h: VioHealth) -> u8 {
    match h {
        VioHealth::Good => 0,
        VioHealth::Degraded => 1,
        VioHealth::Lost => 2,
    }
}

impl PolarScan {
    /// The bytes [`PolarScan::encode`] appends.
    pub fn encoded_len(&self) -> usize {
        POLAR_HEADER_LEN + self.range_mm.len() * POLAR_BIN_LEN
    }

    /// Append the record to `out`. Bins past the shorter of `range_mm` and
    /// `quality` are dropped.
    pub fn encode(&self, out: &mut Vec<u8>) {
        let n = self.range_mm.len().min(self.quality.len());
        out.reserve(POLAR_HEADER_LEN + n * POLAR_BIN_LEN);
        out.extend_from_slice(&self.t_ns.to_le_bytes());
        out.extend_from_slice(&self.angle_start_rad.to_le_bytes());
        out.extend_from_slice(&self.angle_inc_rad.to_le_bytes());
        out.extend_from_slice(&(n as u32).to_le_bytes());
        for v in self.t.iter().chain(&self.q) {
            out.extend_from_slice(&v.to_le_bytes());
        }
        out.push(health_byte(self.health));
        for (r, q) in self.range_mm.iter().zip(&self.quality).take(n) {
            out.extend_from_slice(&r.to_le_bytes());
            out.push(*q);
        }
    }

    /// Parse one record from the front of `buf`, returning it and the bytes
    /// it used.
    pub fn decode(buf: &[u8]) -> Result<(Self, usize), PolarError> {
        if buf.len() < POLAR_HEADER_LEN {
            return Err(PolarError::Truncated {
                need: POLAR_HEADER_LEN,
                have: buf.len(),
            });
        }
        let f32_at = |o: usize| f32::from_le_bytes(buf[o..o + 4].try_into().expect("4 bytes"));
        let t_ns = u64::from_le_bytes(buf[0..8].try_into().expect("8 bytes"));
        let n = u32::from_le_bytes(buf[16..20].try_into().expect("4 bytes")) as usize;
        let need = POLAR_HEADER_LEN + n * POLAR_BIN_LEN;
        if buf.len() < need {
            return Err(PolarError::Truncated {
                need,
                have: buf.len(),
            });
        }
        let health = match buf[POLAR_HEADER_LEN - 1] {
            0 => VioHealth::Good,
            1 => VioHealth::Degraded,
            2 => VioHealth::Lost,
            b => return Err(PolarError::Health(b)),
        };
        let mut range_mm = Vec::with_capacity(n);
        let mut quality = Vec::with_capacity(n);
        for k in 0..n {
            let o = POLAR_HEADER_LEN + k * POLAR_BIN_LEN;
            range_mm.push(u16::from_le_bytes([buf[o], buf[o + 1]]));
            quality.push(buf[o + 2]);
        }
        Ok((
            Self {
                t_ns,
                angle_start_rad: f32_at(8),
                angle_inc_rad: f32_at(12),
                t: [f32_at(20), f32_at(24), f32_at(28)],
                q: [f32_at(32), f32_at(36), f32_at(40), f32_at(44)],
                health,
                range_mm,
                quality,
            },
            need,
        ))
    }

    /// Every record in a decompressed stream; a truncated tail (a session cut
    /// by power loss) ends the list rather than failing it.
    pub fn decode_all(mut buf: &[u8]) -> Result<Vec<Self>, PolarError> {
        let mut out = Vec::new();
        while !buf.is_empty() {
            match Self::decode(buf) {
                Ok((scan, used)) => {
                    out.push(scan);
                    buf = &buf[used..];
                }
                Err(PolarError::Truncated { .. }) => break,
                Err(e) => return Err(e),
            }
        }
        Ok(out)
    }

    /// The bin's endpoint in the sensor FRD frame, metres, or `None` for no
    /// return.
    pub fn body_point(&self, bin: usize) -> Option<[f32; 2]> {
        let r = *self.range_mm.get(bin)?;
        if r == 0 {
            return None;
        }
        let a = self.angle_start_rad + bin as f32 * self.angle_inc_rad;
        let m = r as f32 * 1e-3;
        Some([m * a.cos(), m * a.sin()])
    }

    /// World-from-sensor rotation as a row-major 3×3.
    pub fn rotation(&self) -> [f32; 9] {
        let [w, x, y, z] = self.q;
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - w * z),
            2.0 * (x * z + w * y),
            2.0 * (x * y + w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - w * x),
            2.0 * (x * z - w * y),
            2.0 * (y * z + w * x),
            1.0 - 2.0 * (x * x + y * y),
        ]
    }

    /// Every return in `map_ned`, metres.
    pub fn world_points(&self) -> impl Iterator<Item = [f32; 3]> + '_ {
        let r = self.rotation();
        (0..self.range_mm.len()).filter_map(move |k| {
            let [bx, by] = self.body_point(k)?;
            Some([
                r[0] * bx + r[1] * by + self.t[0],
                r[3] * bx + r[4] * by + self.t[1],
                r[6] * bx + r[7] * by + self.t[2],
            ])
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn scan() -> PolarScan {
        PolarScan {
            t_ns: 42,
            angle_start_rad: 0.0,
            angle_inc_rad: std::f32::consts::FRAC_PI_2,
            t: [1.0, 2.0, -3.0],
            q: [1.0, 0.0, 0.0, 0.0],
            health: VioHealth::Degraded,
            range_mm: vec![1000, 0, 2500, 65535],
            quality: vec![200, 0, 90, 10],
        }
    }

    #[test]
    fn a_record_round_trips_at_three_bytes_a_bin() {
        let s = scan();
        let mut buf = Vec::new();
        s.encode(&mut buf);
        assert_eq!(buf.len(), POLAR_HEADER_LEN + 4 * 3);
        assert_eq!(buf.len(), s.encoded_len());
        let (back, used) = PolarScan::decode(&buf).unwrap();
        assert_eq!(used, buf.len());
        assert_eq!(back, s);
    }

    #[test]
    fn a_stream_decodes_and_a_cut_tail_is_dropped() {
        let mut buf = Vec::new();
        scan().encode(&mut buf);
        scan().encode(&mut buf);
        let whole = buf.len();
        buf.truncate(whole - 2);
        assert_eq!(PolarScan::decode_all(&buf).unwrap().len(), 1);
        assert!(PolarScan::decode_all(&[]).unwrap().is_empty());
    }

    #[test]
    fn returns_land_in_the_world_through_the_pose() {
        let s = scan();
        let pts: Vec<_> = s.world_points().collect();
        // Bin 1 is empty; bin 2 points along body -x (180°).
        assert_eq!(pts.len(), 3);
        assert!((pts[0][0] - 2.0).abs() < 1e-5 && (pts[0][1] - 2.0).abs() < 1e-5);
        assert!((pts[1][0] + 1.5).abs() < 1e-5);
        assert!((pts[0][2] + 3.0).abs() < 1e-6);
    }
}
