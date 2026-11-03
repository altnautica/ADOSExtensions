//! Planar scan to `OBSTACLE_DISTANCE` (#330).
//!
//! The scan topic carries `ranges_m[i]` at `angle_start_rad + i *
//! angle_increment_rad`, clockwise from body +x (FRD yaw), `0.0` for no
//! return. Each sector `k` covers `[k*w - w/2, k*w + w/2)` around its
//! centre `k*w` (`w = 360 / sectors` degrees) and reports the nearest
//! return in centimetres. A sector with no return in range reads
//! `max_distance + 1` (MAVLink: nothing seen up to max); slots past
//! `sectors` read `UINT16_MAX` (unused). The message goes out in
//! `MAV_FRAME_BODY_FRD` with `increment_f` set, which ArduPilot's MAVLink
//! proximity backend reads ahead of the integer `increment`.

use rmpv::Value;

use crate::external_nav::COMPONENT_EXTNAV;
use crate::mavlink_emit::encode_v2;
use crate::nav_config::{as_f64, MAX_SECTORS};

pub const OBSTACLE_DISTANCE_ID: u32 = 330;
pub const OBSTACLE_DISTANCE_CRC_EXTRA: u8 = 23;
/// Base fields (158 bytes) plus `increment_f`, `angle_offset`, `frame`.
pub const OBSTACLE_DISTANCE_LEN: usize = 167;
/// Reported sensor range, cm (a 0.05-12 m planar lidar).
pub const MIN_DISTANCE_CM: u16 = 5;
pub const MAX_DISTANCE_CM: u16 = 1200;
const MAV_FRAME_BODY_FRD: u8 = 12;
const MAV_DISTANCE_SENSOR_LASER: u8 = 0;

/// One decoded scan.
#[derive(Debug, Clone, PartialEq)]
pub struct Scan {
    pub t_ns: i64,
    pub angle_start_rad: f64,
    pub angle_increment_rad: f64,
    pub ranges_m: Vec<f32>,
}

fn get<'a>(v: &'a Value, key: &str) -> Option<&'a Value> {
    match v {
        Value::Map(e) => e
            .iter()
            .find(|(k, _)| k.as_str() == Some(key))
            .map(|(_, val)| val),
        _ => None,
    }
}

/// Parse an `event.subscribe` delivery of the scan topic. `None` for a
/// payload that is not a `body_frd` scan.
pub fn parse_scan_event(args: &Value) -> Option<Scan> {
    let p = get(args, "payload")?;
    if get(p, "frame").and_then(Value::as_str) != Some("body_frd") {
        return None;
    }
    let Value::Array(raw) = get(p, "ranges_m")? else {
        return None;
    };
    let increment = get(p, "angle_increment_rad").and_then(as_f64)?;
    if !(increment.is_finite() && increment > 0.0) {
        return None;
    }
    Some(Scan {
        t_ns: get(p, "t_ns").and_then(Value::as_i64).unwrap_or(0),
        angle_start_rad: get(p, "angle_start_rad").and_then(as_f64).unwrap_or(0.0),
        angle_increment_rad: increment,
        ranges_m: raw
            .iter()
            .map(|v| as_f64(v).unwrap_or(0.0) as f32)
            .collect(),
    })
}

/// The `OBSTACLE_DISTANCE` fields, extensions included.
#[derive(Debug, Clone, PartialEq)]
pub struct ObstacleDistance {
    pub time_usec: u64,
    pub distances: [u16; 72],
    pub increment_deg: f32,
}

impl ObstacleDistance {
    /// Nearest obstacle in metres across the used sectors, `None` when
    /// nothing is in range.
    pub fn min_obstacle_m(&self) -> Option<f32> {
        self.distances
            .iter()
            .filter(|&&d| (MIN_DISTANCE_CM..=MAX_DISTANCE_CM).contains(&d))
            .min()
            .map(|&d| f32::from(d) / 100.0)
    }

    /// MAVLink 2 frame on component 197.
    pub fn encode(&self, sequence: u8) -> Vec<u8> {
        let mut p = Vec::with_capacity(OBSTACLE_DISTANCE_LEN);
        p.extend_from_slice(&self.time_usec.to_le_bytes());
        for d in self.distances {
            p.extend_from_slice(&d.to_le_bytes());
        }
        p.extend_from_slice(&MIN_DISTANCE_CM.to_le_bytes());
        p.extend_from_slice(&MAX_DISTANCE_CM.to_le_bytes());
        p.push(MAV_DISTANCE_SENSOR_LASER);
        p.push(self.increment_deg.round().clamp(1.0, 255.0) as u8);
        p.extend_from_slice(&self.increment_deg.to_le_bytes());
        p.extend_from_slice(&0f32.to_le_bytes()); // angle_offset
        p.push(MAV_FRAME_BODY_FRD);
        encode_v2(
            COMPONENT_EXTNAV,
            sequence,
            OBSTACLE_DISTANCE_ID,
            OBSTACLE_DISTANCE_CRC_EXTRA,
            p,
        )
    }
}

/// Bin a scan into `sectors` equal sectors (1-72).
pub fn sectorise(scan: &Scan, sectors: u8, time_usec: u64) -> ObstacleDistance {
    let sectors = sectors.clamp(1, MAX_SECTORS) as usize;
    let width = 360.0 / sectors as f64;
    let no_return = MAX_DISTANCE_CM + 1;
    let mut distances = [u16::MAX; 72];
    distances[..sectors].fill(no_return);
    let (min_m, max_m) = (
        f64::from(MIN_DISTANCE_CM) / 100.0,
        f64::from(MAX_DISTANCE_CM) / 100.0,
    );
    for (i, &r) in scan.ranges_m.iter().enumerate() {
        let r = f64::from(r);
        if !(r.is_finite() && r >= min_m && r <= max_m) {
            continue;
        }
        let deg = (scan.angle_start_rad + i as f64 * scan.angle_increment_rad).to_degrees();
        let shifted = (deg + width / 2.0).rem_euclid(360.0);
        let k = ((shifted / width) as usize).min(sectors - 1);
        let cm = (r * 100.0).round() as u16;
        distances[k] = distances[k].min(cm);
    }
    ObstacleDistance {
        time_usec,
        distances,
        increment_deg: width as f32,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mavlink_emit::x25_crc;
    use ados_protocol::mavlink::{parse_v2, MavMessage};

    fn scan(ranges: Vec<f32>) -> Scan {
        let n = ranges.len() as f64;
        Scan {
            t_ns: 0,
            angle_start_rad: 0.0,
            angle_increment_rad: std::f64::consts::TAU / n,
            ranges_m: ranges,
        }
    }

    #[test]
    fn empty_scan_reads_no_return_everywhere() {
        let o = sectorise(&scan(vec![0.0; 360]), 72, 0);
        assert!(o.distances.iter().all(|&d| d == MAX_DISTANCE_CM + 1));
        assert_eq!(o.min_obstacle_m(), None);
        assert_eq!(o.increment_deg, 5.0);
    }

    #[test]
    fn returns_land_in_the_sector_around_their_bearing() {
        let mut r = vec![0.0; 360];
        r[0] = 3.0; // dead ahead
        r[90] = 1.5; // right (FRD: clockwise)
        r[358] = 2.0; // 2 deg left of ahead: still sector 0
        r[183] = 20.0; // beyond range: ignored
        r[270] = 0.02; // under min range: ignored
        let o = sectorise(&scan(r), 72, 7);
        assert_eq!(o.distances[0], 200);
        assert_eq!(o.distances[18], 150);
        assert_eq!(o.distances[36], MAX_DISTANCE_CM + 1);
        assert_eq!(o.distances[54], MAX_DISTANCE_CM + 1);
        assert_eq!(o.min_obstacle_m(), Some(1.5));
        // Sector 0 spans [-2.5, 2.5) deg; 3 deg is sector 1, -3 deg sector 71.
        let mut edge = vec![0.0; 720];
        edge[6] = 4.0; // 3 deg
        edge[714] = 4.5; // -3 deg
        let o = sectorise(&scan(edge), 72, 0);
        assert_eq!(o.distances[1], 400);
        assert_eq!(o.distances[71], 450);
        assert_eq!(o.distances[0], MAX_DISTANCE_CM + 1);
    }

    #[test]
    fn fewer_sectors_leave_unused_slots() {
        let mut r = vec![0.0; 360];
        r[100] = 5.0;
        let o = sectorise(&scan(r), 8, 0);
        assert_eq!(o.increment_deg, 45.0);
        assert_eq!(o.distances[2], 500); // 100 deg -> sector centred on 90
        assert!(o.distances[8..].iter().all(|&d| d == u16::MAX));
    }

    #[test]
    fn scan_event_parses_body_frd_only() {
        let payload = |frame: &str| {
            Value::Map(vec![(
                Value::from("payload"),
                Value::Map(vec![
                    (Value::from("t_ns"), Value::from(5i64)),
                    (Value::from("frame"), Value::from(frame)),
                    (Value::from("angle_start_rad"), Value::F64(0.0)),
                    (Value::from("angle_increment_rad"), Value::F64(0.017453)),
                    (
                        Value::from("ranges_m"),
                        Value::Array(vec![Value::F32(1.0), Value::F64(2.0)]),
                    ),
                ]),
            )])
        };
        let s = parse_scan_event(&payload("body_frd")).unwrap();
        assert_eq!(s.ranges_m, vec![1.0, 2.0]);
        assert_eq!(s.t_ns, 5);
        assert!(parse_scan_event(&payload("body_flu")).is_none());
    }

    #[test]
    fn obstacle_frame_packs_extension_fields() {
        let mut r = vec![0.0; 360];
        r[0] = 3.0;
        let frame = sectorise(&scan(r), 72, 99).encode(3);
        assert_eq!(frame[1] as usize, OBSTACLE_DISTANCE_LEN);
        assert_eq!(frame[6], COMPONENT_EXTNAV);
        assert_eq!(u32::from_le_bytes([frame[7], frame[8], frame[9], 0]), 330);
        let p = &frame[10..10 + OBSTACLE_DISTANCE_LEN];
        assert_eq!(p[157], 5); // increment
        assert_eq!(f32::from_le_bytes(p[158..162].try_into().unwrap()), 5.0);
        assert_eq!(p[166], MAV_FRAME_BODY_FRD);
        let crc_at = 10 + OBSTACLE_DISTANCE_LEN;
        let crc = u16::from_le_bytes([frame[crc_at], frame[crc_at + 1]]);
        assert_eq!(crc, x25_crc(&frame[1..crc_at], OBSTACLE_DISTANCE_CRC_EXTRA));
        let (_, msg) = parse_v2(&frame).unwrap();
        let MavMessage::OBSTACLE_DISTANCE(d) = msg else {
            panic!("expected OBSTACLE_DISTANCE");
        };
        assert_eq!(d.time_usec, 99);
        assert_eq!(d.distances[0], 300);
        assert_eq!(d.distances[1], MAX_DISTANCE_CM + 1);
        assert_eq!((d.min_distance, d.max_distance), (5, 1200));
    }
}
