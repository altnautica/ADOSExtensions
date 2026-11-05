//! The flight-controller messages the EKF pose and the navigation lane's
//! heading read, decoded from the raw MAVLink frames the plugin host delivers
//! (`mavlink.read`).
//!
//! MAVLink 2 truncates trailing zero bytes of a payload, so a short payload is
//! zero-extended to the message's full length before decoding.
//!
//! The attitude arrives as `ATTITUDE_QUATERNION` or as `ATTITUDE` (Euler
//! angles): ArduPilot streams only `ATTITUDE` unless asked for the quaternion,
//! and the drone stream table asks for `ATTITUDE` alone outside a capture
//! session. Both decode to the same [`AttitudeQuaternion`].
//!
//! `OPTICAL_FLOW` and the downward `DISTANCE_SENSOR` give the ground
//! velocity the planar matcher follows along a corridor
//! ([`crate::nav::fc_flow`]).

/// `RAW_IMU` (27): ArduPilot fills it with the calibrated IMU, milli-g and
/// milli-rad/s, body FRD.
pub const MSG_RAW_IMU: u32 = 27;
/// `GPS_RAW_INT` (24): the receiver's fix, read for its `fix_type`.
pub const MSG_GPS_RAW_INT: u32 = 24;
/// `ATTITUDE` (30): body FRD → local NED as roll, pitch and yaw, radians.
pub const MSG_ATTITUDE: u32 = 30;
/// `ATTITUDE_QUATERNION` (31): body FRD → local NED.
pub const MSG_ATTITUDE_QUATERNION: u32 = 31;
/// `LOCAL_POSITION_NED` (32): EKF position in the local NED frame.
pub const MSG_LOCAL_POSITION_NED: u32 = 32;
/// `GPS_GLOBAL_ORIGIN` (49): the geodetic origin of that NED frame.
pub const MSG_GPS_GLOBAL_ORIGIN: u32 = 49;
/// `OPTICAL_FLOW` (100): the flow sensor's angular rates, body FRD axes.
pub const MSG_OPTICAL_FLOW: u32 = 100;
/// `DISTANCE_SENSOR` (132): one rangefinder reading.
pub const MSG_DISTANCE_SENSOR: u32 = 132;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AttitudeQuaternion {
    pub time_boot_ms: u32,
    /// `[w, x, y, z]`.
    pub q: [f64; 4],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LocalPositionNed {
    pub time_boot_ms: u32,
    pub ned: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsGlobalOrigin {
    pub lat_deg: f64,
    pub lon_deg: f64,
    /// MSL altitude, metres.
    pub alt_msl_m: f64,
}

/// One `RAW_IMU` sample in SI units, body FRD.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RawImu {
    pub time_usec: u64,
    /// rad/s.
    pub gyro: [f64; 3],
    /// Specific force, m/s² (z ≈ -9.81 level and still).
    pub accel: [f64; 3],
}

/// `OPTICAL_FLOW` as ArduPilot fills it: `flow_comp_m_x/y` carry the flow
/// rate minus the sensor's body rate, rad/s about the body FRD x and y axes
/// (not metres, whatever the field names say), and `time_usec` carries its
/// boot milliseconds; PX4 sends microseconds there.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpticalFlow {
    pub time_usec: u64,
    /// Body-rate compensated flow, rad/s, about body x and y (FRD).
    pub comp: [f64; 2],
    /// 0 (no flow) to 255.
    pub quality: u8,
}

/// `DISTANCE_SENSOR`, metres, with the sensor's own valid band.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DistanceSensor {
    pub time_boot_ms: u32,
    pub distance_m: f64,
    pub min_m: f64,
    pub max_m: f64,
    /// `MAV_SENSOR_ORIENTATION`; 25 (`PITCH_270`) faces down.
    pub orientation: u8,
}
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FcMessage {
    Attitude(AttitudeQuaternion),
    Position(LocalPositionNed),
    Origin(GpsGlobalOrigin),
    /// `GPS_RAW_INT.fix_type` (`GPS_FIX_TYPE`: 3 is a 3D fix).
    GpsFix(u8),
    Imu(RawImu),
    Flow(OpticalFlow),
    Range(DistanceSensor),
}

/// `(msgid, payload)` of a MAVLink 1 or 2 frame, or `None` when malformed.
pub fn split_frame(frame: &[u8]) -> Option<(u32, &[u8])> {
    match *frame.first()? {
        0xFD => {
            let len = *frame.get(1)? as usize;
            let id = u32::from_le_bytes([*frame.get(7)?, *frame.get(8)?, *frame.get(9)?, 0]);
            Some((id, frame.get(10..10 + len)?))
        }
        0xFE => {
            let len = *frame.get(1)? as usize;
            Some((*frame.get(5)? as u32, frame.get(6..6 + len)?))
        }
        _ => None,
    }
}

/// The sender's component id of a MAVLink 1 or 2 frame.
fn component(frame: &[u8]) -> Option<u8> {
    match *frame.first()? {
        0xFD => frame.get(6).copied(),
        0xFE => frame.get(4).copied(),
        _ => None,
    }
}

/// `MAV_COMP_ID_AUTOPILOT1`.
const COMP_AUTOPILOT: u8 = 1;

fn padded<const N: usize>(payload: &[u8]) -> [u8; N] {
    let mut out = [0u8; N];
    let n = payload.len().min(N);
    out[..n].copy_from_slice(&payload[..n]);
    out
}

fn u32_at(b: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([b[at], b[at + 1], b[at + 2], b[at + 3]])
}

fn i32_at(b: &[u8], at: usize) -> i32 {
    u32_at(b, at) as i32
}

fn f32_at(b: &[u8], at: usize) -> f64 {
    f32::from_bits(u32_at(b, at)) as f64
}

fn i16_at(b: &[u8], at: usize) -> f64 {
    i16::from_le_bytes([b[at], b[at + 1]]) as f64
}

/// Milli-g to m/s².
const MG: f64 = 9.80665e-3;

/// Decode one raw frame into the message it carries.
pub fn decode(frame: &[u8]) -> Option<FcMessage> {
    let (id, payload) = split_frame(frame)?;
    Some(match id {
        MSG_ATTITUDE => {
            let b: [u8; 28] = padded(payload);
            FcMessage::Attitude(AttitudeQuaternion {
                time_boot_ms: u32_at(&b, 0),
                q: quat_from_euler(f32_at(&b, 4), f32_at(&b, 8), f32_at(&b, 12)),
            })
        }
        MSG_ATTITUDE_QUATERNION => {
            let b: [u8; 32] = padded(payload);
            FcMessage::Attitude(AttitudeQuaternion {
                time_boot_ms: u32_at(&b, 0),
                q: [f32_at(&b, 4), f32_at(&b, 8), f32_at(&b, 12), f32_at(&b, 16)],
            })
        }
        MSG_LOCAL_POSITION_NED => {
            let b: [u8; 28] = padded(payload);
            FcMessage::Position(LocalPositionNed {
                time_boot_ms: u32_at(&b, 0),
                ned: [f32_at(&b, 4), f32_at(&b, 8), f32_at(&b, 12)],
            })
        }
        MSG_GPS_GLOBAL_ORIGIN => {
            let b: [u8; 12] = padded(payload);
            FcMessage::Origin(GpsGlobalOrigin {
                lat_deg: i32_at(&b, 0) as f64 * 1e-7,
                lon_deg: i32_at(&b, 4) as f64 * 1e-7,
                alt_msl_m: i32_at(&b, 8) as f64 * 1e-3,
            })
        }
        MSG_GPS_RAW_INT => {
            let b: [u8; 30] = padded(payload);
            FcMessage::GpsFix(b[28])
        }
        MSG_RAW_IMU => {
            let b: [u8; 26] = padded(payload);
            FcMessage::Imu(RawImu {
                time_usec: u64::from_le_bytes(b[0..8].try_into().expect("8 bytes")),
                accel: [i16_at(&b, 8) * MG, i16_at(&b, 10) * MG, i16_at(&b, 12) * MG],
                gyro: [i16_at(&b, 14) * 1e-3, i16_at(&b, 16) * 1e-3, i16_at(&b, 18) * 1e-3],
            })
        }
        // The FC's own flow only: a companion's (the vision-nav plugin sends
        // one to the FC) fills the fields with other units.
        MSG_OPTICAL_FLOW if component(frame) == Some(COMP_AUTOPILOT) => {
            let b: [u8; 26] = padded(payload);
            FcMessage::Flow(OpticalFlow {
                time_usec: u64::from_le_bytes(b[0..8].try_into().expect("8 bytes")),
                comp: [f32_at(&b, 8), f32_at(&b, 12)],
                quality: b[25],
            })
        }
        MSG_DISTANCE_SENSOR => {
            let b: [u8; 14] = padded(payload);
            let cm = |at: usize| u16::from_le_bytes([b[at], b[at + 1]]) as f64 * 0.01;
            FcMessage::Range(DistanceSensor {
                time_boot_ms: u32_at(&b, 0),
                min_m: cm(4),
                max_m: cm(6),
                distance_m: cm(8),
                orientation: b[12],
            })
        }
        _ => return None,
    })
}

/// `[w, x, y, z]` of the ZYX (yaw, pitch, roll) rotation, body FRD → NED.
pub fn quat_from_euler(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let (sr, cr) = (roll / 2.0).sin_cos();
    let (sp, cp) = (pitch / 2.0).sin_cos();
    let (sy, cy) = (yaw / 2.0).sin_cos();
    [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]
}

/// Build a MAVLink 2 frame around `payload` (zero CRC; the decoder does not
/// check it, the host router already did). For tests and SITL fixtures.
pub fn frame_v2(msgid: u32, payload: &[u8]) -> Vec<u8> {
    let id = msgid.to_le_bytes();
    let mut f = vec![
        0xFD,
        payload.len() as u8,
        0,
        0,
        0,
        1,
        1,
        id[0],
        id[1],
        id[2],
    ];
    f.extend_from_slice(payload);
    f.extend_from_slice(&[0, 0]);
    f
}

#[cfg(test)]
mod tests {
    use super::*;

    fn le32(v: f32) -> [u8; 4] {
        v.to_le_bytes()
    }

    /// ArduPilot's drone stream carries `ATTITUDE`, not the quaternion: its
    /// Euler angles decode to the same attitude `ATTITUDE_QUATERNION` gives.
    #[test]
    fn euler_attitude_decodes_to_the_quaternion_of_the_same_rotation() {
        let (roll, pitch, yaw) = (0.1f32, -0.2f32, 2.5f32);
        let mut p = Vec::new();
        p.extend_from_slice(&777u32.to_le_bytes());
        for v in [roll, pitch, yaw, 0.0, 0.0, 0.0] {
            p.extend_from_slice(&le32(v));
        }
        while p.last() == Some(&0) {
            p.pop();
        }
        let FcMessage::Attitude(a) = decode(&frame_v2(MSG_ATTITUDE, &p)).unwrap() else {
            panic!()
        };
        assert_eq!(a.time_boot_ms, 777);
        let m = crate::pose::se3::matrix_from_quat(a.q);
        // R = Rz(yaw)·Ry(pitch)·Rx(roll): its yaw, pitch and roll read back.
        let (r0, p0, y0) = (f64::from(roll), f64::from(pitch), f64::from(yaw));
        assert!((m[3].atan2(m[0]) - y0).abs() < 1e-6);
        assert!(((-m[6]).asin() - p0).abs() < 1e-6);
        assert!((m[7].atan2(m[8]) - r0).abs() < 1e-6);
        let n = a.q.iter().map(|v| v * v).sum::<f64>();
        assert!((n - 1.0).abs() < 1e-12);
    }

    #[test]
    fn attitude_and_position_decode() {
        let mut p = Vec::new();
        p.extend_from_slice(&1234u32.to_le_bytes());
        for v in [1.0f32, 0.0, 0.0, 0.0, 0.1, 0.2, 0.3] {
            p.extend_from_slice(&le32(v));
        }
        let FcMessage::Attitude(a) = decode(&frame_v2(31, &p)).unwrap() else {
            panic!()
        };
        assert_eq!(a.time_boot_ms, 1234);
        assert_eq!(a.q, [1.0, 0.0, 0.0, 0.0]);

        let mut p = Vec::new();
        p.extend_from_slice(&50u32.to_le_bytes());
        for v in [1.5f32, -2.0, 0.0, 0.0, 0.0, 0.0] {
            p.extend_from_slice(&le32(v));
        }
        // MAVLink 2 trims the trailing zero velocity bytes.
        while p.last() == Some(&0) {
            p.pop();
        }
        let FcMessage::Position(l) = decode(&frame_v2(32, &p)).unwrap() else {
            panic!()
        };
        assert_eq!(l.ned, [1.5, -2.0, 0.0]);
    }

    #[test]
    fn origin_and_v1_frames_decode() {
        let mut p = Vec::new();
        p.extend_from_slice(&129_716_000i32.to_le_bytes());
        p.extend_from_slice(&775_946_000i32.to_le_bytes());
        p.extend_from_slice(&920_400i32.to_le_bytes());
        let mut v1 = vec![0xFE, p.len() as u8, 0, 1, 1, 49];
        v1.extend_from_slice(&p);
        v1.extend_from_slice(&[0, 0]);
        let FcMessage::Origin(o) = decode(&v1).unwrap() else {
            panic!()
        };
        assert!((o.lat_deg - 12.9716).abs() < 1e-9);
        assert!((o.alt_msl_m - 920.4).abs() < 1e-9);
        assert!(decode(&frame_v2(0, &[0; 9])).is_none());
        assert!(decode(&[0xFD, 40, 0]).is_none());
    }

    #[test]
    fn gps_raw_int_gives_its_fix_type() {
        let mut p = vec![0u8; 30];
        p[8..12].copy_from_slice(&129_716_000i32.to_le_bytes());
        p[28] = 3;
        p[29] = 14;
        assert_eq!(decode(&frame_v2(MSG_GPS_RAW_INT, &p)), Some(FcMessage::GpsFix(3)));
        // No fix: MAVLink 2 trims every zero byte from the satellite count on.
        p[28] = 0;
        p[29] = 0;
        while p.last() == Some(&0) {
            p.pop();
        }
        assert_eq!(decode(&frame_v2(MSG_GPS_RAW_INT, &p)), Some(FcMessage::GpsFix(0)));
    }

    /// A `RAW_IMU` frame, milli-g and milli-rad/s.
    fn raw_imu(time_usec: u64, accel_mg: [i16; 3], gyro_mrad: [i16; 3]) -> Vec<u8> {
        let mut p = time_usec.to_le_bytes().to_vec();
        for v in accel_mg.iter().chain(gyro_mrad.iter()).chain([0i16; 3].iter()) {
            p.extend_from_slice(&v.to_le_bytes());
        }
        frame_v2(MSG_RAW_IMU, &p)
    }

    #[test]
    fn raw_imu_decodes_to_si_units() {
        let Some(FcMessage::Imu(s)) = decode(&raw_imu(5_000_123, [0, 0, -1000], [10, -20, 1047]))
        else {
            panic!("RAW_IMU")
        };
        assert_eq!(s.time_usec, 5_000_123);
        assert!((s.accel[2] + 9.80665).abs() < 1e-9 && s.accel[0] == 0.0);
        assert!((s.gyro[0] - 0.010).abs() < 1e-12);
        assert!((s.gyro[1] + 0.020).abs() < 1e-12);
        assert!((s.gyro[2] - 1.047).abs() < 1e-12);
    }

    #[test]
    fn optical_flow_and_distance_sensor_decode() {
        // ArduPilot's OPTICAL_FLOW: boot ms in time_usec, flow_x/y, sensor id,
        // quality, then the flow_rate_x/y extension.
        let mut p = 2_028u64.to_le_bytes().to_vec();
        for v in [-1.36f32, 3.6, 2.58] {
            p.extend_from_slice(&le32(v));
        }
        p.extend_from_slice(&(-2i16).to_le_bytes());
        p.extend_from_slice(&0i16.to_le_bytes());
        p.extend_from_slice(&[0, 255]);
        p.extend_from_slice(&le32(-1.4));
        p.extend_from_slice(&le32(3.7));
        let Some(FcMessage::Flow(f)) = decode(&frame_v2(MSG_OPTICAL_FLOW, &p)) else {
            panic!("OPTICAL_FLOW")
        };
        assert_eq!(f.time_usec, 2_028);
        assert!((f.comp[0] + 1.36).abs() < 1e-6 && (f.comp[1] - 3.6).abs() < 1e-6);
        assert_eq!(f.quality, 255);
        // The same message from a companion component (198) is not the FC's.
        let mut companion = frame_v2(MSG_OPTICAL_FLOW, &p);
        companion[6] = 198;
        assert_eq!(decode(&companion), None);

        // A downward laser: 5 cm to 12 m, reading 1.14 m.
        let mut p = 37_000u32.to_le_bytes().to_vec();
        for v in [5u16, 1200, 114] {
            p.extend_from_slice(&v.to_le_bytes());
        }
        p.extend_from_slice(&[0, 0, 25, 0]);
        let Some(FcMessage::Range(r)) = decode(&frame_v2(MSG_DISTANCE_SENSOR, &p)) else {
            panic!("DISTANCE_SENSOR")
        };
        assert_eq!((r.time_boot_ms, r.orientation), (37_000, 25));
        assert!((r.distance_m - 1.14).abs() < 1e-9);
        assert!((r.min_m - 0.05).abs() < 1e-9 && (r.max_m - 12.0).abs() < 1e-9);
    }
}
