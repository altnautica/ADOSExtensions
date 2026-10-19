//! The three flight-controller messages the EKF pose reads, decoded from the
//! raw MAVLink frames the plugin host delivers (`mavlink.read`).
//!
//! MAVLink 2 truncates trailing zero bytes of a payload, so a short payload is
//! zero-extended to the message's full length before decoding.

/// `ATTITUDE_QUATERNION` (31): body FRD → local NED.
pub const MSG_ATTITUDE_QUATERNION: u32 = 31;
/// `LOCAL_POSITION_NED` (32): EKF position in the local NED frame.
pub const MSG_LOCAL_POSITION_NED: u32 = 32;
/// `GPS_GLOBAL_ORIGIN` (49): the geodetic origin of that NED frame.
pub const MSG_GPS_GLOBAL_ORIGIN: u32 = 49;

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

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FcMessage {
    Attitude(AttitudeQuaternion),
    Position(LocalPositionNed),
    Origin(GpsGlobalOrigin),
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

/// Decode one raw frame into the message it carries.
pub fn decode(frame: &[u8]) -> Option<FcMessage> {
    let (id, payload) = split_frame(frame)?;
    Some(match id {
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
        _ => return None,
    })
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
}
