//! The RPLIDAR serial protocol, standard scan subset: request encoders,
//! response descriptors, the `GET_INFO` / `GET_HEALTH` payloads and the 5-byte
//! standard scan node. Pure functions over bytes; no I/O.
//!
//! Requests without a payload are two bytes, `A5 <cmd>`. Every response that
//! carries data starts with a 7-byte descriptor `A5 5A`, then a little-endian
//! 32-bit word whose low 30 bits are the response length and whose top 2 bits
//! are the send mode, then one data-type byte.

/// First byte of every request and response descriptor.
pub const SYNC: u8 = 0xA5;
/// Second byte of a response descriptor.
pub const SYNC2: u8 = 0x5A;

/// Bytes in a response descriptor.
pub const DESCRIPTOR_LEN: usize = 7;
/// Bytes in a standard scan node.
pub const NODE_LEN: usize = 5;
/// Bytes in a `GET_INFO` response.
pub const INFO_LEN: usize = 20;
/// Bytes in a `GET_HEALTH` response.
pub const HEALTH_LEN: usize = 3;

/// Request command bytes.
pub const CMD_STOP: u8 = 0x25;
pub const CMD_RESET: u8 = 0x40;
pub const CMD_SCAN: u8 = 0x20;
pub const CMD_GET_INFO: u8 = 0x50;
pub const CMD_GET_HEALTH: u8 = 0x52;

/// Response data types.
pub const TYPE_SCAN: u8 = 0x81;
pub const TYPE_INFO: u8 = 0x04;
pub const TYPE_HEALTH: u8 = 0x06;

/// `STOP`: leave scanning; no response.
pub fn stop() -> [u8; 2] {
    [SYNC, CMD_STOP]
}

/// `RESET`: reboot the core; no response.
pub fn reset() -> [u8; 2] {
    [SYNC, CMD_RESET]
}

/// `SCAN`: start a standard scan; answered by [`SCAN_DESCRIPTOR`] then nodes.
pub fn scan() -> [u8; 2] {
    [SYNC, CMD_SCAN]
}

/// `GET_INFO`: answered by [`INFO_DESCRIPTOR`] and a 20-byte payload.
pub fn get_info() -> [u8; 2] {
    [SYNC, CMD_GET_INFO]
}

/// `GET_HEALTH`: answered by [`HEALTH_DESCRIPTOR`] and a 3-byte payload.
pub fn get_health() -> [u8; 2] {
    [SYNC, CMD_GET_HEALTH]
}

/// How many responses follow a descriptor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SendMode {
    /// One response of the descriptor's length.
    Single,
    /// Responses of the descriptor's length until `STOP`.
    Multiple,
    /// Mode bits 2 and 3, reserved by the protocol.
    Reserved(u8),
}

/// A parsed response descriptor.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Descriptor {
    /// Length of each response, bytes (30 bits).
    pub len: u32,
    pub mode: SendMode,
    pub data_type: u8,
}

/// The descriptor a `SCAN` request is answered with (`A5 5A 05 00 00 40 81`).
pub const SCAN_DESCRIPTOR: Descriptor = Descriptor {
    len: NODE_LEN as u32,
    mode: SendMode::Multiple,
    data_type: TYPE_SCAN,
};

/// The descriptor a `GET_INFO` request is answered with.
pub const INFO_DESCRIPTOR: Descriptor = Descriptor {
    len: INFO_LEN as u32,
    mode: SendMode::Single,
    data_type: TYPE_INFO,
};

/// The descriptor a `GET_HEALTH` request is answered with.
pub const HEALTH_DESCRIPTOR: Descriptor = Descriptor {
    len: HEALTH_LEN as u32,
    mode: SendMode::Single,
    data_type: TYPE_HEALTH,
};

/// Parse a 7-byte response descriptor; `None` when the sync bytes are wrong.
pub fn parse_descriptor(b: &[u8; DESCRIPTOR_LEN]) -> Option<Descriptor> {
    if b[0] != SYNC || b[1] != SYNC2 {
        return None;
    }
    let word = u32::from_le_bytes([b[2], b[3], b[4], b[5]]);
    let mode = match (word >> 30) as u8 {
        0 => SendMode::Single,
        1 => SendMode::Multiple,
        m => SendMode::Reserved(m),
    };
    Some(Descriptor {
        len: word & 0x3FFF_FFFF,
        mode,
        data_type: b[6],
    })
}

/// Encode a descriptor (the inverse of [`parse_descriptor`]).
pub fn encode_descriptor(d: &Descriptor) -> [u8; DESCRIPTOR_LEN] {
    let mode = match d.mode {
        SendMode::Single => 0u32,
        SendMode::Multiple => 1,
        SendMode::Reserved(m) => u32::from(m & 0b11),
    };
    let w = ((d.len & 0x3FFF_FFFF) | (mode << 30)).to_le_bytes();
    [SYNC, SYNC2, w[0], w[1], w[2], w[3], d.data_type]
}

/// The `GET_INFO` payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceInfo {
    pub model: u8,
    pub firmware_minor: u8,
    pub firmware_major: u8,
    pub hardware: u8,
    pub serial: [u8; 16],
}

impl DeviceInfo {
    /// `major.minor`, the minor part two digits as the vendor prints it.
    pub fn firmware(&self) -> String {
        format!("{}.{:02}", self.firmware_major, self.firmware_minor)
    }

    /// The serial number as 32 upper-case hex digits.
    pub fn serial_hex(&self) -> String {
        self.serial.iter().map(|b| format!("{b:02X}")).collect()
    }
}

/// Parse the 20-byte `GET_INFO` payload: model, firmware minor, firmware
/// major, hardware, then the 16-byte serial number.
pub fn parse_info(b: &[u8; INFO_LEN]) -> DeviceInfo {
    let mut serial = [0u8; 16];
    serial.copy_from_slice(&b[4..20]);
    DeviceInfo {
        model: b[0],
        firmware_minor: b[1],
        firmware_major: b[2],
        hardware: b[3],
        serial,
    }
}

/// Device self-check result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HealthStatus {
    Good,
    Warning,
    Error,
    Unknown(u8),
}

/// The `GET_HEALTH` payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Health {
    pub status: HealthStatus,
    pub error_code: u16,
}

/// Parse the 3-byte `GET_HEALTH` payload: status, then a little-endian code.
pub fn parse_health(b: &[u8; HEALTH_LEN]) -> Health {
    let status = match b[0] {
        0 => HealthStatus::Good,
        1 => HealthStatus::Warning,
        2 => HealthStatus::Error,
        s => HealthStatus::Unknown(s),
    };
    Health {
        status,
        error_code: u16::from_le_bytes([b[1], b[2]]),
    }
}

/// One standard scan sample, device convention (angle clockwise from the
/// device's forward mark).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Node {
    /// First sample of a new revolution.
    pub start: bool,
    /// 0..63.
    pub quality: u8,
    pub angle_deg: f32,
    pub dist_m: f32,
}

/// The node's check bits are consistent: byte 0 bit 0 (S) differs from bit 1
/// (!S), and byte 1 bit 0 (C) is 1. A node failing this means the stream is
/// out of step.
pub fn node_framing_ok(b: &[u8; NODE_LEN]) -> bool {
    let s = b[0] & 1;
    let not_s = (b[0] >> 1) & 1;
    s != not_s && b[1] & 1 == 1
}

/// Decode a standard scan node. `None` when the check bits are inconsistent
/// or the distance is 0 (no return).
pub fn decode_standard_node(b: [u8; NODE_LEN]) -> Option<Node> {
    if !node_framing_ok(&b) {
        return None;
    }
    let angle_q6 = (u16::from(b[1]) | u16::from(b[2]) << 8) >> 1;
    let dist_q2 = u16::from(b[3]) | u16::from(b[4]) << 8;
    if dist_q2 == 0 {
        return None;
    }
    Some(Node {
        start: b[0] & 1 == 1,
        quality: b[0] >> 2,
        angle_deg: f32::from(angle_q6) / 64.0,
        dist_m: f32::from(dist_q2) / 4.0 / 1000.0,
    })
}

/// Encode a standard scan node (the inverse of [`decode_standard_node`]);
/// the angle is rounded to 1/64 degree and the distance to 1/4 mm.
pub fn encode_standard_node(start: bool, quality: u8, angle_deg: f32, dist_m: f32) -> [u8; 5] {
    let s = u8::from(start);
    let b0 = (quality.min(63) << 2) | ((1 - s) << 1) | s;
    let angle_q6 = ((angle_deg.rem_euclid(360.0) * 64.0).round() as u32 % (360 * 64)) as u16;
    let a = (angle_q6 << 1) | 1;
    let dist_q2 = (dist_m * 4000.0).round().clamp(0.0, f32::from(u16::MAX)) as u16;
    let [a0, a1] = a.to_le_bytes();
    let [d0, d1] = dist_q2.to_le_bytes();
    [b0, a0, a1, d0, d1]
}

/// Device angle (clockwise from forward, degrees) from a sensor-frame FLU
/// azimuth (counter-clockwise from +x, degrees): `(360 - θ) mod 360`.
pub fn device_from_flu_deg(flu_deg: f64) -> f64 {
    (360.0 - flu_deg).rem_euclid(360.0)
}

/// Sensor-frame FLU azimuth (counter-clockwise from +x, degrees) from a
/// device angle (clockwise from forward, degrees): `(360 - θ) mod 360`, the
/// mirror being its own inverse.
pub fn flu_from_device_deg(device_deg: f64) -> f64 {
    (360.0 - device_deg).rem_euclid(360.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_valid_node_decodes_quality_angle_and_distance() {
        // quality 47, not a start; angle 90.5 deg = q6 5792; 1234.5 mm = q2 4938.
        let q6: u16 = 5792;
        let a = (q6 << 1) | 1;
        let d: u16 = 4938;
        let bytes = [
            (47 << 2) | 0b10,
            a as u8,
            (a >> 8) as u8,
            d as u8,
            (d >> 8) as u8,
        ];
        let n = decode_standard_node(bytes).expect("valid node");
        assert!(!n.start);
        assert_eq!(n.quality, 47);
        assert_eq!(n.angle_deg, 90.5);
        assert!((n.dist_m - 1.2345).abs() < 1e-6);
    }

    #[test]
    fn the_start_flag_is_bit_zero_with_its_complement_in_bit_one() {
        let n = decode_standard_node([(10 << 2) | 0b01, 0x01, 0x00, 0x10, 0x00]).unwrap();
        assert!(n.start);
        assert_eq!(n.angle_deg, 0.0);
        assert!((n.dist_m - 0.004).abs() < 1e-7);
    }

    #[test]
    fn inconsistent_check_bits_are_rejected() {
        // C bit clear.
        assert_eq!(decode_standard_node([0b10, 0x00, 0x10, 0x10, 0x00]), None);
        // S == !S, both 0 and both 1.
        assert_eq!(decode_standard_node([0b00, 0x01, 0x10, 0x10, 0x00]), None);
        assert_eq!(decode_standard_node([0b11, 0x01, 0x10, 0x10, 0x00]), None);
        assert!(!node_framing_ok(&[0b11, 0x01, 0x10, 0x10, 0x00]));
    }

    #[test]
    fn a_zero_distance_node_is_dropped_though_its_framing_is_valid() {
        let b = [(20 << 2) | 0b10, 0x81, 0x16, 0x00, 0x00];
        assert!(node_framing_ok(&b));
        assert_eq!(decode_standard_node(b), None);
    }

    #[test]
    fn the_scan_descriptor_is_five_byte_multiple_responses() {
        let bytes = [0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81];
        assert_eq!(parse_descriptor(&bytes), Some(SCAN_DESCRIPTOR));
        assert_eq!(encode_descriptor(&SCAN_DESCRIPTOR), bytes);
        let info = [0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04];
        assert_eq!(parse_descriptor(&info), Some(INFO_DESCRIPTOR));
        assert_eq!(parse_descriptor(&[0xA5, 0x5B, 5, 0, 0, 0x40, 0x81]), None);
    }

    #[test]
    fn info_payload_parses_model_firmware_hardware_and_serial() {
        let mut b = [0u8; INFO_LEN];
        b[0] = 0x41;
        b[1] = 1;
        b[2] = 1;
        b[3] = 18;
        for (i, x) in b[4..].iter_mut().enumerate() {
            *x = 0xA0 + i as u8;
        }
        let info = parse_info(&b);
        assert_eq!(info.model, 0x41);
        assert_eq!(info.firmware(), "1.01");
        assert_eq!(info.hardware, 18);
        assert_eq!(info.serial_hex(), "A0A1A2A3A4A5A6A7A8A9AAABACADAEAF");
    }

    #[test]
    fn health_payload_parses_status_and_code() {
        let h = parse_health(&[2, 0x34, 0x12]);
        assert_eq!(h.status, HealthStatus::Error);
        assert_eq!(h.error_code, 0x1234);
        assert_eq!(parse_health(&[0, 0, 0]).status, HealthStatus::Good);
    }

    #[test]
    fn requests_are_sync_then_command() {
        assert_eq!(stop(), [0xA5, 0x25]);
        assert_eq!(reset(), [0xA5, 0x40]);
        assert_eq!(scan(), [0xA5, 0x20]);
        assert_eq!(get_info(), [0xA5, 0x50]);
        assert_eq!(get_health(), [0xA5, 0x52]);
    }

    #[test]
    fn flu_and_device_angles_mirror_at_the_pinned_samples() {
        for (flu, dev) in [(0.0, 0.0), (90.0, 270.0), (300.0, 60.0)] {
            assert_eq!(device_from_flu_deg(flu), dev);
            assert_eq!(flu_from_device_deg(dev), flu);
        }
    }

    #[test]
    fn encoded_nodes_decode_back() {
        let b = encode_standard_node(true, 60, 271.25, 3.5);
        let n = decode_standard_node(b).unwrap();
        assert!(n.start);
        assert_eq!(n.quality, 60);
        assert_eq!(n.angle_deg, 271.25);
        assert!((n.dist_m - 3.5).abs() < 1e-6);
    }
}
