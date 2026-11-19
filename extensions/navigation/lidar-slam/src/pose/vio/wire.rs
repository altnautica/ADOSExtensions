//! The `basalt-ados` wire formats: the framed binary input the capture service
//! writes to the child's socket, and the JSON lines the child prints.
//!
//! Input, little-endian, one 5-byte header per message:
//!
//! ```text
//! u8 type | u32 len | payload[len]
//! type 0 HELLO   u32 version (= PROTOCOL_VERSION), always the first message
//! type 1 IMU     u64 t_ns | f64 gyro[3] rad/s | f64 accel[3] m/s^2   (len 56)
//! type 2 STEREO  u64 t_ns | u32 width | u32 height | left | right    (Gray8)
//! ```
//!
//! IMU vectors are in the IMU sensor frame; times are host `CLOCK_MONOTONIC`
//! nanoseconds. A channel item may hold several messages back to back.
//!
//! Output, one JSON object per line:
//!
//! ```text
//! {"status":"initializing"|"tracking"|"lost"}
//! {"t_ns":u64,"T_w_i":[16 row-major],"cov":[36]|null,"features":n|null}
//! ```

use serde::Deserialize;

pub const PROTOCOL_VERSION: u32 = 1;

pub const MSG_HELLO: u8 = 0;
pub const MSG_IMU: u8 = 1;
pub const MSG_STEREO: u8 = 2;

pub const HEADER_LEN: usize = 5;
pub const IMU_LEN: usize = 56;
pub const STEREO_HEADER_LEN: usize = 16;

fn header(out: &mut Vec<u8>, kind: u8, len: usize) {
    out.push(kind);
    out.extend_from_slice(&(len as u32).to_le_bytes());
}

/// The HELLO message.
pub fn encode_hello(out: &mut Vec<u8>) {
    header(out, MSG_HELLO, 4);
    out.extend_from_slice(&PROTOCOL_VERSION.to_le_bytes());
}

/// One IMU sample, IMU sensor frame.
pub fn encode_imu(out: &mut Vec<u8>, t_ns: u64, gyro: [f64; 3], accel: [f64; 3]) {
    header(out, MSG_IMU, IMU_LEN);
    out.extend_from_slice(&t_ns.to_le_bytes());
    for v in gyro.iter().chain(accel.iter()) {
        out.extend_from_slice(&v.to_le_bytes());
    }
}

/// Length of a stereo message for a `width` x `height` pair, header included.
pub fn stereo_message_len(width: u32, height: u32) -> usize {
    HEADER_LEN + STEREO_HEADER_LEN + 2 * width as usize * height as usize
}

/// One Gray8 stereo pair. `left` and `right` must each be `width * height`
/// bytes (checked by the caller).
pub fn encode_stereo(
    out: &mut Vec<u8>,
    t_ns: u64,
    width: u32,
    height: u32,
    left: &[u8],
    right: &[u8],
) {
    debug_assert_eq!(left.len(), width as usize * height as usize);
    debug_assert_eq!(right.len(), left.len());
    out.reserve(stereo_message_len(width, height));
    header(
        out,
        MSG_STEREO,
        STEREO_HEADER_LEN + left.len() + right.len(),
    );
    out.extend_from_slice(&t_ns.to_le_bytes());
    out.extend_from_slice(&width.to_le_bytes());
    out.extend_from_slice(&height.to_le_bytes());
    out.extend_from_slice(left);
    out.extend_from_slice(right);
}

/// The child's tracking status.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ChildStatus {
    Initializing,
    Tracking,
    Lost,
}

/// One state line: the IMU pose in Basalt's gravity-aligned world.
#[derive(Debug, Clone, PartialEq)]
pub struct ChildState {
    pub t_ns: u64,
    /// Row-major 4x4 `T_w_i`.
    pub t_w_i: [f64; 16],
    /// Row-major 6x6 over (x, y, z, rot_x, rot_y, rot_z), when reported.
    pub cov: Option<[f64; 36]>,
    pub features: Option<u32>,
}

/// One parsed stdout line. Short-lived on the reader's stack, so the large
/// state variant stays unboxed.
#[derive(Debug, Clone, PartialEq)]
#[allow(clippy::large_enum_variant)]
pub enum ChildLine {
    Status(ChildStatus),
    State(ChildState),
}

#[derive(Deserialize)]
#[serde(untagged)]
enum RawLine {
    Status {
        status: ChildStatus,
    },
    State {
        t_ns: u64,
        #[serde(rename = "T_w_i")]
        t_w_i: Vec<f64>,
        #[serde(default)]
        cov: Option<Vec<f64>>,
        #[serde(default)]
        features: Option<u32>,
    },
}

/// Parse one stdout line; `None` for anything that is not a well-formed
/// status or state line (including non-finite poses).
pub fn parse_line(line: &str) -> Option<ChildLine> {
    match serde_json::from_str::<RawLine>(line.trim()).ok()? {
        RawLine::Status { status } => Some(ChildLine::Status(status)),
        RawLine::State {
            t_ns,
            t_w_i,
            cov,
            features,
        } => {
            let t_w_i: [f64; 16] = t_w_i.try_into().ok()?;
            if t_w_i.iter().any(|v| !v.is_finite()) {
                return None;
            }
            let cov = match cov {
                None => None,
                Some(c) => {
                    let c: [f64; 36] = c.try_into().ok()?;
                    c.iter().all(|v| v.is_finite()).then_some(c)
                }
            };
            Some(ChildLine::State(ChildState {
                t_ns,
                t_w_i,
                cov,
                features,
            }))
        }
    }
}
