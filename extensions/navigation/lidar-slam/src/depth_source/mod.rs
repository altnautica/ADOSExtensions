//! Depth cameras for the 3D map: one seam over every supported source.
//! `navigation.depth.source` ([`config`]) picks the driver:
//!
//! - `we`: the World Engine depth stream, `ADOSDEPTH1` frames over UDP
//!   ([`we`], wire format in [`wire`]);
//! - `realsense`: an Intel RealSense D435i / D455 through librealsense2,
//!   opened at run time ([`realsense`], Unix);
//! - `off`: none.
//!
//! Every driver hands out [`DepthFrame`]s: a 16-bit millimetre depth image
//! along the optical axis (0 = no return), its pinhole intrinsics and the
//! camera's mount on the body, stamped on the host monotonic clock when the
//! frame was complete. [`DepthFrame::to_points`] turns one into body-frame
//! points (forward, right, down) for the map. A driver whose library or
//! camera is absent fails to open with [`DepthSourceError::Unavailable`];
//! it never panics.

pub mod config;
#[cfg(unix)]
pub mod realsense;
pub mod we;
pub mod wire;

#[cfg(test)]
mod tests;

use std::time::Duration;

use serde::Serialize;

pub use config::{DepthSettings, DepthSourceKind};

/// A frame is "streaming" while the newest one is younger than this.
pub const STREAMING_WINDOW: Duration = Duration::from_secs(1);

/// The camera's mount: translation (body FRD, m) and the unit quaternion
/// (w, x, y, z) taking the camera's own FRD axes (forward along the optical
/// axis, right, down) into body FRD.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Extrinsic {
    pub t: [f32; 3],
    pub q: [f32; 4],
}

impl Extrinsic {
    pub fn identity() -> Self {
        Self {
            t: [0.0; 3],
            q: [1.0, 0.0, 0.0, 0.0],
        }
    }

    /// From roll, pitch, yaw in degrees (Z-Y-X): all zero looks along body
    /// +x, pitch −90 straight down.
    pub fn from_rpy_deg(t: [f32; 3], rpy_deg: [f64; 3]) -> Self {
        let [r, p, y] = rpy_deg.map(|a| a.to_radians() * 0.5);
        let (sr, cr) = r.sin_cos();
        let (sp, cp) = p.sin_cos();
        let (sy, cy) = y.sin_cos();
        let q = [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ];
        Self::from_quat_wxyz(t, q)
    }

    /// From a quaternion (w, x, y, z), normalised.
    pub fn from_quat_wxyz(t: [f32; 3], q: [f64; 4]) -> Self {
        let n = q.iter().map(|x| x * x).sum::<f64>().sqrt();
        Self {
            t,
            q: q.map(|x| (x / n) as f32),
        }
    }

    /// The rotation taking optical-frame vectors (x right, y down, z along
    /// the optical axis: OpenCV's, the World Engine's and RealSense's) into
    /// body FRD, as columns: the images of optical x, y and z.
    pub fn optical_to_body(&self) -> [[f32; 3]; 3] {
        let [w, x, y, z] = self.q;
        // Columns of R(q): body images of camera forward, right, down.
        let fwd = [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y + w * z),
            2.0 * (x * z - w * y),
        ];
        let right = [
            2.0 * (x * y - w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z + w * x),
        ];
        let down = [
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ];
        // Optical x is camera right, y camera down, z camera forward.
        [right, down, fwd]
    }
}

impl Default for Extrinsic {
    fn default() -> Self {
        Self::identity()
    }
}

/// One depth image.
#[derive(Debug, Clone, PartialEq)]
pub struct DepthFrame {
    /// Host monotonic ns when the frame was complete on this host.
    pub stamp_ns: u64,
    /// The source's own time of the frame: sim ns for the World Engine,
    /// the device timestamp (ms × 10⁶) for a RealSense.
    pub source_ns: u64,
    /// The source's frame number.
    pub seq: u64,
    pub w: u32,
    pub h: u32,
    /// Pinhole intrinsics, px (pixel centres at integer coordinates).
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    /// `w · h` depths, row-major from the top-left pixel, mm along the
    /// optical axis; 0 = no return.
    pub depth_mm: Vec<u16>,
    pub extrinsic: Extrinsic,
}

impl DepthFrame {
    /// Body-frame points (FRD, m) of every `stride`-th pixel on both axes
    /// with a return no farther than `max_range_m` (Euclidean, from the
    /// camera). `out` is cleared first; it allocates only when its capacity
    /// is short of the sampled pixel count.
    pub fn to_points(&self, stride: usize, max_range_m: f32, out: &mut Vec<[f32; 3]>) {
        out.clear();
        let stride = stride.max(1);
        let (w, h) = (self.w as usize, self.h as usize);
        if self.depth_mm.len() < w * h || self.fx <= 0.0 || self.fy <= 0.0 {
            return;
        }
        out.reserve(w.div_ceil(stride) * h.div_ceil(stride));
        let [ox, oy, oz] = self.extrinsic.optical_to_body();
        let t = self.extrinsic.t;
        let max2 = max_range_m * max_range_m;
        let (ifx, ify) = (1.0 / self.fx, 1.0 / self.fy);
        for v in (0..h).step_by(stride) {
            let yn = (v as f32 - self.cy) * ify;
            let row = &self.depth_mm[v * w..(v + 1) * w];
            for u in (0..w).step_by(stride) {
                let mm = row[u];
                if mm == 0 {
                    continue;
                }
                let z = f32::from(mm) * 0.001;
                let xn = (u as f32 - self.cx) * ifx;
                if z * z * (1.0 + xn * xn + yn * yn) > max2 {
                    continue;
                }
                out.push([
                    t[0] + z * (ox[0] * xn + oy[0] * yn + oz[0]),
                    t[1] + z * (ox[1] * xn + oy[1] * yn + oz[1]),
                    t[2] + z * (ox[2] * xn + oy[2] * yn + oz[2]),
                ]);
            }
        }
    }
}

/// Counters a driver keeps since it opened.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
pub struct DepthHealth {
    /// Frames handed out.
    pub frames: u64,
    /// Frames the source numbered but this host never completed.
    pub drops: u64,
    /// Datagrams or frames refused as malformed.
    pub rejected: u64,
    /// `stamp_ns` of the newest frame; 0 before the first.
    pub last_stamp_ns: u64,
}

/// What a status surface shows of the depth source.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DepthState {
    Off,
    /// The library or the camera is absent (or the stream refused).
    Unavailable,
    /// Open, no frame within [`STREAMING_WINDOW`].
    Waiting,
    Streaming,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct DepthStatus {
    pub source: DepthSourceKind,
    pub state: DepthState,
    pub reason: Option<String>,
    #[serde(flatten)]
    pub health: DepthHealth,
}

impl DepthStatus {
    pub fn off() -> Self {
        Self {
            source: DepthSourceKind::Off,
            state: DepthState::Off,
            reason: None,
            health: DepthHealth::default(),
        }
    }

    pub fn unavailable(source: DepthSourceKind, err: &DepthSourceError) -> Self {
        Self {
            source,
            state: DepthState::Unavailable,
            reason: Some(err.to_string()),
            health: DepthHealth::default(),
        }
    }
}

#[derive(Debug, thiserror::Error)]
pub enum DepthSourceError {
    /// The library or the camera is absent, or the camera refused the
    /// stream; a later open may succeed.
    #[error("depth source unavailable: {0}")]
    Unavailable(String),
    /// The open stream failed (camera unplugged, socket error).
    #[error("depth stream failed: {0}")]
    Stream(String),
    #[error("depth source i/o: {0}")]
    Io(#[from] std::io::Error),
}

/// One running depth camera.
pub trait DepthSource: Send {
    fn kind(&self) -> DepthSourceKind;

    /// The next whole frame, waiting at most `timeout`; `None` when none
    /// completed in time.
    fn next_frame(&mut self, timeout: Duration) -> Result<Option<DepthFrame>, DepthSourceError>;

    fn health(&self) -> DepthHealth;

    /// Hand a frame back so its buffer is reused for a later one.
    fn recycle(&mut self, _frame: DepthFrame) {}

    /// The status block at host monotonic time `now_ns`.
    fn status(&self, now_ns: u64) -> DepthStatus {
        let health = self.health();
        let fresh = health.last_stamp_ns != 0
            && now_ns.saturating_sub(health.last_stamp_ns) < STREAMING_WINDOW.as_nanos() as u64;
        DepthStatus {
            source: self.kind(),
            state: if fresh {
                DepthState::Streaming
            } else {
                DepthState::Waiting
            },
            reason: None,
            health,
        }
    }
}

/// Open the configured source; `None` when `source` is `off`. Relative
/// library paths resolve against the install directory.
pub fn open(s: &DepthSettings) -> Result<Option<Box<dyn DepthSource>>, DepthSourceError> {
    let extrinsic = s.extrinsic();
    match s.source {
        DepthSourceKind::Off => Ok(None),
        DepthSourceKind::We => Ok(Some(Box::new(we::WeSource::open(&we::WeConfig {
            host: s.we_host.clone(),
            port: s.we_port,
            extrinsic,
        })?))),
        #[cfg(unix)]
        DepthSourceKind::Realsense => Ok(Some(Box::new(realsense::RealsenseSource::open(
            &realsense::RealsenseConfig {
                library: crate::install_path(&s.realsense.library),
                width: s.realsense.width,
                height: s.realsense.height,
                fps: s.realsense.fps,
                serial: s.realsense.serial.clone(),
                extrinsic,
            },
        )?))),
        #[cfg(not(unix))]
        DepthSourceKind::Realsense => Err(DepthSourceError::Unavailable(
            "RealSense needs a Unix build".into(),
        )),
    }
}
