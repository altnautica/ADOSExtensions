//! `navigation.depth`: which depth camera feeds the 3D map, where it sits on
//! the body, and the per-driver settings. [`resolve`] reads and validates the
//! key; a value that would not run is refused with the offending field named.
//!
//! ```json
//! {
//!   "source": "we",
//!   "extrinsic": { "translation_m": [0.13, 0.0, 0.0], "rpy_deg": [0.0, -20.0, 0.0] },
//!   "we_host": "192.0.2.1",
//!   "we_port": 7810
//! }
//! ```
//!
//! The extrinsic is the camera's mount on the body: `translation_m` is its
//! position in body FRD (forward, right, down) from the body origin, and the
//! rotation is either `rpy_deg` (roll, pitch, yaw, Z-Y-X; all zero looks
//! along body +x, pitch −90 looks straight down) or `quat_wxyz` (the unit
//! quaternion taking the camera's own FRD axes into body FRD), never both.
//! Neither means the camera looks straight ahead.

use std::collections::BTreeMap;

use ados_mapping_pose::config::{typed, ConfigError};
use serde::Deserialize;
use serde_json::Value;

use super::Extrinsic;

/// The configured depth source (`navigation.depth.source`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DepthSourceKind {
    /// No depth camera.
    #[default]
    Off,
    /// An Intel RealSense D435i / D455 through librealsense2.
    Realsense,
    /// The World Engine depth stream (`ADOSDEPTH1` frames over UDP).
    We,
    /// The vision engine's camera depth (`vision.depth.v2` on the plugin
    /// bus); needs `vision.depth.read`.
    Vision,
}

/// The camera mount as configured (`navigation.depth.extrinsic`).
#[derive(Debug, Clone, PartialEq, Deserialize, Default)]
#[serde(default, deny_unknown_fields)]
pub struct ExtrinsicConfig {
    pub translation_m: [f64; 3],
    pub rpy_deg: Option<[f64; 3]>,
    pub quat_wxyz: Option<[f64; 4]>,
}

/// RealSense stream settings (`navigation.depth.realsense`).
#[derive(Debug, Clone, PartialEq, Eq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct RealsenseSettings {
    /// The librealsense2 shared library (relative to the install directory).
    pub library: String,
    /// Depth stream size: 848×480 or 640×480.
    pub width: u32,
    pub height: u32,
    /// Frames per second: 15 or 30.
    pub fps: u32,
    /// Only the camera with this serial number; empty takes the first found.
    pub serial: String,
}

impl Default for RealsenseSettings {
    fn default() -> Self {
        Self {
            library: format!("lib/{}-linux/librealsense2.so", std::env::consts::ARCH),
            width: 848,
            height: 480,
            fps: 30,
            serial: String::new(),
        }
    }
}

/// The resolved `navigation.depth` key.
#[derive(Debug, Clone, PartialEq, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct DepthSettings {
    pub source: DepthSourceKind,
    pub extrinsic: ExtrinsicConfig,
    /// Address of the World Engine host serving the depth stream.
    pub we_host: String,
    /// Its depth port: 7810 + the vehicle's SITL lease.
    pub we_port: u16,
    pub realsense: RealsenseSettings,
    /// `vision`: the camera whose depth is leased and mapped (required).
    pub vision_camera: String,
    /// `vision`: a frame whose scale fit is older than this (ms) is counted,
    /// not mapped.
    pub vision_max_fit_age_ms: u32,
}

impl Default for DepthSettings {
    fn default() -> Self {
        Self {
            source: DepthSourceKind::Off,
            extrinsic: ExtrinsicConfig::default(),
            we_host: "127.0.0.1".into(),
            we_port: 7810,
            realsense: RealsenseSettings::default(),
            vision_camera: String::new(),
            vision_max_fit_age_ms: VISION_MAX_FIT_AGE_MS,
        }
    }
}

/// The default oldest scale fit a `vision` frame is mapped with, ms.
pub const VISION_MAX_FIT_AGE_MS: u32 = 2000;
/// The largest `vision_max_fit_age_ms` accepted.
pub const VISION_MAX_FIT_AGE_CAP_MS: u32 = 60_000;

/// The RealSense stream sizes the driver asks for.
pub const REALSENSE_SIZES: [(u32, u32); 2] = [(848, 480), (640, 480)];
/// The RealSense frame rates the driver asks for.
pub const REALSENSE_FPS: [u32; 2] = [15, 30];

impl DepthSettings {
    /// Every field that would not run, as `field: reason`.
    pub fn validate(&self) -> Result<(), String> {
        self.extrinsic_checked()?;
        match self.source {
            DepthSourceKind::Off => {}
            DepthSourceKind::We => {
                if self.we_host.trim().is_empty() {
                    return Err("we_host: empty".into());
                }
                if self.we_port == 0 {
                    return Err("we_port: 0 is not a port".into());
                }
            }
            DepthSourceKind::Vision => {
                if self.vision_camera.trim().is_empty() {
                    return Err("vision_camera: empty".into());
                }
                if self.vision_max_fit_age_ms > VISION_MAX_FIT_AGE_CAP_MS {
                    return Err(format!(
                        "vision_max_fit_age_ms: {} is over {VISION_MAX_FIT_AGE_CAP_MS}",
                        self.vision_max_fit_age_ms
                    ));
                }
            }
            DepthSourceKind::Realsense => {
                let r = &self.realsense;
                if r.library.trim().is_empty() {
                    return Err("realsense.library: empty".into());
                }
                if !REALSENSE_SIZES.contains(&(r.width, r.height)) {
                    return Err(format!(
                        "realsense.width/height: {}x{} is not 848x480 or 640x480",
                        r.width, r.height
                    ));
                }
                if !REALSENSE_FPS.contains(&r.fps) {
                    return Err(format!("realsense.fps: {} is not 15 or 30", r.fps));
                }
            }
        }
        Ok(())
    }

    /// The mount as a rigid transform; see [`Self::validate`] for the refusals.
    pub fn extrinsic(&self) -> Extrinsic {
        self.extrinsic_checked()
            .unwrap_or_else(|_| Extrinsic::identity())
    }

    fn extrinsic_checked(&self) -> Result<Extrinsic, String> {
        let e = &self.extrinsic;
        let finite = |v: &[f64]| v.iter().all(|x| x.is_finite());
        if !finite(&e.translation_m) {
            return Err("extrinsic.translation_m: not finite".into());
        }
        if e.translation_m.iter().any(|x| x.abs() > 10.0) {
            return Err("extrinsic.translation_m: more than 10 m from the body origin".into());
        }
        let t = e.translation_m.map(|x| x as f32);
        match (e.rpy_deg, e.quat_wxyz) {
            (Some(_), Some(_)) => Err("extrinsic: give rpy_deg or quat_wxyz, not both".into()),
            (Some(rpy), None) if !finite(&rpy) => Err("extrinsic.rpy_deg: not finite".into()),
            (Some(rpy), None) => Ok(Extrinsic::from_rpy_deg(t, rpy)),
            (None, Some(q)) => {
                let n = q.iter().map(|x| x * x).sum::<f64>().sqrt();
                if !finite(&q) || !(0.5..=1.5).contains(&n) {
                    return Err(format!(
                        "extrinsic.quat_wxyz: norm {n} is not a unit quaternion"
                    ));
                }
                Ok(Extrinsic::from_quat_wxyz(t, q))
            }
            (None, None) => Ok(Extrinsic {
                t,
                ..Extrinsic::identity()
            }),
        }
    }
}

/// `key` as [`DepthSettings`], validated; defaults (source off) when absent.
pub fn resolve(
    values: &BTreeMap<String, Value>,
    key: &'static str,
) -> Result<DepthSettings, ConfigError> {
    let s: DepthSettings = typed(values, key, DepthSettings::default())?;
    s.validate().map_err(|reason| ConfigError { key, reason })?;
    Ok(s)
}
