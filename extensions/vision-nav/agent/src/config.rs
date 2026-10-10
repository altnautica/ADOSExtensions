//! Per-drone configuration model + validation.
//!
//! Mirrors `config-schema.json` at the extension root, defaults
//! included (a test parses the schema defaults and compares them with
//! [`VisionNavConfig::default`]). The plugin merges the static
//! `on_configure` map with the live per-drone keys (see
//! [`crate::live_config`]) and hands the result to
//! [`VisionNavConfig::from_map`]. Validation rejects an optical-flow
//! mode with a forward- or side-facing camera.

use std::collections::BTreeMap;

use rmpv::Value;

/// Operating mode. The estimator registry maps each mode to its
/// estimator; a mode flip selects a different estimator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    Off,
    OpticalFlow,
    OpticalFlowDegraded,
}

impl Mode {
    /// The wire string for this mode (config + heartbeat use the same
    /// spelling as the schema enum).
    pub fn as_str(self) -> &'static str {
        match self {
            Mode::Off => "off",
            Mode::OpticalFlow => "optical_flow",
            Mode::OpticalFlowDegraded => "optical_flow_degraded",
        }
    }

    /// Parse a mode string, or `None` for an unknown value.
    pub fn parse(s: &str) -> Option<Mode> {
        Some(match s {
            "off" => Mode::Off,
            "optical_flow" => Mode::OpticalFlow,
            "optical_flow_degraded" => Mode::OpticalFlowDegraded,
            _ => return None,
        })
    }
}

/// Which way the camera lens points.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Orientation {
    Forward,
    Downward,
    Side,
    Auto,
}

impl Orientation {
    fn parse(s: &str) -> Option<Orientation> {
        Some(match s {
            "forward" => Orientation::Forward,
            "downward" => Orientation::Downward,
            "side" => Orientation::Side,
            "auto" => Orientation::Auto,
            _ => return None,
        })
    }
}

/// Flight firmware. It picks the flow message: ArduPilot and iNav get
/// `OPTICAL_FLOW` (#100), PX4 gets `OPTICAL_FLOW_RAD` (#106).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Firmware {
    Ardupilot,
    Px4,
    Inav,
}

impl Firmware {
    fn parse(s: &str) -> Option<Firmware> {
        Some(match s {
            "ardupilot" => Firmware::Ardupilot,
            "px4" => Firmware::Px4,
            "inav" => Firmware::Inav,
            _ => return None,
        })
    }
}

/// Rangefinder wiring.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Topology {
    Companion,
    Fc,
    None,
}

impl Topology {
    fn parse(s: &str) -> Option<Topology> {
        Some(match s {
            "companion" => Topology::Companion,
            "fc" => Topology::Fc,
            "none" => Topology::None,
            _ => return None,
        })
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Topology::Companion => "companion",
            Topology::Fc => "fc",
            Topology::None => "none",
        }
    }
}

/// Camera capture settings.
#[derive(Debug, Clone, PartialEq)]
pub struct CameraConfig {
    pub device_path: String,
    pub bus_type: String, // "uvc" | "csi"
    pub orientation: Orientation,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
}

impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            device_path: "/dev/video0".to_string(),
            bus_type: "uvc".to_string(),
            orientation: Orientation::Auto,
            width: 640,
            height: 480,
            fps: 30,
        }
    }
}

/// Distance source used to scale optical flow into metric velocity.
#[derive(Debug, Clone, PartialEq)]
pub struct RangefinderConfig {
    pub topology: Topology,
    pub driver: String, // tfluna_uart | fc_relay
    pub device: Option<String>,
    pub baud: u32,
}

impl Default for RangefinderConfig {
    fn default() -> Self {
        Self {
            topology: Topology::Fc,
            driver: "fc_relay".to_string(),
            device: None,
            baud: 115_200,
        }
    }
}

/// Flight firmware identification.
#[derive(Debug, Clone, PartialEq)]
pub struct FirmwareConfig {
    pub firmware: Firmware,
}

impl Default for FirmwareConfig {
    fn default() -> Self {
        Self {
            firmware: Firmware::Ardupilot,
        }
    }
}

/// Top-level plugin configuration.
#[derive(Debug, Clone, PartialEq)]
pub struct VisionNavConfig {
    pub mode: Mode,
    /// Whether the estimator is engaged. When false the effective mode is
    /// `Off`: the pipeline runs the null estimator and emits no flow, so
    /// the flight controller falls back to its own estimator sources. The
    /// engage Skill writes this per-drone key so the operator engages and
    /// disengages vision navigation without changing the configured `mode`.
    pub active: bool,
    pub camera: CameraConfig,
    pub rangefinder: RangefinderConfig,
    pub firmware: FirmwareConfig,
    /// Quality gate (0-255) the pipeline applies before emitting flow.
    pub flow_quality_min: i32,
    /// Outdoor mode: the scale ladder may use the GPS altitude rung,
    /// measured from the first 3D fix.
    pub outdoor: bool,
}

impl VisionNavConfig {
    /// The mode the pipeline actually runs. The configured `mode` when the
    /// estimator is engaged; `Off` when disengaged so the null estimator runs
    /// and nothing reaches the flight controller.
    pub fn effective_mode(&self) -> Mode {
        if self.active {
            self.mode
        } else {
            Mode::Off
        }
    }

    /// The fail-closed configuration: no mode, not engaged. A config
    /// that fails validation lands here rather than on a mode the
    /// operator did not select.
    pub fn disengaged() -> Self {
        Self {
            mode: Mode::Off,
            active: false,
            ..Self::default()
        }
    }
}

impl Default for VisionNavConfig {
    /// The schema defaults. Mode `off` fails safe: a drone with no stored
    /// config emits nothing.
    fn default() -> Self {
        Self {
            mode: Mode::Off,
            active: true,
            camera: CameraConfig::default(),
            rangefinder: RangefinderConfig::default(),
            firmware: FirmwareConfig::default(),
            flow_quality_min: 50,
            outdoor: false,
        }
    }
}

/// A config validation failure, with an operator-readable message.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigError(pub String);

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.0)
    }
}
impl std::error::Error for ConfigError {}

impl VisionNavConfig {
    /// Parse + validate a config map. An empty map yields the defaults.
    /// Unknown keys are ignored, so a stored config carrying keys an
    /// older build offered still loads.
    pub fn from_map(map: &BTreeMap<String, Value>) -> Result<Self, ConfigError> {
        let mut cfg = VisionNavConfig::default();

        if let Some(v) = map.get("mode").and_then(Value::as_str) {
            cfg.mode = Mode::parse(v).ok_or_else(|| ConfigError(format!("unknown mode {v:?}")))?;
        }
        if let Some(b) = map.get("active").and_then(Value::as_bool) {
            cfg.active = b;
        }
        if let Some(v) = map.get("camera") {
            cfg.camera = parse_camera(v)?;
        }
        if let Some(v) = map.get("rangefinder") {
            cfg.rangefinder = parse_rangefinder(v)?;
        }
        if let Some(v) = map.get("firmware") {
            cfg.firmware = parse_firmware(v)?;
        }
        if let Some(q) = map.get("flow_quality_min").and_then(Value::as_i64) {
            cfg.flow_quality_min = q.clamp(0, 255) as i32;
        }
        if let Some(b) = map.get("outdoor").and_then(Value::as_bool) {
            cfg.outdoor = b;
        }

        cfg.validate()?;
        Ok(cfg)
    }

    /// Build from an rmpv `Value` map.
    pub fn from_value(v: &Value) -> Result<Self, ConfigError> {
        let map = value_to_map(v);
        Self::from_map(&map)
    }

    fn validate(&self) -> Result<(), ConfigError> {
        // Optical-flow modes need a downward (or auto) camera.
        if matches!(self.mode, Mode::OpticalFlow | Mode::OpticalFlowDegraded)
            && matches!(
                self.camera.orientation,
                Orientation::Forward | Orientation::Side
            )
        {
            return Err(ConfigError(format!(
                "Mode {:?} needs a downward-facing camera.",
                self.mode.as_str()
            )));
        }
        Ok(())
    }
}

fn parse_camera(v: &Value) -> Result<CameraConfig, ConfigError> {
    let map = value_to_map(v);
    let mut cam = CameraConfig::default();
    if let Some(s) = map.get("device_path").and_then(Value::as_str) {
        cam.device_path = s.to_string();
    }
    if let Some(s) = map.get("bus_type").and_then(Value::as_str) {
        cam.bus_type = s.to_string();
    }
    if let Some(s) = map.get("orientation").and_then(Value::as_str) {
        cam.orientation = Orientation::parse(s)
            .ok_or_else(|| ConfigError(format!("unknown camera orientation {s:?}")))?;
    }
    if let Some(n) = map.get("width").and_then(Value::as_i64) {
        cam.width = n.clamp(64, 4096) as u32;
    }
    if let Some(n) = map.get("height").and_then(Value::as_i64) {
        cam.height = n.clamp(64, 4096) as u32;
    }
    if let Some(n) = map.get("fps").and_then(Value::as_i64) {
        cam.fps = n.clamp(1, 240) as u32;
    }
    Ok(cam)
}

fn parse_rangefinder(v: &Value) -> Result<RangefinderConfig, ConfigError> {
    let map = value_to_map(v);
    let mut rf = RangefinderConfig::default();
    if let Some(s) = map.get("topology").and_then(Value::as_str) {
        rf.topology = Topology::parse(s)
            .ok_or_else(|| ConfigError(format!("unknown rangefinder topology {s:?}")))?;
    }
    if let Some(s) = map.get("driver").and_then(Value::as_str) {
        rf.driver = s.to_string();
    }
    if let Some(s) = map.get("device").and_then(Value::as_str) {
        rf.device = Some(s.to_string());
    }
    if let Some(n) = map.get("baud").and_then(Value::as_i64) {
        rf.baud = n.clamp(1200, 1_000_000) as u32;
    }
    Ok(rf)
}

fn parse_firmware(v: &Value) -> Result<FirmwareConfig, ConfigError> {
    let map = value_to_map(v);
    let mut fw = FirmwareConfig::default();
    if let Some(s) = map.get("type").and_then(Value::as_str) {
        fw.firmware =
            Firmware::parse(s).ok_or_else(|| ConfigError(format!("unknown firmware {s:?}")))?;
    }
    Ok(fw)
}

/// Coerce an rmpv map (or nil) into a string-keyed map for ergonomic
/// field reads. Non-map values yield an empty map.
fn value_to_map(v: &Value) -> BTreeMap<String, Value> {
    let mut out = BTreeMap::new();
    if let Value::Map(entries) = v {
        for (k, val) in entries {
            if let Some(key) = k.as_str() {
                out.insert(key.to_string(), val.clone());
            }
        }
    }
    out
}

#[cfg(test)]
#[path = "config_tests.rs"]
mod tests;
