//! Per-drone configuration model + validation.
//!
//! Mirrors `config-schema.json` at the extension root. The host hands
//! the plugin a config map at `on_configure`;
//! [`VisionNavConfig::from_value`] parses and validates it. Validation
//! rejects the same thing the schema does: an optical-flow mode with a
//! forward- or side-facing camera.

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

/// Flight firmware. Optical flow runs on all three; each one consumes
/// `OPTICAL_FLOW_RAD` through a different parameter set.
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
#[derive(Debug, Clone)]
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
#[derive(Debug, Clone)]
pub struct RangefinderConfig {
    pub topology: Topology,
    pub driver: String, // tfluna_uart | garmin_lidarlite_i2c | vl53l1x_i2c | fc_relay
    pub device: Option<String>,
    pub baud: Option<u32>,
}

impl Default for RangefinderConfig {
    fn default() -> Self {
        Self {
            topology: Topology::Fc,
            driver: "fc_relay".to_string(),
            device: None,
            baud: None,
        }
    }
}

/// Flight firmware identification.
#[derive(Debug, Clone)]
pub struct FirmwareConfig {
    pub firmware: Firmware,
    pub ekf_source_set_index: Option<u8>,
}

impl Default for FirmwareConfig {
    fn default() -> Self {
        Self {
            firmware: Firmware::Ardupilot,
            ekf_source_set_index: None,
        }
    }
}

/// Optional synthetic-origin pre-arm helper.
#[derive(Debug, Clone)]
pub struct PreArmConfig {
    pub auto_set_origin: bool,
    pub origin_lat: f64,
    pub origin_lon: f64,
    pub origin_alt_m: f64,
}

impl Default for PreArmConfig {
    fn default() -> Self {
        Self {
            auto_set_origin: false,
            origin_lat: 0.0,
            origin_lon: 0.0,
            origin_alt_m: 0.0,
        }
    }
}

/// Top-level plugin configuration.
#[derive(Debug, Clone)]
pub struct VisionNavConfig {
    pub mode: Mode,
    /// Whether the estimator is engaged. When false the effective mode is
    /// `Off`: the pipeline runs the null estimator and emits no pose, so the
    /// flight controller falls back to its own estimator sources. The engage
    /// Skill toggles this per-drone key so the operator arms and disarms
    /// vision navigation without changing the configured `mode`. Defaults to
    /// true so an existing config with no `active` key runs its mode as before.
    pub active: bool,
    pub camera: CameraConfig,
    pub rangefinder: RangefinderConfig,
    pub firmware: FirmwareConfig,
    pub pre_arm: PreArmConfig,
    /// Quality gate the pipeline applies before emitting flow frames.
    pub flow_quality_min: i32,
}

impl VisionNavConfig {
    /// The mode the pipeline actually runs. The configured `mode` when the
    /// estimator is engaged; `Off` when disengaged so the null estimator runs
    /// and no pose reaches the flight controller.
    pub fn effective_mode(&self) -> Mode {
        if self.active {
            self.mode
        } else {
            Mode::Off
        }
    }

    /// The fail-closed configuration: no mode, not engaged. A config the
    /// host hands over that fails validation lands here rather than on
    /// the defaults, so a drone whose stored config names a mode this
    /// build no longer offers emits nothing instead of silently running
    /// a mode the operator did not select.
    pub fn disengaged() -> Self {
        Self {
            mode: Mode::Off,
            active: false,
            ..Self::default()
        }
    }
}

impl Default for VisionNavConfig {
    fn default() -> Self {
        Self {
            mode: Mode::OpticalFlow,
            active: true,
            camera: CameraConfig::default(),
            rangefinder: RangefinderConfig::default(),
            firmware: FirmwareConfig::default(),
            pre_arm: PreArmConfig::default(),
            flow_quality_min: 50,
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
    /// Parse + validate a config map handed by the host. An empty /
    /// missing map yields the defaults. Unknown keys are ignored
    /// (forward-compatible, same as the Python `extra="ignore"`).
    pub fn from_map(map: &BTreeMap<String, Value>) -> Result<Self, ConfigError> {
        let mut cfg = VisionNavConfig::default();

        if let Some(v) = map.get("mode").and_then(Value::as_str) {
            cfg.mode = Mode::parse(v)
                .ok_or_else(|| ConfigError(format!("unknown mode {v:?}")))?;
        }
        // The engage Skill writes this per-drone flag. Absent = engaged, so an
        // existing config keeps running its configured mode.
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
        if let Some(v) = map.get("pre_arm") {
            cfg.pre_arm = parse_pre_arm(v);
        }
        if let Some(q) = map.get("flow_quality_min").and_then(Value::as_i64) {
            cfg.flow_quality_min = q.clamp(0, 255) as i32;
        }

        cfg.validate()?;
        Ok(cfg)
    }

    /// Build from an rmpv `Value` (the shape `on_configure` delivers).
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
        rf.baud = Some(n.clamp(1200, 1_000_000) as u32);
    }
    Ok(rf)
}

fn parse_firmware(v: &Value) -> Result<FirmwareConfig, ConfigError> {
    let map = value_to_map(v);
    let mut fw = FirmwareConfig::default();
    if let Some(s) = map.get("type").and_then(Value::as_str) {
        fw.firmware = Firmware::parse(s)
            .ok_or_else(|| ConfigError(format!("unknown firmware {s:?}")))?;
    }
    if let Some(n) = map.get("ekf_source_set_index").and_then(Value::as_i64) {
        fw.ekf_source_set_index = Some(n.clamp(1, 3) as u8);
    }
    Ok(fw)
}

fn parse_pre_arm(v: &Value) -> PreArmConfig {
    let map = value_to_map(v);
    let mut pa = PreArmConfig::default();
    if let Some(b) = map.get("auto_set_origin").and_then(Value::as_bool) {
        pa.auto_set_origin = b;
    }
    if let Some(f) = map.get("origin_lat").and_then(as_f64) {
        pa.origin_lat = f;
    }
    if let Some(f) = map.get("origin_lon").and_then(as_f64) {
        pa.origin_lon = f;
    }
    if let Some(f) = map.get("origin_alt_m").and_then(as_f64) {
        pa.origin_alt_m = f;
    }
    pa
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

fn as_f64(v: &Value) -> Option<f64> {
    v.as_f64().or_else(|| v.as_i64().map(|i| i as f64))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn map(pairs: &[(&str, Value)]) -> Value {
        Value::Map(
            pairs
                .iter()
                .map(|(k, v)| (Value::from(*k), v.clone()))
                .collect(),
        )
    }

    #[test]
    fn empty_config_is_defaults() {
        let cfg = VisionNavConfig::from_value(&Value::Map(vec![])).unwrap();
        assert_eq!(cfg.mode, Mode::OpticalFlow);
        assert_eq!(cfg.flow_quality_min, 50);
        assert_eq!(cfg.firmware.firmware, Firmware::Ardupilot);
    }

    #[test]
    fn mode_this_build_does_not_offer_is_rejected() {
        // A stored config naming a mode an older build offered must be
        // rejected outright, not coerced into a mode the operator did
        // not pick.
        let v = map(&[("mode", Value::from("estimator_from_an_older_build"))]);
        let err = VisionNavConfig::from_value(&v).unwrap_err();
        assert!(err.0.contains("unknown mode"), "{}", err.0);
    }

    #[test]
    fn optical_flow_rejects_forward_camera() {
        let v = map(&[
            ("mode", Value::from("optical_flow")),
            ("camera", map(&[("orientation", Value::from("forward"))])),
        ]);
        assert!(VisionNavConfig::from_value(&v).is_err());
    }

    #[test]
    fn disengaged_config_emits_nothing() {
        // The fail-closed config a rejected `on_configure` falls back to:
        // no mode, not engaged, so the null estimator runs.
        let cfg = VisionNavConfig::disengaged();
        assert_eq!(cfg.effective_mode(), Mode::Off);
    }

    #[test]
    fn unknown_keys_ignored() {
        let v = map(&[("mode", Value::from("off")), ("bogus", Value::from(1i64))]);
        let cfg = VisionNavConfig::from_value(&v).unwrap();
        assert_eq!(cfg.mode, Mode::Off);
    }

    #[test]
    fn active_defaults_true_and_effective_mode_follows_configured() {
        // No active key: engaged, effective mode == configured mode.
        let cfg = VisionNavConfig::from_value(&Value::Map(vec![])).unwrap();
        assert!(cfg.active);
        assert_eq!(cfg.effective_mode(), Mode::OpticalFlow);
    }

    #[test]
    fn disengaged_effective_mode_is_off() {
        // active=false disengages: the effective mode is Off regardless of the
        // configured mode, so the null estimator runs and no pose is emitted.
        let v = map(&[
            ("mode", Value::from("optical_flow")),
            ("active", Value::from(false)),
        ]);
        let cfg = VisionNavConfig::from_value(&v).unwrap();
        assert!(!cfg.active);
        assert_eq!(cfg.mode, Mode::OpticalFlow);
        assert_eq!(cfg.effective_mode(), Mode::Off);
    }
}
