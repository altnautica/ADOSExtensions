//! The `external_nav` and `obstacle` config blocks.
//!
//! `external_nav` feeds the flight controller a pose from another
//! plugin's shared pose topic (`ODOMETRY` plus EKF source-set switching);
//! `obstacle` turns a shared planar scan into `OBSTACLE_DISTANCE`. Both
//! mirror the matching `config-schema.json` objects, defaults included.

use rmpv::Value;

use crate::config::{value_to_map, ConfigError};

/// The shared pose topic the default config subscribes to.
pub const DEFAULT_POSE_TOPIC: &str = "plugin.world-engine.pose";
/// The shared planar-scan topic the default config subscribes to.
pub const DEFAULT_SCAN_TOPIC: &str = "plugin.world-engine.scan";
/// `OBSTACLE_DISTANCE` carries exactly this many distance slots.
pub const MAX_SECTORS: u8 = 72;

/// The `external_nav` block.
#[derive(Debug, Clone, PartialEq)]
pub struct ExternalNavConfig {
    /// Shared topic carrying the pose descriptor.
    pub pose_topic: String,
    /// EKF origin `[lat_deg, lon_deg, alt_m_amsl]`, sent as
    /// `SET_GPS_GLOBAL_ORIGIN` when the flight controller has none, is
    /// disarmed, and GPS is not primary ([`crate::origin`]). Required in
    /// `external_nav` mode unless `gps_primary`.
    pub origin_lla: Option<[f64; 3]>,
    /// `ODOMETRY` rate cap in Hz.
    pub rate_hz: f32,
    /// EKF source set (1-3) whose position source is ExternalNav.
    pub extnav_set: u8,
    /// EKF source set (1-3) used when the external pose is lost (the
    /// flight controller's flow set) or, with `gps_primary`, the GPS set.
    pub fallback_set: u8,
    /// GPS is the primary source; the external pose takes over only
    /// while the GPS fix is lost.
    pub gps_primary: bool,
}

impl Default for ExternalNavConfig {
    fn default() -> Self {
        Self {
            pose_topic: DEFAULT_POSE_TOPIC.to_string(),
            origin_lla: None,
            rate_hz: 30.0,
            extnav_set: 1,
            fallback_set: 2,
            gps_primary: false,
        }
    }
}

/// The `obstacle` block.
#[derive(Debug, Clone, PartialEq)]
pub struct ObstacleConfig {
    pub enabled: bool,
    /// Shared topic carrying the planar scan.
    pub scan_topic: String,
    /// Sector count, 1-72; each sector spans `360 / sectors` degrees.
    pub sectors: u8,
    /// `OBSTACLE_DISTANCE` rate cap in Hz.
    pub rate_hz: f32,
}

impl Default for ObstacleConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            scan_topic: DEFAULT_SCAN_TOPIC.to_string(),
            sectors: MAX_SECTORS,
            rate_hz: 10.0,
        }
    }
}

/// Read a number of any rmpv numeric kind as `f64`.
pub fn as_f64(v: &Value) -> Option<f64> {
    match v {
        Value::F64(x) => Some(*x),
        Value::F32(x) => Some(f64::from(*x)),
        Value::Integer(i) => i.as_f64(),
        _ => None,
    }
}

fn source_set(v: &Value, key: &str) -> Result<u8, ConfigError> {
    match v.as_i64() {
        Some(n @ 1..=3) => Ok(n as u8),
        _ => Err(ConfigError(format!("external_nav.{key} must be 1, 2 or 3"))),
    }
}

pub(crate) fn parse_external_nav(v: &Value) -> Result<ExternalNavConfig, ConfigError> {
    let map = value_to_map(v);
    let mut cfg = ExternalNavConfig::default();
    if let Some(s) = map.get("pose_topic").and_then(Value::as_str) {
        cfg.pose_topic = s.to_string();
    }
    match map.get("origin_lla") {
        None | Some(Value::Nil) => {}
        Some(Value::Array(a)) if a.len() == 3 => {
            let mut out = [0.0; 3];
            for (slot, item) in out.iter_mut().zip(a) {
                *slot = as_f64(item).ok_or_else(|| {
                    ConfigError("external_nav.origin_lla must hold three numbers".into())
                })?;
            }
            if !(-90.0..=90.0).contains(&out[0]) || !(-180.0..=180.0).contains(&out[1]) {
                return Err(ConfigError(
                    "external_nav.origin_lla latitude or longitude out of range".into(),
                ));
            }
            cfg.origin_lla = Some(out);
        }
        Some(_) => {
            return Err(ConfigError(
                "external_nav.origin_lla must be [lat, lon, alt_m]".into(),
            ))
        }
    }
    if let Some(r) = map.get("rate_hz").and_then(as_f64) {
        cfg.rate_hz = r.clamp(1.0, 50.0) as f32;
    }
    if let Some(v) = map.get("extnav_set") {
        cfg.extnav_set = source_set(v, "extnav_set")?;
    }
    if let Some(v) = map.get("fallback_set") {
        cfg.fallback_set = source_set(v, "fallback_set")?;
    }
    if let Some(b) = map.get("gps_primary").and_then(Value::as_bool) {
        cfg.gps_primary = b;
    }
    if cfg.extnav_set == cfg.fallback_set {
        return Err(ConfigError(
            "external_nav.extnav_set and fallback_set must differ".into(),
        ));
    }
    Ok(cfg)
}

pub(crate) fn parse_obstacle(v: &Value) -> Result<ObstacleConfig, ConfigError> {
    let map = value_to_map(v);
    let mut cfg = ObstacleConfig::default();
    if let Some(b) = map.get("enabled").and_then(Value::as_bool) {
        cfg.enabled = b;
    }
    if let Some(s) = map.get("scan_topic").and_then(Value::as_str) {
        cfg.scan_topic = s.to_string();
    }
    if let Some(n) = map.get("sectors").and_then(Value::as_i64) {
        cfg.sectors = n.clamp(1, i64::from(MAX_SECTORS)) as u8;
    }
    if let Some(r) = map.get("rate_hz").and_then(as_f64) {
        cfg.rate_hz = r.clamp(1.0, 20.0) as f32;
    }
    Ok(cfg)
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
    fn external_nav_parses_origin_and_sets() {
        let v = map(&[
            (
                "origin_lla",
                Value::Array(vec![
                    Value::F64(52.07),
                    Value::F64(-0.62),
                    Value::from(110i64),
                ]),
            ),
            ("extnav_set", Value::from(1i64)),
            ("fallback_set", Value::from(3i64)),
            ("gps_primary", Value::from(true)),
            ("rate_hz", Value::F64(200.0)),
        ]);
        let cfg = parse_external_nav(&v).unwrap();
        assert_eq!(cfg.origin_lla, Some([52.07, -0.62, 110.0]));
        assert_eq!(cfg.fallback_set, 3);
        assert!(cfg.gps_primary);
        assert_eq!(cfg.rate_hz, 50.0);
        assert_eq!(cfg.pose_topic, DEFAULT_POSE_TOPIC);
    }

    #[test]
    fn external_nav_rejects_bad_sets_and_origin() {
        let same = map(&[
            ("extnav_set", Value::from(2i64)),
            ("fallback_set", Value::from(2i64)),
        ]);
        assert!(parse_external_nav(&same).is_err());
        let out_of_range = map(&[("extnav_set", Value::from(4i64))]);
        assert!(parse_external_nav(&out_of_range).is_err());
        let short = map(&[("origin_lla", Value::Array(vec![Value::F64(1.0)]))]);
        assert!(parse_external_nav(&short).is_err());
        let bad_lat = map(&[(
            "origin_lla",
            Value::Array(vec![Value::F64(95.0), Value::F64(0.0), Value::F64(0.0)]),
        )]);
        assert!(parse_external_nav(&bad_lat).is_err());
    }

    #[test]
    fn obstacle_sectors_clamp_to_the_message_size() {
        let v = map(&[
            ("enabled", Value::from(true)),
            ("sectors", Value::from(500i64)),
        ]);
        let cfg = parse_obstacle(&v).unwrap();
        assert!(cfg.enabled);
        assert_eq!(cfg.sectors, MAX_SECTORS);
        assert_eq!(cfg.scan_topic, DEFAULT_SCAN_TOPIC);
    }
}
