//! Companion-state machine + the `navigation` heartbeat snapshot.
//!
//! The plugin owns a `navigation` block on the agent's heartbeat,
//! pushed through `ctx.telemetry.extend("navigation", ..)`. The payload
//! shape mirrors the GCS `VisionNavTelemetry` type and the cloud relay
//! `cmd_droneStatus.navigation` validator. Field names are camelCase
//! end-to-end (snake_case drift silently drops the block at the relay).
//!
//! [`CompanionState`] is the comp-198 companion node's state; the
//! degradation ladder transitions it (converged -> ACTIVE; a 2 s
//! degraded/failed streak -> CRITICAL) and it maps to a HEARTBEAT
//! `system_status` and to the GCS-facing estimator state.

use rmpv::Value;

pub use crate::pre_arm::CompanionState;

/// MAVLink `MAV_STATE_*` for the companion HEARTBEAT system_status.
pub fn companion_system_status(state: CompanionState) -> u8 {
    match state {
        CompanionState::Inactive => 3,    // MAV_STATE_STANDBY
        CompanionState::Active => 4,      // MAV_STATE_ACTIVE
        CompanionState::Critical => 6,    // MAV_STATE_CRITICAL
        CompanionState::Terminating => 8, // MAV_STATE_FLIGHT_TERMINATION
    }
}

/// Map the companion state to the GCS-facing estimator-state string.
fn companion_to_estimator_state(state: CompanionState) -> &'static str {
    match state {
        CompanionState::Inactive => "off",
        CompanionState::Active => "converged",
        CompanionState::Critical => "degraded",
        CompanionState::Terminating => "failed",
    }
}

/// The auto-detect summary surfaced verbatim on the heartbeat.
#[derive(Debug, Clone, Default)]
pub struct AutodetectSummary {
    pub suggested_mode: Option<String>,
    pub suggested_mode_reason: Option<String>,
    pub detected_camera_count: Option<i32>,
    pub detected_rangefinder_driver: Option<String>,
}

/// The mutable snapshot the pipeline refreshes each tick. `to_value`
/// serializes the `navigation` block (all keys camelCase).
#[derive(Debug, Clone, Default)]
pub struct HealthSnapshot {
    pub rangefinder_topology: Option<String>,
    pub recommended_camera_id: Option<String>,
    pub mode: Option<String>,
    pub available_estimators: Vec<String>,
    pub companion_state: Option<CompanionState>,
    pub flow_quality: Option<i32>,
    pub flow_rate_hz: Option<f32>,
    pub flow_distance_m: Option<f32>,
    pub flow_scale_source: Option<String>,
    pub estimator_state: Option<String>,
    pub imu_source: Option<String>,
    pub imu_rate_hz: Option<f32>,
    pub camera_imu_sync_offset_ms: Option<f32>,
    pub camera_intrinsics_loaded: bool,
    pub vio_state: Option<String>,
    pub vio_reset_counter: u32,
    pub vio_quality: Option<i32>,
    pub feature_count: Option<i32>,
    pub pre_arm_report: Option<Value>,
    pub autodetect: AutodetectSummary,
}

impl HealthSnapshot {
    /// Serialize as the `navigation` payload. Optional fields ride as
    /// `Nil` so an older GCS render path stays correct and the block is
    /// a strict superset of the previous shape.
    pub fn to_value(&self) -> Value {
        let companion = self.companion_state.unwrap_or(CompanionState::Inactive);
        let estimator_state = self
            .estimator_state
            .clone()
            .unwrap_or_else(|| companion_to_estimator_state(companion).to_string());

        let mut entries: Vec<(Value, Value)> = vec![
            (k("opticalFlowSupported"), Value::Boolean(true)),
            (k("vioSupported"), Value::Boolean(true)),
            (k("rangefinderTopology"), opt_str(&self.rangefinder_topology)),
            (k("recommendedCameraId"), opt_str(&self.recommended_camera_id)),
            (k("flowQuality"), opt_i32(self.flow_quality)),
            (k("flowRateHz"), opt_f32(self.flow_rate_hz)),
            (k("flowDistanceM"), opt_f32(self.flow_distance_m)),
            (k("vioState"), opt_str(&self.vio_state.clone().or(Some("absent".into())))),
            (k("vioResetCounter"), Value::from(self.vio_reset_counter)),
            (k("vioQuality"), opt_i32(self.vio_quality)),
            (k("companionState"), Value::from(companion.as_str())),
            (k("mode"), opt_str(&self.mode)),
            (
                k("availableEstimators"),
                Value::Array(self.available_estimators.iter().map(|s| Value::from(s.as_str())).collect()),
            ),
            (k("estimatorState"), Value::from(estimator_state)),
            (k("flowScaleSource"), opt_str(&self.flow_scale_source)),
            (k("imuSource"), opt_str(&self.imu_source)),
            (k("imuRateHz"), opt_f32(self.imu_rate_hz)),
            (k("cameraImuSyncOffsetMs"), opt_f32(self.camera_imu_sync_offset_ms)),
            (k("cameraIntrinsicsLoaded"), Value::Boolean(self.camera_intrinsics_loaded)),
            (k("estimatorFeatureCount"), opt_i32(self.feature_count)),
            (
                k("preArmReport"),
                self.pre_arm_report.clone().unwrap_or(Value::Nil),
            ),
            (k("suggestedMode"), opt_str(&self.autodetect.suggested_mode)),
            (
                k("suggestedModeReason"),
                opt_str(&self.autodetect.suggested_mode_reason),
            ),
            (k("detectedCameraCount"), opt_i32(self.autodetect.detected_camera_count)),
            (
                k("detectedRangefinderDriver"),
                opt_str(&self.autodetect.detected_rangefinder_driver),
            ),
        ];
        // Keep insertion order stable for tests.
        entries.shrink_to_fit();
        Value::Map(entries)
    }
}

fn k(s: &str) -> Value {
    Value::from(s)
}
fn opt_str(s: &Option<String>) -> Value {
    match s {
        Some(v) => Value::from(v.as_str()),
        None => Value::Nil,
    }
}
fn opt_i32(v: Option<i32>) -> Value {
    match v {
        Some(x) => Value::from(x),
        None => Value::Nil,
    }
}
fn opt_f32(v: Option<f32>) -> Value {
    match v {
        Some(x) => Value::F64(x as f64),
        None => Value::Nil,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn field<'a>(v: &'a Value, key: &str) -> Option<&'a Value> {
        match v {
            Value::Map(e) => e.iter().find(|(k, _)| k.as_str() == Some(key)).map(|(_, val)| val),
            _ => None,
        }
    }

    #[test]
    fn snapshot_is_camelcase_superset() {
        let snap = HealthSnapshot {
            mode: Some("optical_flow".into()),
            companion_state: Some(CompanionState::Active),
            flow_quality: Some(120),
            available_estimators: vec!["off".into(), "optical_flow".into()],
            ..Default::default()
        };
        let v = snap.to_value();
        assert_eq!(field(&v, "mode").unwrap().as_str(), Some("optical_flow"));
        assert_eq!(field(&v, "flowQuality").unwrap().as_i64(), Some(120));
        assert_eq!(field(&v, "companionState").unwrap().as_str(), Some("active"));
        // Active companion maps the GCS estimator state to converged.
        assert_eq!(field(&v, "estimatorState").unwrap().as_str(), Some("converged"));
        // vioSupported is now true (the Rust agent ships VIO modes).
        assert_eq!(field(&v, "vioSupported").unwrap().as_bool(), Some(true));
    }

    #[test]
    fn unset_optional_fields_serialize_nil() {
        let snap = HealthSnapshot::default();
        let v = snap.to_value();
        assert!(matches!(field(&v, "flowDistanceM"), Some(Value::Nil)));
        assert!(matches!(field(&v, "imuSource"), Some(Value::Nil)));
    }

    #[test]
    fn companion_system_status_maps() {
        assert_eq!(companion_system_status(CompanionState::Active), 4);
        assert_eq!(companion_system_status(CompanionState::Critical), 6);
    }
}
