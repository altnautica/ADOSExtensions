//! Mode-aware pre-arm gate.
//!
//! Runs every tick and reports a structured set of checks the GCS
//! pre-arm card consumes. Each check is ok / pending / blocking with an
//! operator-readable reason; the aggregate `armable` flag is true only
//! when every check is ok.
//!
//! Mode rules:
//!
//! * `off` — no checks; armable passthrough.
//! * `optical_flow` — companion active, flow quality above gate,
//!   rangefinder healthy.
//! * `optical_flow_degraded` — companion active, flow quality above
//!   gate, any scale source healthy (rangefinder optional).
//!
//! Pure: no MAVLink, no telemetry, no async. The pipeline builds the
//! [`PreArmInputs`] snapshot and serializes the [`PreArmReport`] onto
//! the heartbeat.

use rmpv::Value;

use crate::config::Mode;

pub const DEFAULT_FLOW_QUALITY_GATE: i32 = 50;

/// Check severity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Severity {
    Ok,
    Pending,
    Blocking,
}

impl Severity {
    fn as_str(self) -> &'static str {
        match self {
            Severity::Ok => "ok",
            Severity::Pending => "pending",
            Severity::Blocking => "blocking",
        }
    }
}

/// One pre-arm check result. `id` is a stable machine key the GCS maps
/// to a row icon + i18n string.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PreArmCheck {
    pub id: &'static str,
    pub severity: Severity,
    pub detail: String,
}

impl PreArmCheck {
    fn ok(id: &'static str, detail: impl Into<String>) -> Self {
        Self {
            id,
            severity: Severity::Ok,
            detail: detail.into(),
        }
    }
    fn pending(id: &'static str, detail: impl Into<String>) -> Self {
        Self {
            id,
            severity: Severity::Pending,
            detail: detail.into(),
        }
    }
    fn blocking(id: &'static str, detail: impl Into<String>) -> Self {
        Self {
            id,
            severity: Severity::Blocking,
            detail: detail.into(),
        }
    }
}

/// Aggregate pre-arm state for the heartbeat.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PreArmReport {
    pub mode: Mode,
    pub armable: bool,
    pub checks: Vec<PreArmCheck>,
}

impl PreArmReport {
    /// Wire-friendly map for the navigation heartbeat block.
    pub fn to_value(&self) -> Value {
        let checks: Vec<Value> = self
            .checks
            .iter()
            .map(|c| {
                Value::Map(vec![
                    (Value::from("id"), Value::from(c.id)),
                    (Value::from("severity"), Value::from(c.severity.as_str())),
                    (Value::from("detail"), Value::from(c.detail.as_str())),
                ])
            })
            .collect();
        Value::Map(vec![
            (Value::from("mode"), Value::from(self.mode.as_str())),
            (Value::from("armable"), Value::Boolean(self.armable)),
            (Value::from("checks"), Value::Array(checks)),
        ])
    }
}

/// Snapshot of the inputs the gate consults, built by the pipeline per
/// tick.
#[derive(Debug, Clone)]
pub struct PreArmInputs {
    pub mode: Mode,
    pub companion_state: CompanionState,
    pub flow_quality: Option<i32>,
    pub flow_scale_source: Option<String>,
    pub rangefinder_topology: Option<String>, // companion|fc|both|None
}

/// Companion node state mirrored from the comp-198 heartbeat machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompanionState {
    Inactive,
    Active,
    Critical,
    Terminating,
}

impl CompanionState {
    pub fn as_str(self) -> &'static str {
        match self {
            CompanionState::Inactive => "inactive",
            CompanionState::Active => "active",
            CompanionState::Critical => "critical",
            CompanionState::Terminating => "terminating",
        }
    }
}

/// The gate. The flow-quality gate mirrors the config's
/// `flow_quality_min`.
pub struct PreArmGate {
    flow_quality_gate: i32,
}

impl Default for PreArmGate {
    fn default() -> Self {
        Self {
            flow_quality_gate: DEFAULT_FLOW_QUALITY_GATE,
        }
    }
}

impl PreArmGate {
    pub fn with_flow_quality_gate(flow_quality_gate: i32) -> Self {
        Self { flow_quality_gate }
    }

    pub fn evaluate(&self, inputs: &PreArmInputs) -> PreArmReport {
        let checks = match inputs.mode {
            Mode::Off => Vec::new(),
            Mode::OpticalFlow => self.of_checks(inputs, true),
            Mode::OpticalFlowDegraded => self.of_checks(inputs, false),
        };
        let armable = if checks.is_empty() {
            true
        } else {
            checks.iter().all(|c| c.severity == Severity::Ok)
        };
        PreArmReport {
            mode: inputs.mode,
            armable,
            checks,
        }
    }

    fn of_checks(&self, inputs: &PreArmInputs, require_rangefinder: bool) -> Vec<PreArmCheck> {
        let mut checks = vec![self.companion_check(inputs), self.flow_quality_check(inputs)];
        if require_rangefinder {
            checks.push(self.rangefinder_check(inputs));
        } else {
            checks.push(self.scale_source_check(inputs));
        }
        checks
    }

    fn companion_check(&self, inputs: &PreArmInputs) -> PreArmCheck {
        match inputs.companion_state {
            CompanionState::Active => PreArmCheck::ok("companion_active", ""),
            CompanionState::Inactive => PreArmCheck::pending(
                "companion_active",
                "Companion has not reported active yet.",
            ),
            other => PreArmCheck::blocking(
                "companion_active",
                format!("Companion state {:?}.", other.as_str()),
            ),
        }
    }

    fn flow_quality_check(&self, inputs: &PreArmInputs) -> PreArmCheck {
        match inputs.flow_quality {
            None => {
                PreArmCheck::pending("flow_quality", "Awaiting first optical-flow sample.")
            }
            Some(q) if q >= self.flow_quality_gate => {
                PreArmCheck::ok("flow_quality", format!("Quality {q}/255."))
            }
            Some(q) => PreArmCheck::blocking(
                "flow_quality",
                format!("Quality {q} below gate {}.", self.flow_quality_gate),
            ),
        }
    }

    fn rangefinder_check(&self, inputs: &PreArmInputs) -> PreArmCheck {
        match inputs.rangefinder_topology.as_deref() {
            Some("companion") | Some("fc") | Some("both") => PreArmCheck::ok(
                "rangefinder",
                format!("Topology: {}", inputs.rangefinder_topology.as_deref().unwrap()),
            ),
            _ => PreArmCheck::blocking(
                "rangefinder",
                "Rangefinder required in this mode. Switch to Optical Flow \
                 (degraded) to fly without one.",
            ),
        }
    }

    fn scale_source_check(&self, inputs: &PreArmInputs) -> PreArmCheck {
        match inputs.flow_scale_source.as_deref() {
            Some("rangefinder") | Some("baro") | Some("gps") | Some("vision") => PreArmCheck::ok(
                "scale_source",
                format!("Scale source: {}.", inputs.flow_scale_source.as_deref().unwrap()),
            ),
            _ => PreArmCheck::blocking(
                "scale_source",
                "No altitude or depth source healthy.",
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base(mode: Mode) -> PreArmInputs {
        PreArmInputs {
            mode,
            companion_state: CompanionState::Inactive,
            flow_quality: None,
            flow_scale_source: None,
            rangefinder_topology: None,
        }
    }

    #[test]
    fn off_is_always_armable() {
        let r = PreArmGate::default().evaluate(&base(Mode::Off));
        assert!(r.armable);
        assert!(r.checks.is_empty());
    }

    #[test]
    fn optical_flow_armable_when_all_ok() {
        let mut i = base(Mode::OpticalFlow);
        i.companion_state = CompanionState::Active;
        i.flow_quality = Some(120);
        i.rangefinder_topology = Some("companion".to_string());
        let r = PreArmGate::default().evaluate(&i);
        assert!(r.armable, "{:?}", r.checks);
    }

    #[test]
    fn optical_flow_blocks_without_rangefinder() {
        let mut i = base(Mode::OpticalFlow);
        i.companion_state = CompanionState::Active;
        i.flow_quality = Some(120);
        // no rangefinder topology
        let r = PreArmGate::default().evaluate(&i);
        assert!(!r.armable);
        assert!(r
            .checks
            .iter()
            .any(|c| c.id == "rangefinder" && c.severity == Severity::Blocking));
    }

    #[test]
    fn degraded_accepts_scale_source_in_lieu_of_rangefinder() {
        let mut i = base(Mode::OpticalFlowDegraded);
        i.companion_state = CompanionState::Active;
        i.flow_quality = Some(120);
        i.flow_scale_source = Some("baro".to_string());
        let r = PreArmGate::default().evaluate(&i);
        assert!(r.armable, "{:?}", r.checks);
    }
}
