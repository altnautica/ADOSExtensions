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
//! * `optical_flow` — companion active, flow quality above gate, and a
//!   configured rangefinder with a fresh reading inside its min-max
//!   range.
//! * `optical_flow_degraded` — companion active, flow quality above
//!   gate, and a live scale source (rangefinder, baro, GPS or vision).
//!   The 1.5 m static fallback blocks.
//!
//! The gate is advisory: it reports readiness to the GCS and does not
//! stop the flight controller from arming.
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
    /// This tick's reading, `None` when no fresh reading exists.
    pub rangefinder_reading_m: Option<f32>,
    pub rangefinder_min_m: f32,
    pub rangefinder_max_m: f32,
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
        let mut checks = vec![
            self.companion_check(inputs),
            self.flow_quality_check(inputs),
        ];
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
            CompanionState::Inactive => {
                PreArmCheck::pending("companion_active", "Companion has not reported active yet.")
            }
            other => PreArmCheck::blocking(
                "companion_active",
                format!("Companion state {:?}.", other.as_str()),
            ),
        }
    }

    fn flow_quality_check(&self, inputs: &PreArmInputs) -> PreArmCheck {
        match inputs.flow_quality {
            None => PreArmCheck::pending("flow_quality", "Awaiting first optical-flow sample."),
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
            Some(topology @ ("companion" | "fc" | "both")) => {
                let (min, max) = (inputs.rangefinder_min_m, inputs.rangefinder_max_m);
                match inputs.rangefinder_reading_m {
                    None => PreArmCheck::blocking("rangefinder", "No fresh rangefinder reading."),
                    Some(d) if max > min && !(min..=max).contains(&d) => PreArmCheck::blocking(
                        "rangefinder",
                        format!("Rangefinder reading {d:.2} m outside {min:.2}-{max:.2} m."),
                    ),
                    Some(d) => {
                        PreArmCheck::ok("rangefinder", format!("Topology: {topology}, {d:.2} m"))
                    }
                }
            }
            _ => PreArmCheck::blocking(
                "rangefinder",
                "Rangefinder required in this mode. Switch to Optical Flow \
                 (degraded) to fly without one.",
            ),
        }
    }

    fn scale_source_check(&self, inputs: &PreArmInputs) -> PreArmCheck {
        match inputs.flow_scale_source.as_deref() {
            Some(source @ ("rangefinder" | "baro" | "gps" | "vision")) => {
                PreArmCheck::ok("scale_source", format!("Scale source: {source}."))
            }
            Some("static") => PreArmCheck::blocking(
                "scale_source",
                "Only the 1.5 m static fallback is available; no live altitude source.",
            ),
            _ => PreArmCheck::blocking("scale_source", "No altitude or depth source healthy."),
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
            rangefinder_reading_m: None,
            rangefinder_min_m: 0.2,
            rangefinder_max_m: 8.0,
        }
    }

    #[test]
    fn off_is_always_armable() {
        let r = PreArmGate::default().evaluate(&base(Mode::Off));
        assert!(r.armable);
        assert!(r.checks.is_empty());
    }

    #[test]
    fn optical_flow_armable_when_all_ok_with_a_fresh_reading() {
        let mut i = base(Mode::OpticalFlow);
        i.companion_state = CompanionState::Active;
        i.flow_quality = Some(120);
        i.rangefinder_topology = Some("companion".to_string());
        i.rangefinder_reading_m = Some(2.0);
        let r = PreArmGate::default().evaluate(&i);
        assert!(r.armable, "{:?}", r.checks);
    }

    fn rangefinder_severity(i: &PreArmInputs) -> Severity {
        PreArmGate::default()
            .evaluate(i)
            .checks
            .into_iter()
            .find(|c| c.id == "rangefinder")
            .unwrap()
            .severity
    }

    #[test]
    fn topology_without_a_reading_blocks() {
        let mut i = base(Mode::OpticalFlow);
        i.companion_state = CompanionState::Active;
        i.flow_quality = Some(120);
        i.rangefinder_topology = Some("fc".to_string());
        assert_eq!(rangefinder_severity(&i), Severity::Blocking);
        assert!(!PreArmGate::default().evaluate(&i).armable);
    }

    #[test]
    fn out_of_range_reading_blocks() {
        let mut i = base(Mode::OpticalFlow);
        i.rangefinder_topology = Some("companion".to_string());
        i.rangefinder_reading_m = Some(12.0);
        assert_eq!(rangefinder_severity(&i), Severity::Blocking);
        i.rangefinder_reading_m = Some(0.1);
        assert_eq!(rangefinder_severity(&i), Severity::Blocking);
    }

    #[test]
    fn degraded_blocks_on_static_scale() {
        let mut i = base(Mode::OpticalFlowDegraded);
        i.companion_state = CompanionState::Active;
        i.flow_quality = Some(120);
        i.flow_scale_source = Some("static".to_string());
        let r = PreArmGate::default().evaluate(&i);
        assert!(!r.armable);
        assert!(r
            .checks
            .iter()
            .any(|c| c.id == "scale_source" && c.severity == Severity::Blocking));
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
