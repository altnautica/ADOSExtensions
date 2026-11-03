//! Pre-arm checks for `external_nav` mode.
//!
//! * `extnav_source`: the pose topic is subscribed and its payloads parse.
//!   A refused subscription (the publishing plugin is not installed, or
//!   the grant is missing) blocks as "External nav source missing".
//! * `extnav_health`: good passes, degraded is pending, lost blocks.
//! * `ekf_origin`: the flight controller reports an EKF origin.
//! * `obstacle_feed` (when the obstacle feed is enabled): scans arrive.

use crate::external_nav::ExtnavHealth;
use crate::pre_arm::PreArmCheck;

/// The obstacle feed's state for its pre-arm row.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ObstacleFeed {
    Denied,
    Waiting,
    Live,
}

/// What the external-nav runtime knows this tick.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExtnavPreArmInputs {
    /// The pose-topic subscription was refused.
    pub source_denied: bool,
    /// Why the last pose payload was rejected, when it was.
    pub pose_error: Option<String>,
    /// Health of the fresh pose; `None` when there is none.
    pub health: Option<ExtnavHealth>,
    pub origin_seen: bool,
    /// `None` when the obstacle feed is disabled.
    pub obstacle: Option<ObstacleFeed>,
}

pub(crate) fn extnav_checks(inputs: Option<&ExtnavPreArmInputs>) -> Vec<PreArmCheck> {
    let Some(i) = inputs else {
        return vec![PreArmCheck::blocking(
            "extnav_source",
            "External nav source missing.",
        )];
    };
    let source = if i.source_denied {
        PreArmCheck::blocking(
            "extnav_source",
            "External nav source missing: the pose topic is not published by an \
             installed plugin, or reading it is not granted.",
        )
    } else if let Some(err) = &i.pose_error {
        PreArmCheck::blocking("extnav_source", format!("Pose rejected: {err}."))
    } else if i.health.is_none() {
        PreArmCheck::pending("extnav_source", "Awaiting a pose.")
    } else {
        PreArmCheck::ok("extnav_source", "")
    };
    let health = match i.health {
        Some(ExtnavHealth::Good) => PreArmCheck::ok("extnav_health", "Tracking good."),
        Some(ExtnavHealth::Degraded) => {
            PreArmCheck::pending("extnav_health", "Tracking degraded; position noise raised.")
        }
        Some(ExtnavHealth::Lost) => PreArmCheck::blocking("extnav_health", "Tracking lost."),
        None => PreArmCheck::pending("extnav_health", "No fresh pose."),
    };
    let origin = if i.origin_seen {
        PreArmCheck::ok("ekf_origin", "")
    } else {
        PreArmCheck::pending("ekf_origin", "EKF origin not set yet.")
    };
    let mut checks = vec![source, health, origin];
    match i.obstacle {
        None => {}
        Some(ObstacleFeed::Live) => checks.push(PreArmCheck::ok("obstacle_feed", "")),
        Some(ObstacleFeed::Waiting) => {
            checks.push(PreArmCheck::pending("obstacle_feed", "Awaiting a scan."))
        }
        Some(ObstacleFeed::Denied) => checks.push(PreArmCheck::blocking(
            "obstacle_feed",
            "Scan topic not available.",
        )),
    }
    checks
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Mode;
    use crate::pre_arm::{CompanionState, PreArmGate, PreArmInputs, Severity};

    fn inputs(extnav: Option<ExtnavPreArmInputs>) -> PreArmInputs {
        PreArmInputs {
            mode: Mode::ExternalNav,
            companion_state: CompanionState::Inactive,
            flow_quality: None,
            flow_scale_source: None,
            rangefinder_topology: None,
            rangefinder_reading_m: None,
            rangefinder_min_m: 0.0,
            rangefinder_max_m: 0.0,
            extnav,
        }
    }

    fn ready() -> ExtnavPreArmInputs {
        ExtnavPreArmInputs {
            source_denied: false,
            pose_error: None,
            health: Some(ExtnavHealth::Good),
            origin_seen: true,
            obstacle: Some(ObstacleFeed::Live),
        }
    }

    fn severity(r: &crate::pre_arm::PreArmReport, id: &str) -> Severity {
        r.checks.iter().find(|c| c.id == id).unwrap().severity
    }

    #[test]
    fn ready_external_nav_is_armable() {
        let r = PreArmGate::default().evaluate(&inputs(Some(ready())));
        assert!(r.armable, "{:?}", r.checks);
        assert_eq!(r.checks.len(), 4);
    }

    #[test]
    fn a_missing_source_blocks() {
        let r = PreArmGate::default().evaluate(&inputs(Some(ExtnavPreArmInputs {
            source_denied: true,
            health: None,
            ..ready()
        })));
        assert!(!r.armable);
        assert_eq!(severity(&r, "extnav_source"), Severity::Blocking);
        let detail = &r.checks[0].detail;
        assert!(
            detail.starts_with("External nav source missing"),
            "{detail}"
        );
        let none = PreArmGate::default().evaluate(&inputs(None));
        assert_eq!(severity(&none, "extnav_source"), Severity::Blocking);
    }

    #[test]
    fn lost_blocks_degraded_and_origin_wait() {
        let lost = PreArmGate::default().evaluate(&inputs(Some(ExtnavPreArmInputs {
            health: Some(ExtnavHealth::Lost),
            ..ready()
        })));
        assert_eq!(severity(&lost, "extnav_health"), Severity::Blocking);
        let degraded = PreArmGate::default().evaluate(&inputs(Some(ExtnavPreArmInputs {
            health: Some(ExtnavHealth::Degraded),
            origin_seen: false,
            obstacle: None,
            ..ready()
        })));
        assert_eq!(severity(&degraded, "extnav_health"), Severity::Pending);
        assert_eq!(severity(&degraded, "ekf_origin"), Severity::Pending);
        assert_eq!(degraded.checks.len(), 3);
        assert!(!degraded.armable);
    }
}
