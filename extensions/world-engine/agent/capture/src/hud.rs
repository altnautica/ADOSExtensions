//! The on-device capture HUD: the `capture_hud` page on the host's onboard
//! display, and the front-panel buttons that drive capture.
//!
//! The display service reserves one data-driven page (page id `plugin`, its
//! content a title plus label/value rows written through `display.page.set`,
//! capability `display.oled.page`); the capture service fills it from the
//! capture state about once a second. The agent's visor screen order lists it
//! as `plugin:capture_hud`.
//!
//! Buttons (`button.subscribe`): the visor button map assigns `passthrough` to
//! B1 and B2 short presses, so only plugin subscribers act on them. B1 toggles
//! recording, B2 marks a new room. A press is matched on the host's button
//! label (`B1`/`B2`, whatever pin the board wires them to) and must carry the
//! `passthrough` action, so a board whose map gives B1 another job never
//! starts a session by accident.

use ados_protocol::buttons::ButtonPress;
use world_engine_protocol::capture_ctl::CaptureStateView;

/// The page's title.
pub const PAGE_TITLE: &str = "Capture";
/// The action the host's button map gives presses reserved for plugins.
pub const PASSTHROUGH: &str = "passthrough";
/// Front-panel labels of the REC and MARK buttons.
pub const REC_LABEL: &str = "B1";
pub const MARK_LABEL: &str = "B2";

/// What a front-panel press asks for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HudAction {
    ToggleRecording,
    Mark,
}

/// The capture action of a press, if any.
pub fn button_action(press: &ButtonPress) -> Option<HudAction> {
    if press.is_long() || press.action.as_deref() != Some(PASSTHROUGH) {
        return None;
    }
    match press.label.as_deref() {
        Some(REC_LABEL) => Some(HudAction::ToggleRecording),
        Some(MARK_LABEL) => Some(HudAction::Mark),
        _ => None,
    }
}

fn mmss(s: f64) -> String {
    let s = s.max(0.0) as u64;
    format!("{}:{:02}", s / 60, s % 60)
}

fn gib(bytes: u64) -> String {
    format!("{:.0} GiB", bytes as f64 / (1u64 << 30) as f64)
}

/// The page rows for a capture state.
pub fn page_rows(st: &CaptureStateView) -> Vec<(String, String)> {
    let mut rows = Vec::new();
    let head = match st.state.as_str() {
        "recording" => format!("REC {}", mmss(st.duration_s.unwrap_or(0.0))),
        "paused" => format!("PAUSED {}", mmss(st.duration_s.unwrap_or(0.0))),
        "closing" => "SAVING".to_string(),
        _ => "READY".to_string(),
    };
    rows.push(("State".into(), head));
    let room = st.room.clone().unwrap_or_else(|| "-".into());
    rows.push(("Room".into(), format!("{room} F{}", st.floor)));
    let fps: Vec<String> = st
        .frames
        .per_camera
        .iter()
        .map(|c| format!("{:.0}", c.fps))
        .collect();
    rows.push((
        "Frames".into(),
        format!("{} fps, {:.1}% drop", fps.join("/"), st.frames.dropped_pct),
    ));
    rows.push((
        "Pose".into(),
        format!("{} {}", st.pose.source, st.pose.health),
    ));
    if let Some(pct) = st.coverage.room_pct {
        rows.push(("Coverage".into(), format!("{pct:.0}%")));
    }
    // Only chips that need the operator: an all-ok HUD stays short.
    let alerts: Vec<String> = st
        .health
        .iter()
        .filter(|h| h.level != "ok")
        .map(|h| match h.id.as_str() {
            "blur" => "slow down".to_string(),
            "texture" => "low texture".to_string(),
            "corridor" => "corridor: face a corner".to_string(),
            other => format!("{other} {}", h.level),
        })
        .collect();
    rows.push((
        "Health".into(),
        if alerts.is_empty() {
            "ok".into()
        } else {
            alerts.join(", ")
        },
    ));
    if let Some(free) = st.storage.free_bytes {
        rows.push(("Storage".into(), format!("{} free", gib(free))));
    }
    rows
}

#[cfg(test)]
mod tests {
    use super::*;
    use world_engine_protocol::capture_ctl::{
        CoverageSummary, ExposureView, FramesView, HealthView, PoseView, StorageView,
    };

    fn press(label: Option<&str>, kind: &str, action: Option<&str>) -> ButtonPress {
        ButtonPress {
            pin: 17,
            label: label.map(str::to_string),
            kind: kind.into(),
            action: action.map(str::to_string),
            timestamp_ms: 0,
        }
    }

    #[test]
    fn only_passthrough_short_presses_on_b1_b2_act() {
        let a = |p| button_action(&p);
        let pt = Some("passthrough");
        assert_eq!(
            a(press(Some("B1"), "short", pt)),
            Some(HudAction::ToggleRecording)
        );
        assert_eq!(a(press(Some("B2"), "short", pt)), Some(HudAction::Mark));
        assert_eq!(a(press(Some("B1"), "long", pt)), None);
        assert_eq!(a(press(Some("B1"), "short", Some("cycle_screen"))), None);
        assert_eq!(a(press(Some("B3"), "short", pt)), None);
        assert_eq!(a(press(None, "short", pt)), None);
        assert_eq!(a(press(Some("B1"), "short", None)), None);
    }

    #[test]
    fn the_page_shows_state_room_and_only_raised_chips() {
        let st = CaptureStateView {
            state: "recording".into(),
            session_id: Some("s".into()),
            started_at_ns: Some(1),
            duration_s: Some(312.4),
            site: None,
            floor: 1,
            room: Some("Kitchen".into()),
            segment: Some(3),
            frames: FramesView::default(),
            pose: PoseView {
                source: "vio".into(),
                health: "ok".into(),
                tracked_features: None,
                drift_hint_m: None,
            },
            coverage: CoverageSummary {
                room_pct: Some(72.0),
                ..Default::default()
            },
            health: vec![
                HealthView {
                    id: "blur".into(),
                    level: "warn".into(),
                    value: Some(30.0),
                },
                HealthView {
                    id: "clock".into(),
                    level: "ok".into(),
                    value: Some(0.4),
                },
            ],
            exposure: ExposureView {
                locked: true,
                exposure_us: Some(2000),
                gain: Some(2.0),
                mode: None,
                applied: false,
                reason: None,
            },
            storage: StorageView {
                free_bytes: Some(812 << 30),
                est_remaining_s: None,
            },
        };
        let rows = page_rows(&st);
        assert_eq!(rows[0], ("State".to_string(), "REC 5:12".to_string()));
        assert_eq!(rows[1].1, "Kitchen F1");
        assert!(rows.iter().any(|(k, v)| k == "Health" && v == "slow down"));
        assert!(rows.iter().any(|(k, v)| k == "Coverage" && v == "72%"));
        assert!(rows
            .iter()
            .any(|(k, v)| k == "Storage" && v == "812 GiB free"));
    }
}
