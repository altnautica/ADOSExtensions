//! The capture control socket (`atlas-control.sock`), version 2.
//!
//! The capture service binds it; the link (the extension's HTTP API, the MCP
//! tool handler) is the client. One connection per exchange: one
//! newline-delimited JSON [`CtlRequest`] in, one newline-delimited JSON
//! [`CtlReply`] out.
//!
//! | `cmd`      | args            | `result` on success              |
//! |------------|-----------------|----------------------------------|
//! | `status`   | -               | [`crate::atlas::CaptureStatus`]  |
//! | `state`    | -               | [`CaptureStateView`]             |
//! | `coverage` | -               | [`CoverageView`]                 |
//! | `start`    | [`StartArgs`]   | `{"session_id": ".."}`           |
//! | `stop`     | -               | [`CaptureStateView`]             |
//! | `pause`    | -               | [`CaptureStateView`]             |
//! | `resume`   | -               | [`CaptureStateView`]             |
//! | `mark`     | [`MarkArgs`]    | `{"segment": n}`                 |
//! | `exposure` | [`ExposureView`]| [`ExposureView`]                 |
//!
//! A refusal is `{"ok": false, "error": "<CODE>", "detail": ".."}` with a
//! code from [`codes`]; the HTTP layer maps codes to statuses with
//! [`http_status`].

use serde::{Deserialize, Serialize};
use serde_json::Value;

/// Wire version of this control protocol.
pub const CAPTURE_CTL_VERSION: u16 = 2;

/// How long the link waits for a control reply (`stop` excepted). The
/// capture service answers every other request, `exposure` included, inside
/// this.
pub const CONTROL_REQUEST_TIMEOUT_MS: u64 = 3_000;

/// Refusal codes.
pub mod codes {
    pub const STORAGE_FULL: &str = "E_STORAGE_FULL";
    pub const ALREADY_RECORDING: &str = "E_ALREADY_RECORDING";
    pub const NOT_RECORDING: &str = "E_NOT_RECORDING";
    pub const BAD_REQUEST: &str = "E_BAD_REQUEST";
    pub const NO_RIG: &str = "E_NO_RIG";
    pub const RECORDER: &str = "E_RECORDER";
    /// The clock service names a non-host timebase but no clock sample has
    /// arrived on the sensor bus yet.
    pub const CLOCK_NOT_READY: &str = "E_CLOCK_NOT_READY";
}

/// The HTTP status a refusal code maps to.
pub fn http_status(code: &str) -> u16 {
    match code {
        codes::STORAGE_FULL
        | codes::ALREADY_RECORDING
        | codes::NOT_RECORDING
        | codes::CLOCK_NOT_READY => 409,
        codes::BAD_REQUEST | codes::NO_RIG => 400,
        _ => 500,
    }
}

/// An operator-typed anchor for a session without GNSS.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct OperatorAnchor {
    pub lat_deg: f64,
    pub lon_deg: f64,
    pub h_ellipsoid_m: f64,
    #[serde(default)]
    pub heading_deg: Option<f64>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct StartArgs {
    #[serde(default)]
    pub site: Option<String>,
    #[serde(default)]
    pub floor: Option<i64>,
    #[serde(default)]
    pub room: Option<String>,
    #[serde(default)]
    pub operator_anchor: Option<OperatorAnchor>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MarkArgs {
    pub room: String,
    /// The new room's floor; absent keeps the session's current floor.
    #[serde(default)]
    pub floor: Option<i32>,
}

/// Exposure regime (A9): `walking` caps exposure at 4 ms, `slow` at 10 ms,
/// `short` keeps it at or under 2 ms (flicker-immune).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ExposureMode {
    #[default]
    Walking,
    Slow,
    Short,
}

/// `exposure` in the state and the `PUT .../exposure` body/reply. `locked`
/// is the request; `applied` is true only when every enabled capture camera
/// took the values, which are then the first camera's read-back. Otherwise
/// the requested (quantised) values are echoed with `applied: false` and
/// `reason` names why: `pending` while the cameras are still being set (a
/// later state read carries the outcome), or the host's code (`not_v4l2`,
/// `control_unsupported`, `gain_unit_unknown`, `not_capture_camera`, ..).
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct ExposureView {
    #[serde(default)]
    pub locked: bool,
    /// Exposure time, microseconds.
    #[serde(default)]
    pub exposure_us: Option<u32>,
    /// Analogue gain as a multiplier (×; 1.0 is unity), not dB.
    #[serde(default)]
    pub gain: Option<f32>,
    #[serde(default)]
    pub mode: Option<ExposureMode>,
    #[serde(default)]
    pub applied: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reason: Option<String>,
}

/// A control request.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "cmd", rename_all = "snake_case")]
pub enum CtlRequest {
    Status,
    State,
    Coverage,
    Start(StartArgs),
    Stop,
    Pause,
    Resume,
    Mark(MarkArgs),
    Exposure(ExposureView),
}

/// A control reply.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CtlReply {
    pub ok: bool,
    #[serde(default)]
    pub result: Value,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub detail: Option<String>,
}

impl CtlReply {
    pub fn ok(result: impl Serialize) -> Self {
        Self {
            ok: true,
            result: serde_json::to_value(result).unwrap_or(Value::Null),
            error: None,
            detail: None,
        }
    }

    pub fn err(code: &str, detail: impl Into<String>) -> Self {
        Self {
            ok: false,
            result: Value::Null,
            error: Some(code.to_string()),
            detail: Some(detail.into()),
        }
    }
}

/// `frames.per_camera[]`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CameraFramesView {
    pub id: String,
    pub fps: f64,
    pub dropped: u64,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct FramesView {
    pub total: u64,
    pub dropped_pct: f64,
    pub per_camera: Vec<CameraFramesView>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct PoseView {
    /// `fc_ekf|vio|lidar|replay|none`.
    pub source: String,
    /// `ok|degraded|lost`.
    pub health: String,
    pub tracked_features: Option<u32>,
    pub drift_hint_m: Option<f64>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct CoverageSummary {
    pub voxels: u64,
    pub room_pct: Option<f64>,
    pub loop_closed: bool,
    pub distance_from_start_m: Option<f64>,
}

/// One health chip; ids exactly `blur|texture|flicker|corridor|storage|clock`.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct HealthView {
    pub id: String,
    /// `ok|warn|bad`.
    pub level: String,
    pub value: Option<f64>,
}

#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct StorageView {
    pub free_bytes: Option<u64>,
    /// Whole seconds of recording left at the current write rate; `None`
    /// when unknown.
    pub est_remaining_s: Option<u64>,
}

/// `GET /atlas/capture/state`.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct CaptureStateView {
    /// `idle|recording|paused|closing`.
    pub state: String,
    pub session_id: Option<String>,
    pub started_at_ns: Option<u64>,
    pub duration_s: Option<f64>,
    pub site: Option<String>,
    pub floor: i64,
    pub room: Option<String>,
    pub segment: Option<u32>,
    pub frames: FramesView,
    pub pose: PoseView,
    pub coverage: CoverageSummary,
    pub health: Vec<HealthView>,
    pub exposure: ExposureView,
    pub storage: StorageView,
}

/// `GET /atlas/capture/coverage`: top-down coverage cells `[ix, iy, hits]`
/// on a `cell_m` grid and the walked trajectory, local ENU metres.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
pub struct CoverageView {
    pub cell_m: f64,
    pub cells: Vec<[i64; 3]>,
    pub trajectory: Vec<[f64; 3]>,
    pub start: Option<[f64; 3]>,
    pub current: Option<[f64; 3]>,
    pub heading_deg: Option<f64>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn requests_use_the_cmd_tag() {
        let r: CtlRequest =
            serde_json::from_value(json!({"cmd": "mark", "room": "Kitchen", "floor": 1})).unwrap();
        assert_eq!(
            r,
            CtlRequest::Mark(MarkArgs {
                room: "Kitchen".into(),
                floor: Some(1)
            })
        );
        let r: CtlRequest = serde_json::from_value(json!({"cmd": "mark", "room": "Hall"})).unwrap();
        assert_eq!(
            r,
            CtlRequest::Mark(MarkArgs {
                room: "Hall".into(),
                floor: None
            }),
            "an absent floor keeps the current one, it is not floor 0"
        );
        let s: CtlRequest = serde_json::from_value(json!({"cmd": "start"})).unwrap();
        assert_eq!(s, CtlRequest::Start(StartArgs::default()));
        assert_eq!(
            serde_json::to_value(CtlRequest::Status).unwrap(),
            json!({"cmd": "status"})
        );
    }

    #[test]
    fn refusals_map_to_http_statuses() {
        assert_eq!(http_status(codes::STORAGE_FULL), 409);
        assert_eq!(http_status(codes::BAD_REQUEST), 400);
        assert_eq!(http_status(codes::CLOCK_NOT_READY), 409);
    }

    #[test]
    fn exposure_applied_defaults_false() {
        let v: ExposureView = serde_json::from_value(json!({"locked": true})).unwrap();
        assert!(!v.applied);
        assert_eq!(serde_json::to_value(v).unwrap()["applied"], false);
    }

    #[test]
    fn an_exposure_put_may_omit_every_field() {
        let v: ExposureView = serde_json::from_value(json!({})).unwrap();
        assert_eq!(v, ExposureView::default());
        let v: ExposureView =
            serde_json::from_value(json!({"exposure_us": 4000, "gain": 2.0})).unwrap();
        assert!(!v.locked);
        assert_eq!(v.mode, None);
        assert_eq!(v.exposure_us, Some(4000));
    }

    #[test]
    fn storage_figures_serialize_as_integers() {
        let s = StorageView {
            free_bytes: Some(812_345_678_901),
            est_remaining_s: Some(14_400),
        };
        let text = serde_json::to_string(&s).unwrap();
        assert_eq!(
            text,
            r#"{"free_bytes":812345678901,"est_remaining_s":14400}"#
        );
        let unknown = serde_json::to_value(StorageView::default()).unwrap();
        assert!(unknown["est_remaining_s"].is_null());
    }

    #[test]
    fn refusal_body_carries_the_code() {
        let e = serde_json::to_value(CtlReply::err(codes::STORAGE_FULL, "full")).unwrap();
        assert_eq!(e["error"], "E_STORAGE_FULL");
        assert_eq!(e["ok"], false);
    }
}
