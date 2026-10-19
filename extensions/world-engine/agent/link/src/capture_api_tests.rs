use axum::http::StatusCode;

use super::*;
use crate::api::drone_router;
use crate::api::tests::{call, test_api};
use crate::control::tests::fake_control_socket;
use crate::testing::FakeHost;
use world_engine_protocol::capture_ctl::{codes as ctl_codes, CtlReply, ExposureMode};

fn live_view() -> CaptureStateView {
    let mut v = idle_view(StorageView {
        free_bytes: Some(812_345_678_901),
        est_remaining_s: Some(14_400.0),
    });
    v.state = "recording".into();
    v.session_id = Some("visor-1-20261019T101500Z-3fa2".into());
    v.segment = Some(3);
    v.pose.source = "vio".into();
    v.pose.health = "ok".into();
    v
}

/// A capture service that records, refuses a second start and answers the
/// rest.
fn service(req: CtlRequest) -> CtlReply {
    match req {
        CtlRequest::Start(args) if args.site.as_deref() == Some("busy") => {
            CtlReply::err(ctl_codes::ALREADY_RECORDING, "a session is recording")
        }
        CtlRequest::Start(args) if args.site.as_deref() == Some("early") => {
            CtlReply::err(ctl_codes::CLOCK_NOT_READY, "no clock sample yet")
        }
        CtlRequest::Start(_) => {
            CtlReply::ok(json!({"session_id": "visor-1-20261019T101500Z-3fa2"}))
        }
        CtlRequest::State | CtlRequest::Stop | CtlRequest::Pause | CtlRequest::Resume => {
            CtlReply::ok(live_view())
        }
        CtlRequest::Mark(m) if m.room.is_empty() => CtlReply::err(ctl_codes::BAD_REQUEST, "room"),
        CtlRequest::Mark(_) => CtlReply::ok(json!({"segment": 3})),
        // Not applied to the sensor: the lock request echoed with the reason.
        CtlRequest::Exposure(e) => CtlReply::ok(ExposureView {
            applied: false,
            reason: Some("not_v4l2".into()),
            ..e
        }),
        CtlRequest::Coverage => CtlReply::ok(CoverageView {
            cell_m: 0.5,
            cells: vec![[0, 1, 4]],
            trajectory: vec![[0.0, 0.0, 0.0], [1.0, 0.5, 0.0]],
            start: Some([0.0, 0.0, 0.0]),
            current: Some([1.0, 0.5, 0.0]),
            heading_deg: Some(90.0),
        }),
        CtlRequest::Status => CtlReply::err(ctl_codes::RECORDER, "unused"),
    }
}

#[tokio::test]
async fn capture_actions_reach_the_control_socket_with_their_args() {
    let dir = tempfile::tempdir().unwrap();
    let sock = dir.path().join("atlas-control.sock");
    let recorded = fake_control_socket(&sock, service);
    let app = drone_router(test_api(FakeHost::new(), &sock, dir.path()));

    let start = json!({"site": "Warehouse 3", "floor": 0, "room": "Dock", "operator_anchor": null});
    let (st, body) = call(app.clone(), "POST", "/atlas/capture/start", Some(start)).await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(
        body,
        json!({"ok": true, "session_id": "visor-1-20261019T101500Z-3fa2"})
    );
    let sent: Value = serde_json::from_str(&recorded.lock()[0]).unwrap();
    assert_eq!(sent["cmd"], "start");
    assert_eq!(sent["site"], "Warehouse 3");

    // No body is a start with no metadata.
    let (st, _) = call(app.clone(), "POST", "/atlas/capture/start", None).await;
    assert_eq!(st, StatusCode::OK);

    let (st, body) = call(
        app.clone(),
        "POST",
        "/atlas/capture/start",
        Some(json!({"site": "busy"})),
    )
    .await;
    assert_eq!(st, StatusCode::CONFLICT);
    assert_eq!(body["error"], "E_ALREADY_RECORDING");

    let (st, body) = call(
        app.clone(),
        "POST",
        "/atlas/capture/mark",
        Some(json!({"room": "Kitchen", "floor": 0})),
    )
    .await;
    assert_eq!(
        (st, body),
        (StatusCode::OK, json!({"ok": true, "segment": 3}))
    );
    let (st, body) = call(
        app.clone(),
        "POST",
        "/atlas/capture/mark",
        Some(json!({"floor": 0})),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST);
    assert_eq!(body["error"], "E_BAD_REQUEST");

    let (st, body) = call(app.clone(), "POST", "/atlas/capture/stop", None).await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(body["ok"], true);
    assert_eq!(body["state"], "recording");
    for verb in ["pause", "resume"] {
        let (st, _) = call(app.clone(), "POST", &format!("/atlas/capture/{verb}"), None).await;
        assert_eq!(st, StatusCode::OK);
    }

    let exp = json!({"locked": true, "exposure_us": 4000, "gain": 2.0, "mode": "walking"});
    let (st, body) = call(
        app.clone(),
        "PUT",
        "/atlas/capture/exposure",
        Some(exp.clone()),
    )
    .await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(
        body,
        json!({"locked": true, "applied": false, "reason": "not_v4l2", "exposure_us": 4000, "gain": 2.0, "mode": "walking"})
    );
    let sent: ExposureView = serde_json::from_value(
        serde_json::from_str::<Value>(recorded.lock().last().unwrap()).unwrap(),
    )
    .unwrap();
    assert_eq!(sent.mode, Some(ExposureMode::Walking));

    let (st, body) = call(
        app,
        "POST",
        "/atlas/capture/start",
        Some(json!({"site": "early"})),
    )
    .await;
    assert_eq!(st, StatusCode::CONFLICT);
    assert_eq!(body["error"], "E_CLOCK_NOT_READY");
}

#[tokio::test]
async fn state_and_coverage_read_the_service() {
    let dir = tempfile::tempdir().unwrap();
    let sock = dir.path().join("atlas-control.sock");
    fake_control_socket(&sock, service);
    let app = drone_router(test_api(FakeHost::new(), &sock, dir.path()));
    let (st, body) = call(app.clone(), "GET", "/atlas/capture/state", None).await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(body["state"], "recording");
    assert_eq!(body["segment"], 3);
    assert_eq!(body["pose"]["source"], "vio");
    assert_eq!(body["storage"]["free_bytes"], 812_345_678_901u64);
    let (st, body) = call(app, "GET", "/atlas/capture/coverage", None).await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(body["cell_m"], 0.5);
    assert_eq!(body["cells"], json!([[0, 1, 4]]));
    assert_eq!(body["heading_deg"], 90.0);
}

#[tokio::test]
async fn without_the_service_state_is_an_idle_view_and_actions_are_unavailable() {
    let dir = tempfile::tempdir().unwrap();
    let app = drone_router(test_api(
        FakeHost::new(),
        &dir.path().join("none.sock"),
        dir.path(),
    ));
    let (st, body) = call(app.clone(), "GET", "/atlas/capture/state", None).await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(body["state"], "idle");
    assert!(body["session_id"].is_null());
    assert_eq!(
        body["frames"],
        json!({"total": 0, "dropped_pct": 0.0, "per_camera": []})
    );
    assert_eq!(body["pose"]["source"], "none");
    assert_eq!(body["health"], json!([]));
    assert_eq!(body["exposure"]["locked"], false);
    assert!(body["storage"].is_object());

    let (st, body) = call(app.clone(), "GET", "/atlas/capture/coverage", None).await;
    assert_eq!(st, StatusCode::OK);
    assert_eq!(body["cell_m"], 0.0);

    let (st, body) = call(app, "POST", "/atlas/capture/start", None).await;
    assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE);
    assert_eq!(body["error"], "E_CAPTURE_UNAVAILABLE");
}
