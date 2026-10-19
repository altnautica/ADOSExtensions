use axum::body::Body;
use axum::http::Request;
use tower::ServiceExt;

use super::*;
use crate::api::drone_router;
use crate::api::tests::{call, test_api};
use crate::host::WhepAnswer;
use crate::testing::FakeHost;

const LOCATION: &str = "/whep/3fa2c1?camera=cam_fl";

async fn raw(
    app: Router,
    method: &str,
    uri: &str,
    body: &str,
) -> (StatusCode, Option<String>, String) {
    let req = Request::builder()
        .method(method)
        .uri(uri)
        .header("content-type", "application/sdp")
        .body(Body::from(body.to_string()))
        .unwrap();
    let resp = app.oneshot(req).await.unwrap();
    let status = resp.status();
    let location = resp
        .headers()
        .get(header::LOCATION)
        .map(|v| v.to_str().unwrap().to_string());
    let bytes = axum::body::to_bytes(resp.into_body(), usize::MAX)
        .await
        .unwrap();
    (status, location, String::from_utf8(bytes.to_vec()).unwrap())
}

#[tokio::test]
async fn a_whep_offer_and_close_go_through_the_host() {
    let dir = tempfile::tempdir().unwrap();
    let host = FakeHost::new();
    *host.whep_reply.lock() = Ok(WhepAnswer {
        sdp: "v=0 answer".into(),
        location: Some(LOCATION.into()),
    });
    let app = drone_router(test_api(
        host.clone(),
        &dir.path().join("none.sock"),
        dir.path(),
    ));
    let (st, loc, body) = raw(
        app.clone(),
        "POST",
        "/atlas/capture/whep?camera=cam_fl",
        "v=0 offer",
    )
    .await;
    assert_eq!(st, StatusCode::CREATED);
    assert_eq!(body, "v=0 answer");
    let loc = loc.unwrap();
    assert_eq!(loc, "whep?location=%2Fwhep%2F3fa2c1%3Fcamera%3Dcam_fl");
    assert_eq!(
        host.whep_offers.lock()[0],
        ("cam_fl".to_string(), "v=0 offer".to_string())
    );

    // The GCS deletes `atlas/capture/<location>`.
    let (st, body) = call(
        app.clone(),
        "DELETE",
        &format!("/atlas/capture/{loc}"),
        None,
    )
    .await;
    assert_eq!((st, body), (StatusCode::OK, json!({"ok": true})));
    assert_eq!(host.whep_closed.lock()[0], LOCATION);

    let (st, _, _) = raw(
        app.clone(),
        "POST",
        "/atlas/capture/whep?camera=../x",
        "v=0",
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST);
    let (st, _) = call(app, "DELETE", "/atlas/capture/whep", None).await;
    assert_eq!(st, StatusCode::BAD_REQUEST);
}

#[tokio::test]
async fn host_refusals_map_to_403_and_503() {
    let dir = tempfile::tempdir().unwrap();
    let host = FakeHost::new();
    let app = drone_router(test_api(
        host.clone(),
        &dir.path().join("none.sock"),
        dir.path(),
    ));
    let (st, _, body) = raw(
        app.clone(),
        "POST",
        "/atlas/capture/whep?camera=cam_fl",
        "v=0",
    )
    .await;
    assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE);
    assert!(body.contains("E_WHEP_UNREACHABLE"));
    *host.whep_reply.lock() = Err("capability_denied: video.whep.relay".into());
    let (st, _, body) = raw(app, "POST", "/atlas/capture/whep?camera=cam_fl", "v=0").await;
    assert_eq!(st, StatusCode::FORBIDDEN);
    assert!(body.contains("E_CAPABILITY_DENIED"));
    assert_eq!(
        host_error(ClientError::Rpc("boom".into()), codes::RIG_UNREACHABLE).status,
        StatusCode::BAD_GATEWAY
    );
}

#[tokio::test]
async fn the_camera_id_goes_to_the_host_as_is_and_an_unknown_camera_is_404() {
    let dir = tempfile::tempdir().unwrap();
    let host = FakeHost::new();
    *host.whep_reply.lock() = Ok(WhepAnswer {
        sdp: "v=0 answer".into(),
        location: Some("/whep/s1?camera=main".into()),
    });
    let app = drone_router(test_api(
        host.clone(),
        &dir.path().join("none.sock"),
        dir.path(),
    ));
    // The primary capture camera is asked for by its own id (the host maps it
    // to the `main` leg); the host's id rule allows upper case and `-`.
    let (st, _, _) = raw(
        app.clone(),
        "POST",
        "/atlas/capture/whep?camera=Front-1",
        "v=0",
    )
    .await;
    assert_eq!(st, StatusCode::CREATED);
    assert_eq!(host.whep_offers.lock()[0].0, "Front-1");

    *host.whep_reply.lock() =
        Err("unknown_camera: \"rear\" is not a configured, enabled camera".into());
    let (st, _, body) = raw(app, "POST", "/atlas/capture/whep?camera=rear", "v=0").await;
    assert_eq!(st, StatusCode::NOT_FOUND);
    assert!(body.contains("E_UNKNOWN_CAMERA"), "{body}");
}

#[test]
fn camera_ids_follow_the_host_path_segment_rule() {
    for ok in ["main", "cam_fl", "Front-1", "a".repeat(100).as_str()] {
        assert!(camera_id_ok(ok), "{ok}");
    }
    for bad in ["", "../x", "a/b", "a b", "a.b", "a?b"] {
        assert!(!camera_id_ok(bad), "{bad}");
    }
}

#[tokio::test]
async fn a_rig_is_checked_then_relayed_with_the_core_reply_verbatim() {
    let dir = tempfile::tempdir().unwrap();
    let host = FakeHost::new();
    let app = drone_router(test_api(
        host.clone(),
        &dir.path().join("none.sock"),
        dir.path(),
    ));
    let (st, body) = call(
        app.clone(),
        "PUT",
        "/atlas/rig",
        Some(json!({"rig_id": "x"})),
    )
    .await;
    assert_eq!(st, StatusCode::BAD_REQUEST);
    assert_eq!(body["error"], "E_RIG_INVALID");
    assert!(host.rigs.lock().is_empty());

    let rig = json!({"contract": "ados.capture/1", "rig_id": "visor-01", "calib_version": "v1", "sensors": [
        {"id": "imu0", "kind": "imu", "role": "nav", "T_body_sensor": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1], "time_offset_ns": 0}
    ]});
    let (st, body) = call(app.clone(), "PUT", "/atlas/rig", Some(rig.clone())).await;
    assert_eq!(st, StatusCode::SERVICE_UNAVAILABLE);
    assert_eq!(body["error"], "E_RIG_UNREACHABLE");

    let ok = json!({"ok": true, "rig_id": "visor-01", "calib_version": "v1"});
    *host.rig_reply.lock() = Ok((200, ok.clone()));
    let (st, body) = call(app.clone(), "PUT", "/atlas/rig", Some(rig.clone())).await;
    assert_eq!((st, body), (StatusCode::OK, ok));
    assert_eq!(host.rigs.lock().last().unwrap(), &rig);

    let refused = json!({"error": "E_RIG_INVALID", "detail": "sensor imu0: role"});
    *host.rig_reply.lock() = Ok((400, refused.clone()));
    let (st, body) = call(app.clone(), "PUT", "/atlas/rig", Some(rig.clone())).await;
    assert_eq!((st, body), (StatusCode::BAD_REQUEST, refused));

    *host.rig_reply.lock() = Err("capability_denied: sensors.rig.write".into());
    let (st, body) = call(app, "PUT", "/atlas/rig", Some(rig)).await;
    assert_eq!(st, StatusCode::FORBIDDEN);
    assert_eq!(body["error"], "E_CAPABILITY_DENIED");
}
