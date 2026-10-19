//! The device calibration store ([`world_engine_capture::calib`]):
//!
//! - `GET /atlas/calib` → `{"rig_id", "active", "versions":[{calib_version,
//!   created_utc, source, report}]}` for the current rig; `rig_id` and
//!   `active` null, `versions` empty when the store holds nothing.
//! - `GET /atlas/calib/{version}` → that version's contract rig document;
//!   `404 E_NO_CALIB` when absent.
//!
//! The current rig is plugin config `atlas.rig_id` when set, else the only
//! rig the store holds.

use axum::extract::{Path, State};
use axum::http::StatusCode;
use axum::response::Response;
use axum::routing::get;
use axum::Router;
use serde_json::{json, Value};
use world_engine_capture::calib::CalibStore;

use crate::api::DroneApi;
use crate::host::config_string;
use crate::reply::{codes, respond, ApiError};

/// The plugin config key naming the device's rig.
pub const RIG_ID_KEY: &str = "atlas.rig_id";

pub fn routes() -> Router<DroneApi> {
    Router::new()
        .route("/atlas/calib", get(summary))
        .route("/atlas/calib/:version", get(version))
}

fn no_calib(detail: impl Into<String>) -> ApiError {
    ApiError::new(StatusCode::NOT_FOUND, codes::NO_CALIB, detail)
}

impl DroneApi {
    /// The store and the current rig id (if any).
    async fn calib(&self) -> Result<(Option<CalibStore>, Option<String>), ApiError> {
        let Some(root) = self.calib_root.clone() else {
            return Ok((None, None));
        };
        let store = tokio::task::spawn_blocking(move || CalibStore::open(root))
            .await
            .map_err(ApiError::io)?
            .map_err(ApiError::io)?;
        let rig_id = match config_string(self.host.as_ref(), RIG_ID_KEY).await {
            Some(id) => Some(id),
            None => {
                let rigs = store.rigs();
                (rigs.len() == 1).then(|| rigs[0].clone())
            }
        };
        Ok((Some(store), rig_id))
    }

    /// `GET /atlas/calib`.
    pub async fn calib_summary(&self) -> Result<Value, ApiError> {
        let (Some(store), Some(rig_id)) = self.calib().await? else {
            return Ok(json!({ "rig_id": null, "active": null, "versions": [] }));
        };
        let active = store.active_version(&rig_id);
        let versions = store.list(&rig_id).map_err(ApiError::io)?;
        Ok(json!({ "rig_id": rig_id, "active": active, "versions": versions }))
    }

    /// `GET /atlas/calib/{version}`: the rig document.
    pub async fn calib_rig(&self, version: &str) -> Result<Value, ApiError> {
        let (Some(store), Some(rig_id)) = self.calib().await? else {
            return Err(no_calib("no rig in the calibration store"));
        };
        match store.get(&rig_id, version) {
            Ok(Some(record)) => serde_json::to_value(record.rig).map_err(ApiError::io),
            Ok(None) => Err(no_calib(format!("no calibration {rig_id}/{version}"))),
            Err(e) => Err(ApiError::bad_request(e.to_string())),
        }
    }
}

async fn summary(State(api): State<DroneApi>) -> Response {
    respond(api.calib_summary().await)
}

async fn version(State(api): State<DroneApi>, Path(v): Path<String>) -> Response {
    respond(api.calib_rig(&v).await)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::api::drone_router;
    use crate::api::tests::{call, test_api};
    use crate::testing::FakeHost;
    use world_engine_capture::calib::{import_kalibr, CalibRecord};

    const CAMCHAIN: &str = "cam0:\n  camera_model: pinhole\n  intrinsics: [400.0, 400.0, 320.0, 240.0]\n  distortion_model: radtan\n  distortion_coeffs: [0.1, 0.01, 0.0, 0.0]\n  resolution: [640, 480]\n";

    #[tokio::test]
    async fn the_store_is_listed_for_the_current_rig() {
        let dir = tempfile::tempdir().unwrap();
        let host = FakeHost::new();
        let app = drone_router(test_api(
            host.clone(),
            &dir.path().join("none.sock"),
            dir.path(),
        ));
        let (st, body) = call(app.clone(), "GET", "/atlas/calib", None).await;
        assert_eq!(
            (st, body),
            (
                StatusCode::OK,
                json!({"rig_id": null, "active": null, "versions": []})
            )
        );

        let store = CalibStore::open(dir.path().join("calib")).unwrap();
        for (v, t) in [
            ("v1", "2026-10-18T08:00:00Z"),
            ("v2", "2026-10-19T08:00:00Z"),
        ] {
            store
                .put(&CalibRecord {
                    calib_version: v.into(),
                    created_utc: t.into(),
                    source: "kalibr".into(),
                    report: json!({"reprojection_rms_px": 0.3}),
                    rig: import_kalibr("visor-01", v, CAMCHAIN, None).unwrap(),
                })
                .unwrap();
        }
        store.set_active("visor-01", "v2").unwrap();
        let (st, body) = call(app.clone(), "GET", "/atlas/calib", None).await;
        assert_eq!(st, StatusCode::OK);
        assert_eq!(body["rig_id"], "visor-01");
        assert_eq!(body["active"], "v2");
        assert_eq!(body["versions"][0]["calib_version"], "v1");
        assert_eq!(body["versions"][1]["report"]["reprojection_rms_px"], 0.3);
        assert!(body["versions"][0].get("rig").is_none());

        let (st, rig) = call(app.clone(), "GET", "/atlas/calib/v1", None).await;
        assert_eq!(st, StatusCode::OK);
        assert_eq!(rig["calib_version"], "v1");
        assert_eq!(rig["sensors"][0]["camera"]["model"], "opencv");
        let (st, body) = call(app.clone(), "GET", "/atlas/calib/v9", None).await;
        assert_eq!(st, StatusCode::NOT_FOUND);
        assert_eq!(body["error"], "E_NO_CALIB");

        // A configured rig id that has no versions.
        host.set_config(RIG_ID_KEY, json!("other-rig"));
        let (_, body) = call(app, "GET", "/atlas/calib", None).await;
        assert_eq!(
            body,
            json!({"rig_id": "other-rig", "active": null, "versions": []})
        );
    }
}
