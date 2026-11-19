use super::*;
use axum::body::Body;
use axum::http::Request;
use parking_lot::Mutex;
use tower::ServiceExt;

#[derive(Default)]
struct Memory(Mutex<BTreeMap<String, Value>>);

#[async_trait]
impl ConfigStore for Memory {
    async fn get(&self, key: &str) -> Result<Value, String> {
        Ok(self.0.lock().get(key).cloned().unwrap_or(Value::Null))
    }
    async fn set(&self, key: &str, value: Value) -> Result<(), String> {
        self.0.lock().insert(key.to_string(), value);
        Ok(())
    }
}

fn app() -> (Router, Arc<Memory>, Arc<Shared>) {
    let store = Arc::new(Memory::default());
    let shared = Arc::new(Shared::default());
    (router(shared.clone(), store.clone()), store, shared)
}

async fn put(app: Router, body: Value) -> (StatusCode, Value) {
    let req = Request::put("/navigation/config")
        .header("content-type", "application/json")
        .body(Body::from(body.to_string()))
        .unwrap();
    let resp = app.oneshot(req).await.unwrap();
    let code = resp.status();
    let bytes = axum::body::to_bytes(resp.into_body(), 1 << 20)
        .await
        .unwrap();
    (code, serde_json::from_slice(&bytes).unwrap())
}

#[tokio::test]
async fn a_patch_is_resolved_whole_before_anything_is_written() {
    let (app, store, _) = app();
    let (code, body) = put(
        app.clone(),
        json!({"navigation.pose_provider": "lidar", "capture.enabled": true}),
    )
    .await;
    assert_eq!(code, StatusCode::BAD_REQUEST);
    assert!(
        body["detail"].as_str().unwrap().contains("capture.enabled"),
        "{body}"
    );
    assert!(store.0.lock().is_empty(), "a refused patch writes nothing");

    let (code, body) = put(
        app.clone(),
        json!({"navigation.pose_provider": "lidar", "navigation.map_placement": "everywhere"}),
    )
    .await;
    assert_eq!(code, StatusCode::BAD_REQUEST);
    assert!(
        body["detail"]
            .as_str()
            .unwrap()
            .contains("navigation.map_placement"),
        "{body}"
    );
    assert!(store.0.lock().is_empty());

    let (code, _) = put(
        app,
        json!({"navigation.pose_provider": "off", "navigation.takeoff_heading_deg": 90}),
    )
    .await;
    assert_eq!(code, StatusCode::OK);
    let stored = store.0.lock().clone();
    assert_eq!(stored["navigation.pose_provider"], "off");
    assert_eq!(stored["navigation.takeoff_heading_deg"], 90);
}

#[tokio::test]
async fn the_sweep_route_serves_the_whole_cloud_from_the_running_lane() {
    let (app, _, shared) = app();
    let get = || {
        Request::get("/navigation/sweep")
            .body(Body::empty())
            .unwrap()
    };
    let resp = app.clone().oneshot(get()).await.unwrap();
    assert_eq!(resp.status(), StatusCode::SERVICE_UNAVAILABLE, "no run");

    let (tx, mut rx) = tokio::sync::mpsc::channel::<crate::lane::WholeSweep>(1);
    *shared.whole.lock() = Some(tx);
    tokio::spawn(async move {
        if let Some(reply) = rx.recv().await {
            let pts: Vec<u8> = [1.0f32, 2.0, 3.0]
                .iter()
                .flat_map(|f| f.to_le_bytes())
                .collect();
            let _ = reply.send((pts, 0.05, 9));
        }
    });
    let resp = app.oneshot(get()).await.unwrap();
    assert_eq!(resp.status(), StatusCode::OK);
    let b = axum::body::to_bytes(resp.into_body(), 1 << 20)
        .await
        .unwrap();
    assert_eq!(b.len(), crate::sweep::HEADER_LEN + 12);
    assert_eq!(u64::from_le_bytes(b[0..8].try_into().unwrap()), 9);
    assert_eq!(
        u32::from_le_bytes(b[16..20].try_into().unwrap()),
        1,
        "reset"
    );
}
