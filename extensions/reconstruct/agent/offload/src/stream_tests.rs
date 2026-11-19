use std::time::Duration;

use crate::PIN_KEY;
use ados_mapping_protocol::atlas::{PolarScan, VioHealth};
use ados_mapping_transport::atlas_event_router;
use ados_mapping_transport::testing::FakeHost;
use serde_json::json;

use super::*;

fn scan(t_ns: u64) -> Value {
    let s = PolarScan {
        t_ns,
        angle_start_rad: 0.0,
        angle_inc_rad: 0.01,
        t: [1.0, 2.0, 0.0],
        q: [1.0, 0.0, 0.0, 0.0],
        health: VioHealth::Good,
        range_mm: vec![1000, 0, 2000],
        quality: vec![40, 0, 50],
    };
    // As the host delivers it: the named msgpack map Navigation published.
    rmpv::ext::to_value(serde_json::to_value(&s).unwrap()).unwrap()
}

fn status(placement: &str) -> Value {
    rmpv::ext::to_value(json!({"placement": placement, "sweep_points": 0})).unwrap()
}

async fn wait_for(view: &SharedStream, f: impl Fn(&StreamView) -> bool) {
    for _ in 0..200 {
        if f(&view.lock()) {
            return;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    panic!("timed out: {:?}", view.lock());
}

#[tokio::test]
async fn scans_reach_the_node_only_while_navigation_streams_the_map() {
    let (tx, mut rx) = tokio::sync::mpsc::channel(16);
    let listener = tokio::net::TcpListener::bind("127.0.0.1:0").await.unwrap();
    let addr = listener.local_addr().unwrap();
    tokio::spawn(async move { axum::serve(listener, atlas_event_router(tx)).await.unwrap() });

    let dir = tempfile::tempdir().unwrap();
    let host = FakeHost::new();
    host.set_config(PIN_KEY, json!(addr.to_string()));
    let view = SharedStream::default();
    let (stop, shutdown) = watch::channel(false);
    let task = tokio::spawn(
        Streamer {
            host: host.clone(),
            credentials: CredentialStore::new(Some(dir.path()), PIN_KEY),
            device_id: "drone-7".into(),
            view: view.clone(),
        }
        .run(shutdown),
    );
    for _ in 0..200 {
        if host.event_callbacks.lock().len() == 2 {
            break;
        }
        tokio::time::sleep(Duration::from_millis(10)).await;
    }

    host.deliver_event(SCAN_POLAR_TOPIC, scan(1));
    host.deliver_event(MAP_STATUS_TOPIC, status("hybrid"));
    host.deliver_event(SCAN_POLAR_TOPIC, scan(2));
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert_eq!(
        view.lock().forwarded,
        0,
        "no stream placement: nothing forwarded"
    );

    host.deliver_event(MAP_STATUS_TOPIC, status("stream"));
    host.deliver_event(SCAN_POLAR_TOPIC, scan(3));
    wait_for(&view, |v| v.forwarded == 1).await;
    let event = rx.recv().await.unwrap();
    assert_eq!(event.topic, LIVE_SCAN_TOPIC);
    assert_eq!(event.device_id.as_deref(), Some("drone-7"));
    let (sent, _) = PolarScan::decode(&event.payload).unwrap();
    assert_eq!((sent.t_ns, sent.range_mm.clone()), (3, vec![1000, 0, 2000]));
    let v = view.lock().clone();
    assert!(v.streaming);
    assert_eq!(v.node.as_deref(), Some(addr.to_string().as_str()));
    assert_eq!((v.dropped, v.failed), (0, 0));

    host.deliver_event(MAP_STATUS_TOPIC, status("post_flight"));
    host.deliver_event(SCAN_POLAR_TOPIC, scan(4));
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert_eq!(view.lock().forwarded, 1);
    assert!(!view.lock().streaming);

    stop.send(true).unwrap();
    task.await.unwrap();
}
