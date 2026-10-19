//! The exposure lock through the real capture loop: a camera slower than the
//! reply budget answers the PUT `pending`, and a later state read carries
//! the applied read-back values.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use ados_protocol::shutdown::Shutdown;
use async_trait::async_trait;
use parking_lot::Mutex;
use tokio::sync::{mpsc, oneshot};
use world_engine_capture::camera_control::{CameraControl, CameraOutcome, REPLY_BUDGET};
use world_engine_capture::pose::replay::ReplayPose;
use world_engine_capture::sensor_bus::SensorState;
use world_engine_capture::vehicle::VehicleFeed;
use world_engine_capture::{
    run_capture_loop, AtlasFrameSource, AtlasPublisher, AtlasRuntimeConfig, ControlMsg,
    EngineFrameSource, LoopDeps, Paths,
};
use world_engine_protocol::capture_ctl::{CtlReply, CtlRequest, ExposureView};

/// Takes the values after `delay`, reading back a slightly different exposure.
struct SlowCamera {
    delay: Duration,
}

#[async_trait]
impl CameraControl for SlowCamera {
    async fn set_exposure(&self, _: &str, us: Option<u32>, gain: Option<f32>) -> CameraOutcome {
        tokio::time::sleep(self.delay).await;
        CameraOutcome {
            applied: true,
            exposure_us: us.map(|u| u - 10),
            gain,
            reason: None,
        }
    }
}

async fn ask(tx: &mpsc::Sender<ControlMsg>, req: CtlRequest) -> CtlReply {
    let (reply, rx) = oneshot::channel();
    tx.send(ControlMsg { req, reply }).await.unwrap();
    rx.await.unwrap()
}

fn exposure_of(state: &CtlReply) -> ExposureView {
    serde_json::from_value(state.result["exposure"].clone()).unwrap()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_slow_camera_answers_pending_and_the_state_catches_up() {
    let dir = tempfile::tempdir().unwrap();
    let values: BTreeMap<String, serde_json::Value> = [
        ("atlas.enabled".to_string(), serde_json::json!(true)),
        (
            "atlas.cameras".to_string(),
            serde_json::json!([{"id": "cam_fl"}, {"id": "cam_fr", "role": "aux"}]),
        ),
    ]
    .into_iter()
    .collect();
    let cfg = AtlasRuntimeConfig::from_values("sim-visor", &values).unwrap();
    let (_bus_tx, bus_rx) = mpsc::channel(8);
    let (ctl_tx, ctl_rx) = mpsc::channel(8);
    let (_ftx, frx) = mpsc::channel(8);
    let publisher = AtlasPublisher::bind(&dir.path().join("atlas.sock").to_string_lossy())
        .await
        .unwrap();
    let cancel = Shutdown::new();
    let deps = LoopDeps {
        cfg,
        profile: "visor".into(),
        frames: AtlasFrameSource::Engine(EngineFrameSource::from_receiver(frx)),
        router: None,
        pose: Arc::new(ReplayPose::new(Vec::new())),
        fc: None,
        vehicle: VehicleFeed::new(),
        sensors: Arc::new(Mutex::new(SensorState::default())),
        bus_rx,
        lidar_rx: None,
        publisher,
        control_rx: ctl_rx,
        hud_rx: None,
        view_tx: None,
        camera: Some(Arc::new(SlowCamera {
            delay: REPLY_BUDGET + Duration::from_millis(300),
        })),
        paths: Paths {
            storage_sidecar: dir.path().join("storage.json"),
            clock_sidecar: dir.path().join("clock.json"),
            rig: dir.path().join("rig.json"),
            active_marker: None,
        },
    };
    let lp = tokio::spawn(run_capture_loop(deps, cancel.clone()));

    let lock = ExposureView {
        locked: true,
        exposure_us: Some(2_000),
        gain: Some(2.0),
        ..Default::default()
    };
    let put = ask(&ctl_tx, CtlRequest::Exposure(lock)).await;
    assert!(put.ok, "{put:?}");
    let pending: ExposureView = serde_json::from_value(put.result).unwrap();
    assert!(pending.locked && !pending.applied, "{pending:?}");
    assert_eq!(pending.reason.as_deref(), Some("pending"));
    // The loop keeps serving while the cameras are still being set.
    let now = exposure_of(&ask(&ctl_tx, CtlRequest::State).await);
    assert_eq!(now.reason.as_deref(), Some("pending"));

    tokio::time::sleep(Duration::from_millis(800)).await;
    let later = exposure_of(&ask(&ctl_tx, CtlRequest::State).await);
    assert!(later.applied && later.locked, "{later:?}");
    assert_eq!(later.exposure_us, Some(1_990));
    assert_eq!(later.gain, Some(2.0));
    assert!(later.reason.is_none());

    cancel.trigger();
    lp.await.unwrap();
}
