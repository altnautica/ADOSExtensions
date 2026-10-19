//! A hub-timebase session run through the capture loop: the hub clock is
//! hours away from the host clock and the clock service interleaves host
//! fallback samples and a source switch. Every record must land on the
//! session's own mapping: one segment, monotonic IMU stamps, and a duration
//! equal to the span actually recorded.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use ados_capture_contract::v1::{ClockSample, ImuBatch, ImuSample};
use ados_capture_contract::{topics, SessionReader, Timebase};
use ados_config::capture_storage::CaptureState;
use ados_protocol::shutdown::Shutdown;
use parking_lot::Mutex;
use tokio::sync::{mpsc, oneshot};
use world_engine_protocol::capture_ctl::{CtlReply, CtlRequest, StartArgs};

use super::{run_capture_loop, LoopDeps, Paths};
use crate::clock::mono_ns;
use crate::control::ControlMsg;
use crate::frame_source::{AtlasFrameSource, SyntheticFrameSource};
use crate::pose::fc_ekf::FcEkfPose;
use crate::publish::AtlasPublisher;
use crate::runtime::AtlasRuntimeConfig;
use crate::sensor_bus::{BusEvent, SensorState};
use crate::vehicle::VehicleFeed;

/// The hub free-run clock reads three hours ahead of the host clock.
const HUB_OFFSET_NS: u64 = 3 * 3_600 * 1_000_000_000;

fn clock(t_ns: u64, host_ns: u64) -> ClockSample {
    ClockSample {
        t_ns,
        host_mono_ns: host_ns,
        utc_ns: None,
        locked: true,
        sigma_ns: 44_000,
    }
}

async fn request(tx: &mpsc::Sender<ControlMsg>, req: CtlRequest) -> CtlReply {
    let (reply, rx) = oneshot::channel();
    tx.send(ControlMsg { req, reply }).await.unwrap();
    rx.await.unwrap()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_hub_session_records_on_its_own_timebase() {
    let dir = tempfile::tempdir().unwrap();
    let root = dir.path().join("capture");
    std::fs::create_dir_all(&root).unwrap();
    let paths = Paths {
        storage_sidecar: dir.path().join("capture-storage.json"),
        clock_sidecar: dir.path().join("clock.json"),
        rig: dir.path().join("rig.json"),
        active_marker: None,
    };
    crate::recording::tests::write_sidecar(&paths.storage_sidecar, &root, CaptureState::Ok);
    std::fs::write(
        &paths.clock_sidecar,
        r#"{"timebase":"hub_freerun","locked":true,"sigma_ns":44000}"#,
    )
    .unwrap();
    std::fs::write(
        &paths.rig,
        serde_json::to_vec(&crate::recorder::tests::rig()).unwrap(),
    )
    .unwrap();

    let sensors = Arc::new(Mutex::new(SensorState::default()));
    let (bus_tx, bus_rx) = mpsc::channel(1024);
    let (ctl_tx, ctl_rx) = mpsc::channel(4);
    let cancel = Shutdown::new();
    let deps = LoopDeps {
        cfg: AtlasRuntimeConfig::from_values("bench", &BTreeMap::new()).unwrap(),
        profile: "visor".into(),
        frames: AtlasFrameSource::Synthetic(SyntheticFrameSource::new(Vec::new())),
        router: None,
        pose: Arc::new(FcEkfPose::new()),
        fc: None,
        vehicle: VehicleFeed::new(),
        sensors: sensors.clone(),
        bus_rx,
        lidar_rx: None,
        publisher: AtlasPublisher::bind(&dir.path().join("atlas.sock").to_string_lossy())
            .await
            .unwrap(),
        control_rx: ctl_rx,
        hud_rx: None,
        view_tx: None,
        camera: None,
        paths,
    };
    let task = tokio::spawn(run_capture_loop(deps, cancel.clone()));
    // What the ingest task does: fold into the shared state, then forward.
    let publish = |ev: BusEvent| {
        sensors.lock().apply(&ev);
        bus_tx.try_send(ev).unwrap();
    };

    // Only the host fallback so far: not on the hub timebase yet.
    let now = mono_ns();
    publish(BusEvent::Clock(clock(now, now)));
    tokio::time::sleep(Duration::from_millis(50)).await;
    let refused = request(&ctl_tx, CtlRequest::Start(StartArgs::default())).await;
    assert_eq!(refused.error.as_deref(), Some("E_CLOCK_NOT_READY"));

    let now = mono_ns();
    publish(BusEvent::Clock(clock(now + HUB_OFFSET_NS, now)));
    tokio::time::sleep(Duration::from_millis(50)).await;
    let started = request(&ctl_tx, CtlRequest::Start(StartArgs::default())).await;
    assert!(started.ok, "{started:?}");
    let id = started.result["session_id"].as_str().unwrap().to_string();
    let start_host = mono_ns();

    let mut seq = 0;
    for k in 0..125u64 {
        let now = mono_ns();
        publish(BusEvent::Imu {
            src: "imu0".into(),
            batch: ImuBatch {
                samples: (0..4u64)
                    .map(|i| ImuSample {
                        t_ns: now.saturating_sub((3 - i) * 5_000_000),
                        seq: {
                            seq += 1;
                            seq
                        },
                        gyro: vec![0.0; 3],
                        accel: vec![0.0, 0.0, 9.81],
                        temp_c: None,
                    })
                    .collect(),
            },
        });
        if k % 10 == 0 {
            // Hub samples, the host fallback, and one switch to another source.
            let t = match k / 10 {
                3 => 1_700_000_000_000_000_000,
                n if n % 2 == 1 => now,
                _ => now + HUB_OFFSET_NS,
            };
            publish(BusEvent::Clock(clock(t, now)));
        }
        tokio::time::sleep(Duration::from_millis(20)).await;
    }
    let stop_host = mono_ns();
    assert!(request(&ctl_tx, CtlRequest::Stop).await.ok);

    let sdir = root.join(&id);
    for _ in 0..200 {
        if sdir.join("manifest.json").exists() {
            break;
        }
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    cancel.trigger();
    task.await.unwrap();

    let r = SessionReader::open(&sdir).unwrap();
    r.verify().unwrap();
    let m = r.manifest().clone();
    assert_eq!(m.timebase, Timebase::HubFreerun);
    assert_eq!(m.segments.len(), 1, "{:?}", m.segments);
    let span = (stop_host - start_host) as f64 / 1e9;
    let t_end = m.segments.iter().map(|s| s.t_end_ns).max().unwrap();
    let duration = (t_end - m.t0_ns) as f64 / 1e9;
    assert!(
        (duration - span).abs() < 0.5,
        "duration {duration} s, span {span} s"
    );
    assert!(m.t0_ns > HUB_OFFSET_NS && m.t0_ns - HUB_OFFSET_NS <= start_host);

    let (lo, hi) = (m.t0_ns - 100_000_000, m.t0_ns + ((span + 0.5) * 1e9) as u64);
    let (mut imu_prev, mut imu, mut clocks) = (0, 0, 0);
    r.for_each_message(|msg| {
        assert!(
            (lo..=hi).contains(&msg.t_ns),
            "{} at {} outside {lo}..{hi}",
            msg.topic,
            msg.t_ns
        );
        if msg.topic == topics::CLOCK {
            clocks += 1;
        }
        if msg.topic.starts_with("/imu/") {
            assert!(msg.t_ns >= imu_prev, "{} < {imu_prev}", msg.t_ns);
            imu_prev = msg.t_ns;
            let b: ImuBatch = msg.decode().unwrap();
            for s in &b.samples {
                assert!((lo..=hi).contains(&s.t_ns), "imu sample {}", s.t_ns);
            }
            imu += 1;
        }
        Ok(())
    })
    .unwrap();
    assert!(imu >= 100, "{imu} imu batches");
    assert!(clocks >= 10, "{clocks} clock samples");
}
