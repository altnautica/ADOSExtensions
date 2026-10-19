//! SITL capture: the real capture loop records a session from synthetic
//! framebus frames (two triggered cameras on simulation time) and sensor-bus
//! IMU, trigger and clock frames, and the session validates with the contract
//! reader: both cameras, every 1 kHz IMU sample, triggers joined, timebase
//! `sim`.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use ados_capture_contract::foxglove::RawImage;
use ados_capture_contract::v1::{ClockSample, ImuBatch, ImuSample, TriggerRecord};
use ados_capture_contract::{SessionReader, Timebase};
use ados_config::capture_storage::{CaptureState, CaptureStorageSidecar, CaptureUsage};
use ados_protocol::framebus::{FrameFormat, TIMEBASE_SIM};
use ados_protocol::shutdown::Shutdown;
use parking_lot::Mutex;
use tokio::sync::{mpsc, oneshot};
use world_engine_capture::pose::replay::ReplayPose;
use world_engine_capture::pose::{diagonal_cov, se3, PoseEstimate};
use world_engine_capture::sensor_bus::{BusEvent, SensorState};
use world_engine_capture::vehicle::VehicleFeed;
use world_engine_capture::{
    run_capture_loop, AtlasFrameSource, AtlasPublisher, AtlasRuntimeConfig, CapturedFrame,
    ControlMsg, EngineFrameSource, LoopDeps, Paths, PoseSource, VioHealth,
};
use world_engine_protocol::capture_ctl::{CtlReply, CtlRequest, StartArgs};

/// Host monotonic ns at simulation time 0.
const HOST0: u64 = 50_000_000_000;
/// Simulation time of the first frame.
const SIM0: u64 = 1_000_000_000;
const FRAMES: u64 = 20;
const FRAME_NS: u64 = 50_000_000;

fn rig_json() -> serde_json::Value {
    let cam = |id: &str, role: &str, y: f64| {
        serde_json::json!({"id": id, "kind": "camera", "role": role,
            "T_body_sensor": [0.0,0.0,1.0,0.1, -1.0,0.0,0.0,y, 0.0,-1.0,0.0,0.0, 0.0,0.0,0.0,1.0],
            "camera": {"model": "pinhole", "width": 64, "height": 48, "fx": 50.0, "fy": 50.0,
                       "cx": 32.0, "cy": 24.0, "dist": [], "calibrated": true, "readout_ns": 0},
            "time_offset_ns": 0})
    };
    serde_json::json!({"contract": "ados.capture/1", "rig_id": "sim-rig", "calib_version": "1",
        "sensors": [cam("cam_fl", "front_left", 0.05), cam("cam_fr", "front_right", -0.05),
            {"id": "imu0", "kind": "imu", "role": "nav",
             "T_body_sensor": [1.0,0.0,0.0,0.0, 0.0,1.0,0.0,0.0, 0.0,0.0,1.0,0.0, 0.0,0.0,0.0,1.0],
             "time_offset_ns": 0}]})
}

fn frame(cam: &str, k: u64) -> CapturedFrame {
    CapturedFrame {
        camera_id: cam.into(),
        ts_ms: (k * 50) as i64,
        ts_ns: HOST0 + SIM0 + k * FRAME_NS,
        ts_sensor_ns: Some(SIM0 + k * FRAME_NS),
        timebase: TIMEBASE_SIM,
        trigger_seq: Some(k),
        exposure_ns: Some(2_000_000),
        seq: k,
        width: 64,
        height: 48,
        format: FrameFormat::Gray8,
        pixels: (0..64 * 48)
            .map(|i| ((i * 7 + k as usize) % 251) as u8)
            .collect(),
    }
}

fn bus_events() -> Vec<BusEvent> {
    let mut ev = vec![BusEvent::Clock(ClockSample {
        t_ns: 0,
        host_mono_ns: HOST0,
        utc_ns: None,
        locked: true,
        sigma_ns: 1_000,
    })];
    // 1.2 s of IMU at 1 kHz in 10 ms batches, from sim 0.9 s.
    for b in 0..120u64 {
        ev.push(BusEvent::Imu {
            src: "imu0".into(),
            batch: ImuBatch {
                samples: (0..10)
                    .map(|i| {
                        let n = b * 10 + i;
                        ImuSample {
                            t_ns: HOST0 + 900_000_000 + n * 1_000_000,
                            seq: n,
                            gyro: vec![0.0, 0.0, 0.01],
                            accel: vec![0.0, 0.0, 9.81],
                            temp_c: None,
                        }
                    })
                    .collect(),
            },
        });
    }
    for k in 0..FRAMES {
        for cam in ["cam_fl", "cam_fr"] {
            ev.push(BusEvent::Trigger {
                camera: cam.into(),
                rec: TriggerRecord {
                    seq: k,
                    t_mid_ns: HOST0 + SIM0 + k * FRAME_NS,
                    exposure_ns: 2_000_000,
                    gain: 1.0,
                },
            });
        }
    }
    ev
}

fn write_sidecars(dir: &std::path::Path) -> Paths {
    let root = dir.join("capture");
    std::fs::create_dir_all(&root).unwrap();
    let sc = CaptureStorageSidecar {
        version: 1,
        updated_ns: 0,
        capture_root: root.to_string_lossy().into_owned(),
        volume: None,
        capture: CaptureUsage {
            used_bytes: 0,
            budget_bytes: 1 << 40,
            budget_pct: 85,
            floor_free_bytes: 0,
            sessions: 0,
            uploaded: 0,
            evict_unuploaded: false,
            state: CaptureState::Ok,
        },
        write_probe: None,
    };
    let paths = Paths {
        storage_sidecar: dir.join("capture-storage.json"),
        clock_sidecar: dir.join("clock.json"),
        rig: dir.join("rig.json"),
        active_marker: Some(dir.join("active")),
    };
    std::fs::write(&paths.storage_sidecar, serde_json::to_vec(&sc).unwrap()).unwrap();
    std::fs::write(
        &paths.clock_sidecar,
        br#"{"timebase":"sim","locked":true,"sigma_ns":1000}"#,
    )
    .unwrap();
    std::fs::write(&paths.rig, serde_json::to_vec(&rig_json()).unwrap()).unwrap();
    paths
}

async fn ask(tx: &mpsc::Sender<ControlMsg>, req: CtlRequest) -> CtlReply {
    let (reply, rx) = oneshot::channel();
    tx.send(ControlMsg { req, reply }).await.unwrap();
    rx.await.unwrap()
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn a_two_camera_sim_session_records_and_validates() {
    let dir = tempfile::tempdir().unwrap();
    let paths = write_sidecars(dir.path());
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
    let poses: Vec<PoseEstimate> = (0..30u64)
        .map(|k| PoseEstimate {
            t_ns: HOST0 + SIM0 - 200_000_000 + k * FRAME_NS,
            r: se3::IDENTITY,
            t: [k as f64 * 0.1, 0.0, 1.5],
            cov: diagonal_cov(0.05, 0.01),
            source: PoseSource::Replay,
            health: VioHealth::Good,
        })
        .collect();
    let sensors = Arc::new(Mutex::new(SensorState::default()));
    let events = bus_events();
    for e in &events {
        sensors.lock().apply(e);
    }
    let (bus_tx, bus_rx) = mpsc::channel(1024);
    let (ctl_tx, ctl_rx) = mpsc::channel(8);
    let frames: Vec<CapturedFrame> = (0..FRAMES)
        .flat_map(|k| [frame("cam_fl", k), frame("cam_fr", k)])
        .collect();
    let publisher = AtlasPublisher::bind(&dir.path().join("atlas.sock").to_string_lossy())
        .await
        .unwrap();
    let cancel = Shutdown::new();
    // Start before any frame flows: the loop serves control first.
    let start = {
        let tx = ctl_tx.clone();
        tokio::spawn(async move { ask(&tx, CtlRequest::Start(StartArgs::default())).await })
    };
    // Frames flow through an engine source fed after the start.
    let (ftx, frx) = mpsc::channel(64);
    let deps = LoopDeps {
        cfg,
        profile: "visor".into(),
        frames: AtlasFrameSource::Engine(EngineFrameSource::from_receiver(frx)),
        router: None,
        pose: Arc::new(ReplayPose::new(poses)),
        fc: None,
        vehicle: VehicleFeed::new(),
        sensors,
        bus_rx,
        lidar_rx: None,
        publisher,
        control_rx: ctl_rx,
        hud_rx: None,
        view_tx: None,
        paths: paths.clone(),
    };
    let lp = tokio::spawn(run_capture_loop(deps, cancel.clone()));
    let started = start.await.unwrap();
    assert!(started.ok, "{started:?}");
    let session_id = started.result["session_id"].as_str().unwrap().to_string();
    for e in events {
        bus_tx.send(e).await.unwrap();
    }
    tokio::time::sleep(Duration::from_millis(200)).await;
    for f in frames {
        ftx.send(f).await.unwrap();
    }
    tokio::time::sleep(Duration::from_millis(500)).await;
    let state = ask(&ctl_tx, CtlRequest::State).await;
    assert_eq!(state.result["state"], "recording");
    let stop = ask(&ctl_tx, CtlRequest::Stop).await;
    assert!(stop.ok);
    cancel.trigger();
    lp.await.unwrap();

    let sdir = dir.path().join("capture").join(&session_id);
    assert!(!sdir.join(".recording").exists());
    assert!(!dir.path().join("active").exists());
    let r = SessionReader::open(&sdir).unwrap();
    r.verify().unwrap();
    assert_eq!(r.manifest().timebase, Timebase::Sim);
    assert_eq!(r.rig().rig_id, "sim-rig");
    let (mut fl, mut fr, mut imu, mut trig, mut poses_seen) = (0, 0, 0, 0, 0);
    let mut image_times = Vec::new();
    r.for_each_message(|m| {
        match m.topic {
            "/cam/cam_fl/image" => {
                fl += 1;
                let img: RawImage = m.decode()?;
                assert_eq!(img.encoding, "mono8");
                image_times.push(m.t_ns);
            }
            "/cam/cam_fr/image" => fr += 1,
            "/imu/imu0" => {
                let b: ImuBatch = m.decode()?;
                imu += b.samples.len();
                // Samples are on simulation time.
                assert!(b.samples.iter().all(|s| s.t_ns < 2_200_000_000));
            }
            "/cam/cam_fl/trigger" | "/cam/cam_fr/trigger" => trig += 1,
            "/pose/replay" => poses_seen += 1,
            _ => {}
        }
        Ok(())
    })
    .unwrap();
    assert_eq!((fl, fr), (FRAMES as usize, FRAMES as usize));
    assert_eq!(imu, 1200, "every 1 kHz sample preserved");
    assert_eq!(trig, 2 * FRAMES as usize);
    assert!(poses_seen > 0);
    // Images carry their trigger's mid-exposure on simulation time.
    image_times.sort_unstable();
    assert_eq!(image_times[0], SIM0);
    assert_eq!(image_times[1] - image_times[0], FRAME_NS);
}
