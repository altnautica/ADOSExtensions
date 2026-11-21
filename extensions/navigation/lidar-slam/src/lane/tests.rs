//! The lane's outputs: `map_status.imu_feed` follows the pose provider the
//! run actually uses (`navigation.lidar.imu = "fc"` only names the IMU a
//! planar LiDAR pose would level with); `scan_raw` streams only while a
//! capture session records; `pose_track` carries every estimate in order.

use std::collections::BTreeMap;
use std::time::{Duration, Instant};

use ados_mapping_pose::{PoseProvider, PoseTrack};
use ados_mapping_protocol::capture::{
    CaptureState, MapStatus, PoseSource, PoseTrackBatch, ScanChunk, VioHealth, POSE_COV_LEN,
};

use super::*;
use crate::publish::Event;
use crate::settings::KEY_LIDAR;

/// A LiDAR pose fed by the test: the run's provider is the LiDAR one.
#[derive(Default)]
struct LidarPose {
    track: PoseTrack,
}

impl PoseProvider for LidarPose {
    fn source(&self) -> PoseSource {
        PoseSource::Lidar
    }
    fn track(&self) -> &PoseTrack {
        &self.track
    }
}

fn deps(pose: Option<Arc<dyn PoseProvider>>, planar: bool) -> (LaneDeps, mpsc::Receiver<Event>) {
    let values = BTreeMap::from([(KEY_LIDAR.to_string(), serde_json::json!({ "imu": "fc" }))]);
    let cfg = NavConfig::from_values(&values).unwrap();
    let (out, rx) = Publisher::new();
    let (_bus_tx, bus_rx) = mpsc::channel(4);
    let deps = LaneDeps {
        settings: crate::supervisor::nav_settings(&NavConfig {
            memory_budget_mb: Some(64),
            ..cfg.clone()
        }),
        cfg,
        pose,
        fc: FcEkfPose::new(),
        rig: None,
        lidar_planar: planar,
        frames: None,
        bus_rx,
        lidar_rx: None,
        out,
        capture: CaptureWatch::default(),
        shared: Arc::new(Shared::default()),
        scratch_dir: None,
        voxfeed: None,
    };
    (deps, rx)
}

/// Run a lane and return the first map status it publishes.
async fn first_map_status(pose: Option<Arc<dyn PoseProvider>>, planar: bool) -> MapStatus {
    let (deps, mut rx) = deps(pose, planar);
    let cancel = Shutdown::new();
    let task = tokio::spawn(run_lane(deps, cancel.clone()));
    let status = tokio::time::timeout(Duration::from_secs(10), async {
        loop {
            let (topic, body) = rx.recv().await.expect("lane alive");
            if topic == crate::topics::MAP_STATUS {
                return MapStatus::from_msgpack(&body).unwrap();
            }
        }
    })
    .await
    .expect("no map status within 10 s");
    cancel.trigger();
    task.await.unwrap();
    status
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn no_pose_provider_reports_no_provider_and_no_imu_feed() {
    let status = first_map_status(None, true).await;
    assert_eq!(status.pose_provider, None);
    assert_eq!(status.imu_feed, None, "nothing uses the FC IMU feed");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn an_fc_imu_under_the_planar_lidar_pose_reports_its_feed() {
    let status = first_map_status(Some(Arc::new(LidarPose::default())), true).await;
    assert!(
        status.imu_feed.is_some(),
        "the planar LiDAR pose levels with the FC IMU"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn an_fc_imu_under_a_3d_lidar_pose_reports_no_imu_feed() {
    let status = first_map_status(Some(Arc::new(LidarPose::default())), false).await;
    assert_eq!(status.imu_feed, None, "KISS-ICP uses no FC IMU feed");
}

fn scan(t_ns: u64, n: usize) -> LidarScan {
    LidarScan {
        sensor_id: "lidar0".into(),
        t_ns,
        points: (0..n).map(|i| [i as f32, 1.0, 0.0]).collect(),
        t_offset_ns: vec![0; n],
        intensity: vec![9; n],
    }
}

fn drain(rx: &mut mpsc::Receiver<Event>, topic: &str) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    while let Ok((t, body)) = rx.try_recv() {
        if t == topic {
            out.push(body);
        }
    }
    out
}

#[tokio::test]
async fn raw_scans_stream_only_while_a_capture_session_records() {
    let (mut d, mut rx) = deps(None, false);
    let mut lane = Lane::new(&mut d);
    lane.on_lidar(scan(1, 8_000)).await;
    assert!(
        drain(&mut rx, crate::topics::SCAN_RAW).is_empty(),
        "Capture absent"
    );

    d.capture.set(Instant::now(), CaptureState::Capturing);
    lane.on_lidar(scan(2, 8_000)).await;
    let parts: Vec<ScanChunk> = drain(&mut rx, crate::topics::SCAN_RAW)
        .iter()
        .map(|b| ScanChunk::from_msgpack(b).unwrap())
        .collect();
    assert_eq!(parts.len(), 3, "8,000 points in parts of 3,500");
    assert!(parts
        .iter()
        .all(|c| c.seq == 2 && c.t_ns == 2 && c.parts == 3));
    assert_eq!(parts.iter().map(ScanChunk::len).sum::<usize>(), 8_000);

    d.capture.set(Instant::now(), CaptureState::Paused);
    lane.on_lidar(scan(3, 10)).await;
    assert!(drain(&mut rx, crate::topics::SCAN_RAW).is_empty(), "paused");
}

fn estimate(t_ns: u64, x: f64) -> PoseEstimate {
    PoseEstimate {
        t_ns,
        r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        t: [x, 0.0, 0.0],
        cov: [0.0; POSE_COV_LEN],
        source: PoseSource::Lidar,
        health: VioHealth::Good,
        sigma_xy_m: Some(0.05),
        heading_ok: true,
        health_reason: None,
    }
}

#[tokio::test]
async fn the_pose_track_carries_every_estimate_between_two_batches_in_order() {
    let pose = Arc::new(LidarPose::default());
    let (mut d, mut rx) = deps(Some(pose.clone()), true);
    let mut lane = Lane::new(&mut d);
    for i in 0..5 {
        pose.track.push(estimate(1_000 + i, i as f64));
    }
    lane.on_nav_tick().await;
    for i in 5..7 {
        pose.track.push(estimate(1_000 + i, i as f64));
    }
    lane.on_nav_tick().await;
    lane.on_track_tick().await;
    lane.on_track_tick().await;
    let batches: Vec<PoseTrackBatch> = drain(&mut rx, crate::topics::POSE_TRACK)
        .iter()
        .map(|b| PoseTrackBatch::from_msgpack(b).unwrap())
        .collect();
    assert_eq!(batches.len(), 1, "an empty interval publishes no batch");
    let ts: Vec<u64> = batches[0].estimates.iter().map(|e| e.t_ns).collect();
    assert_eq!(ts, (1_000..1_007).collect::<Vec<_>>());
    assert_eq!(batches[0].source, PoseSource::Lidar);
    assert_eq!(batches[0].estimates[6].t, [6.0, 0.0, 0.0]);
}
