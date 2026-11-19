//! `map_status.imu_feed` follows the pose provider the run actually uses:
//! `atlas.lidar.imu = "fc"` only names the IMU a planar LiDAR pose would
//! level with, so a run whose pose fell back to the flight controller's EKF,
//! or runs KISS-ICP on a 3D LiDAR, reports no FC IMU feed.

use std::collections::BTreeMap;
use std::sync::Arc;
use std::time::Duration;

use ados_protocol::shutdown::Shutdown;
use parking_lot::Mutex;
use tokio::io::AsyncReadExt;
use tokio::sync::mpsc;
use ados_mapping_protocol::atlas::{
    AtlasEvent, MapStatus, PoseSource, PLUGIN_ATLAS_MAP_STATUS_TOPIC,
};

use super::{run_capture_loop, LoopDeps, NavDeps, Paths};
use ados_mapping_pose::frame_source::{AtlasFrameSource, SyntheticFrameSource};
use ados_mapping_pose::fc_ekf::FcEkfPose;
use ados_mapping_pose::{PoseProvider, PoseTrack};
use crate::publish::AtlasPublisher;
use crate::runtime::{AtlasRuntimeConfig, SharedSteps};
use ados_mapping_pose::sensor_bus::SensorState;
use ados_mapping_pose::vehicle::VehicleFeed;

/// A LiDAR pose that never estimates: the run's provider is the LiDAR one.
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

/// Run the capture loop with `atlas.lidar.imu = "fc"`, `pose` and a planar
/// (`planar`) or 3D LiDAR, and return the first map status it publishes.
async fn first_map_status(pose: Arc<dyn PoseProvider>, planar: bool) -> MapStatus {
    let dir = tempfile::tempdir().unwrap();
    let paths = Paths {
        storage_sidecar: dir.path().join("capture-storage.json"),
        clock_sidecar: dir.path().join("clock.json"),
        rig: dir.path().join("rig.json"),
        active_marker: None,
    };
    let values = BTreeMap::from([(
        crate::runtime::KEY_LIDAR.to_string(),
        serde_json::json!({ "imu": "fc" }),
    )]);
    let sock = dir.path().join("atlas.sock");
    let (_bus_tx, bus_rx) = mpsc::channel(4);
    let (_ctl_tx, ctl_rx) = mpsc::channel(4);
    let cancel = Shutdown::new();
    let deps = LoopDeps {
        cfg: AtlasRuntimeConfig::from_values("bench", &values).unwrap(),
        profile: "drone".into(),
        frames: AtlasFrameSource::Synthetic(SyntheticFrameSource::new(Vec::new())),
        router: None,
        pose,
        fc: None,
        vehicle: VehicleFeed::new(),
        sensors: Arc::new(Mutex::new(SensorState::default())),
        bus_rx,
        lidar_rx: None,
        publisher: AtlasPublisher::bind(&sock.to_string_lossy()).await.unwrap(),
        control_rx: ctl_rx,
        hud_rx: None,
        view_tx: None,
        camera: None,
        paths,
        nav: NavDeps {
            lidar_planar: planar,
            ..Default::default()
        },
        reconstruct_steps: SharedSteps::default(),
    };
    let mut sub = tokio::net::UnixStream::connect(&sock).await.unwrap();
    let task = tokio::spawn(run_capture_loop(deps, cancel.clone()));
    let status = tokio::time::timeout(Duration::from_secs(10), async {
        loop {
            let mut len = [0u8; 4];
            sub.read_exact(&mut len).await.unwrap();
            let mut body = vec![0u8; u32::from_be_bytes(len) as usize];
            sub.read_exact(&mut body).await.unwrap();
            let ev = AtlasEvent::decode(&body).unwrap();
            if ev.topic == PLUGIN_ATLAS_MAP_STATUS_TOPIC {
                return MapStatus::from_msgpack(&ev.payload).unwrap();
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
async fn an_fc_imu_under_the_fc_ekf_pose_reports_no_imu_feed() {
    let status = first_map_status(Arc::new(FcEkfPose::new()), true).await;
    assert_eq!(status.imu_feed, None, "the FC EKF pose uses no FC IMU feed");
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn an_fc_imu_under_the_planar_lidar_pose_reports_its_feed() {
    let status = first_map_status(Arc::new(LidarPose::default()), true).await;
    assert!(
        status.imu_feed.is_some(),
        "the planar LiDAR pose levels with the FC IMU"
    );
}

#[tokio::test(flavor = "multi_thread", worker_threads = 2)]
async fn an_fc_imu_under_a_3d_lidar_pose_reports_no_imu_feed() {
    let status = first_map_status(Arc::new(LidarPose::default()), false).await;
    assert_eq!(status.imu_feed, None, "KISS-ICP uses no FC IMU feed");
}
