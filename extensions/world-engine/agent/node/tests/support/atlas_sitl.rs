//! Shared helpers for the Atlas SITL gate harness (`atlas_sitl*.rs`): a mock
//! engine, persistable keyframes and the bag that closes their session.
// Each test binary uses a subset of these helpers.
#![allow(dead_code)]

use ados_capture_contract::v1::ImuSample;
use ados_capture_contract::{
    CameraModel, CameraModelModel, GeoAnchor, GeoAnchorDatum, GeoAnchorSource,
};
use std::sync::Arc;
use tokio::sync::broadcast;
use world_engine_node::{Cluster, Engine, JobStore, MockDetector, MockReconstructor, Scheduler};
use world_engine_protocol::atlas::{
    CameraRole, CaptureState, CaptureStatus, ImageEncoding, KeyframeEnvelope, KeyframeFlags,
    KeyframeImage, KeyframeTier, Pose, PoseSource, TimeAlignment, VioHealth,
    ATLAS_CAPTURE_STATE_TOPIC, ATLAS_KEYFRAME_TOPIC, POSE_COV_LEN,
};
use world_engine_transport::AtlasEvent;

pub fn engine() -> Engine {
    let store = JobStore::open(":memory:").unwrap();
    let scheduler = Scheduler::new(store, Arc::new(MockReconstructor), Arc::new(MockDetector));
    Engine::new(scheduler, Cluster::new_master("compute-sitl"), 1)
}

/// A real, persistable single-camera keyframe for session `g0` (the session the
/// [`bagged`] helper closes), so a simulated capture actually writes a dataset the
/// bag can finalize into a reconstruct job — the pipeline the G0 / G2 / loopback
/// gates prove. A keyframe whose bytes never decode would count as received but
/// persist nothing, and an empty bag enqueues no job.
pub fn keyframe(i: usize) -> AtlasEvent {
    AtlasEvent::new(
        ATLAS_KEYFRAME_TOPIC,
        None,
        keyframe_env("g0", "front", i as u64).to_msgpack().unwrap(),
    )
}

pub fn bagged(keyframes: u64) -> AtlasEvent {
    let status = CaptureStatus {
        session_id: "g0".into(),
        state: CaptureState::Bagged,
        keyframes,
        vio_health: VioHealth::Good,
        camera_count: 1,
        ingest_rate_hz: 9.0,
        capped: false,
        anchored: true,
        pose_tier: PoseSource::FcEkf,
        dropped_keyframes: 0,
    };
    AtlasEvent::new(
        ATLAS_CAPTURE_STATE_TOPIC,
        None,
        status.to_msgpack().unwrap(),
    )
}

/// A full keyframe envelope for `session_id` / `camera_id`, with an IMU sample so
/// the camera subtree + the IMU scalars are both produced. Synthetic intrinsics +
/// an identity pose translated along x by the keyframe id (no real camera). The
/// session is a parameter so a capture's keyframes and its terminal bag carry the
/// SAME session — the persister keys the dataset by it, so a mismatch would leave
/// the bag with nothing to reconstruct.
pub fn keyframe_env(session_id: &str, camera_id: &str, kf_id: u64) -> KeyframeEnvelope {
    KeyframeEnvelope {
        session_id: session_id.into(),
        kf_id,
        ts_unix_ms: 1000 + kf_id as i64,
        camera_id: camera_id.into(),
        camera_role: CameraRole::Primary,
        tier: KeyframeTier::Full,
        image: KeyframeImage {
            encoding: ImageEncoding::Jpeg,
            width: 1280,
            height: 720,
            bytes: vec![],
        },
        camera: CameraModel {
            model: CameraModelModel::Pinhole,
            width: 1280,
            height: 720,
            fx: 900.0,
            fy: 900.0,
            cx: 640.0,
            cy: 360.0,
            dist: vec![],
            calibrated: true,
            readout_ns: 0,
        },
        pose: Pose {
            r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            t: [kf_id as f64, 0.0, 0.0],
            cov: None,
        },
        pose_cov: vec![0.0009; POSE_COV_LEN],
        pose_source: PoseSource::FcEkf,
        time: TimeAlignment::unmeasured(),
        anchor: Some(GeoAnchor {
            lat_deg: 12.97,
            lon_deg: 77.59,
            h_ellipsoid_m: 920.0,
            geoid_undulation_m: 0.0,
            datum: GeoAnchorDatum::Wgs84,
            source: GeoAnchorSource::GnssMean,
        }),
        imu_window: vec![ImuSample {
            t_ns: 0,
            seq: 0,
            gyro: vec![0.1, 0.2, 0.3],
            accel: vec![0.0, 0.0, 9.81],
            temp_c: None,
        }],
        flags: KeyframeFlags::default(),
    }
}

/// Mirror the delta WS handler's per-device filter against one subscriber: a
/// tuple tagged for another device is dropped from this device's view. The
/// broadcast delivers the same tuple to every subscriber, so this receives
/// exactly one and applies the `dev != device_id` filter, returning the event
/// only when it belongs to `device_id`.
pub async fn deliver_for(
    rx: &mut broadcast::Receiver<(String, AtlasEvent)>,
    device_id: &str,
) -> Option<AtlasEvent> {
    match rx.recv().await {
        Ok((dev, event)) if dev == device_id => Some(event),
        Ok(_) => None, // another drone's delta — not this view
        Err(_) => None,
    }
}
