use super::*;
use std::collections::BTreeSet;

use ados_capture_contract::v1::ImuSample;
use ados_capture_contract::{
    session_id, CameraModel, CameraModelModel, ContractId, ManifestProfile, RigRef, RigSensor,
    RigSensorKind, RigSensorRole, SessionReader, Timebase,
};

use crate::pose::{diagonal_cov, se3, PoseEstimate};
use world_engine_protocol::atlas::{PoseSource, VioHealth};

const I4: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

pub fn rig() -> Rig {
    let cam = |id: &str, role| RigSensor {
        id: id.into(),
        kind: RigSensorKind::Camera,
        role,
        t_body_sensor: I4.into(),
        camera: Some(CameraModel {
            model: CameraModelModel::Pinhole,
            width: 64,
            height: 48,
            fx: 50.0,
            fy: 50.0,
            cx: 32.0,
            cy: 24.0,
            dist: vec![],
            calibrated: true,
            readout_ns: 0,
        }),
        imu_noise: None,
        time_offset_ns: 0,
        lever_arm_m: None,
    };
    Rig {
        contract: ContractId::AdosCapture1,
        rig_id: "test-rig".into(),
        calib_version: "1".into(),
        sensors: vec![
            cam("cam_fl", RigSensorRole::FrontLeft),
            cam("cam_fr", RigSensorRole::FrontRight),
        ],
    }
}

pub fn manifest(rig: &Rig) -> Manifest {
    Manifest {
        contract: ContractId::AdosCapture1,
        session_id: session_id("visor-1", 1_792_368_000, 0x3fa2),
        device_id: "visor-1".into(),
        profile: ManifestProfile::Visor,
        timebase: Timebase::Sim,
        t0_ns: 1_000_000_000,
        utc_at_t0_ns: None,
        sigma_ns: -1,
        anchor: None,
        rig: RigRef {
            rig_id: rig.rig_id.clone(),
            calib_version: rig.calib_version.clone(),
        },
        operator: Operator {
            site: None,
            floor: None,
            rooms: vec![],
        },
        segments: vec![],
        health: SessionHealth {
            blur: false,
            texture: false,
            flicker: false,
            corridor: false,
            frames_dropped_pct: 0.0,
        },
    }
}

#[test]
fn every_channel_is_recorded_and_segments_roll() {
    let dir = tempfile::tempdir().unwrap();
    let marker = dir.path().join("active");
    let rig = rig();
    let rec = Recorder::start(RecorderConfig {
        root: dir.path().to_path_buf(),
        manifest: manifest(&rig),
        rig: rig.clone(),
        limits: SegmentLimits {
            max_bytes: 4 << 30,
            max_duration_ns: 500_000_000,
        },
        encoding: ImageEncoding::Raw,
        active_marker: Some(marker.clone()),
    })
    .unwrap();
    let sdir = rec.dir().to_path_buf();
    assert!(
        sdir.join(".recording").exists(),
        "marker before the first segment"
    );
    assert!(marker.exists());
    for k in 0..30u64 {
        let t = 1_000_000_000 + k * 50_000_000;
        for cam in ["cam_fl", "cam_fr"] {
            assert!(rec.send(Item::Image {
                camera: cam.into(),
                t_ns: t,
                format: FrameFormat::Gray8,
                width: 64,
                height: 48,
                pixels: vec![k as u8; 64 * 48],
            }));
            rec.send(Item::Trigger {
                camera: cam.into(),
                rec: TriggerRecord {
                    seq: k,
                    t_mid_ns: t,
                    exposure_ns: 2_000_000,
                    gain: 1.0,
                },
            });
        }
        let batch = ImuBatch {
            samples: (0..50)
                .map(|i| ImuSample {
                    t_ns: t + i * 1_000_000,
                    seq: k * 50 + i,
                    gyro: vec![0.0; 3],
                    accel: vec![0.0, 0.0, 9.81],
                    temp_c: None,
                })
                .collect(),
        };
        rec.send(Item::Imu {
            src: "imu0".into(),
            t_ns: t,
            batch,
        });
        let (pose, cov) = convert::pose(
            &PoseEstimate {
                t_ns: t,
                r: se3::IDENTITY,
                t: [k as f64, 0.0, 0.0],
                cov: diagonal_cov(0.1, 0.01),
                source: PoseSource::Vio,
                health: VioHealth::Good,
            },
            t,
        );
        rec.send(Item::Pose {
            source: "vio",
            t_ns: t,
            pose,
            cov,
        });
    }
    rec.send(Item::Clock(ClockSample {
        t_ns: 1_000_000_000,
        host_mono_ns: 5,
        utc_ns: None,
        locked: true,
        sigma_ns: 0,
    }));
    rec.send(Item::Health {
        t_ns: 2_000_000_000,
        json: serde_json::json!({"blur": "ok"}),
    });
    rec.send(Item::Keyframe {
        file: "000001.jpg".into(),
        jpeg: vec![0xFF, 0xD8],
    });
    let stats = rec.stats();
    let m = rec
        .finish(
            Operator {
                site: Some("Lab".into()),
                floor: Some(0),
                rooms: vec![],
            },
            manifest(&rig).health,
        )
        .unwrap();
    assert!(!sdir.join(".recording").exists());
    assert!(!marker.exists());
    assert!(sdir.join("keyframes/000001.jpg").exists());
    assert!(m.segments.len() >= 3, "{} segments", m.segments.len());
    assert_eq!(m.operator.site.as_deref(), Some("Lab"));
    assert!(stats.error.is_none());

    let r = SessionReader::open(&sdir).unwrap();
    r.verify().unwrap();
    let mut topics_seen = BTreeSet::new();
    let mut imu = 0;
    r.for_each_message(|msg| {
        topics_seen.insert(msg.topic.to_string());
        if msg.topic == "/imu/imu0" {
            imu += msg.decode::<ImuBatch>()?.samples.len();
        }
        Ok(())
    })
    .unwrap();
    assert_eq!(imu, 1500);
    for t in [
        "/cam/cam_fl/image",
        "/cam/cam_fr/image",
        "/cam/cam_fl/trigger",
        "/imu/imu0",
        "/pose/vio",
        "/pose/vio/cov",
        "/clock",
        "/health",
    ] {
        assert!(topics_seen.contains(t), "{t} missing from {topics_seen:?}");
    }
}

#[test]
fn a_dropped_recorder_leaves_recording_but_clears_the_active_marker() {
    let dir = tempfile::tempdir().unwrap();
    let marker = dir.path().join("active");
    let rig = rig();
    let rec = Recorder::start(RecorderConfig {
        root: dir.path().to_path_buf(),
        manifest: manifest(&rig),
        rig,
        limits: SegmentLimits::default(),
        encoding: ImageEncoding::Raw,
        active_marker: Some(marker.clone()),
    })
    .unwrap();
    let sdir = rec.dir().to_path_buf();
    drop(rec);
    assert!(sdir.join(".recording").exists());
    assert!(!marker.exists());
}
