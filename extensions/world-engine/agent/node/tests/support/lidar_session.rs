//! A synthetic LiDAR + IMU + camera session walking along +x in a box room,
//! for the LiDAR pipeline tests.

use std::path::{Path, PathBuf};

use ados_capture_contract::foxglove::{
    PackedElementField, PointCloud, Pose, PoseInFrame, Quaternion, RawImage, Vector3,
};
use ados_capture_contract::v1::{ImuBatch, ImuSample};
use ados_capture_contract::*;

pub const T0: u64 = 1_000_000_000;
const IDENTITY: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];
/// Camera looking along +x (OpenCV axes: x right = -y body, y down = -z body).
const T_BODY_CAM: [f64; 16] = [
    0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

fn sensor(id: &str, kind: RigSensorKind, role: RigSensorRole, t: [f64; 16]) -> RigSensor {
    RigSensor {
        id: id.into(),
        kind,
        role,
        t_body_sensor: t.into(),
        camera: None,
        imu_noise: None,
        time_offset_ns: 0,
        lever_arm_m: None,
    }
}

fn rig() -> Rig {
    let mut cam = sensor(
        "cam0",
        RigSensorKind::Camera,
        RigSensorRole::FrontLeft,
        T_BODY_CAM,
    );
    cam.camera = Some(CameraModel {
        model: CameraModelModel::Pinhole,
        width: 64,
        height: 48,
        fx: 40.0,
        fy: 40.0,
        cx: 32.0,
        cy: 24.0,
        dist: vec![],
        calibrated: false,
        readout_ns: 0,
    });
    Rig {
        contract: ContractId::AdosCapture1,
        rig_id: "lidar_rig".into(),
        calib_version: "1".into(),
        sensors: vec![
            cam,
            sensor("lidar0", RigSensorKind::Lidar, RigSensorRole::Up, IDENTITY),
            sensor("imu0", RigSensorKind::Imu, RigSensorRole::Nav, IDENTITY),
        ],
    }
}

/// Body position at frame `i`: walking along x inside a 6 x 4 m room.
pub fn body_at(i: u64) -> [f64; 3] {
    [0.5 + 0.1 * i as f64, 2.0, 1.5]
}

/// A 6 x 4 x 2.5 m room in the body frame of a body at `at` (identity
/// rotation), every 10 cm.
fn room_points(at: [f64; 3]) -> Vec<u8> {
    let mut data = Vec::new();
    let mut push = |p: [f64; 3]| {
        for v in [p[0] - at[0], p[1] - at[1], p[2] - at[2]] {
            data.extend_from_slice(&(v as f32).to_le_bytes());
        }
    };
    for xi in 0..=60 {
        for yi in 0..=40 {
            push([xi as f64 * 0.1, yi as f64 * 0.1, 0.0]);
        }
        for zi in 0..=25 {
            push([xi as f64 * 0.1, 0.0, zi as f64 * 0.1]);
            push([xi as f64 * 0.1, 4.0, zi as f64 * 0.1]);
        }
    }
    for yi in 0..=40 {
        for zi in 0..=25 {
            push([0.0, yi as f64 * 0.1, zi as f64 * 0.1]);
            push([6.0, yi as f64 * 0.1, zi as f64 * 0.1]);
        }
    }
    data
}

/// The session (geo-anchored) under `root`; `track` records the body walk on
/// `/pose/lidar`.
pub fn build_session(root: &Path, track: bool) -> PathBuf {
    let manifest = Manifest {
        contract: ContractId::AdosCapture1,
        session_id: session_id("visor-1", 1_792_368_000, 7),
        device_id: "visor-1".into(),
        profile: ManifestProfile::Visor,
        timebase: Timebase::HubFreerun,
        t0_ns: T0,
        utc_at_t0_ns: None,
        sigma_ns: -1,
        anchor: Some(GeoAnchor {
            lat_deg: 12.9716,
            lon_deg: 77.5946,
            h_ellipsoid_m: 834.6,
            geoid_undulation_m: -86.4,
            datum: GeoAnchorDatum::Wgs84,
            source: GeoAnchorSource::Sim,
        }),
        rig: RigRef {
            rig_id: "lidar_rig".into(),
            calib_version: "1".into(),
        },
        operator: Operator {
            site: Some("Test flat".into()),
            floor: Some(0),
            rooms: vec![Room {
                name: "Living".into(),
                floor: 0,
                t_start_ns: T0,
                t_end_ns: T0 + 30 * 100_000_000,
            }],
        },
        segments: vec![],
        health: SessionHealth {
            blur: false,
            texture: false,
            flicker: false,
            corridor: false,
            frames_dropped_pct: 0.0,
        },
    };
    let mut w = SessionWriter::create(root, manifest, &rig()).unwrap();
    for i in 0..30u64 {
        let t = T0 + i * 100_000_000;
        let p = body_at(i);
        let pose = PoseInFrame {
            timestamp: None,
            frame_id: "world".into(),
            pose: Some(Pose {
                position: Some(Vector3 {
                    x: p[0],
                    y: p[1],
                    z: p[2],
                }),
                orientation: Some(Quaternion {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0,
                }),
            }),
        };
        if track {
            w.write_proto(&topics::pose("lidar"), t, &pose).unwrap();
        }
        let data: Vec<u8> = (0..64 * 48)
            .map(|k| (((k % 64) / 4 + (k / 64) / 4 + i as usize) % 2 * 200) as u8)
            .collect();
        let img = RawImage {
            timestamp: None,
            frame_id: "cam0".into(),
            width: 64,
            height: 48,
            encoding: "mono8".into(),
            step: 64,
            data,
        };
        w.write_proto(&topics::cam_image("cam0"), t, &img).unwrap();
        let samples = (0..10u64)
            .map(|k| ImuSample {
                t_ns: t + k * 10_000_000,
                seq: i * 10 + k,
                gyro: vec![0.0, 0.0, 0.0],
                accel: vec![0.0, 0.0, 9.81],
                temp_c: None,
            })
            .collect();
        w.write_proto(&topics::imu("imu0"), t, &ImuBatch { samples })
            .unwrap();
        if i % 5 == 0 {
            let field = |name: &str, offset: u32| PackedElementField {
                name: name.into(),
                offset,
                r#type: 7,
            };
            let pc = PointCloud {
                timestamp: None,
                frame_id: "lidar0".into(),
                pose: None,
                point_stride: 12,
                fields: vec![field("x", 0), field("y", 4), field("z", 8)],
                data: room_points(p),
            };
            w.write_proto(&topics::lidar_points("lidar0"), t, &pc)
                .unwrap();
        }
    }
    let dir = w.dir().to_path_buf();
    w.finish().unwrap();
    dir
}
