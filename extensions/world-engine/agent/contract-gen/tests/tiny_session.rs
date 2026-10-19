//! The golden `contract/fixtures/tiny-session/` session: built here, decoded
//! here. `ADOS_CAPTURE_REGEN_FIXTURES=1 cargo test -p ados-capture-contract
//! --test tiny_session` rewrites the committed copy; every other run checks
//! that the committed copy decodes to exactly what the writer produces now.

use std::path::{Path, PathBuf};

use ados_capture_contract::foxglove::RawImage;
use ados_capture_contract::v1::{ClockSample, ImuBatch, ImuSample, TriggerRecord};
use ados_capture_contract::{
    session_id, topics, CameraModel, CameraModelModel, ContractId, GeoAnchor, GeoAnchorDatum,
    GeoAnchorSource, Manifest, ManifestProfile, Operator, Rig, RigRef, RigSensor, RigSensorKind,
    RigSensorRole, SegmentLimits, SessionHealth, SessionReader, SessionWriter, Timebase,
};

const T0: u64 = 1_000_000_000;
const IDENTITY: [f64; 16] = [
    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
];

fn rig() -> Rig {
    Rig {
        contract: ContractId::AdosCapture1,
        rig_id: "tiny-rig".into(),
        calib_version: "1".into(),
        sensors: vec![
            RigSensor {
                id: "cam0".into(),
                kind: RigSensorKind::Camera,
                role: RigSensorRole::FrontLeft,
                t_body_sensor: [
                    0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    1.0,
                ]
                .into(),
                camera: Some(CameraModel {
                    model: CameraModelModel::Pinhole,
                    width: 64,
                    height: 48,
                    fx: 50.0,
                    fy: 50.0,
                    cx: 32.0,
                    cy: 24.0,
                    dist: vec![],
                    calibrated: false,
                    readout_ns: 0,
                }),
                imu_noise: None,
                time_offset_ns: 0,
                lever_arm_m: None,
            },
            RigSensor {
                id: "imu0".into(),
                kind: RigSensorKind::Imu,
                role: RigSensorRole::Nav,
                t_body_sensor: IDENTITY.into(),
                camera: None,
                imu_noise: None,
                time_offset_ns: 0,
                lever_arm_m: None,
            },
        ],
    }
}

fn manifest() -> Manifest {
    Manifest {
        contract: ContractId::AdosCapture1,
        session_id: session_id("sim-1", 1_792_368_000, 1),
        device_id: "sim-1".into(),
        profile: ManifestProfile::Sim,
        timebase: Timebase::Sim,
        t0_ns: T0,
        utc_at_t0_ns: None,
        sigma_ns: 0,
        anchor: Some(GeoAnchor {
            lat_deg: 12.9716,
            lon_deg: 77.5946,
            h_ellipsoid_m: 834.6,
            geoid_undulation_m: -86.4,
            datum: GeoAnchorDatum::Wgs84,
            source: GeoAnchorSource::Sim,
        }),
        rig: RigRef {
            rig_id: "tiny-rig".into(),
            calib_version: "1".into(),
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

fn image(seq: u64) -> RawImage {
    let data = (0..64 * 48)
        .map(|i| ((i % 64) as u64 * 4 + seq * 16) as u8)
        .collect();
    RawImage {
        timestamp: Some(prost_types::Timestamp {
            seconds: ((T0 + seq * 50_000_000) / 1_000_000_000) as i64,
            nanos: ((T0 + seq * 50_000_000) % 1_000_000_000) as i32,
        }),
        frame_id: "cam0".into(),
        width: 64,
        height: 48,
        encoding: "mono8".into(),
        step: 64,
        data,
    }
}

/// Write the tiny session under `root`; returns the session directory.
fn build(root: &Path) -> PathBuf {
    let mut w = SessionWriter::create(root, manifest(), &rig()).unwrap();
    for batch in 0..2u64 {
        let samples = (0..10u64)
            .map(|i| {
                let seq = batch * 10 + i;
                ImuSample {
                    t_ns: T0 + seq * 1_000_000,
                    seq,
                    gyro: vec![0.01, -0.02, 0.003 * seq as f64],
                    accel: vec![0.0, 0.0, 9.80665],
                    temp_c: Some(31.5),
                }
            })
            .collect();
        let t = T0 + batch * 10_000_000;
        w.write_proto(&topics::imu("imu0"), t, &ImuBatch { samples })
            .unwrap();
    }
    for seq in 0..2u64 {
        let t_mid = T0 + seq * 50_000_000;
        let trig = TriggerRecord {
            seq,
            t_mid_ns: t_mid,
            exposure_ns: 2_000_000,
            gain: 1.0,
        };
        w.write_proto(&topics::cam_trigger("cam0"), t_mid, &trig)
            .unwrap();
        w.write_proto(&topics::cam_image("cam0"), t_mid, &image(seq))
            .unwrap();
    }
    let clock = ClockSample {
        t_ns: T0,
        host_mono_ns: 5_000_000_000,
        utc_ns: None,
        locked: true,
        sigma_ns: 0,
    };
    w.write_proto(topics::CLOCK, T0, &clock).unwrap();
    let dir = w.dir().to_path_buf();
    w.finish().unwrap();
    dir
}

#[derive(Debug, PartialEq)]
struct Decoded {
    frames: Vec<(u64, u32, u32, Vec<u8>)>,
    imu: Vec<ImuSample>,
    triggers: Vec<TriggerRecord>,
    clocks: Vec<ClockSample>,
    timebase: Timebase,
}

fn decode(dir: &Path) -> Decoded {
    let r = SessionReader::open(dir).unwrap();
    r.verify().unwrap();
    let mut d = Decoded {
        frames: vec![],
        imu: vec![],
        triggers: vec![],
        clocks: vec![],
        timebase: r.manifest().timebase,
    };
    r.for_each_message(|m| {
        match m.topic {
            "/cam/cam0/image" => {
                let img: RawImage = m.decode()?;
                d.frames.push((m.t_ns, img.width, img.height, img.data));
            }
            "/cam/cam0/trigger" => d.triggers.push(m.decode()?),
            "/imu/imu0" => d.imu.extend(m.decode::<ImuBatch>()?.samples),
            "/clock" => d.clocks.push(m.decode()?),
            other => panic!("unexpected topic {other}"),
        }
        Ok(())
    })
    .unwrap();
    d
}

fn committed_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../contract/fixtures/tiny-session")
}

#[test]
fn tiny_session_fixture_is_current_and_decodes() {
    if std::env::var_os("ADOS_CAPTURE_REGEN_FIXTURES").is_some() {
        let _ = std::fs::remove_dir_all(committed_root());
        build(&committed_root());
    }
    let tmp = tempfile::tempdir().unwrap();
    let fresh = decode(&build(tmp.path()));
    let committed = decode(&committed_root().join(manifest().session_id));

    assert_eq!(committed.frames.len(), 2);
    assert!(committed
        .frames
        .iter()
        .all(|f| f.1 == 64 && f.2 == 48 && f.3.len() == 64 * 48));
    assert_eq!(committed.imu.len(), 20);
    assert!(committed
        .imu
        .iter()
        .enumerate()
        .all(|(i, s)| s.seq == i as u64));
    assert_eq!(committed.clocks.len(), 1);
    assert_eq!(committed.timebase, Timebase::Sim);
    assert_eq!(
        committed, fresh,
        "committed tiny-session is stale; regenerate it"
    );
}

#[test]
fn a_tampered_segment_fails_checksum_verification() {
    let tmp = tempfile::tempdir().unwrap();
    let dir = build(tmp.path());
    let seg = dir.join("segments/000000.mcap");
    let mut bytes = std::fs::read(&seg).unwrap();
    let mid = bytes.len() / 2;
    bytes[mid] ^= 0xff;
    std::fs::write(&seg, bytes).unwrap();
    assert!(SessionReader::open(&dir).unwrap().verify().is_err());
}

#[test]
fn segments_roll_on_duration_and_size() {
    let tmp = tempfile::tempdir().unwrap();
    let mut w = SessionWriter::create(tmp.path(), manifest(), &rig())
        .unwrap()
        .with_limits(SegmentLimits {
            max_bytes: u64::MAX,
            max_duration_ns: 1_000_000_000,
        });
    for i in 0..5u64 {
        let clock = ClockSample {
            t_ns: T0 + i * 600_000_000,
            ..Default::default()
        };
        w.write_proto(topics::CLOCK, clock.t_ns, &clock).unwrap();
    }
    let m = w.finish().unwrap();
    // 0.0 0.6 | 1.2 1.8 | 2.4
    let files: Vec<&str> = m.segments.iter().map(|s| s.file.as_str()).collect();
    assert_eq!(
        files,
        [
            "segments/000000.mcap",
            "segments/000001.mcap",
            "segments/000002.mcap"
        ]
    );
    assert_eq!(m.segments[1].t_start_ns, T0 + 1_200_000_000);
    assert_eq!(m.segments[1].t_end_ns, T0 + 1_800_000_000);

    let tmp = tempfile::tempdir().unwrap();
    let mut w = SessionWriter::create(tmp.path(), manifest(), &rig())
        .unwrap()
        .with_limits(SegmentLimits {
            max_bytes: 1,
            max_duration_ns: u64::MAX,
        });
    for seq in 0..3u64 {
        w.write_proto(&topics::cam_image("cam0"), T0 + seq, &image(seq))
            .unwrap();
    }
    let m = w.finish().unwrap();
    assert_eq!(
        m.segments.len(),
        3,
        "any written segment is over a 1-byte limit"
    );
    let r = SessionReader::open(&tmp.path().join(&m.session_id)).unwrap();
    r.verify().unwrap();
    let mut n = 0;
    r.for_each_message(|_| {
        n += 1;
        Ok(())
    })
    .unwrap();
    assert_eq!(n, 3);
}

#[test]
fn out_of_order_and_off_timebase_stamps_do_not_count_as_elapsed() {
    let tmp = tempfile::tempdir().unwrap();
    let mut w = SessionWriter::create(tmp.path(), manifest(), &rig())
        .unwrap()
        .with_limits(SegmentLimits {
            max_bytes: u64::MAX,
            max_duration_ns: 1_000_000_000,
        });
    // 0.1 s steps over 1.4 s, each followed by a stamp 0.05 s in the past and
    // one hours away (a message off the session timebase): only the real
    // 1.4 s counts, so the segment rolls once, at 1.0 s.
    for i in 0..15u64 {
        let t = T0 + i * 100_000_000;
        for at in [t, t - 50_000_000, T0 + 40_000_000_000_000] {
            let clock = ClockSample {
                t_ns: at,
                ..Default::default()
            };
            w.write_proto(topics::CLOCK, at, &clock).unwrap();
        }
    }
    let m = w.finish().unwrap();
    assert_eq!(m.segments.len(), 2, "{:?}", m.segments);
    assert_eq!(m.segments[1].t_start_ns, T0 + 950_000_000);
}

#[test]
fn decode_refuses_the_wrong_message_type() {
    let tmp = tempfile::tempdir().unwrap();
    let r = SessionReader::open(&build(tmp.path())).unwrap();
    let mut refused = 0;
    r.for_each_message(|m| {
        if m.topic == topics::CLOCK && m.decode::<ImuBatch>().is_err() {
            refused += 1;
        }
        Ok(())
    })
    .unwrap();
    assert_eq!(refused, 1);
}

#[test]
fn the_descriptor_set_names_every_channel_message() {
    use prost::Message;
    let fds =
        prost_types::FileDescriptorSet::decode(ados_capture_contract::FILE_DESCRIPTOR_SET).unwrap();
    let names: Vec<String> = fds
        .file
        .iter()
        .flat_map(|f| {
            let pkg = f.package().to_string();
            f.message_type
                .iter()
                .map(move |m| format!("{pkg}.{}", m.name()))
        })
        .collect();
    for want in [
        "ados.capture.v1.ImuBatch",
        "ados.capture.v1.TriggerRecord",
        "ados.capture.v1.ClockSample",
        "ados.capture.v1.GnssRaw",
        "ados.capture.v1.PoseCov",
        "foxglove.RawImage",
        "foxglove.CompressedImage",
        "foxglove.LocationFix",
        "foxglove.PointCloud",
        "foxglove.PoseInFrame",
        "google.protobuf.Timestamp",
    ] {
        assert!(
            names.iter().any(|n| n == want),
            "{want} missing from the descriptor set"
        );
    }
}
