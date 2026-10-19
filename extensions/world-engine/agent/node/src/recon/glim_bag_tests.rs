use super::*;
use ados_capture_contract::foxglove::packed_element_field::NumericType;
use ados_capture_contract::foxglove::PackedElementField;
use ados_capture_contract::v1::ImuSample;
use ados_capture_contract::{topics, Manifest, Rig, SessionWriter};

const T0: u64 = 1_000_000_000;

fn tiny(name: &str) -> Vec<u8> {
    let p = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../contract/fixtures/tiny-session/sim-1-20261019T000000Z-0001")
        .join(name);
    fs::read(p).unwrap()
}

/// The tiny session's manifest and rig (camera + `imu0`) plus `lidar0`
/// (10 cm up, recorded 1 µs late), with two sweeps of two timed points and
/// two IMU batches of three samples.
fn session(root: &Path) -> std::path::PathBuf {
    let manifest: Manifest = serde_json::from_slice(&tiny("manifest.json")).unwrap();
    let mut rig: Rig = serde_json::from_slice(&tiny("rig.json")).unwrap();
    rig.sensors.push(
        serde_json::from_value(serde_json::json!({
            "id": "lidar0", "kind": "lidar", "role": "up", "time_offset_ns": 1000,
            "T_body_sensor": [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.1,
                              0.0, 0.0, 0.0, 1.0]
        }))
        .unwrap(),
    );
    let mut w = SessionWriter::create(root, manifest, &rig).unwrap();
    let field = |name: &str, offset: u32, ty: NumericType| PackedElementField {
        name: name.into(),
        offset,
        r#type: ty as i32,
    };
    for k in 0..2u64 {
        let t = T0 + k * 100_000_000;
        let mut data = Vec::new();
        for (p, dt) in [([1.0f32, 2.0, 3.0], 0u32), ([-1.0, 0.5, 0.25], 50_000_000)] {
            p.iter().for_each(|v| data.extend(v.to_le_bytes()));
            data.extend(dt.to_le_bytes());
        }
        let pc = PointCloud {
            timestamp: None,
            frame_id: "lidar0".into(),
            pose: None,
            point_stride: 16,
            fields: vec![
                field("x", 0, NumericType::Float32),
                field("y", 4, NumericType::Float32),
                field("z", 8, NumericType::Float32),
                field("t_offset_ns", 12, NumericType::Uint32),
            ],
            data,
        };
        w.write_proto(&topics::lidar_points("lidar0"), t, &pc)
            .unwrap();
        let samples = (0..3u64)
            .map(|i| ImuSample {
                t_ns: t + i * 5_000_000,
                seq: k * 3 + i,
                gyro: vec![0.1, -0.2, i as f64],
                accel: vec![0.0, 0.0, 9.81],
                temp_c: None,
            })
            .collect();
        w.write_proto(&topics::imu("imu0"), t + 10_000_000, &ImuBatch { samples })
            .unwrap();
    }
    let dir = w.dir().to_path_buf();
    w.finish().unwrap();
    dir
}

/// Little-endian CDR reader mirroring the encoder's alignment.
struct Rd<'a>(&'a [u8], usize);

impl Rd<'_> {
    fn take(&mut self, n: usize, align: usize) -> &[u8] {
        while !(self.1 - 4).is_multiple_of(align) {
            self.1 += 1;
        }
        self.1 += n;
        &self.0[self.1 - n..self.1]
    }
    fn u32(&mut self) -> u32 {
        u32::from_le_bytes(self.take(4, 4).try_into().unwrap())
    }
    fn f64(&mut self) -> f64 {
        f64::from_le_bytes(self.take(8, 8).try_into().unwrap())
    }
    fn string(&mut self) -> String {
        let n = self.u32() as usize;
        String::from_utf8(self.take(n, 1)[..n - 1].to_vec()).unwrap()
    }
    /// `(stamp_ns, frame_id)` of a std_msgs/Header.
    fn header(&mut self) -> (u64, String) {
        let sec = self.u32() as u64;
        let nsec = self.u32() as u64;
        (sec * 1_000_000_000 + nsec, self.string())
    }
}

#[test]
fn the_bag_holds_timed_sweeps_and_imu_samples() {
    let dir = tempfile::tempdir().unwrap();
    let reader = SessionReader::open(&session(&dir.path().join("s"))).unwrap();
    let bag_dir = dir.path().join("bag");
    let bag = export(&reader, &bag_dir).unwrap();
    assert_eq!(bag.lidar_id, "lidar0");
    assert_eq!(bag.imu_id.as_deref(), Some("imu0"));
    assert_eq!(bag.imu_samples, 6);
    assert_eq!(bag.cloud_times, vec![T0 + 1000, T0 + 100_001_000]);
    assert!(bag.point_times);
    assert_eq!(bag.t_body_lidar[11], 0.1);

    let bytes = fs::read(bag_dir.join("bag_0.mcap")).unwrap();
    let (mut clouds, mut imus) = (Vec::new(), Vec::new());
    for m in mcap::MessageStream::new(&bytes).unwrap() {
        let m = m.unwrap();
        let schema = m.channel.schema.as_ref().unwrap();
        assert_eq!(schema.encoding, "ros2msg");
        assert_eq!(m.channel.message_encoding, "cdr");
        assert_eq!(&m.data[..4], &[0, 1, 0, 0]);
        let mut r = Rd(&m.data, 4);
        let (stamp, frame) = r.header();
        assert_eq!(stamp, m.log_time);
        match (m.channel.topic.as_str(), schema.name.as_str()) {
            (POINTS_TOPIC, "sensor_msgs/msg/PointCloud2") => {
                assert_eq!(frame, LIDAR_FRAME);
                assert_eq!((r.u32(), r.u32()), (1, 2));
                let names: Vec<String> = (0..r.u32())
                    .map(|_| {
                        let n = r.string();
                        r.u32();
                        r.take(1, 1);
                        r.u32();
                        n
                    })
                    .collect();
                assert_eq!(names, ["x", "y", "z", "t"]);
                r.take(1, 1);
                assert_eq!((r.u32(), r.u32()), (16, 32));
                let rest = r.0.len() - r.1;
                let data = r.take(rest, 1)[4..36].to_vec();
                let f = |i: usize| f32::from_le_bytes(data[4 * i..4 * i + 4].try_into().unwrap());
                assert_eq!([f(0), f(1), f(2), f(3)], [1.0, 2.0, 3.0, 0.0]);
                assert_eq!([f(4), f(5), f(6)], [-1.0, 0.5, 0.25]);
                assert!((f(7) - 0.05).abs() < 1e-7, "{}", f(7));
                clouds.push(stamp);
            }
            (IMU_TOPIC, "sensor_msgs/msg/Imu") => {
                assert_eq!(frame, IMU_FRAME);
                let q: Vec<f64> = (0..4).map(|_| r.f64()).collect();
                assert_eq!(q, [0.0, 0.0, 0.0, 1.0]);
                assert_eq!(r.f64(), -1.0);
                (0..8).for_each(|_| assert_eq!(r.f64(), 0.0));
                let gyro: Vec<f64> = (0..3).map(|_| r.f64()).collect();
                (0..9).for_each(|_| assert_eq!(r.f64(), 0.0));
                let accel: Vec<f64> = (0..3).map(|_| r.f64()).collect();
                assert_eq!(accel, [0.0, 0.0, 9.81]);
                imus.push((stamp, gyro[2]));
            }
            other => panic!("unexpected channel {other:?}"),
        }
    }
    assert_eq!(clouds, bag.cloud_times);
    imus.sort_by_key(|i| i.0);
    assert_eq!(imus.len(), 6);
    assert_eq!(imus[1], (T0 + 5_000_000, 1.0));

    let meta: serde_yaml_ng::Value =
        serde_yaml_ng::from_str(&fs::read_to_string(bag_dir.join("metadata.yaml")).unwrap())
            .unwrap();
    let info = &meta["rosbag2_bagfile_information"];
    assert_eq!(info["storage_identifier"].as_str(), Some("mcap"));
    assert_eq!(info["message_count"].as_u64(), Some(8));
    assert_eq!(
        info["starting_time"]["nanoseconds_since_epoch"].as_u64(),
        Some(T0)
    );
    assert_eq!(info["relative_file_paths"][0].as_str(), Some("bag_0.mcap"));
    // The key rosbag2 parses (singular `count`).
    let topics = info["topics_with_message_count"].as_sequence().unwrap();
    let counts: Vec<(String, u64)> = topics
        .iter()
        .map(|t| {
            (
                t["topic_metadata"]["name"].as_str().unwrap().to_string(),
                t["message_count"].as_u64().unwrap(),
            )
        })
        .collect();
    assert_eq!(
        counts,
        [("/points".to_string(), 2), ("/imu".to_string(), 6)]
    );
}

#[test]
fn tum_lines_read_back_exactly() {
    let dir = tempfile::tempdir().unwrap();
    let path = dir.path().join("traj.txt");
    fs::write(
        &path,
        "# t x y z qx qy qz qw\n1700000000.123456789 1.0 2.0 3.0 0 0 0.7071068 0.7071068\n\n0.5 0 0 0 0 0 0 1\n",
    )
    .unwrap();
    let poses = read_tum(&path).unwrap();
    assert_eq!(poses.len(), 2);
    assert_eq!(poses[0].0, 500_000_000);
    assert_eq!(poses[1].0, 1_700_000_000_123_456_789);
    let m = poses[1].1;
    assert_eq!(geom::translation(&m), [1.0, 2.0, 3.0]);
    // 90° about z: x maps to y.
    let x = geom::apply(&m, [1.0, 0.0, 0.0]);
    assert!(
        (x[0] - 1.0).abs() < 1e-6 && (x[1] - 3.0).abs() < 1e-6,
        "{x:?}"
    );

    let out = dir.path().join("out.txt");
    write_tum(&out, &poses).unwrap();
    let back = read_tum(&out).unwrap();
    assert_eq!(back[1].0, poses[1].0);
    for (a, b) in back[1].1.iter().zip(&poses[1].1) {
        assert!((a - b).abs() < 1e-8);
    }
    fs::write(&path, "1.0 1 2 3\n").unwrap();
    assert_eq!(read_tum(&path).unwrap_err().code, "E_IO");
}
