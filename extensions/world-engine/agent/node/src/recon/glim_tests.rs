use super::*;
use crate::recon::prepare::{read_frames, FrameRecord};
use crate::recon::session_data::PoseTrack;
use crate::recon::StandInRunner;
use ados_capture_contract::CameraModelModel;

fn rig() -> Rig {
    serde_json::from_value(json!({
        "contract": "ados.capture/1", "rig_id": "r", "calib_version": "1",
        "sensors": [
            {"id": "cam0", "kind": "camera", "role": "front_left", "time_offset_ns": 5_000_000,
             "T_body_sensor": [0.0, 0.0, 1.0, 0.1, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.2,
                               0.0, 0.0, 0.0, 1.0],
             "camera": {"model": "pinhole", "width": 64, "height": 48, "fx": 40.0, "fy": 40.0,
                        "cx": 32.0, "cy": 24.0, "dist": [], "calibrated": false, "readout_ns": 0}},
            {"id": "lidar0", "kind": "lidar", "role": "up", "time_offset_ns": 0,
             "T_body_sensor": [0.0, -1.0, 0.0, 0.05, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.3,
                               0.0, 0.0, 0.0, 1.0],
             "lidar": crate::calib::test_util::mid360()}
        ]
    }))
    .unwrap()
}

/// A curved walk: body at `(cos a, sin a, 0.1 a)` facing along the path.
fn walk(n: u64) -> Vec<(u64, Mat4)> {
    (0..n)
        .map(|i| {
            let a = 0.2 * i as f64;
            let h = (a + std::f64::consts::FRAC_PI_2) / 2.0;
            (
                1_000_000_000 + i * 100_000_000,
                geom::from_quat_trans([h.cos(), 0.0, 0.0, h.sin()], [a.cos(), a.sin(), 0.1 * a]),
            )
        })
        .collect()
}

fn close(a: &[f64], b: &[f64], tol: f64) {
    for (i, (x, y)) in a.iter().zip(b).enumerate() {
        assert!((x - y).abs() <= tol, "[{i}] {x} vs {y}\n{a:?}\n{b:?}");
    }
}

#[test]
fn comments_are_stripped_outside_strings() {
    let s =
        "{\"url\": \"http://x/*y*/\", /* block\n */ \"a\": 1, // tail \"q\"\n \"b\": \"\\\"//\"}";
    let v: Value = serde_json::from_str(&strip_comments(s)).unwrap();
    assert_eq!(v, json!({"url": "http://x/*y*/", "a": 1, "b": "\"//"}));
}

fn stock(dir: &Path) -> std::path::PathBuf {
    let copy = copy_config(dir, "img");
    assert!(stand_in(&copy).unwrap());
    let args = copy.args_lossy();
    assert!(args.contains(&format!(
        "type=bind,src={CONFIG_SOURCE},dst=/config,readonly"
    )));
    dir.join(STOCK)
}

#[test]
fn config_gets_topics_extrinsic_and_imu_mode() {
    let dir = tempfile::tempdir().unwrap();
    let stock = stock(dir.path());
    let t_body_lidar = rig().sensors[1].t_body_sensor.0;
    let mut bag = GlimBag {
        lidar_id: "lidar0".into(),
        t_body_lidar,
        imu_id: Some("imu0".into()),
        imu_samples: 10,
        cloud_times: vec![1],
        point_times: true,
    };
    let out = dir.path().join("config");
    write_config(&stock, &out, &bag, GlimOdometry::default()).unwrap();
    let ros = read_jsonc(&out.join("config_ros.json")).unwrap();
    assert_eq!(ros["glim_ros"]["imu_topic"], "/imu");
    assert_eq!(ros["glim_ros"]["points_topic"], "/points");
    assert_eq!(ros["glim_ros"]["acc_scale"], 1.0);
    assert_eq!(
        ros["glim_ros"]["extension_modules"][0],
        "libmemory_monitor.so"
    );
    let sensors = read_jsonc(&out.join("config_sensors.json")).unwrap();
    let t: Vec<f64> = serde_json::from_value(sensors["sensors"]["T_lidar_imu"].clone()).unwrap();
    // T_lidar_body maps the body origin to minus the LiDAR mount, in LiDAR axes.
    let want = geom::inverse_rigid(&t_body_lidar);
    let got = geom::from_quat_trans([t[6], t[3], t[4], t[5]], [t[0], t[1], t[2]]);
    close(&got, &want, 1e-12);
    assert_eq!(sensors["sensors"]["global_shutter_lidar"], false);
    let global = read_jsonc(&out.join("config.json")).unwrap();
    assert_eq!(
        [
            &global["global"]["config_odometry"],
            &global["global"]["config_sub_mapping"],
            &global["global"]["config_global_mapping"]
        ],
        [
            "config_odometry_cpu.json",
            "config_sub_mapping_cpu.json",
            "config_global_mapping_cpu.json"
        ]
    );

    let out = dir.path().join("config_gpu");
    write_config(&stock, &out, &bag, GlimOdometry::Gpu).unwrap();
    let global = read_jsonc(&out.join("config.json")).unwrap();
    assert_eq!(
        [
            &global["global"]["config_odometry"],
            &global["global"]["config_sub_mapping"],
            &global["global"]["config_global_mapping"]
        ],
        [
            "config_odometry_gpu.json",
            "config_sub_mapping_gpu.json",
            "config_global_mapping_gpu.json"
        ]
    );

    bag.imu_id = None;
    bag.point_times = false;
    let out = dir.path().join("config_lidar_only");
    write_config(&stock, &out, &bag, GlimOdometry::Cpu).unwrap();
    let global = read_jsonc(&out.join("config.json")).unwrap();
    assert_eq!(
        global["global"]["config_odometry"],
        "config_odometry_ct.json"
    );
    let sub = read_jsonc(&out.join("config_sub_mapping_cpu.json")).unwrap();
    assert_eq!(sub["sub_mapping"]["enable_imu"], false);
    let glob = read_jsonc(&out.join("config_global_mapping_cpu.json")).unwrap();
    assert_eq!(glob["global_mapping"]["enable_imu"], false);
    let ros = read_jsonc(&out.join("config_ros.json")).unwrap();
    assert_eq!(ros["glim_ros"]["imu_topic"], "");
    let sensors = read_jsonc(&out.join("config_sensors.json")).unwrap();
    assert_eq!(sensors["sensors"]["global_shutter_lidar"], true);

    let empty = dir.path().join("empty");
    fs::create_dir_all(&empty).unwrap();
    let e = write_config(&empty, &dir.path().join("x"), &bag, GlimOdometry::Cpu).unwrap_err();
    assert_eq!(
        (e.code.as_str(), e.tool.as_deref()),
        ("E_TOOL_MISSING", Some("glim"))
    );
    assert_eq!(GlimOdometry::parse("gpu"), Some(GlimOdometry::Gpu));
    assert_eq!(GlimOdometry::parse("cuda"), None);
}

#[test]
fn alignment_recovers_a_known_rigid_transform() {
    let session = walk(20);
    let track = PoseTrack {
        source: "lidar".into(),
        samples: session.clone(),
    };
    let off = stand_in::stand_in_offset();
    let glim: Vec<(u64, Mat4)> = session
        .iter()
        .map(|(t, m)| (*t, geom::mul(&off, m)))
        .collect();
    let (t, rmse) = align(&glim, &track).unwrap();
    close(&t, &geom::inverse_rigid(&off), 1e-9);
    assert!(rmse < 1e-9, "{rmse}");

    // A straight walk still fixes the roll about the line (orientation marks).
    let line: Vec<(u64, Mat4)> = (0..10u64)
        .map(|i| {
            (
                i * 100_000_000,
                geom::from_quat_trans([1.0, 0.0, 0.0, 0.0], [0.1 * i as f64, 0.0, 0.0]),
            )
        })
        .collect();
    let track = PoseTrack {
        source: "lidar".into(),
        samples: line.clone(),
    };
    let glim: Vec<(u64, Mat4)> = line.iter().map(|(t, m)| (*t, geom::mul(&off, m))).collect();
    close(
        &align(&glim, &track).unwrap().0,
        &geom::inverse_rigid(&off),
        1e-9,
    );

    let far: Vec<(u64, Mat4)> = glim.iter().map(|(t, m)| (t + 60_000_000_000, *m)).collect();
    assert_eq!(align(&far, &track).unwrap_err().code, "E_INPUT");
}

fn frame(t_ns: u64) -> FrameRecord {
    FrameRecord {
        name: format!("cam0/{t_ns}.png"),
        camera_id: "cam0".into(),
        t_ns,
        t_world_cam: Some(geom::IDENTITY),
        width: 64,
        height: 48,
        fx: 40.0,
        fy: 40.0,
        cx: 32.0,
        cy: 24.0,
        model: CameraModelModel::Pinhole,
        dist: vec![],
        calibrated: false,
        t_body_cam: None,
        rig_instant: None,
        prior_sigma_m: None,
    }
}

/// A dataset whose frames are at `times`, each with an image file.
fn dataset(dir: &Path, times: &[u64]) {
    let frames: Vec<FrameRecord> = times.iter().map(|t| frame(*t)).collect();
    fs::create_dir_all(dir.join("images/cam0")).unwrap();
    for f in &frames {
        fs::write(dir.join("images").join(&f.name), b"png").unwrap();
    }
    fs::write(
        dir.join("frames.json"),
        serde_json::to_vec(&frames).unwrap(),
    )
    .unwrap();
}

#[test]
fn frames_are_reposed_from_the_trajectory() {
    let dir = tempfile::tempdir().unwrap();
    let rig = rig();
    // The camera offset (+5 ms) moves the second frame onto a sample.
    dataset(dir.path(), &[1_000_000_000, 1_095_000_000, 9_000_000_000]);
    let samples = walk(5);
    let traj = PoseTrack {
        source: "glim".into(),
        samples: samples.clone(),
    };
    // With a recorded track, a frame outside the trajectory keeps its pose.
    let n = rewrite_frames(dir.path(), &rig, &traj, true).unwrap();
    assert_eq!(
        n,
        Reposed {
            glim: 2,
            recorded: 1,
            dropped: 0
        }
    );
    let back = read_frames(dir.path()).unwrap();
    let body_cam = rig.sensors[0].t_body_sensor.0;
    close(
        &back[1].t_world_cam.unwrap(),
        &geom::mul(&samples[1].1, &body_cam),
        1e-12,
    );
    let first = geom::mul(&traj.at(1_005_000_000).unwrap(), &body_cam);
    close(&back[0].t_world_cam.unwrap(), &first, 1e-12);
    assert_eq!(back[2].t_world_cam, Some(geom::IDENTITY));

    // Without one, it is dropped with its image.
    let n = rewrite_frames(dir.path(), &rig, &traj, false).unwrap();
    assert_eq!((n.glim, n.dropped), (2, 1));
    let back = read_frames(dir.path()).unwrap();
    assert_eq!(back.len(), 2);
    assert!(!dir.path().join("images/cam0/9000000000.png").exists());
    assert!(dir.path().join("images/cam0/1000000000.png").exists());
}

#[test]
fn a_poorly_aligned_trajectory_keeps_the_recorded_poses() {
    let dir = tempfile::tempdir().unwrap();
    let ds = dir.path().join("dataset");
    dataset(&ds, &[1_000_000_000, 1_300_000_000, 9_000_000_000]);
    let before = fs::read(ds.join("frames.json")).unwrap();
    let track = PoseTrack {
        source: "lidar".into(),
        samples: walk(20),
    };
    let off = stand_in::stand_in_offset();
    let good: Vec<(u64, Mat4)> = walk(20)
        .into_iter()
        .map(|(t, m)| (t, geom::mul(&off, &m)))
        .collect();
    // Every other pose 1 m off: no rigid fit comes within 0.30 m.
    let bad: Vec<(u64, Mat4)> = good
        .iter()
        .enumerate()
        .map(|(i, (t, m))| {
            let mut m = *m;
            m[11] += if i % 2 == 0 { 1.0 } else { -1.0 };
            (*t, m)
        })
        .collect();
    let s = apply_trajectory(
        dir.path(),
        &ds,
        &rig(),
        bad,
        Some(&track),
        GlimOdometry::Cpu,
    )
    .unwrap();
    assert!(
        !s.used && s.rejected.as_deref() == Some("align_rmse"),
        "{s:?}"
    );
    assert!(s.align_rmse_m.unwrap() > MAX_ALIGN_RMSE_M);
    assert_eq!(fs::read(ds.join("frames.json")).unwrap(), before);
    let v = serde_json::to_value(&s).unwrap();
    assert_eq!(
        (&v["used"], &v["rejected"], &v["odometry"]),
        (&json!(false), &json!("align_rmse"), &json!("cpu"))
    );

    let s = apply_trajectory(
        dir.path(),
        &ds,
        &rig(),
        good,
        Some(&track),
        GlimOdometry::Gpu,
    )
    .unwrap();
    assert!(
        s.used && s.rejected.is_none() && s.align_rmse_m.unwrap() < 1e-9,
        "{s:?}"
    );
    assert_eq!(
        (s.frames_glim, s.frames_recorded, s.frames_dropped),
        (2, 1, 0)
    );
    assert!(serde_json::to_value(&s).unwrap().get("rejected").is_none());
    // Posed in the track's world: the camera sits at the body pose.
    let back = read_frames(&ds).unwrap();
    let want = geom::mul(
        &track.at(1_005_000_000).unwrap(),
        &rig().sensors[0].t_body_sensor.0,
    );
    close(&back[0].t_world_cam.unwrap(), &want, 1e-9);
}

#[test]
fn a_trajectory_without_a_track_must_stay_under_the_speed_limit() {
    let dir = tempfile::tempdir().unwrap();
    let ds = dir.path().join("dataset");
    dataset(&ds, &[1_000_000_000, 9_000_000_000]);
    // walk(): ~2 m/s along the arc, under the limit; frames outside drop.
    let s = apply_trajectory(dir.path(), &ds, &rig(), walk(10), None, GlimOdometry::Cpu).unwrap();
    assert!(s.used && s.align_rmse_m.is_none());
    assert_eq!((s.frames_glim, s.frames_dropped), (1, 1));

    let fast: Vec<(u64, Mat4)> = (0..5u64)
        .map(|i| {
            let p = [0.8 * i as f64, 0.0, 0.0];
            (
                i * 100_000_000,
                geom::from_quat_trans([1.0, 0.0, 0.0, 0.0], p),
            )
        })
        .collect();
    assert!((max_speed(&fast).unwrap().0 - 8.0).abs() < 1e-9);
    let e = apply_trajectory(dir.path(), &ds, &rig(), fast, None, GlimOdometry::Cpu).unwrap_err();
    assert_eq!(e.code, "E_GLIM_DIVERGED");
    assert!(e.message.contains("8.00 m/s"), "{}", e.message);
    assert_eq!(read_frames(&ds).unwrap().len(), 1);
}

#[test]
fn missing_glim_is_a_tool_missing_error() {
    let dir = tempfile::tempdir().unwrap();
    let runner = StandInRunner {
        missing: vec!["glim".into()],
        ..Default::default()
    };
    let e = check_available(&runner, dir.path(), GlimOdometry::Cpu).unwrap_err();
    assert_eq!(
        (e.code.as_str(), e.tool.as_deref()),
        ("E_TOOL_MISSING", Some("glim"))
    );
    let ok = StandInRunner::default();
    check_available(&ok, dir.path(), GlimOdometry::Cpu).unwrap();
    let runs = ok.runs_of("glim");
    assert_eq!(runs.len(), 1, "CPU odometry does not ask for runtimes");
    let inspect = runs[0].args_lossy();
    assert_eq!(inspect[..2], ["image", "inspect"]);
    assert!(inspect
        .last()
        .unwrap()
        .starts_with("koide3/glim_ros2@sha256:"));
}

#[test]
fn gpu_odometry_needs_the_nvidia_runtime_and_passes_the_gpu() {
    let dir = tempfile::tempdir().unwrap();
    let ok = StandInRunner::default();
    check_available(&ok, dir.path(), GlimOdometry::Gpu).unwrap();
    let info = ok.runs_of("glim")[1].args_lossy();
    assert_eq!(info, ["info", "--format", "{{json .Runtimes}}"]);

    let no_nvidia = StandInRunner {
        missing: vec![NVIDIA_RUNTIME.into()],
        ..Default::default()
    };
    check_available(&no_nvidia, dir.path(), GlimOdometry::Cpu).unwrap();
    let e = check_available(&no_nvidia, dir.path(), GlimOdometry::Gpu).unwrap_err();
    assert_eq!(
        (e.code.as_str(), e.tool.as_deref()),
        ("E_TOOL_MISSING", Some("nvidia-container-runtime"))
    );

    let gpus = |o| {
        glim_rosbag(dir.path(), "img", o)
            .args_lossy()
            .iter()
            .any(|a| a == "--gpus")
    };
    assert!(gpus(GlimOdometry::Gpu) && !gpus(GlimOdometry::Cpu));
}
