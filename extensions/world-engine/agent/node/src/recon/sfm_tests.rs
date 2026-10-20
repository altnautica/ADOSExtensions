use super::*;
use crate::recon::geom;
use crate::recon::stand_in::StandInRunner;
use ados_capture_contract::CameraModelModel;

fn frame(i: u64, calibrated: bool) -> FrameRecord {
    FrameRecord {
        name: format!("cam0/{i:04}.png"),
        camera_id: "cam0".into(),
        t_ns: i * 100_000_000,
        t_world_cam: Some(geom::from_quat_trans(
            [1.0, 0.0, 0.0, 0.0],
            [i as f64 * 0.3, 0.0, 0.0],
        )),
        width: 64,
        height: 48,
        fx: 40.0,
        fy: 40.0,
        cx: 32.0,
        cy: 24.0,
        model: CameraModelModel::Pinhole,
        dist: vec![],
        calibrated,
        t_body_cam: None,
        rig_instant: None,
        prior_sigma_m: None,
    }
}

fn frames(n: u64, calibrated: bool) -> Vec<FrameRecord> {
    (0..n).map(|i| frame(i, calibrated)).collect()
}

fn unposed(frames: Vec<FrameRecord>) -> Vec<FrameRecord> {
    frames
        .into_iter()
        .map(|f| FrameRecord {
            t_world_cam: None,
            ..f
        })
        .collect()
}

struct Ran {
    summary: SfmSummary,
    runs: Vec<Vec<String>>,
    rig_config: Option<serde_json::Value>,
    _dir: tempfile::TempDir,
    sfm: std::path::PathBuf,
}

impl Ran {
    fn cmd(&self, name: &str) -> Option<&Vec<String>> {
        self.runs.iter().find(|a| a[0] == name)
    }

    fn pos(&self, name: &str) -> usize {
        self.runs
            .iter()
            .position(|a| a[0] == name)
            .unwrap_or(usize::MAX)
    }
}

/// The value after `flag` in `args`.
fn value<'a>(args: &'a [String], flag: &str) -> Option<&'a str> {
    let i = args.iter().position(|a| a == flag)?;
    args.get(i + 1).map(String::as_str)
}

fn run_with(frames: &[FrameRecord], opts: SfmOptions<'_>) -> Ran {
    let dir = tempfile::tempdir().unwrap();
    let dataset = dir.path().join("dataset");
    fs::create_dir_all(dataset.join("images")).unwrap();
    fs::write(
        dataset.join("frames.json"),
        serde_json::to_vec(frames).unwrap(),
    )
    .unwrap();
    let runner = StandInRunner::default();
    let sfm = dir.path().join("sfm");
    let summary = run(&runner, &dataset, &sfm, &opts, &mut |_| {}).unwrap();
    let rig_config = fs::read(sfm.join("rig_config.json"))
        .ok()
        .map(|b| serde_json::from_slice(&b).unwrap());
    let runs = runner
        .runs_of("colmap")
        .into_iter()
        .map(|i| i.args_lossy())
        .collect();
    Ran {
        summary,
        runs,
        rig_config,
        _dir: dir,
        sfm,
    }
}

const REFINE: SfmOptions<'static> = SfmOptions {
    fixed_poses: false,
    refine: true,
    hierarchical: false,
    prior_source: None,
};

#[test]
fn quadratic_pairs_are_never_requested() {
    // A long unposed walk: the sequential matcher, linear overlap only, no
    // vocabulary-tree loop detection. 10 keyframes a second: 20 per 2 s.
    let long = run_with(&unposed(frames(400, true)), REFINE);
    let seq = long
        .cmd("sequential_matcher")
        .expect("sequential matcher ran");
    assert_eq!(
        value(seq, "--SequentialMatching.quadratic_overlap"),
        Some("0")
    );
    assert_eq!(value(seq, "--SequentialMatching.loop_detection"), Some("0"));
    assert_eq!(value(seq, "--SequentialMatching.overlap"), Some("20"));
    // A posed session of any size matches only the pairs its poses list.
    for n in [40, 400] {
        let posed = run_with(&frames(n, true), REFINE);
        let imp = posed.cmd("matches_importer").expect("pairs imported");
        assert_eq!(value(imp, "--match_type"), Some("pairs"));
        let list = fs::read_to_string(posed.sfm.join("pairs.txt")).unwrap();
        assert!(!list.is_empty());
        assert!(posed.cmd("sequential_matcher").is_none());
        assert!(posed.cmd("exhaustive_matcher").is_none());
    }
    for args in &long.runs {
        assert_ne!(
            value(args, "--SequentialMatching.quadratic_overlap"),
            Some("1")
        );
    }
}

#[test]
fn every_matcher_verifies_relative_pose_with_the_inlier_floor() {
    for ran in [
        run_with(&unposed(frames(4, true)), REFINE),
        run_with(&unposed(frames(200, true)), REFINE),
        run_with(&frames(4, true), REFINE),
    ] {
        let m = [
            "exhaustive_matcher",
            "sequential_matcher",
            "matches_importer",
        ]
        .iter()
        .find_map(|n| ran.cmd(n))
        .expect("a matcher ran");
        assert_eq!(
            value(m, "--TwoViewGeometry.compute_relative_pose"),
            Some("1")
        );
        assert_eq!(value(m, "--TwoViewGeometry.min_num_inliers"), Some("30"));
    }
}

#[test]
fn a_refined_posed_session_maps_with_weighted_priors() {
    let ran = run_with(&frames(4, true), REFINE);
    assert_eq!(ran.summary.method, "pose_prior_mapper");
    assert!(ran.summary.world_frame);
    assert_eq!(ran.summary.pairs_rejected_by_prior, 0);
    assert!(ran.cmd("point_triangulator").is_none());
    assert!(ran.cmd("bundle_adjuster").is_none());
    let m = ran.cmd("pose_prior_mapper").unwrap();
    assert_eq!(value(m, "--use_robust_loss_on_prior_position"), Some("1"));
    assert_eq!(value(m, "--overwrite_priors_covariance"), Some("0"));
    assert_eq!(value(m, "--Mapper.min_num_matches"), Some("30"));
    assert_eq!(value(m, "--Mapper.abs_pose_min_num_inliers"), Some("50"));
    // Calibrated intrinsics stay fixed; uncalibrated ones are refined.
    assert_eq!(value(m, "--Mapper.ba_refine_focal_length"), Some("0"));
    assert_eq!(value(m, "--Mapper.ba_refine_extra_params"), Some("0"));
    let free = run_with(&frames(4, false), REFINE);
    let m = free.cmd("pose_prior_mapper").unwrap();
    assert!(value(m, "--Mapper.ba_refine_focal_length").is_none());
    // The priors went into the database before mapping.
    assert!(ran.pos("matches_importer") < ran.pos("pose_prior_mapper"));
}

/// The position covariance diagonal of every `pose_priors` row.
fn prior_variances(ran: &Ran) -> Vec<[f64; 3]> {
    let conn = rusqlite::Connection::open(ran.sfm.join("database.db")).unwrap();
    let mut stmt = conn
        .prepare("SELECT position_covariance FROM pose_priors ORDER BY corr_data_id")
        .unwrap();
    stmt.query_map([], |r| r.get::<_, Vec<u8>>(0))
        .unwrap()
        .map(|b| {
            let b = b.unwrap();
            let v: Vec<f64> = b
                .as_chunks::<8>()
                .0
                .iter()
                .map(|c| f64::from_le_bytes(*c))
                .collect();
            [v[0], v[4], v[8]]
        })
        .collect()
}

#[test]
fn prior_sigma_follows_the_source_unless_the_track_recorded_one() {
    let close = |a: [f64; 3], b: [f64; 3]| a.iter().zip(b).all(|(x, y)| (x - y).abs() < 1e-12);
    let planar = run_with(
        &frames(4, true),
        SfmOptions {
            prior_source: Some("lidar2d"),
            ..REFINE
        },
    );
    let v = prior_variances(&planar);
    assert_eq!(v.len(), 4);
    assert!(
        v.iter().all(|v| close(*v, [0.0009, 0.0009, 0.0625])),
        "{v:?}"
    );
    let mut recorded = frames(4, true);
    recorded[2].prior_sigma_m = Some([0.2, 0.2, 0.2]);
    let ekf = run_with(
        &recorded,
        SfmOptions {
            prior_source: Some("fc_ekf"),
            ..REFINE
        },
    );
    let v = prior_variances(&ekf);
    assert!(close(v[0], [0.25; 3]) && close(v[2], [0.04; 3]), "{v:?}");
    assert_eq!(default_prior_sigma_m(Some("vio")), [0.05; 3]);
}

#[test]
fn fixed_poses_are_only_triangulated() {
    for opts in [
        SfmOptions {
            fixed_poses: true,
            prior_source: Some("lidar"),
            ..REFINE
        },
        SfmOptions {
            refine: false,
            ..REFINE
        },
    ] {
        let ran = run_with(&frames(4, true), opts);
        assert_eq!(ran.summary.method, "known_poses");
        assert!(ran.cmd("pose_prior_mapper").is_none());
        let t = ran.cmd("point_triangulator").expect("triangulated");
        assert_eq!(value(t, "--Mapper.min_num_matches"), Some("30"));
        // Matching still drops pairs that contradict the trajectory.
        assert!(ran.cmd("matches_importer").is_some());
    }
}

#[test]
fn calibrated_intrinsics_stay_fixed_in_the_hierarchical_mapper() {
    let ran = run_with(
        &unposed(frames(4, true)),
        SfmOptions {
            hierarchical: true,
            ..REFINE
        },
    );
    let args = ran
        .cmd("hierarchical_mapper")
        .expect("hierarchical mapper ran");
    assert_eq!(value(args, "--Mapper.ba_refine_focal_length"), Some("0"));
    assert_eq!(value(args, "--Mapper.min_num_matches"), Some("30"));
}

/// The stereo mounts of the sim Visor rig (body FLU, OpenCV cameras 10 cm apart).
fn stereo_mount(y: f64) -> geom::Mat4 {
    [
        0.0, 0.0, 1.0, 0.1, -1.0, 0.0, 0.0, y, 0.0, -1.0, 0.0, 0.3, 0.0, 0.0, 0.0, 1.0,
    ]
}

/// Six unposed stereo frames, exported by `prepare` as three rig instants;
/// `cam_fr` at `fr_calibrated`.
fn stereo_frames(fr_calibrated: bool) -> Vec<FrameRecord> {
    let mut out = Vec::new();
    for (cam, y, calibrated) in [("cam_fl", 0.05, true), ("cam_fr", -0.05, fr_calibrated)] {
        for t in [300_000_000u64, 9_900_000_000, 10_200_000_000] {
            out.push(FrameRecord {
                name: crate::recon::prepare::frame_name(cam, t),
                camera_id: cam.into(),
                t_ns: t,
                t_world_cam: None,
                t_body_cam: Some(stereo_mount(y)),
                calibrated,
                rig_instant: Some(t),
                ..frame(0, true)
            });
        }
    }
    out
}

#[test]
fn frame_names_sort_in_time_order() {
    let a = crate::recon::prepare::frame_name("cam_fl", 10_152_000_000);
    let b = crate::recon::prepare::frame_name("cam_fl", 100_152_000_000);
    assert!(a < b, "{a} vs {b}");
    assert_eq!(a, "cam_fl/00000000010152000000.png");
}

#[test]
fn a_calibrated_stereo_rig_is_configured_before_matching_and_kept_in_mapping() {
    let ran = run_with(&stereo_frames(true), REFINE);
    assert_eq!(ran.summary.rig_frames_paired, 3);
    assert!(ran.pos("rig_configurator") < ran.pos("exhaustive_matcher"));
    let m = ran.cmd("exhaustive_matcher").unwrap();
    assert_eq!(value(m, "--FeatureMatching.rig_verification"), Some("1"));
    let config = ran.rig_config.clone().expect("rig config written");
    let cams = config[0]["cameras"].as_array().unwrap();
    assert_eq!(cams[0]["image_prefix"], "cam_fl/");
    assert_eq!(cams[0]["ref_sensor"], true);
    assert_eq!(cams[1]["image_prefix"], "cam_fr/");
    // cam_fr sits 10 cm to the right of cam_fl (+x of the OpenCV camera).
    let t: Vec<f64> = serde_json::from_value(cams[1]["cam_from_rig_translation"].clone()).unwrap();
    let q: Vec<f64> = serde_json::from_value(cams[1]["cam_from_rig_rotation"].clone()).unwrap();
    assert!(
        (t[0] + 0.1).abs() < 1e-12 && t[1].abs() < 1e-12 && t[2].abs() < 1e-12,
        "{t:?}"
    );
    assert!((q[0] - 1.0).abs() < 1e-12, "{q:?}");
    let mapper = ran.cmd("mapper").unwrap();
    assert_eq!(
        value(mapper, "--Mapper.ba_refine_sensor_from_rig"),
        Some("0")
    );
}

#[test]
fn a_posed_stereo_rig_is_configured_too() {
    let posed: Vec<FrameRecord> = stereo_frames(true)
        .into_iter()
        .enumerate()
        .map(|(i, f)| FrameRecord {
            t_world_cam: Some(geom::from_quat_trans(
                [1.0, 0.0, 0.0, 0.0],
                [i as f64 * 0.5, 0.0, 0.0],
            )),
            ..f
        })
        .collect();
    let ran = run_with(&posed, REFINE);
    assert_eq!(
        (ran.summary.rig_frames_paired, ran.summary.method.as_str()),
        (3, "pose_prior_mapper")
    );
    assert!(ran.pos("rig_configurator") < ran.pos("matches_importer"));
    let imp = ran.cmd("matches_importer").unwrap();
    assert_eq!(value(imp, "--FeatureMatching.rig_verification"), Some("1"));
    let m = ran.cmd("pose_prior_mapper").unwrap();
    assert_eq!(value(m, "--Mapper.ba_refine_sensor_from_rig"), Some("0"));
    // Triangulating against fixed poses runs without the rig.
    let fixed = run_with(
        &posed,
        SfmOptions {
            fixed_poses: true,
            ..REFINE
        },
    );
    assert_eq!(fixed.summary.rig_frames_paired, 0);
    assert!(fixed.cmd("rig_configurator").is_none());
    let imp = fixed.cmd("matches_importer").unwrap();
    assert!(value(imp, "--FeatureMatching.rig_verification").is_none());
}

#[test]
fn no_rig_for_an_uncalibrated_camera_or_unpaired_frames() {
    let ran = run_with(&stereo_frames(false), REFINE);
    assert_eq!(ran.summary.rig_frames_paired, 0);
    assert!(ran.rig_config.is_none());
    assert!(ran.cmd("rig_configurator").is_none());
    let m = ran.cmd("exhaustive_matcher").unwrap();
    assert!(value(m, "--FeatureMatching.rig_verification").is_none());
    // Exported outside a rig, or an instant short of a camera.
    let unpaired: Vec<FrameRecord> = stereo_frames(true)
        .into_iter()
        .map(|f| FrameRecord {
            rig_instant: None,
            ..f
        })
        .collect();
    let mut short = stereo_frames(true);
    short.pop();
    for frames in [unpaired, short] {
        let ran = run_with(&frames, REFINE);
        assert_eq!(ran.summary.rig_frames_paired, 0);
        assert!(ran.rig_config.is_none());
        assert!(ran.cmd("rig_configurator").is_none());
    }
}
