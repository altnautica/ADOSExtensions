use std::fs;

use ados_capture_contract::CameraModelModel;

use super::*;
use crate::recon::prepare::FrameRecord;
use crate::recon::preset::Preset;
use crate::recon::products_stage::ProductsInput;
use crate::recon::StandInRunner;

#[test]
fn look_at_points_the_optical_axis_at_the_target_with_y_down() {
    let t = look_at([0.0, 0.0, 1.6], [3.0, 0.0, 1.6]);
    // z (third column) is the forward axis.
    assert_eq!([t[2], t[6], t[10]], [1.0, 0.0, 0.0]);
    // y (second column) points down.
    assert_eq!([t[1], t[5], t[9]], [0.0, 0.0, -1.0]);
    // x = y × z is right-handed: looking +x with z up, right is -y.
    assert_eq!([t[0], t[4], t[8]], [0.0, -1.0, 0.0]);
    assert_eq!([t[3], t[7], t[11]], [0.0, 0.0, 1.6]);
}

fn frame(cam: &str, t_ns: u64) -> FrameRecord {
    FrameRecord {
        name: format!("{cam}/{t_ns:020}.png"),
        camera_id: cam.into(),
        t_ns,
        t_world_cam: None,
        width: 64,
        height: 48,
        fx: 50.0,
        fy: 50.0,
        cx: 32.0,
        cy: 24.0,
        model: CameraModelModel::Pinhole,
        dist: Vec::new(),
        calibrated: true,
        t_body_cam: None,
        rig_instant: None,
    }
}

/// A walk east along y = 2 at 1.6 m eye height, 0.5 m per keyframe, a
/// stereo pair per instant, every camera looking north: COLMAP's
/// `images.txt` (camera-from-world) and the dataset's `frames.json`.
fn walk(job: &Path, steps: usize) {
    let txt = job.join("sfm").join("txt");
    fs::create_dir_all(&txt).unwrap();
    fs::create_dir_all(job.join("dataset")).unwrap();
    // Camera looking +y (north) with y down: x = east, y = -up, z = north.
    let r_wc = [[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]];
    // R_cw = R_wcᵀ as a quaternion (w, x, y, z): a +90° turn about x.
    let q = [
        std::f64::consts::FRAC_1_SQRT_2,
        std::f64::consts::FRAC_1_SQRT_2,
        0.0,
        0.0,
    ];
    let mut images = String::from("# images\n");
    let mut frames = Vec::new();
    for i in 0..steps {
        let c = [i as f64 * 0.5, 2.0, 1.6];
        // t_cw = -R_cw c, with R_cw = r_wcᵀ.
        let t: Vec<f64> = (0..3)
            .map(|row| -(0..3).map(|k| r_wc[k][row] * c[k]).sum::<f64>())
            .collect();
        for (n, cam) in ["cam0", "cam1"].iter().enumerate() {
            let f = frame(cam, 1_000 + i as u64);
            images.push_str(&format!(
                "{} {} {} {} {} {} {} {} 1 {}\n\n",
                2 * i + n + 1,
                q[0],
                q[1],
                q[2],
                q[3],
                t[0],
                t[1],
                t[2],
                f.name
            ));
            frames.push(f);
        }
    }
    fs::write(txt.join("images.txt"), images).unwrap();
    fs::write(
        job.join("dataset").join("frames.json"),
        serde_json::to_vec(&frames).unwrap(),
    )
    .unwrap();
}

#[test]
fn tour_writer_emits_esdf_and_thinned_nodes() {
    let tmp = tempfile::tempdir().unwrap();
    let job = tmp.path().join("job");
    walk(&job, 20);
    let viewer = tmp.path().join("viewer");
    fs::create_dir_all(&viewer).unwrap();
    fs::write(viewer.join("index.html"), "<!doctype html>").unwrap();
    let out = job.join("out");
    fs::create_dir_all(&out).unwrap();
    fs::write(out.join("scene.spz"), [1u8]).unwrap();
    fs::write(out.join("occupancy.esdf"), [0u8; 8]).unwrap();
    fs::write(out.join("occupancy.json"), b"{}").unwrap();

    let spec = Preset::Tour.spec();
    let input = ProductsInput {
        job_dir: &job,
        session_id: "s",
        title: "Walk",
        anchor: None,
        generation: 1,
        spec: &spec,
        tour_viewer: &viewer,
    };
    let kf = keyframes(&job);
    assert_eq!(kf.len(), 20, "one keyframe per capture instant");
    let a = build_tour(
        &StandInRunner::default(),
        &input,
        &out,
        None,
        &kf,
        &Placement::identity(None),
        &mut |_| {},
    )
    .unwrap();
    assert_eq!(a.meta["nodes"], 7);

    let tour: serde_json::Value =
        serde_json::from_slice(&fs::read(out.join("tour").join("tour.json")).unwrap()).unwrap();
    assert_eq!(tour["esdf"], "occupancy.esdf");
    assert!(out.join("tour").join("occupancy.esdf").is_file());
    assert!(out.join("tour").join("occupancy.json").is_file());
    let nodes = tour["nodes"].as_array().unwrap();
    let xs: Vec<f64> = nodes
        .iter()
        .map(|n| n["position"][0].as_f64().unwrap())
        .collect();
    assert_eq!(xs, [0.0, 1.5, 3.0, 4.5, 6.0, 7.5, 9.0]);
    for n in nodes {
        assert_eq!(n["position"][1], 2.0);
        assert_eq!(n["position"][2], 1.6);
        assert!((n["yaw_deg"].as_f64().unwrap() - 90.0).abs() < 1e-6, "{n}");
        assert!(n["id"].as_str().is_some_and(|s| !s.is_empty()));
    }

    // Without the ESDF header the grid is useless to the viewer: no `esdf`.
    fs::remove_file(out.join("occupancy.json")).unwrap();
    build_tour(
        &StandInRunner::default(),
        &input,
        &out,
        None,
        &kf,
        &Placement::identity(None),
        &mut |_| {},
    )
    .unwrap();
    let tour: serde_json::Value =
        serde_json::from_slice(&fs::read(out.join("tour").join("tour.json")).unwrap()).unwrap();
    assert!(tour["esdf"].is_null());
    assert!(!out.join("tour").join("occupancy.esdf").exists());
}
