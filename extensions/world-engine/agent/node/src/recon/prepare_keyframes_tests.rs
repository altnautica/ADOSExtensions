use super::*;
use crate::recon::preset::Preset;

const PERIOD_NS: u64 = 33_333_333;
/// `cam_fr` exposes this long after `cam_fl` (unsynchronised stereo).
const SKEW_NS: u64 = 2_000_000;
const T0: u64 = 1_000_000_000;

/// A stereo session of `n` instants `period` apart, time-sorted like the
/// session index; `cam_fr` misses the frames at `missing`, `seq` gives
/// trigger sequences.
fn stereo(n: u64, period: u64, missing: &[u64], seq: bool) -> Vec<FrameRef> {
    let mut out = Vec::new();
    for k in 0..n {
        let t = T0 + k * period;
        out.push(FrameRef {
            cam: "cam_fl".into(),
            t_ns: t,
            trigger_seq: seq.then_some(k),
        });
        if !missing.contains(&k) {
            out.push(FrameRef {
                cam: "cam_fr".into(),
                t_ns: t + SKEW_NS,
                trigger_seq: seq.then_some(k),
            });
        }
    }
    out
}

/// One camera at 30 Hz for `n` frames.
fn mono(n: u64) -> Vec<FrameRef> {
    (0..n)
        .map(|k| FrameRef {
            cam: "cam0".into(),
            t_ns: T0 + k * PERIOD_NS,
            trigger_seq: None,
        })
        .collect()
}

/// A body walking +x at `speed` m/s, sampled every 10 ms over `secs`.
fn walk(speed: f64, secs: f64) -> PoseTrack {
    let samples = (0..=(secs * 100.0) as u64)
        .map(|k| {
            let t = T0 + k * 10_000_000;
            let x = speed * (k as f64 * 0.01);
            (
                t,
                geom::from_quat_trans([1.0, 0.0, 0.0, 0.0], [x, 0.0, 0.0]),
            )
        })
        .collect();
    PoseTrack {
        source: "vio".into(),
        samples,
    }
}

fn spec(max_images: usize) -> PresetSpec {
    PresetSpec {
        max_images,
        ..Preset::Preview.spec()
    }
}

/// The instant that opened each window.
fn firsts(sel: &Selection) -> Vec<&FrameSet> {
    sel.windows.iter().map(|w| &w[0]).collect()
}

/// Every set holds one frame of each camera, from the same instant.
fn assert_paired(sel: &Selection) {
    assert!(sel.rig);
    for s in sel.windows.iter().flatten() {
        let cams: Vec<&str> = s.frames.iter().map(|f| f.cam.as_str()).collect();
        assert_eq!(cams, ["cam_fl", "cam_fr"], "{s:?}");
        assert_eq!(s.t_ns, s.frames[0].t_ns);
        assert_eq!(s.frames[1].t_ns, s.frames[0].t_ns + SKEW_NS, "{s:?}");
    }
}

#[test]
fn a_frame_missed_mid_session_keeps_every_later_instant_paired() {
    let gap_t = T0 + 37 * PERIOD_NS;
    for seq in [false, true] {
        let frames = stereo(120, PERIOD_NS, &[37], seq);
        let sel = select_keyframes(&frames, None, None, &spec(1000));
        assert_paired(&sel);
        assert_eq!(sel.incomplete, 1, "seq {seq}");
        assert!(sel.windows.iter().flatten().all(|s| s.t_ns != gap_t));
        // Time-only selection (no poses) every 250 ms keeps going past the gap.
        let late = firsts(&sel).iter().filter(|s| s.t_ns > gap_t).count();
        assert!(late >= 9, "seq {seq}: {late} instants after the gap");
    }
}

#[test]
fn cameras_with_their_own_trigger_counters_pair_by_time() {
    // Each camera counts its own triggers: no sequence holds both cameras.
    let mut frames = stereo(40, PERIOD_NS, &[], true);
    for f in frames.iter_mut().filter(|f| f.cam == "cam_fr") {
        f.trigger_seq = f.trigger_seq.map(|s| s + 10_000);
    }
    let sel = select_keyframes(&frames, None, None, &spec(1000));
    assert_paired(&sel);
    assert_eq!(sel.incomplete, 0);
    assert!(!sel.windows.is_empty());
}

#[test]
fn one_trigger_sequence_300_ms_apart_is_not_an_instant() {
    // Same sequences at 25 Hz, but cam_fr stamped 300 ms (7.5 frames) late:
    // neither the sequence nor the time pairs them, so the frames leave the rig.
    const PERIOD: u64 = 40_000_000;
    let mut frames = stereo(40, PERIOD, &[], true);
    for f in frames.iter_mut().filter(|f| f.cam == "cam_fr") {
        f.t_ns += 300_000_000 - SKEW_NS;
    }
    frames.sort_by_key(|f| f.t_ns);
    let sel = select_keyframes(&frames, None, None, &spec(1000));
    assert!(!sel.rig);
    assert_eq!(sel.incomplete, 0);
    // Every window holds one camera's frames only.
    for w in &sel.windows {
        assert!(w.iter().all(|s| s.frames.len() == 1));
        assert!(w.iter().all(|s| s.frames[0].cam == w[0].frames[0].cam));
    }
    for cam in ["cam_fl", "cam_fr"] {
        assert!(firsts(&sel).iter().any(|s| s.frames[0].cam == cam));
    }
}

#[test]
fn thinning_to_the_image_cap_keeps_pairs_together() {
    // Instants 300 ms apart: every one passes the time criterion.
    let frames = stereo(200, 300_000_000, &[], false);
    let sel = select_keyframes(&frames, None, None, &spec(21));
    assert_eq!(sel.windows.len(), 10);
    assert_paired(&sel);
}

#[test]
fn sharpest_frame_in_window_is_picked() {
    // 0.2 m/s past a 0.10 m baseline: a window opens every 0.5 s and spans
    // the 0.3 s after it.
    let frames = mono(90);
    let track = walk(0.2, 3.5);
    let hq = Preset::Hq.spec();
    let sel = select_keyframes(&frames, Some(&track), None, &hq);
    let second = &sel.windows[1];
    let t_open = second[0].t_ns;
    assert!(second.iter().all(|s| s.t_ns - t_open <= SHARP_WINDOW_NS));
    assert!(second.len() >= 8, "{} candidates", second.len());
    assert!(sel.windows[2][0].t_ns > second.last().unwrap().t_ns);

    // The opening frame is blurred, one 0.2 s later is sharp, and a sharper
    // one 0.4 s later lies outside the window.
    let t_sharp = t_open + 6 * PERIOD_NS;
    let t_outside = t_open + 12 * PERIOD_NS;
    let mut scores: HashMap<(String, u64), f64> = frames
        .iter()
        .map(|f| ((f.cam.clone(), f.t_ns), 10.0))
        .collect();
    scores.insert(("cam0".into(), t_open), 2.0);
    scores.insert(("cam0".into(), t_sharp), 30.0);
    scores.insert(("cam0".into(), t_outside), 100.0);
    let medians = camera_medians(scores.iter().map(|((c, _), s)| (c.as_str(), *s)));
    let picks = pick_sharpest(&sel, &scores, &medians);
    assert_eq!(picks.len(), sel.windows.len());
    assert_eq!(picks[1].t_ns, t_sharp);
    // A window with every candidate equal keeps its opening instant.
    assert_eq!(picks[0].t_ns, sel.windows[0][0].t_ns);

    // The pick survives the blur drop; a pick that is blurred even at its
    // best goes.
    let mut records: Vec<(FrameRecord, f64)> = picks
        .iter()
        .map(|s| {
            let score = scores[&("cam0".to_string(), s.t_ns)];
            (record("cam0", s.t_ns, None), score)
        })
        .collect();
    records[2].1 = 3.0;
    let dir = tempfile::tempdir().unwrap();
    let (blurred, _) = drop_unusable(dir.path(), &mut records, 0, &medians);
    assert_eq!(blurred, 1);
    assert!(records.iter().any(|(r, _)| r.t_ns == t_sharp));
    assert!(records.iter().all(|(r, _)| r.t_ns != picks[2].t_ns));
}

#[test]
fn baseline_scales_with_depth_and_clamps() {
    assert!((baseline_m(Some(2.5), 0.10) - 0.20).abs() < 1e-12);
    assert_eq!(baseline_m(Some(0.2), 0.10), BASELINE_MIN_M);
    assert_eq!(baseline_m(Some(30.0), 0.10), BASELINE_MAX_M);
    assert_eq!(baseline_m(None, 0.10), 0.10);
    assert_eq!(baseline_m(Some(f64::NAN), 0.10), 0.10);

    // The nearest scan within a second sets the depth.
    let mut depth = DepthTrack::default();
    depth.push_scan(T0, [[5.0, 0.0, 0.0], [0.0, 5.0, 0.0], [3.0, 4.0, 0.0]]);
    depth.push_scan(T0 + 2_000_000_000, [[1.0, 0.0, 0.0], [0.0, 0.0, 0.0]]);
    assert_eq!(depth.near(T0 + 400_000_000), Some(5.0));
    assert_eq!(depth.near(T0 + 1_700_000_000), Some(1.0));
    assert_eq!(depth.near(T0 + 5_000_000_000), None);

    // 1 m/s for 3 s: a 5 m deep aisle spaces windows 0.40 m apart, a 1 m
    // deep room 0.08 m apart, no depth at the preset's 0.10 m.
    let frames = mono(90);
    let track = walk(1.0, 3.5);
    let hq = Preset::Hq.spec();
    let deep = flat_depth(5.0);
    let near = flat_depth(1.0);
    let n = |d: Option<&DepthTrack>| {
        select_keyframes(&frames, Some(&track), d, &hq)
            .windows
            .len()
    };
    let (n_deep, n_near, n_none) = (n(Some(&deep)), n(Some(&near)), n(None));
    assert!((7..=9).contains(&n_deep), "{n_deep}");
    assert!(
        n_near > n_none && n_none > n_deep,
        "{n_near} {n_none} {n_deep}"
    );
}

/// A scan every 100 ms over 4 s at `range` m.
fn flat_depth(range: f64) -> DepthTrack {
    let mut d = DepthTrack::default();
    for k in 0..40 {
        d.push_scan(T0 + k * 100_000_000, [[range, 0.0, 0.0]]);
    }
    d
}

fn record(cam: &str, instant: u64, rig_instant: Option<u64>) -> FrameRecord {
    FrameRecord {
        name: super::super::frame_name(cam, instant),
        camera_id: cam.into(),
        t_ns: instant,
        t_world_cam: None,
        width: 4,
        height: 4,
        fx: 2.0,
        fy: 2.0,
        cx: 2.0,
        cy: 2.0,
        model: ados_capture_contract::CameraModelModel::Pinhole,
        dist: vec![],
        calibrated: true,
        t_body_cam: None,
        rig_instant,
    }
}

#[test]
fn a_blurred_frame_on_one_camera_drops_its_whole_instant() {
    let dir = tempfile::tempdir().unwrap();
    let mut records = Vec::new();
    for k in 0..5u64 {
        for cam in ["cam_fl", "cam_fr"] {
            let blurred = cam == "cam_fr" && k == 2;
            records.push((record(cam, k, Some(k)), if blurred { 1.0 } else { 10.0 }));
        }
    }
    let medians = camera_medians(records.iter().map(|(r, s)| (r.camera_id.as_str(), *s)));
    let (blurred, rig_dropped) = drop_unusable(dir.path(), &mut records, 2, &medians);
    assert_eq!((blurred, rig_dropped), (1, 1));
    assert_eq!(records.len(), 8);
    for k in 0..5u64 {
        let n = records
            .iter()
            .filter(|(r, _)| r.rig_instant == Some(k))
            .count();
        assert_eq!(n, if k == 2 { 0 } else { 2 }, "instant {k}");
    }
}
