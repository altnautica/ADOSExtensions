use super::*;
use crate::recon::preset::Preset;

const PERIOD_NS: u64 = 33_333_333;
/// `cam_fr` exposes this long after `cam_fl` (unsynchronised stereo).
const SKEW_NS: u64 = 2_000_000;

/// A stereo session of `n` instants `period` apart, time-sorted like the
/// session index; `cam_fr` misses the frames at `missing`, `seq` gives
/// trigger sequences.
fn stereo(n: u64, period: u64, missing: &[u64], seq: bool) -> Vec<FrameRef> {
    let mut out = Vec::new();
    for k in 0..n {
        let t = 1_000_000_000 + k * period;
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

fn spec(max_images: usize) -> PresetSpec {
    PresetSpec {
        max_images,
        ..Preset::Preview.spec()
    }
}

/// Every set holds one frame of each camera, from the same instant.
fn assert_paired(sel: &Selection) {
    assert!(sel.rig);
    for s in &sel.sets {
        let cams: Vec<&str> = s.frames.iter().map(|f| f.cam.as_str()).collect();
        assert_eq!(cams, ["cam_fl", "cam_fr"], "{s:?}");
        assert_eq!(s.t_ns, s.frames[0].t_ns);
        assert_eq!(s.frames[1].t_ns, s.frames[0].t_ns + SKEW_NS, "{s:?}");
    }
}

#[test]
fn a_frame_missed_mid_session_keeps_every_later_instant_paired() {
    let gap_t = 1_000_000_000 + 37 * PERIOD_NS;
    for seq in [false, true] {
        let frames = stereo(120, PERIOD_NS, &[37], seq);
        let sel = select_keyframes(&frames, None, &spec(1000));
        assert_paired(&sel);
        assert_eq!(sel.incomplete, 1, "seq {seq}");
        assert!(sel.sets.iter().all(|s| s.t_ns != gap_t));
        // Time-only selection (no poses) every 250 ms keeps going past the gap.
        let late = sel.sets.iter().filter(|s| s.t_ns > gap_t).count();
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
    let sel = select_keyframes(&frames, None, &spec(1000));
    assert_paired(&sel);
    assert_eq!(sel.incomplete, 0);
    assert!(!sel.sets.is_empty());
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
    let sel = select_keyframes(&frames, None, &spec(1000));
    assert!(!sel.rig);
    assert_eq!(sel.incomplete, 0);
    assert!(sel.sets.iter().all(|s| s.frames.len() == 1));
    for cam in ["cam_fl", "cam_fr"] {
        assert!(sel.sets.iter().any(|s| s.frames[0].cam == cam));
    }
}

#[test]
fn thinning_to_the_image_cap_keeps_pairs_together() {
    // Instants 300 ms apart: every one passes the time criterion.
    let frames = stereo(200, 300_000_000, &[], false);
    let sel = select_keyframes(&frames, None, &spec(21));
    assert_eq!(sel.sets.len(), 10);
    assert_paired(&sel);
}

fn record(cam: &str, instant: u64) -> FrameRecord {
    FrameRecord {
        name: frame_name(cam, instant),
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
        rig_instant: Some(instant),
    }
}

#[test]
fn a_blurred_frame_on_one_camera_drops_its_whole_instant() {
    let dir = tempfile::tempdir().unwrap();
    let mut records = Vec::new();
    for k in 0..5u64 {
        for cam in ["cam_fl", "cam_fr"] {
            let blurred = cam == "cam_fr" && k == 2;
            records.push((record(cam, k), if blurred { 1.0 } else { 10.0 }));
        }
    }
    let (blurred, rig_dropped) = drop_unusable(dir.path(), &mut records, 2);
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
