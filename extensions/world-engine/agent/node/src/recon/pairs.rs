//! Image pairs for COLMAP's `matches_importer` on a posed session.
//!
//! A sequential matcher pairs by name order and its quadratic overlap and
//! vocabulary-tree loop detection reach far back in time, which is how two
//! identical warehouse bays end up matched. With a pose prior the candidate
//! set is chosen geometrically instead: neighbours in time, views close in
//! space that look the same way, and the cameras of one rig instant.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::fs;
use std::path::Path;

use super::prepare::FrameRecord;
use super::StageError;

/// Seconds of capture a sequential window spans.
const WINDOW_S: f64 = 2.0;
/// The window never shrinks below or grows past these many keyframes.
const MIN_WINDOW: usize = 20;
const MAX_WINDOW: usize = 60;

/// How pairs are chosen from poses.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PairCfg {
    /// Keyframe instants apart in time order that are always paired.
    pub window: usize,
    /// Camera centres at most this far apart (metres) may pair...
    pub radius_m: f64,
    /// ...when their principal axes are less than this apart (degrees).
    pub max_axis_deg: f64,
}

impl PairCfg {
    /// The default spatial rule with time window `window`.
    pub fn new(window: usize) -> Self {
        Self {
            window,
            radius_m: 4.0,
            max_axis_deg: 30.0,
        }
    }
}

/// The capture instant a frame belongs to (its rig instant when exported in
/// one, else its own time).
fn instant(f: &FrameRecord) -> u64 {
    f.rig_instant.unwrap_or(f.t_ns)
}

/// The sequential window, in keyframes: 2 s of capture at the session's
/// keyframe rate (instants of the busiest camera over the time they span),
/// clamped to [20, 60].
pub fn window(frames: &[FrameRecord]) -> usize {
    let mut per_cam: BTreeMap<&str, (usize, u64, u64)> = BTreeMap::new();
    for f in frames {
        let t = instant(f);
        let e = per_cam.entry(&f.camera_id).or_insert((0, t, t));
        e.0 += 1;
        e.1 = e.1.min(t);
        e.2 = e.2.max(t);
    }
    let rate = per_cam
        .values()
        .filter(|(n, a, b)| *n > 1 && b > a)
        .map(|(n, a, b)| *n as f64 / ((b - a) as f64 * 1e-9))
        .fold(0.0, f64::max);
    ((WINDOW_S * rate).round() as usize).clamp(MIN_WINDOW, MAX_WINDOW)
}

/// A camera's centre and principal axis (OpenCV +z) in the world.
type CentreAxis = ([f64; 3], [f64; 3]);

fn centre_axis(t: &[f64; 16]) -> CentreAxis {
    ([t[3], t[7], t[11]], [t[2], t[6], t[10]])
}

/// The pairs to match, each `(a, b)` with `a < b` by name, sorted. A pair is
/// in when its instants are at most `window` apart in time order, when both
/// frames are posed with centres within `radius_m` and principal axes less
/// than `max_axis_deg` apart, or when they are one capture instant seen by
/// two cameras.
pub fn from_poses(frames: &[FrameRecord], cfg: &PairCfg) -> Vec<(String, String)> {
    let instants: Vec<u64> = frames
        .iter()
        .map(instant)
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect();
    let order = |f: &FrameRecord| instants.partition_point(|t| *t < instant(f));
    let cos_max = cfg.max_axis_deg.to_radians().cos();
    let r2 = cfg.radius_m * cfg.radius_m;
    let posed: Vec<(usize, Option<CentreAxis>)> = frames
        .iter()
        .map(|f| (order(f), f.t_world_cam.as_ref().map(centre_axis)))
        .collect();
    let mut out = BTreeSet::new();
    for i in 0..frames.len() {
        for j in i + 1..frames.len() {
            let (a, b) = (&frames[i], &frames[j]);
            if a.name == b.name {
                continue;
            }
            let (oa, pa) = posed[i];
            let (ob, pb) = posed[j];
            let near_in_time = oa.abs_diff(ob) <= cfg.window;
            let same_instant = oa == ob && a.camera_id != b.camera_id;
            let near_in_space = match (pa, pb) {
                (Some((ca, xa)), Some((cb, xb))) => {
                    let d2: f64 = (0..3).map(|k| (ca[k] - cb[k]).powi(2)).sum();
                    let dot: f64 = (0..3).map(|k| xa[k] * xb[k]).sum();
                    d2 <= r2 && dot > cos_max
                }
                _ => false,
            };
            if near_in_time || same_instant || near_in_space {
                let (x, y) = match a.name < b.name {
                    true => (&a.name, &b.name),
                    false => (&b.name, &a.name),
                };
                out.insert((x.clone(), y.clone()));
            }
        }
    }
    out.into_iter().collect()
}

/// Write `pairs` as a `matches_importer` match list (`name1 name2` per line).
pub fn write(path: &Path, pairs: &[(String, String)]) -> Result<(), StageError> {
    let mut s = String::with_capacity(pairs.len() * 64);
    for (a, b) in pairs {
        let _ = writeln!(s, "{a} {b}");
    }
    fs::write(path, s)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::recon::geom;
    use ados_capture_contract::CameraModelModel;

    /// A camera at `c` looking along `yaw_deg` in the world XY plane
    /// (OpenCV camera: +z forward, +y down).
    fn frame(cam: &str, t_ns: u64, c: [f64; 3], yaw_deg: f64) -> FrameRecord {
        let (s, co) = yaw_deg.to_radians().sin_cos();
        // Columns: camera x (right), y (down), z (forward) in the world.
        let t = [
            s, 0.0, co, c[0], -co, 0.0, s, c[1], 0.0, -1.0, 0.0, c[2], 0.0, 0.0, 0.0, 1.0,
        ];
        FrameRecord {
            name: crate::recon::prepare::frame_name(cam, t_ns),
            camera_id: cam.into(),
            t_ns,
            t_world_cam: Some(t),
            width: 64,
            height: 48,
            fx: 40.0,
            fy: 40.0,
            cx: 32.0,
            cy: 24.0,
            model: CameraModelModel::Pinhole,
            dist: vec![],
            calibrated: true,
            t_body_cam: None,
            rig_instant: None,
            prior_sigma_m: None,
        }
    }

    /// Up aisle A (y = 0, looking +x), back down aisle B (y = 4, looking -x)
    /// and up aisle A again, 0.5 m per keyframe at 2 keyframes a second,
    /// stereo at every instant. Returns the frames and each one's leg.
    fn two_aisles() -> (Vec<FrameRecord>, BTreeMap<String, u64>) {
        let (mut out, mut leg) = (Vec::new(), BTreeMap::new());
        let n = 80u64;
        for k in 0..3 * n {
            let t = k * 500_000_000;
            let i = (k % n) as f64 * 0.5;
            let (c, yaw) = match k / n {
                1 => ([(n - 1) as f64 * 0.5 - i, 4.0, 1.5], 180.0),
                _ => ([i, 0.0, 1.5], 0.0),
            };
            for (cam, dy) in [("cam_fl", 0.05), ("cam_fr", -0.05)] {
                let mut f = frame(cam, t, [c[0], c[1] + dy, c[2]], yaw);
                f.rig_instant = Some(t);
                leg.insert(f.name.clone(), k / n);
                out.push(f);
            }
        }
        (out, leg)
    }

    #[test]
    fn pairs_from_poses_keeps_window_radius_and_stereo() {
        let (frames, leg) = two_aisles();
        let cfg = PairCfg::new(window(&frames));
        // 2 keyframes a second: 4 per 2 s, clamped up to 20.
        assert_eq!(cfg.window, MIN_WINDOW);
        let pairs = from_poses(&frames, &cfg);
        let by_name: BTreeMap<&str, &FrameRecord> =
            frames.iter().map(|f| (f.name.as_str(), f)).collect();
        // Every stereo instant is paired across the rig.
        for f in frames.iter().filter(|f| f.camera_id == "cam_fl") {
            let other = f.name.replace("cam_fl", "cam_fr");
            assert!(pairs.contains(&(f.name.clone(), other)), "{}", f.name);
        }
        let mut revisits = 0;
        for (a, b) in &pairs {
            let (fa, fb) = (by_name[a.as_str()], by_name[b.as_str()]);
            let (ca, xa) = centre_axis(fa.t_world_cam.as_ref().unwrap());
            let (cb, xb) = centre_axis(fb.t_world_cam.as_ref().unwrap());
            let dot: f64 = (0..3).map(|k| xa[k] * xb[k]).sum();
            let order = |f: &FrameRecord| f.t_ns / 500_000_000;
            if order(fa).abs_diff(order(fb)) <= cfg.window as u64 {
                continue;
            }
            // Outside the time window only close views looking the same way
            // pair, so nothing pairs across the aisles.
            assert!(geom::dist(ca, cb) <= 4.0, "{a} {b}");
            assert!(dot > 30f64.to_radians().cos(), "{a} {b}");
            assert!((ca[1] - cb[1]).abs() < 1.0, "{a} {b}");
            let legs = [leg[a.as_str()], leg[b.as_str()]];
            assert!(!legs.contains(&1), "{a} {b}");
            revisits += usize::from(legs.contains(&0) && legs.contains(&2));
        }
        // The second pass up aisle A pairs with the first.
        assert!(revisits > 100, "{revisits}");
    }

    #[test]
    fn a_revisit_looking_the_same_way_pairs_outside_the_window() {
        let mut frames: Vec<FrameRecord> = (0..100u64)
            .map(|k| frame("cam", k * 100_000_000, [k as f64 * 0.2, 0.0, 1.5], 0.0))
            .collect();
        // Back at the start, much later, looking the same way: paired. The
        // same place seen the other way round: not.
        frames.push(frame("cam", 60_000_000_000, [0.5, 0.3, 1.5], 10.0));
        frames.push(frame("cam", 61_000_000_000, [0.5, 0.3, 1.5], 180.0));
        let pairs = from_poses(&frames, &PairCfg::new(20));
        let has = |a: &str, b: &str| pairs.iter().any(|(x, y)| x == a && y == b);
        let first = frames[0].name.clone();
        assert!(has(&first, &frames[100].name));
        assert!(!has(&first, &frames[101].name));
        // A frame far along the line, outside the window, does not pair with it.
        assert!(!has(&frames[50].name, &frames[100].name));
    }

    #[test]
    fn window_follows_the_keyframe_rate() {
        // 20 keyframes a second for 10 s: 40 per 2 s.
        let fast: Vec<FrameRecord> = (0..200u64)
            .map(|k| frame("cam", k * 50_000_000, [0.0; 3], 0.0))
            .collect();
        assert_eq!(window(&fast), 40);
        let faster: Vec<FrameRecord> = (0..2000u64)
            .map(|k| frame("cam", k * 10_000_000, [0.0; 3], 0.0))
            .collect();
        assert_eq!(window(&faster), MAX_WINDOW);
        assert_eq!(window(&fast[..1]), MIN_WINDOW);
    }
}
