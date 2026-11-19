//! Keyframe choice for `prepare`: group a session's frames into instants,
//! open a keyframe window wherever the motion since the last one crosses the
//! baseline, rotation or time threshold, then keep the sharpest instant of
//! each window and drop a pick only when even it is blurred.
//!
//! The baseline scales with the scene: `0.08 × median depth`, clamped to
//! [[`BASELINE_MIN_M`], [`BASELINE_MAX_M`]], where the depth is the median
//! range of the LiDAR scan nearest the instant; without a LiDAR scan the
//! preset's baseline holds.

use std::collections::{BTreeMap, HashMap, HashSet};
use std::fs;
use std::path::Path;

use super::super::geom::{self, Mat4};
use super::super::preset::PresetSpec;
use super::super::session_data::{FrameRef, PoseTrack};
use super::FrameRecord;

/// A frame is dropped when its blur score is under this fraction of its
/// camera's median.
const BLUR_KEEP_FRACTION: f64 = 0.35;
/// A multi-camera session is selected as a rig only while at least this share
/// of its sparsest camera's frames fall in instants that hold every camera.
const RIG_MIN_PAIRED_SHARE: f64 = 0.5;
/// A keyframe window spans this long after the instant that opened it.
pub const SHARP_WINDOW_NS: u64 = 300_000_000;
/// Baseline per metre of median scene depth.
pub const BASELINE_PER_DEPTH: f64 = 0.08;
pub const BASELINE_MIN_M: f64 = 0.05;
pub const BASELINE_MAX_M: f64 = 0.50;
/// A LiDAR scan sets the scene depth of instants within this of it.
const DEPTH_MATCH_NS: u64 = 1_000_000_000;

/// The frames of one time instant: one per rig camera, or, outside a rig,
/// one camera's frame.
#[derive(Debug, Clone, PartialEq)]
pub struct FrameSet {
    /// The reference (first by id) camera's frame time; a rig instant's
    /// frames are all named by it, each keeping its own time in its record.
    pub t_ns: u64,
    /// Ordered by camera.
    pub frames: Vec<FrameRef>,
}

/// The keyframe windows chosen from a session.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Selection {
    /// Per keyframe, the instant that crossed a threshold first, then the
    /// later instants of the same camera (or rig) within [`SHARP_WINDOW_NS`]
    /// and before the next window, any of which may stand in for it.
    pub windows: Vec<Vec<FrameSet>>,
    /// The sets are rig instants holding every camera's frame.
    pub rig: bool,
    /// Recorded instants dropped because a camera had no frame in them.
    pub incomplete: usize,
}

/// Median scene depth over time: one median range per LiDAR scan.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct DepthTrack {
    samples: Vec<(u64, f64)>,
}

impl DepthTrack {
    /// Record the scan at `t_ns` from its points' ranges; a scan with no
    /// valid point records nothing.
    pub fn push_scan(&mut self, t_ns: u64, points: impl IntoIterator<Item = [f64; 3]>) {
        let mut r: Vec<f64> = points
            .into_iter()
            .map(|p| (p[0] * p[0] + p[1] * p[1] + p[2] * p[2]).sqrt())
            .filter(|r| r.is_finite() && *r > 1e-3)
            .collect();
        if r.is_empty() {
            return;
        }
        let mid = r.len() / 2;
        let (_, m, _) = r.select_nth_unstable_by(mid, f64::total_cmp);
        let m = *m;
        let at = self.samples.partition_point(|(t, _)| *t <= t_ns);
        self.samples.insert(at, (t_ns, m));
    }

    /// The median range of the scan nearest `t_ns`, when one is within
    /// [`DEPTH_MATCH_NS`].
    pub fn near(&self, t_ns: u64) -> Option<f64> {
        let i = self.samples.partition_point(|(t, _)| *t < t_ns);
        [i.checked_sub(1), Some(i)]
            .into_iter()
            .flatten()
            .filter_map(|j| self.samples.get(j))
            .map(|(t, d)| (t.abs_diff(t_ns), *d))
            .filter(|(dt, _)| *dt <= DEPTH_MATCH_NS)
            .min_by_key(|(dt, _)| *dt)
            .map(|(_, d)| d)
    }

    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }
}

/// The keyframe baseline at median scene depth `depth_m`, or `fallback_m`
/// without a depth.
pub fn baseline_m(depth_m: Option<f64>, fallback_m: f64) -> f64 {
    match depth_m {
        Some(d) if d.is_finite() && d > 0.0 => {
            (BASELINE_PER_DEPTH * d).clamp(BASELINE_MIN_M, BASELINE_MAX_M)
        }
        _ => fallback_m,
    }
}

/// Choose keyframe windows from the time-sorted `frames`: group a
/// multi-camera session into instants and keep only those holding every
/// camera (see the `prepare` module docs for the fallback), open a window at
/// each instant far enough from the last window's in position, rotation or
/// time (per camera outside a rig), then thin uniformly to the preset's image
/// cap, whole windows at a time.
pub fn select_keyframes(
    frames: &[FrameRef],
    poses: Option<&PoseTrack>,
    depth: Option<&DepthTrack>,
    spec: &PresetSpec,
) -> Selection {
    let mut cams: Vec<&str> = frames.iter().map(|f| f.cam.as_str()).collect();
    cams.sort_unstable();
    cams.dedup();
    let (mut sets, mut rig, mut incomplete) = (Vec::new(), false, 0);
    if cams.len() >= 2 {
        let sparsest = cams
            .iter()
            .map(|c| frames.iter().filter(|f| f.cam == *c).count())
            .min()
            .unwrap_or(0);
        let period = frame_period(frames);
        let by_seq = frames
            .iter()
            .all(|f| f.trigger_seq.is_some())
            .then(|| group_by_trigger(frames, period));
        for grouped in by_seq
            .into_iter()
            .chain(std::iter::once_with(|| group_by_time(frames, period)))
        {
            let (complete, partial): (Vec<FrameSet>, Vec<FrameSet>) = grouped
                .into_iter()
                .partition(|s| s.frames.len() == cams.len());
            if complete.len() as f64 >= RIG_MIN_PAIRED_SHARE * sparsest as f64 {
                (sets, rig, incomplete) = (complete, true, partial.len());
                break;
            }
        }
    }
    if !rig {
        sets = frames
            .iter()
            .map(|f| FrameSet {
                t_ns: f.t_ns,
                frames: vec![f.clone()],
            })
            .collect();
    }
    // A rig selects instants together; otherwise each camera on its own.
    fn set_key(rig: bool, s: &FrameSet) -> &str {
        if rig {
            ""
        } else {
            s.frames[0].cam.as_str()
        }
    }
    let min_rot = spec.min_rotation_deg.to_radians();
    let mut last: HashMap<&str, (u64, Option<Mat4>)> = HashMap::new();
    let mut picked: Vec<usize> = Vec::new();
    for (i, s) in sets.iter().enumerate() {
        let pose = poses.and_then(|p| p.at(s.t_ns));
        let take = match last.get(set_key(rig, s)) {
            None => true,
            Some((t_prev, prev)) => {
                let dt = s.t_ns.saturating_sub(*t_prev);
                let step = baseline_m(depth.and_then(|d| d.near(s.t_ns)), spec.min_baseline_m);
                match (prev, pose) {
                    (Some(a), Some(b)) => {
                        geom::dist(geom::translation(a), geom::translation(&b)) >= step
                            || geom::rotation_angle(a, &b) >= min_rot
                            || dt >= 2_000_000_000
                    }
                    _ => dt >= 250_000_000,
                }
            }
        };
        if take {
            last.insert(set_key(rig, s), (s.t_ns, pose));
            picked.push(i);
        }
    }
    let per_set = if rig { cams.len() } else { 1 };
    let cap = (spec.max_images / per_set).max(1);
    if picked.len() > cap {
        let stride = picked.len() as f64 / cap as f64;
        picked = (0..cap)
            .map(|i| picked[(i as f64 * stride) as usize])
            .collect();
    }
    // Each window runs to SHARP_WINDOW_NS or the next window of its key.
    let mut windows = Vec::with_capacity(picked.len());
    for (n, &i) in picked.iter().enumerate() {
        let k = set_key(rig, &sets[i]);
        let end = picked[n + 1..]
            .iter()
            .copied()
            .find(|&j| set_key(rig, &sets[j]) == k)
            .unwrap_or(sets.len());
        let t0 = sets[i].t_ns;
        windows.push(
            sets[i..end]
                .iter()
                .take_while(|s| s.t_ns - t0 <= SHARP_WINDOW_NS)
                .filter(|s| set_key(rig, s) == k)
                .cloned()
                .collect(),
        );
    }
    Selection {
        windows,
        rig,
        incomplete,
    }
}

/// Each camera's median blur score over `scores`.
pub fn camera_medians<'a>(
    scores: impl IntoIterator<Item = (&'a str, f64)>,
) -> HashMap<String, f64> {
    let mut by_cam: HashMap<&str, Vec<f64>> = HashMap::new();
    for (cam, s) in scores {
        by_cam.entry(cam).or_default().push(s);
    }
    by_cam
        .into_iter()
        .map(|(cam, mut s)| {
            s.sort_by(f64::total_cmp);
            (cam.to_string(), s[s.len() / 2])
        })
        .collect()
}

/// The sharpest instant of each window: the highest score relative to its
/// camera's median (for a rig instant, its least sharp camera's). A window
/// whose candidates were not all scored keeps its first instant.
pub fn pick_sharpest(
    selection: &Selection,
    scores: &HashMap<(String, u64), f64>,
    medians: &HashMap<String, f64>,
) -> Vec<FrameSet> {
    let relative = |s: &FrameSet| -> Option<f64> {
        s.frames
            .iter()
            .map(|f| {
                let score = *scores.get(&(f.cam.clone(), f.t_ns))?;
                let median = medians.get(&f.cam).copied().unwrap_or(0.0);
                Some(score / median.max(f64::MIN_POSITIVE))
            })
            .try_fold(f64::INFINITY, |acc, r| r.map(|r| acc.min(r)))
    };
    selection
        .windows
        .iter()
        .filter_map(|w| {
            let mut best: Option<(&FrameSet, f64)> = None;
            for s in w {
                if let Some(r) = relative(s) {
                    if best.is_none_or(|(_, b)| r > b) {
                        best = Some((s, r));
                    }
                }
            }
            best.map(|(s, _)| s).or(w.first()).cloned()
        })
        .collect()
}

/// `2·dt` under a frame period: two frames of one instant.
fn same_instant(dt: u64, period: u64) -> bool {
    dt.saturating_mul(2) < period
}

/// Group frames into instants by trigger sequence; a frame more than half a
/// frame period from its sequence's earliest frame, or a camera's second
/// frame under one sequence, stands alone.
fn group_by_trigger(frames: &[FrameRef], period: u64) -> Vec<FrameSet> {
    let mut by_seq: BTreeMap<u64, Vec<FrameRef>> = BTreeMap::new();
    for f in frames {
        by_seq
            .entry(f.trigger_seq.unwrap_or_default())
            .or_default()
            .push(f.clone());
    }
    let mut sets = Vec::new();
    for mut group in by_seq.into_values() {
        group.sort_by_key(|f| f.t_ns);
        let t0 = group[0].t_ns;
        let mut set: Vec<FrameRef> = Vec::new();
        for f in group {
            if set.iter().all(|g| g.cam != f.cam) && same_instant(f.t_ns - t0, period) {
                set.push(f);
            } else {
                sets.push(vec![f]);
            }
        }
        sets.push(set);
    }
    finish_sets(sets)
}

/// Group time-sorted frames into instants by time: a frame joins the open
/// instant while within half a frame period of its first frame and its
/// camera is not in it yet.
fn group_by_time(frames: &[FrameRef], period: u64) -> Vec<FrameSet> {
    let mut sets: Vec<Vec<FrameRef>> = Vec::new();
    for f in frames {
        match sets.last_mut() {
            Some(s)
                if s.iter().all(|g| g.cam != f.cam)
                    && same_instant(f.t_ns.saturating_sub(s[0].t_ns), period) =>
            {
                s.push(f.clone())
            }
            _ => sets.push(vec![f.clone()]),
        }
    }
    finish_sets(sets)
}

/// Order each instant's frames by camera, time it by its reference camera,
/// and order the instants by time.
fn finish_sets(groups: Vec<Vec<FrameRef>>) -> Vec<FrameSet> {
    let mut sets: Vec<FrameSet> = groups
        .into_iter()
        .map(|mut frames| {
            frames.sort_by(|a, b| a.cam.cmp(&b.cam));
            FrameSet {
                t_ns: frames[0].t_ns,
                frames,
            }
        })
        .collect();
    sets.sort_by_key(|s| s.t_ns);
    sets
}

/// The shortest per-camera median frame interval (0 with no camera recording
/// two frames).
fn frame_period(frames: &[FrameRef]) -> u64 {
    let mut by_cam: HashMap<&str, Vec<u64>> = HashMap::new();
    for f in frames {
        by_cam.entry(&f.cam).or_default().push(f.t_ns);
    }
    by_cam
        .into_values()
        .filter_map(|mut ts| {
            ts.sort_unstable();
            let mut dts: Vec<u64> = ts.windows(2).map(|w| w[1] - w[0]).collect();
            dts.sort_unstable();
            dts.get(dts.len() / 2).copied()
        })
        .min()
        .unwrap_or(0)
}

/// Drop frames whose blur score is under [`BLUR_KEEP_FRACTION`] of their
/// camera's median in `medians`, and with `rig_cams` > 0 every rig instant
/// left without all `rig_cams` usable frames (deleting their PNGs). Returns
/// how many frames were dropped for blur and how many rig instants were
/// dropped.
pub fn drop_unusable(
    dataset_dir: &Path,
    records: &mut Vec<(FrameRecord, f64)>,
    rig_cams: usize,
    medians: &HashMap<String, f64>,
) -> (usize, usize) {
    let sharp = |r: &FrameRecord, s: f64| {
        medians
            .get(&r.camera_id)
            .is_none_or(|m| s >= BLUR_KEEP_FRACTION * m)
    };
    let blurred = records.iter().filter(|(r, s)| !sharp(r, *s)).count();
    // Usable frames per rig instant; an instant short of a camera goes whole.
    let mut usable: HashMap<u64, usize> = HashMap::new();
    let mut instants: HashSet<u64> = HashSet::new();
    for (r, s) in records.iter() {
        if let Some(t) = r.rig_instant {
            instants.insert(t);
            if sharp(r, *s) {
                *usable.entry(t).or_default() += 1;
            }
        }
    }
    let complete = |t: u64| usable.get(&t).copied().unwrap_or(0) == rig_cams;
    let rig_dropped = instants.iter().filter(|t| !complete(**t)).count();
    let (keep, gone): (Vec<_>, Vec<_>) = records
        .drain(..)
        .partition(|(r, s)| sharp(r, *s) && r.rig_instant.is_none_or(complete));
    for (r, _) in gone {
        let _ = fs::remove_file(dataset_dir.join("images").join(&r.name));
    }
    *records = keep;
    (blurred, rig_dropped)
}

#[cfg(test)]
#[path = "prepare_keyframes_tests.rs"]
mod tests;
