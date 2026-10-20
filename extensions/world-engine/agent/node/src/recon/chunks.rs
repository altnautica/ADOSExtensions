//! Spatial chunking of large posed sessions.
//!
//! The camera centres are gridded in XY into square cells sized so no cell
//! core holds more than `chunk_frames` frames; sparse cells merge into their
//! nearest chunk, and every empty cell goes to the nearest owned one, so the
//! cells partition the whole plane. A chunk trains on every frame whose camera
//! centre lies within `overlap_m` of its core. Each chunk runs known-pose
//! triangulation (so every chunk stays in the session ENU frame) and training
//! on its own dataset; the results are merged by keeping, from each chunk,
//! only what lies in its core (see [`super::chunks_merge`]).
//!
//! Frames without a pose prior cannot be placed and are left out of every
//! chunk; known-pose triangulation would not register them either.

use std::collections::{BTreeMap, VecDeque};
use std::fs;
use std::path::Path;

use serde::{Deserialize, Serialize};

use super::chunks_merge;
use super::prepare::{read_frames, FrameRecord};
use super::sfm::{self, SfmSummary};
use super::train::TrainSummary;
use super::{StageError, ToolInvocation, ToolRunner};

/// Frames per chunk core unless the job sets `chunk_frames`.
pub const CHUNK_FRAMES: usize = 400;
/// The smallest `chunk_frames` honoured.
pub const MIN_CHUNK_FRAMES: usize = 10;
/// How far past its core a chunk trains, metres.
pub const OVERLAP_M: f64 = 5.0;
/// The posed share known-pose triangulation needs (as in [`sfm::run`]).
const MIN_POSED_SHARE: f64 = 0.5;
/// Grid cells never shrink below this, metres.
const MIN_CELL_M: f64 = 2.0;
/// The plan, written beside the chunk SfM models for the train stage.
const PLAN_FILE: &str = "plan.json";

/// One chunk of a plan.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct Chunk {
    pub id: usize,
    /// Frames whose camera centre lies in the core.
    pub core_frames: usize,
    /// `[min_x, min_y, max_x, max_y]` of the cells the chunk owns.
    pub core: [f64; 4],
    /// Every frame the chunk trains on (its core expanded by the overlap).
    pub frames: Vec<String>,
}

/// A grid partition of the plane into chunks.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ChunkPlan {
    pub origin: [f64; 2],
    pub cell_m: f64,
    pub nx: usize,
    pub ny: usize,
    /// The chunk owning each cell, row-major with x fastest. Points outside
    /// the grid belong to the nearest edge cell.
    pub owner: Vec<usize>,
    pub overlap_m: f64,
    pub chunks: Vec<Chunk>,
    /// Frames that more than one chunk trains on.
    pub shared_frames: usize,
}

impl ChunkPlan {
    fn cell_of(&self, x: f64, y: f64) -> (usize, usize) {
        let at = |v: f64, o: f64, n: usize| {
            (((v - o) / self.cell_m).floor().max(0.0) as usize).min(n - 1)
        };
        (
            at(x, self.origin[0], self.nx),
            at(y, self.origin[1], self.ny),
        )
    }

    /// The chunk whose core holds `(x, y)`.
    pub fn owner_of(&self, x: f64, y: f64) -> usize {
        let (ix, iy) = self.cell_of(x, y);
        self.owner[iy * self.nx + ix]
    }

    fn cell_rect(&self, cell: usize) -> [f64; 4] {
        let (ix, iy) = ((cell % self.nx) as f64, (cell / self.nx) as f64);
        let (x0, y0) = (
            self.origin[0] + ix * self.cell_m,
            self.origin[1] + iy * self.cell_m,
        );
        [x0, y0, x0 + self.cell_m, y0 + self.cell_m]
    }

    /// The stage-detail view of the plan.
    pub fn detail(&self) -> serde_json::Value {
        let chunks: Vec<serde_json::Value> = self
            .chunks
            .iter()
            .map(|c| {
                serde_json::json!({
                    "id": c.id, "frames": c.frames.len(),
                    "core_frames": c.core_frames, "core": c.core,
                })
            })
            .collect();
        serde_json::json!({
            "chunks": chunks,
            "shared_frames": self.shared_frames,
            "overlap_m": self.overlap_m,
        })
    }
}

/// Distance from `p` to the rectangle `r` (`0` inside it).
fn rect_dist(r: &[f64; 4], p: [f64; 2]) -> f64 {
    let dx = (r[0] - p[0]).max(p[0] - r[2]).max(0.0);
    let dy = (r[1] - p[1]).max(p[1] - r[3]).max(0.0);
    dx.hypot(dy)
}

/// Plan chunks over named camera centres (XY). `None` when the frames fit
/// one chunk.
pub fn plan(
    frames: &[(String, [f64; 2])],
    chunk_frames: usize,
    overlap_m: f64,
) -> Option<ChunkPlan> {
    let max = chunk_frames.max(MIN_CHUNK_FRAMES);
    let n = frames.len();
    if n <= max {
        return None;
    }
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for (_, c) in frames {
        for k in 0..2 {
            lo[k] = lo[k].min(c[k]);
            hi[k] = hi[k].max(c[k]);
        }
    }
    let (w, h) = ((hi[0] - lo[0]).max(1e-3), (hi[1] - lo[1]).max(1e-3));
    let target = n.div_ceil(max) as f64;
    // Start at the cell that would give `target` equal cells and shrink until
    // no cell is over the cap (or the cell floor is reached: frames stacked
    // on one spot cannot be split in space).
    let mut cell_m = (w * h / target)
        .sqrt()
        .max(w.max(h) / target)
        .max(MIN_CELL_M);
    let (nx, ny, cell_idx) = loop {
        let (nx, ny) = ((w / cell_m) as usize + 1, (h / cell_m) as usize + 1);
        let idx: Vec<usize> = frames
            .iter()
            .map(|(_, c)| {
                let ix = (((c[0] - lo[0]) / cell_m) as usize).min(nx - 1);
                let iy = (((c[1] - lo[1]) / cell_m) as usize).min(ny - 1);
                iy * nx + ix
            })
            .collect();
        let mut counts = vec![0usize; nx * ny];
        idx.iter().for_each(|&i| counts[i] += 1);
        if counts.iter().all(|&c| c <= max) || cell_m <= MIN_CELL_M {
            break (nx, ny, idx);
        }
        cell_m = (cell_m * 0.85).max(MIN_CELL_M);
    };

    // One group per non-empty cell, then merge sparse groups into the nearest
    // group (preferring one the merge keeps under the cap).
    let mut counts: BTreeMap<usize, usize> = BTreeMap::new();
    cell_idx
        .iter()
        .for_each(|&i| *counts.entry(i).or_default() += 1);
    let mut groups: Vec<(Vec<usize>, usize)> =
        counts.into_iter().map(|(c, k)| (vec![c], k)).collect();
    let sparse = (max / 4).max(3);
    let gap = |a: &[usize], b: &[usize]| {
        let mut best = usize::MAX;
        for &p in a {
            for &q in b {
                let d = (p % nx).abs_diff(q % nx).max((p / nx).abs_diff(q / nx));
                best = best.min(d);
            }
        }
        best
    };
    while groups.len() > 1 {
        let Some(s) = (0..groups.len())
            .filter(|&g| groups[g].1 < sparse)
            .min_by_key(|&g| (groups[g].1, g))
        else {
            break;
        };
        let t = (0..groups.len())
            .filter(|&g| g != s)
            .min_by_key(|&g| {
                (
                    groups[s].1 + groups[g].1 > max,
                    gap(&groups[s].0, &groups[g].0),
                    groups[g].1,
                    g,
                )
            })
            .expect("more than one group");
        let (cells, k) = groups.remove(s);
        let t = if t > s { t - 1 } else { t };
        groups[t].0.extend(cells);
        groups[t].1 += k;
    }
    if groups.len() <= 1 {
        return None;
    }
    groups.iter_mut().for_each(|g| g.0.sort_unstable());
    groups.sort_by_key(|g| g.0[0]);

    // Every cell to its nearest owned cell's chunk (breadth first).
    let mut owner = vec![usize::MAX; nx * ny];
    let mut queue = VecDeque::new();
    for (id, (cells, _)) in groups.iter().enumerate() {
        for &c in cells {
            owner[c] = id;
            queue.push_back(c);
        }
    }
    while let Some(c) = queue.pop_front() {
        let (x, y) = (c % nx, c / nx);
        let near = [
            (x > 0).then(|| c - 1),
            (x + 1 < nx).then(|| c + 1),
            (y > 0).then(|| c - nx),
            (y + 1 < ny).then(|| c + nx),
        ];
        for d in near.into_iter().flatten() {
            if owner[d] == usize::MAX {
                owner[d] = owner[c];
                queue.push_back(d);
            }
        }
    }

    let mut plan = ChunkPlan {
        origin: lo,
        cell_m,
        nx,
        ny,
        owner,
        overlap_m,
        chunks: Vec::new(),
        shared_frames: 0,
    };
    let mut membership = vec![0usize; n];
    for id in 0..groups.len() {
        let rects: Vec<[f64; 4]> = (0..nx * ny)
            .filter(|&c| plan.owner[c] == id)
            .map(|c| plan.cell_rect(c))
            .collect();
        let core = rects.iter().fold(
            [
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ],
            |a, r| {
                [
                    a[0].min(r[0]),
                    a[1].min(r[1]),
                    a[2].max(r[2]),
                    a[3].max(r[3]),
                ]
            },
        );
        let mut chunk = Chunk {
            id,
            core_frames: 0,
            core,
            frames: Vec::new(),
        };
        for (i, (name, c)) in frames.iter().enumerate() {
            if plan.owner[cell_idx[i]] == id {
                chunk.core_frames += 1;
            }
            if rect_dist(&core, *c) <= overlap_m
                && rects.iter().any(|r| rect_dist(r, *c) <= overlap_m)
            {
                chunk.frames.push(name.clone());
                membership[i] += 1;
            }
        }
        plan.chunks.push(chunk);
    }
    plan.shared_frames = membership.iter().filter(|&&m| m > 1).count();
    Some(plan)
}

/// The chunk plan for a prepared dataset, or `None` when it runs unchunked:
/// too few posed frames for known-pose triangulation, or one chunk holds it.
pub fn plan_dataset(
    dataset: &Path,
    chunk_frames: Option<usize>,
) -> Result<Option<ChunkPlan>, StageError> {
    let frames = read_frames(dataset)?;
    let posed: Vec<(String, [f64; 2])> = frames
        .iter()
        .filter_map(|f| f.t_world_cam.map(|t| (f.name.clone(), [t[3], t[7]])))
        .collect();
    if posed.len() < 3 || (posed.len() as f64) < MIN_POSED_SHARE * frames.len() as f64 {
        return Ok(None);
    }
    Ok(plan(
        &posed,
        chunk_frames.unwrap_or(CHUNK_FRAMES),
        OVERLAP_M,
    ))
}

/// The plan a chunked `sfm` stage wrote under `sfm_dir`.
pub fn read_plan(sfm_dir: &Path) -> Result<ChunkPlan, StageError> {
    Ok(serde_json::from_slice(&fs::read(
        sfm_dir.join("chunks").join(PLAN_FILE),
    )?)?)
}

/// Hard-link `src` to `dst`, copying where links are not possible.
fn link(src: &Path, dst: &Path) -> Result<(), StageError> {
    if let Some(parent) = dst.parent() {
        fs::create_dir_all(parent)?;
    }
    if fs::hard_link(src, dst).is_err() {
        fs::copy(src, dst)?;
    }
    Ok(())
}

/// A chunk's dataset: its images, `frames.json` subset, and the masks and
/// depth priors of those images when present.
fn write_chunk_dataset(
    dataset: &Path,
    chunk: &Chunk,
    frames: &BTreeMap<&str, &FrameRecord>,
    out: &Path,
) -> Result<(), StageError> {
    let mut subset = Vec::with_capacity(chunk.frames.len());
    for name in &chunk.frames {
        let f = frames.get(name.as_str()).ok_or_else(|| {
            StageError::input(format!("chunk frame {name} is not in frames.json"))
        })?;
        link(
            &dataset.join("images").join(name),
            &out.join("images").join(name),
        )?;
        for (dir, ext) in [("masks", "png"), ("depths", "npy")] {
            let rel = format!("{name}.{ext}");
            let src = dataset.join(dir).join(&rel);
            if src.is_file() {
                link(&src, &out.join(dir).join(&rel))?;
            }
        }
        subset.push((*f).clone());
    }
    fs::create_dir_all(out)?;
    fs::write(out.join("frames.json"), serde_json::to_vec_pretty(&subset)?)?;
    Ok(())
}

/// Stage `sfm` per chunk: chunk datasets under `dataset/chunks/<i>/`, models
/// under `sfm_dir/chunks/<i>/`, merged into `sfm_dir/txt` and, as BIN, into
/// `dataset/sparse/0`. Returns the stage detail.
pub fn run_sfm(
    runner: &dyn ToolRunner,
    dataset: &Path,
    sfm_dir: &Path,
    plan: &ChunkPlan,
    fixed_poses: bool,
    refine: bool,
    progress: &mut dyn FnMut(f32),
) -> Result<serde_json::Value, StageError> {
    for d in [
        sfm_dir.to_path_buf(),
        dataset.join("sparse"),
        dataset.join("chunks"),
    ] {
        if d.exists() {
            fs::remove_dir_all(&d)?;
        }
    }
    let chunk_sfm = sfm_dir.join("chunks");
    fs::create_dir_all(&chunk_sfm)?;
    fs::write(chunk_sfm.join(PLAN_FILE), serde_json::to_vec_pretty(plan)?)?;
    let frames = read_frames(dataset)?;
    let by_name: BTreeMap<&str, &FrameRecord> =
        frames.iter().map(|f| (f.name.as_str(), f)).collect();
    let n = plan.chunks.len() as f32;
    let mut models = Vec::with_capacity(plan.chunks.len());
    for (i, c) in plan.chunks.iter().enumerate() {
        let ds = dataset.join("chunks").join(c.id.to_string());
        write_chunk_dataset(dataset, c, &by_name, &ds)?;
        let dir = chunk_sfm.join(c.id.to_string());
        sfm::run(runner, &ds, &dir, fixed_poses, refine, false, &mut |f| {
            progress(0.9 * (i as f32 + f) / n)
        })?;
        models.push((c.id, dir.join("txt")));
    }
    let txt = sfm_dir.join("txt");
    let merged = chunks_merge::merge_models(&models, plan, &txt)?;
    let model0 = dataset.join("sparse").join("0");
    fs::create_dir_all(&model0)?;
    runner.run(
        &ToolInvocation::new("colmap", sfm_dir)
            .arg("model_converter")
            .arg("--input_path")
            .arg(&txt)
            .arg("--output_path")
            .arg(&model0)
            .args(["--output_type", "BIN"]),
        &mut |_| {},
    )?;
    progress(1.0);
    let mut detail = serde_json::to_value(SfmSummary {
        method: "known_poses".into(),
        world_frame: true,
        registered_images: merged.images,
        points: merged.points,
        rig_frames_paired: 0,
    })?;
    if let (Some(d), serde_json::Value::Object(extra)) = (detail.as_object_mut(), plan.detail()) {
        d.extend(extra);
        d.insert(
            "max_shared_pose_delta_m".into(),
            merged.max_shared_pose_delta_m.into(),
        );
    }
    Ok(detail)
}

/// Train one dataset (`dataset`, `train_dir`, progress) into
/// `train_dir/splat.ply`.
pub type TrainOne<'a> =
    dyn Fn(&Path, &Path, &mut dyn FnMut(f32)) -> Result<TrainSummary, StageError> + 'a;

/// Stage `train` per chunk into `train_dir/chunks/<i>/`, merged into
/// `train_dir/splat.ply`. Returns the stage detail.
pub fn run_train(
    plan: &ChunkPlan,
    dataset: &Path,
    train_dir: &Path,
    sfm_detail: &serde_json::Value,
    train_one: &TrainOne<'_>,
    progress: &mut dyn FnMut(f32),
) -> Result<serde_json::Value, StageError> {
    let n = plan.chunks.len() as f32;
    let mut parts = Vec::with_capacity(plan.chunks.len());
    let mut summaries = Vec::with_capacity(plan.chunks.len());
    for (i, c) in plan.chunks.iter().enumerate() {
        let out = train_dir.join("chunks").join(c.id.to_string());
        let ds = dataset.join("chunks").join(c.id.to_string());
        summaries.push(train_one(&ds, &out, &mut |f| {
            progress(0.95 * (i as f32 + f) / n)
        })?);
        parts.push((c.id, out.join("splat.ply")));
    }
    let counts = chunks_merge::merge_splats(&parts, plan, &train_dir.join("splat.ply"))?;
    progress(1.0);
    let psnrs: Vec<f64> = summaries.iter().filter_map(|s| s.psnr).collect();
    let merged = TrainSummary {
        gaussians: Some(counts.iter().map(|c| c.core).sum()),
        psnr: (!psnrs.is_empty()).then(|| psnrs.iter().sum::<f64>() / psnrs.len() as f64),
        ..summaries[0].clone()
    };
    let mut detail = serde_json::to_value(merged)?;
    let chunks: Vec<serde_json::Value> = plan
        .chunks
        .iter()
        .zip(&counts)
        .map(|(c, k)| {
            serde_json::json!({
                "id": c.id, "frames": c.frames.len(), "core_frames": c.core_frames,
                "core": c.core, "gaussians": k.total, "core_gaussians": k.core,
            })
        })
        .collect();
    detail["chunks"] = chunks.into();
    detail["shared_frames"] = plan.shared_frames.into();
    detail["max_shared_pose_delta_m"] = sfm_detail["max_shared_pose_delta_m"].clone();
    Ok(detail)
}

#[cfg(test)]
#[path = "chunks_tests.rs"]
mod tests;
