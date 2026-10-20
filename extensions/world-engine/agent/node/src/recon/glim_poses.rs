//! From GLIM's dumped trajectory to keyframe poses: rigid alignment onto the
//! session's pose track, the divergence guards, and the `frames.json`
//! rewrite.

use std::fs;
use std::path::Path;

use ados_capture_contract::Rig;

use super::super::geom::{self, Mat4};
use super::super::prepare::{read_frames, FrameRecord};
use super::super::session_data::PoseTrack;
use super::super::StageError;
use super::{write_tum, GlimOdometry, GlimSummary};

/// An aligned trajectory further than this (RMS, metres) from the recorded
/// track is rejected and the recorded poses are kept.
pub const MAX_ALIGN_RMSE_M: f64 = 0.30;
/// A trajectory with no track to check against is rejected when it moves
/// faster than this between consecutive poses (m/s).
pub const MAX_SPEED_MPS: f64 = 5.0;

/// The rigid transform (no scale) taking GLIM's world onto the track's, fit
/// at every GLIM pose the track covers, and the RMS position error there.
pub fn align(glim: &[(u64, Mat4)], track: &PoseTrack) -> Result<(Mat4, f64), StageError> {
    let pairs: Vec<(Mat4, Mat4)> = glim
        .iter()
        .filter_map(|(t, g)| track.at(*t).map(|s| (*g, s)))
        .collect();
    if pairs.len() < 3 {
        return Err(StageError::input(
            "the LiDAR-inertial trajectory overlaps the session pose track at fewer than three poses",
        ));
    }
    // Each pose contributes its origin and the tips of its unit axes, so the
    // fit is determined by the orientations too (a straight-line walk alone
    // leaves the roll about the line free).
    let marks = |m: &Mat4| {
        [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        .map(|p| geom::apply(m, p))
    };
    let (src, dst): (Vec<[f64; 3]>, Vec<[f64; 3]>) = pairs
        .iter()
        .flat_map(|(g, s)| marks(g).into_iter().zip(marks(s)))
        .unzip();
    let t = crate::products::align::horn(&src, &dst);
    let sq: f64 = pairs
        .iter()
        .map(|(g, s)| {
            geom::dist(geom::apply(&t, geom::translation(g)), geom::translation(s)).powi(2)
        })
        .sum();
    Ok((t, (sq / pairs.len() as f64).sqrt()))
}

/// The fastest move between consecutive poses: `(m/s, t0, t1)`.
pub fn max_speed(poses: &[(u64, Mat4)]) -> Option<(f64, u64, u64)> {
    poses
        .windows(2)
        .filter(|w| w[1].0 > w[0].0)
        .map(|w| {
            let d = geom::dist(geom::translation(&w[0].1), geom::translation(&w[1].1));
            (d / ((w[1].0 - w[0].0) as f64 * 1e-9), w[0].0, w[1].0)
        })
        .max_by(|a, b| a.0.total_cmp(&b.0))
}

/// How `frames.json` was rewritten.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Reposed {
    /// Frames posed from the trajectory.
    pub posed: usize,
    /// Frames outside the trajectory that kept their recorded pose.
    pub recorded: usize,
    /// Frames outside the trajectory removed from the dataset.
    pub dropped: usize,
}

/// Remove a dropped frame's image and the mask / depth prior derived from it.
fn remove_frame_files(dataset: &Path, f: &FrameRecord) -> Result<(), StageError> {
    for p in [
        dataset.join("images").join(&f.name),
        dataset.join("masks").join(format!("{}.png", f.name)),
        dataset.join("depths").join(format!("{}.npy", f.name)),
    ] {
        if p.is_file() {
            fs::remove_file(p)?;
        }
    }
    Ok(())
}

/// Set every frame's `T_world_cam` from `traj` (body poses) at the camera's
/// capture time. A frame the trajectory does not cover keeps its recorded
/// pose when `keep_recorded`, else it is dropped from the dataset.
pub fn rewrite_frames(
    dataset: &Path,
    rig: &Rig,
    traj: &PoseTrack,
    keep_recorded: bool,
) -> Result<Reposed, StageError> {
    let mut n = Reposed::default();
    let mut kept = Vec::new();
    for mut f in read_frames(dataset)? {
        let cam = rig
            .sensors
            .iter()
            .find(|s| s.id == f.camera_id)
            .ok_or_else(|| {
                StageError::input(format!("camera {} is not in the rig", f.camera_id))
            })?;
        let t = (f.t_ns as i64 + cam.time_offset_ns).max(0) as u64;
        match traj.at(t) {
            Some(body) => {
                f.t_world_cam = Some(geom::mul(&body, &cam.t_body_sensor.0));
                // The recorded track's covariance no longer describes this pose.
                f.prior_sigma_m = None;
                n.posed += 1;
            }
            None if keep_recorded => n.recorded += 1,
            None => {
                remove_frame_files(dataset, &f)?;
                n.dropped += 1;
                continue;
            }
        }
        kept.push(f);
    }
    fs::write(
        dataset.join("frames.json"),
        serde_json::to_vec_pretty(&kept)?,
    )?;
    Ok(n)
}

/// Express `poses` (GLIM's `T_world_body`) in the session world and re-pose
/// the dataset, or keep the recorded poses when the alignment is too poor.
/// `dir` receives `trajectory_world.txt`.
pub fn apply_trajectory(
    dir: &Path,
    dataset: &Path,
    rig: &Rig,
    mut poses: Vec<(u64, Mat4)>,
    track: Option<&PoseTrack>,
    odometry: GlimOdometry,
) -> Result<GlimSummary, StageError> {
    let mut summary = GlimSummary {
        odometry,
        trajectory_poses: poses.len(),
        align_rmse_m: None,
        used: false,
        rejected: None,
        frames_glim: 0,
        frames_recorded: 0,
        frames_dropped: 0,
    };
    match track {
        Some(track) => {
            let (t, rmse) = align(&poses, track)?;
            summary.align_rmse_m = Some(rmse);
            if rmse > MAX_ALIGN_RMSE_M {
                tracing::warn!(
                    rmse_m = rmse,
                    limit_m = MAX_ALIGN_RMSE_M,
                    "GLIM trajectory rejected; keeping the recorded poses"
                );
                summary.rejected = Some("align_rmse".into());
                return Ok(summary);
            }
            for p in &mut poses {
                p.1 = geom::mul(&t, &p.1);
            }
        }
        None => {
            if let Some((v, t0, t1)) = max_speed(&poses).filter(|m| m.0 > MAX_SPEED_MPS) {
                return Err(StageError {
                    code: "E_GLIM_DIVERGED".into(),
                    message: format!(
                        "the GLIM trajectory moves at {v:.2} m/s between {t0} and {t1} ns (limit {MAX_SPEED_MPS} m/s)"
                    ),
                    tool: Some("glim".into()),
                });
            }
        }
    }
    write_tum(&dir.join("trajectory_world.txt"), &poses)?;
    let traj = PoseTrack {
        source: "glim".into(),
        samples: poses,
    };
    let n = rewrite_frames(dataset, rig, &traj, track.is_some())?;
    summary.used = true;
    summary.frames_glim = n.posed;
    summary.frames_recorded = n.recorded;
    summary.frames_dropped = n.dropped;
    Ok(summary)
}
