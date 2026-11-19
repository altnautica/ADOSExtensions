//! The LiDAR side of a reconstruction: which LiDAR the rig carries, the
//! trajectory that poses the keyframes ahead of SfM, and the switches that
//! depend on it.
//!
//! A 3D LiDAR (solid-state or spinning) takes its poses from GLIM's
//! LiDAR-inertial trajectory and keeps them fixed through SfM. A planar
//! LiDAR is matched in-process ([`scan2d`]); its poses fix x, y and heading
//! but not height, so they are priors (source `lidar2d`) the pose-prior
//! mapper refines, never fixed poses.

use ados_capture_contract::{Rig, RigSensorKind, SessionReader};
use serde_json::Value;

use super::glim::{self, GlimOdometry, GlimSummary};
use super::scan2d::{self, Scan2dSummary};
use super::{StageError, ToolRunner};

/// The kind of LiDAR on a rig.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RigLidar {
    None,
    Planar,
    ThreeD,
}

impl RigLidar {
    pub fn of(rig: &Rig) -> Self {
        if glim::rig_has_3d_lidar(rig) {
            Self::ThreeD
        } else if scan2d::rig_planar_lidar(rig).is_some() {
            Self::Planar
        } else {
            Self::None
        }
    }

    /// Share of the sfm stage the trajectory step takes.
    pub fn sfm_share(self) -> f32 {
        match self {
            Self::None => 0.0,
            _ => 0.3,
        }
    }
}

/// The longest range of the rig's LiDARs, metres.
pub fn max_range_m(rig: &Rig) -> Option<f64> {
    rig.sensors
        .iter()
        .filter(|s| s.kind == RigSensorKind::Lidar)
        .filter_map(|s| s.lidar.as_ref().map(|l| l.max_range_m))
        .reduce(f64::max)
}

/// Whether training takes the monocular depth prior. Never with a 3D LiDAR
/// cloud. A planar LiDAR leaves height and everything off its scan plane
/// unmeasured, so its sessions keep the prior. Otherwise only a camera-only
/// session SfM mapped without prior poses: a posed session (VIO or flight
/// controller track) already has its geometry from the track, and on such
/// sessions the prior cost about 3 dB of training PSNR; where it is used the
/// trainer still drops each view whose prior disagrees with the sparse
/// points. `prepare` computes the priors either way (the motion and mirror
/// masks use them).
pub fn train_depth_prior(
    preset_wants: bool,
    has_3d_lidar: bool,
    planar_lidar: bool,
    sfm_detail: &Value,
) -> bool {
    let posed = matches!(
        sfm_detail["method"].as_str(),
        Some("known_poses" | "pose_prior_mapper")
    );
    preset_wants && !has_3d_lidar && (planar_lidar || !posed)
}

/// What the trajectory step decided for SfM, and its stage detail entries.
#[derive(Debug, Clone, PartialEq)]
pub struct LidarPoses {
    /// The pose source of the SfM priors.
    pub prior: Option<String>,
    /// Keep the prior poses fixed (a trusted 3D LiDAR trajectory).
    pub fixed: bool,
    glim: Option<GlimSummary>,
    lidar2d: Option<Scan2dSummary>,
}

impl LidarPoses {
    /// Add the SfM prior's source and the `glim` / `lidar2d` entries to the
    /// sfm stage detail. A used trajectory with no recorded track to align
    /// onto sits in its own frame, so the model is local (no geo anchor).
    pub fn annotate(&self, detail: &mut Value) -> Result<(), StageError> {
        detail["prior_source"] = self.prior.clone().into();
        if let Some(g) = &self.glim {
            if g.used && g.align_rmse_m.is_none() {
                detail["world_frame"] = false.into();
            }
            detail["glim"] = serde_json::to_value(g)?;
        }
        if let Some(s) = &self.lidar2d {
            if s.used && s.align_rmse_m.is_none() {
                detail["world_frame"] = false.into();
            }
            detail["lidar2d"] = serde_json::to_value(s)?;
        }
        Ok(())
    }
}

/// Pose the dataset from the rig's LiDAR ahead of SfM: GLIM for a 3D LiDAR,
/// the planar matcher for a planar one, nothing otherwise. `recorded` is the
/// source of the session's recorded pose track.
pub fn run(
    runner: &dyn ToolRunner,
    reader: &SessionReader,
    job_dir: &std::path::Path,
    dataset: &std::path::Path,
    odometry: GlimOdometry,
    recorded: Option<&str>,
    progress: &mut dyn FnMut(f32),
) -> Result<LidarPoses, StageError> {
    let recorded_lidar = recorded == Some("lidar");
    Ok(match RigLidar::of(reader.rig()) {
        RigLidar::None => LidarPoses {
            prior: recorded.map(str::to_string),
            fixed: recorded_lidar,
            glim: None,
            lidar2d: None,
        },
        RigLidar::ThreeD => {
            let g = glim::run(runner, reader, job_dir, dataset, odometry, progress)?;
            LidarPoses {
                prior: recorded.map(str::to_string),
                fixed: g.as_ref().is_some_and(|g| g.used) || recorded_lidar,
                glim: g,
                lidar2d: None,
            }
        }
        RigLidar::Planar => {
            let s = scan2d::run(reader, job_dir, dataset, progress)?;
            // A recorded LiDAR track of a planar rig is a planar one too.
            let planar = s.as_ref().is_some_and(|s| s.used) || recorded_lidar;
            LidarPoses {
                prior: match planar {
                    true => Some(scan2d::PRIOR_SOURCE.into()),
                    false => recorded.map(str::to_string),
                },
                fixed: false,
                glim: None,
                lidar2d: s,
            }
        }
    })
}

#[cfg(test)]
mod tests {
    use super::train_depth_prior;
    use serde_json::json;

    #[test]
    fn the_mono_depth_prior_trains_pose_free_camera_and_planar_lidar_sessions() {
        let mapped = json!({"method": "mapper"});
        let posed = json!({"method": "known_poses"});
        let prior_mapped = json!({"method": "pose_prior_mapper"});
        assert!(train_depth_prior(true, false, false, &mapped));
        assert!(train_depth_prior(
            true,
            false,
            false,
            &json!({"method": "hierarchical_mapper"})
        ));
        // Camera-only sessions posed by VIO or the flight controller.
        assert!(!train_depth_prior(true, false, false, &posed));
        assert!(!train_depth_prior(true, false, false, &prior_mapped));
        // A planar LiDAR leaves height unmeasured: the prior stays.
        assert!(train_depth_prior(true, false, true, &prior_mapped));
        assert!(!train_depth_prior(false, false, true, &prior_mapped));
        // A 3D LiDAR cloud replaces it.
        assert!(!train_depth_prior(true, true, false, &mapped));
        assert!(!train_depth_prior(false, false, false, &mapped));
    }
}
