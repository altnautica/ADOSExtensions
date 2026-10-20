//! Building a capture run's pose provider and LiDAR source from the config
//! and the rig.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use ados_capture_contract::{LidarModelPattern, Rig};
use tokio::sync::mpsc;
use world_engine_protocol::atlas::PoseSource;

use crate::lidar_scan::LidarScan;
use crate::lidar_source::{self, LidarDriver, LidarSourceError};
use crate::pose::fc_ekf::FcEkfPose;
use crate::pose::lidar::{kiss_config, Launcher, LidarPose, LidarPoseConfig};
use crate::pose::planar::{PlanarLidarConfig, PlanarLidarPose};
use crate::pose::vio::{VioConfig, VioPose};
use crate::pose::PoseProvider;
use crate::rig_source;
use crate::runtime::{resolve_pose_provider, AtlasRuntimeConfig};

/// A relative path against the plugin's install directory (the process cwd).
pub fn install_path(p: &str) -> PathBuf {
    let path = PathBuf::from(p);
    if path.is_absolute() {
        path
    } else {
        std::env::current_dir().unwrap_or_default().join(path)
    }
}

/// The provider a run uses, and the FC EKF handle when that is the one.
pub struct Chosen {
    pub pose: Arc<dyn PoseProvider>,
    pub fc: Option<FcEkfPose>,
}

/// Pick and start the pose provider. A provider that cannot start (no rig
/// for VIO, a missing binary) falls back to the FC EKF, which every drone and
/// hub-stage-A visor has, and says so in the log.
pub fn pose_provider(
    cfg: &AtlasRuntimeConfig,
    profile: &str,
    rig: Option<&Rig>,
    fc: &FcEkfPose,
    work_dir: &Path,
) -> Chosen {
    let stereo = rig.is_some_and(|r| {
        rig_source::has_stereo_imu(r, &cfg.vio.left, &cfg.vio.right, &cfg.vio.imu)
    });
    let lidar_imu =
        rig.is_some_and(|r| rig_source::has_lidar_imu(r, &cfg.lidar.lidar_id, &cfg.lidar.imu));
    let want = resolve_pose_provider(cfg.pose_provider, profile, stereo, lidar_imu);
    let fallback = |why: String| {
        tracing::warn!(wanted = ?want, reason = %why, "pose_provider_fallback_fc_ekf");
        Chosen {
            pose: Arc::new(fc.clone()),
            fc: Some(fc.clone()),
        }
    };
    match (want, rig) {
        (PoseSource::Vio, Some(rig)) => match VioPose::spawn(VioConfig {
            binary: install_path(&cfg.vio.binary),
            rig: rig.clone(),
            scale: cfg.vio.scale,
            left_id: cfg.vio.left.clone(),
            right_id: cfg.vio.right.clone(),
            imu_id: cfg.vio.imu.clone(),
            work_dir: work_dir.join("vio"),
        }) {
            Ok(v) => Chosen {
                pose: Arc::new(v),
                fc: None,
            },
            Err(e) => fallback(e.to_string()),
        },
        (PoseSource::Lidar, Some(rig)) => match lidar_pose(cfg, rig, work_dir) {
            Ok(pose) => Chosen { pose, fc: None },
            Err(why) => fallback(why),
        },
        (PoseSource::Vio | PoseSource::Lidar, None) => fallback("no rig document".into()),
        _ => Chosen {
            pose: Arc::new(fc.clone()),
            fc: Some(fc.clone()),
        },
    }
}

/// LiDAR odometry for the rig's configured LiDAR, picked by its scan pattern:
/// 2D scan matching for a planar LiDAR, KISS-ICP for a 3D one.
fn lidar_pose(
    cfg: &AtlasRuntimeConfig,
    rig: &Rig,
    work_dir: &Path,
) -> Result<Arc<dyn PoseProvider>, String> {
    let id = &cfg.lidar.lidar_id;
    let (_, model) = rig_source::lidar_sensor(rig, id)
        .ok_or_else(|| format!("rig has no lidar sensor `{id}` with a scan model"))?;
    match model.pattern {
        LidarModelPattern::Planar => PlanarLidarPose::spawn(PlanarLidarConfig {
            rig: rig.clone(),
            lidar_id: id.clone(),
            imu_id: cfg.lidar.imu.clone(),
        })
        .map(|p| Arc::new(p) as Arc<dyn PoseProvider>)
        .map_err(|e| e.to_string()),
        LidarModelPattern::Solid | LidarModelPattern::Spinning => {
            LidarPose::spawn(LidarPoseConfig {
                launcher: Launcher::Uv(install_path(&cfg.lidar.uv)),
                project_dir: install_path(&cfg.lidar.project_dir),
                rig: rig.clone(),
                lidar_id: id.clone(),
                work_dir: work_dir.join("lidar"),
                kiss: kiss_config(model),
            })
            .map(|p| Arc::new(p) as Arc<dyn PoseProvider>)
            .map_err(|e| e.to_string())
        }
    }
}

/// Start the live LiDAR driver for the rig's configured LiDAR, chosen by its
/// `lidar.model`. `None` when the rig has no such sensor, the driver's device
/// setting is empty, or the driver cannot start (logged).
pub fn lidar_source(
    cfg: &AtlasRuntimeConfig,
    rig: Option<&Rig>,
    work_dir: &Path,
) -> Option<(Box<dyn LidarDriver>, mpsc::Receiver<LidarScan>)> {
    let id = &cfg.lidar.lidar_id;
    let (_, model) = rig_source::lidar_sensor(rig?, id)?;
    match lidar_source::start(model, &cfg.lidar, id, work_dir) {
        Ok(s) => Some(s),
        Err(LidarSourceError::NotConfigured(key)) => {
            tracing::info!(sensor = %id, key, "lidar_source_not_configured");
            None
        }
        Err(e) => {
            tracing::warn!(sensor = %id, model = %model.model, error = %e, "lidar_source_unavailable");
            None
        }
    }
}
