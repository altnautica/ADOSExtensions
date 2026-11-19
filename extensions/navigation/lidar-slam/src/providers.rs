//! Building a run's pose provider and LiDAR source from the config and the
//! rig.

use std::path::Path;
use std::sync::Arc;

use ados_capture_contract::{LidarModelPattern, Rig};
use ados_mapping_pose::lidar_scan::LidarScan;
use ados_mapping_pose::pose::PoseProvider;
use ados_mapping_protocol::atlas::PoseSource;

use crate::install_path;
use crate::lidar_source::{self, LidarRunner};
use crate::pose::lidar::{kiss_config, Launcher, LidarPose, LidarPoseConfig};
use crate::pose::planar::{PlanarLidarConfig, PlanarLidarPose};
use crate::pose::vio::{VioConfig, VioPose};
use crate::settings::{resolve_pose_provider, NavConfig};

/// The provider the config and rig ask for (`None`: none).
pub fn wanted(cfg: &NavConfig, profile: &str, rig: Option<&Rig>) -> Option<PoseSource> {
    let stereo = rig.is_some_and(|r| {
        ados_mapping_pose::rig::has_stereo_imu(r, &cfg.vio.left, &cfg.vio.right, &cfg.vio.imu)
    });
    let lidar_imu = rig.is_some_and(|r| {
        ados_mapping_pose::rig::has_lidar_imu(r, &cfg.lidar.lidar_id, &cfg.lidar.imu)
    });
    resolve_pose_provider(cfg.pose_provider, profile, stereo, lidar_imu)
}

/// Start the provider `want` names; why it could not start otherwise (no
/// rig, a missing binary, no such LiDAR). Navigation has no fallback pose:
/// the flight controller's own EKF is its own fallback source set.
pub fn start(
    want: PoseSource,
    cfg: &NavConfig,
    rig: Option<&Rig>,
    work_dir: &Path,
) -> Result<Arc<dyn PoseProvider>, String> {
    let rig = rig.ok_or("no rig document")?;
    match want {
        PoseSource::Vio => VioPose::spawn(VioConfig {
            binary: install_path(&cfg.vio.binary),
            rig: rig.clone(),
            scale: cfg.vio.scale,
            left_id: cfg.vio.left.clone(),
            right_id: cfg.vio.right.clone(),
            imu_id: cfg.vio.imu.clone(),
            work_dir: work_dir.join("vio"),
        })
        .map(|v| Arc::new(v) as Arc<dyn PoseProvider>)
        .map_err(|e| e.to_string()),
        PoseSource::Lidar => lidar_pose(cfg, rig, work_dir),
        other => Err(format!("{other:?} is not a navigation pose provider")),
    }
}

/// LiDAR odometry for the rig's configured LiDAR, picked by its scan pattern:
/// 2D scan matching for a planar LiDAR, KISS-ICP for a 3D one.
fn lidar_pose(
    cfg: &NavConfig,
    rig: &Rig,
    work_dir: &Path,
) -> Result<Arc<dyn PoseProvider>, String> {
    let id = &cfg.lidar.lidar_id;
    let (_, model) = ados_mapping_pose::rig::lidar_sensor(rig, id)
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

/// The run's task for the rig's configured LiDAR, its driver chosen by the
/// sensor's `lidar.model`; `None` when the rig declares no such sensor. The
/// driver starts (and, while the device is absent, retries) inside the task.
pub fn lidar_runner(cfg: &NavConfig, rig: Option<&Rig>, work_dir: &Path) -> Option<LidarRunner> {
    let id = cfg.lidar.lidar_id.clone();
    let (_, model) = ados_mapping_pose::rig::lidar_sensor(rig?, &id)?;
    let (model, settings, work_dir) = (model.clone(), cfg.lidar.clone(), work_dir.to_path_buf());
    let name = model.model.clone();
    let sensor = id.clone();
    let starter: lidar_source::Starter =
        Arc::new(move |scans: tokio::sync::mpsc::Sender<LidarScan>| {
            lidar_source::start(&model, &settings, &sensor, &work_dir, scans)
        });
    Some(LidarRunner::new(id, name, starter))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::settings::PoseProviderConfig;

    #[test]
    fn auto_runs_none_without_a_lidar_or_a_visor_stereo_rig() {
        let cfg = NavConfig::default();
        assert_eq!(wanted(&cfg, "drone", None), None);
        let off = NavConfig {
            pose_provider: PoseProviderConfig::Off,
            ..NavConfig::default()
        };
        assert_eq!(wanted(&off, "visor", None), None);
        let lidar = NavConfig {
            pose_provider: PoseProviderConfig::Lidar,
            ..NavConfig::default()
        };
        assert_eq!(wanted(&lidar, "drone", None), Some(PoseSource::Lidar));
        let e = start(PoseSource::Lidar, &lidar, None, Path::new("/tmp")).err();
        assert_eq!(e.as_deref(), Some("no rig document"));
    }
}
