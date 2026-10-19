//! Building a capture run's pose provider and LiDAR source from the config
//! and the rig.

use std::path::{Path, PathBuf};
use std::sync::Arc;

use ados_capture_contract::Rig;
use tokio::sync::mpsc;
use world_engine_protocol::atlas::PoseSource;

use crate::lidar_scan::LidarScan;
use crate::lidar_source::{LidarSource, LidarSourceConfig};
use crate::pose::fc_ekf::FcEkfPose;
use crate::pose::lidar::{default_kiss_config, Launcher, LidarPose, LidarPoseConfig};
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
    let want = resolve_pose_provider(cfg.pose_provider, profile, stereo);
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
        (PoseSource::Lidar, Some(rig)) => match LidarPose::spawn(LidarPoseConfig {
            launcher: Launcher::Uv(install_path(&cfg.lidar.uv)),
            project_dir: install_path(&cfg.lidar.project_dir),
            rig: rig.clone(),
            lidar_id: cfg.lidar.lidar_id.clone(),
            work_dir: work_dir.join("lidar"),
            kiss: default_kiss_config(),
        }) {
            Ok(l) => Chosen {
                pose: Arc::new(l),
                fc: None,
            },
            Err(e) => fallback(e.to_string()),
        },
        (PoseSource::Vio | PoseSource::Lidar, None) => fallback("no rig document".into()),
        _ => Chosen {
            pose: Arc::new(fc.clone()),
            fc: Some(fc.clone()),
        },
    }
}

/// Start the live LiDAR driver when the rig has the configured LiDAR and a
/// host address is set.
pub fn lidar_source(
    cfg: &AtlasRuntimeConfig,
    rig: Option<&Rig>,
    work_dir: &Path,
) -> Option<(LidarSource, mpsc::Receiver<LidarScan>)> {
    let host_ip = cfg.lidar.host_ip.clone()?;
    let rig = rig?;
    if !rig_source::lidar_ids(rig).contains(&cfg.lidar.lidar_id) {
        return None;
    }
    let mut net = ados_livox::SdkNetConfig::new(host_ip);
    net.lidar_ips = cfg.lidar.lidar_ips.clone();
    match LidarSource::start(LidarSourceConfig {
        sensor_id: cfg.lidar.lidar_id.clone(),
        sdk_library: install_path(&cfg.lidar.sdk_library),
        net,
        work_dir: work_dir.join("livox"),
        period_ns: 100_000_000,
    }) {
        Ok(s) => Some(s),
        Err(e) => {
            tracing::warn!(error = %e, "lidar_source_unavailable");
            None
        }
    }
}
