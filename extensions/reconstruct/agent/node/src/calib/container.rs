//! Container runs of the calibration tools. The job's work directory is
//! mounted at `/data`; every path below is relative to it.
//!
//! The Kalibr image's entrypoint is an interactive shell, so its runs replace
//! the entrypoint with `bash -c` and source the catkin workspace first. The
//! LiDAR-camera image's entrypoint sources its ROS 2 workspace and execs the
//! given command.

use std::path::Path;

use crate::recon::{StageError, ToolInvocation};
use crate::tools::container_path;

/// Kalibr working directory, relative to the job work directory.
pub const KALIBR_DIR: &str = "kalibr";
/// Bag-creator input folder inside [`KALIBR_DIR`].
pub const KALIBR_FOLDER: &str = "data";
/// Bag name inside [`KALIBR_DIR`]; Kalibr names its outputs after it.
pub const KALIBR_BAG: &str = "calib.bag";
/// LiDAR-camera working directory, relative to the job work directory.
pub const DVLC_DIR: &str = "dvlc";
/// Directory of ROS 2 bags (one per sub-directory) inside [`DVLC_DIR`].
pub const DVLC_BAGS: &str = "bags";
/// Preprocessed output (and `calib.json`) inside [`DVLC_DIR`].
pub const DVLC_PREPROCESSED: &str = "preprocessed";
/// Image and point topics of the exported ROS 2 bag.
pub const DVLC_IMAGE_TOPIC: &str = "/image";
pub const DVLC_POINTS_TOPIC: &str = "/points";

/// Catkin workspace of the Kalibr image.
const KALIBR_SETUP: &str = "source /catkin_ws/devel/setup.bash";

/// Container path of a work-directory-relative path.
pub fn data(rel: &str) -> String {
    format!("/data/{rel}")
}

/// The pinned image of a container tool, or `E_TOOL_MISSING`.
pub fn pinned_image(tool: &str) -> Result<String, StageError> {
    crate::tools::pins()
        .get(tool)
        .and_then(|p| p.image.clone())
        .ok_or_else(|| {
            StageError::tool_missing(tool, format!("no pinned container image for {tool}"))
        })
}

fn docker_run(work_dir: &Path, extra: &[&str], image: &str) -> ToolInvocation {
    let volume = format!("{}:/data", container_path(work_dir));
    ToolInvocation::new("docker", work_dir)
        .args(["run", "--rm", "-v", volume.as_str()])
        .args(extra.iter().copied())
        .arg(image)
}

fn kalibr(work_dir: &Path, image: &str, command: &str) -> ToolInvocation {
    docker_run(
        work_dir,
        &["-e", "MPLBACKEND=Agg", "--entrypoint", "/bin/bash"],
        image,
    )
    .args([
        "-c".to_string(),
        format!("{KALIBR_SETUP} && rosrun kalibr {command}"),
    ])
}

fn kalibr_path(rel: &str) -> String {
    data(&format!("{KALIBR_DIR}/{rel}"))
}

/// `kalibr_bagcreater --folder <data> --output-bag <calib.bag>`.
pub fn kalibr_bagcreater(work_dir: &Path, image: &str) -> ToolInvocation {
    kalibr(
        work_dir,
        image,
        &format!(
            "kalibr_bagcreater --folder {} --output-bag {}",
            kalibr_path(KALIBR_FOLDER),
            kalibr_path(KALIBR_BAG)
        ),
    )
}

/// `kalibr_calibrate_cameras` over `/cam<i>/image_raw` with one model per
/// camera (`pinhole-radtan` / `pinhole-equi`).
pub fn kalibr_calibrate_cameras(work_dir: &Path, image: &str, models: &[&str]) -> ToolInvocation {
    let topics: Vec<String> = (0..models.len())
        .map(|i| format!("/cam{i}/image_raw"))
        .collect();
    kalibr(
        work_dir,
        image,
        &format!(
            "kalibr_calibrate_cameras --bag {} --topics {} --models {} --target {} --dont-show-report",
            kalibr_path(KALIBR_BAG),
            topics.join(" "),
            models.join(" "),
            kalibr_path("target.yaml")
        ),
    )
}

/// `kalibr_calibrate_imu_camera` with the rig's camera chain and IMU noise.
pub fn kalibr_calibrate_imu_camera(work_dir: &Path, image: &str) -> ToolInvocation {
    kalibr(
        work_dir,
        image,
        &format!(
            "kalibr_calibrate_imu_camera --bag {} --cams {} --imu {} --target {} --dont-show-report",
            kalibr_path(KALIBR_BAG),
            kalibr_path("camchain.yaml"),
            kalibr_path("imu.yaml"),
            kalibr_path("target.yaml")
        ),
    )
}

/// Output file of a Kalibr run (`calib-camchain.yaml`, ...), relative to the
/// work directory.
pub fn kalibr_output(suffix: &str) -> String {
    let stem = KALIBR_BAG.trim_end_matches(".bag");
    format!("{KALIBR_DIR}/{stem}-{suffix}")
}

fn dvlc(work_dir: &Path, image: &str, args: &[String]) -> ToolInvocation {
    docker_run(work_dir, &[], image)
        .args(["ros2", "run", "direct_visual_lidar_calibration"])
        .args(args.iter().cloned())
}

fn csv(v: &[f64]) -> String {
    v.iter()
        .map(|x| format!("{x}"))
        .collect::<Vec<_>>()
        .join(",")
}

/// `preprocess <bags> <preprocessed>` with the topics and camera given
/// explicitly (`camera_model` is `plumb_bob` or `fisheye`).
pub fn dvlc_preprocess(
    work_dir: &Path,
    image: &str,
    camera_model: &str,
    intrinsics: &[f64; 4],
    dist: &[f64],
) -> ToolInvocation {
    let dvlc_path = |rel: &str| data(&format!("{DVLC_DIR}/{rel}"));
    dvlc(
        work_dir,
        image,
        &[
            "preprocess".into(),
            dvlc_path(DVLC_BAGS),
            dvlc_path(DVLC_PREPROCESSED),
            "--image_topic".into(),
            DVLC_IMAGE_TOPIC.into(),
            "--points_topic".into(),
            DVLC_POINTS_TOPIC.into(),
            "--camera_model".into(),
            camera_model.into(),
            "--camera_intrinsics".into(),
            csv(intrinsics),
            "--camera_distortion_coeffs".into(),
            csv(dist),
        ],
    )
}

/// `calibrate <preprocessed> --background --auto_quit` (NID registration
/// from the initial guess in `calib.json`).
pub fn dvlc_calibrate(work_dir: &Path, image: &str) -> ToolInvocation {
    dvlc(
        work_dir,
        image,
        &[
            "calibrate".into(),
            data(&format!("{DVLC_DIR}/{DVLC_PREPROCESSED}")),
            "--background".into(),
            "--auto_quit".into(),
        ],
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    const IMG: &str = "registry.example/calib@sha256:0123";

    fn work() -> PathBuf {
        PathBuf::from("/srv/jobs/j1/calib")
    }

    fn head(rest: &[&str]) -> Vec<String> {
        let vol = format!("{}:/data", container_path(&work()));
        ["run", "--rm", "-v", vol.as_str()]
            .iter()
            .chain(rest)
            .map(|s| s.to_string())
            .collect()
    }

    fn kalibr_args(cmd: &str) -> Vec<String> {
        let mut v = head(&[
            "-e",
            "MPLBACKEND=Agg",
            "--entrypoint",
            "/bin/bash",
            IMG,
            "-c",
        ]);
        v.push(format!(
            "source /catkin_ws/devel/setup.bash && rosrun kalibr {cmd}"
        ));
        v
    }

    #[test]
    fn kalibr_command_lines() {
        let inv = kalibr_bagcreater(&work(), IMG);
        assert_eq!(inv.tool, "docker");
        assert_eq!(inv.cwd, work());
        assert_eq!(
            inv.args_lossy(),
            kalibr_args(
                "kalibr_bagcreater --folder /data/kalibr/data --output-bag /data/kalibr/calib.bag"
            )
        );
        assert_eq!(
            kalibr_calibrate_cameras(&work(), IMG, &["pinhole-radtan", "pinhole-equi"])
                .args_lossy(),
            kalibr_args(
                "kalibr_calibrate_cameras --bag /data/kalibr/calib.bag \
                 --topics /cam0/image_raw /cam1/image_raw \
                 --models pinhole-radtan pinhole-equi \
                 --target /data/kalibr/target.yaml --dont-show-report"
            )
        );
        assert_eq!(
            kalibr_calibrate_imu_camera(&work(), IMG).args_lossy(),
            kalibr_args(
                "kalibr_calibrate_imu_camera --bag /data/kalibr/calib.bag \
                 --cams /data/kalibr/camchain.yaml --imu /data/kalibr/imu.yaml \
                 --target /data/kalibr/target.yaml --dont-show-report"
            )
        );
        assert_eq!(
            kalibr_output("camchain-imucam.yaml"),
            "kalibr/calib-camchain-imucam.yaml"
        );
    }

    #[test]
    fn dvlc_command_lines() {
        let inv = dvlc_preprocess(
            &work(),
            IMG,
            "plumb_bob",
            &[450.0, 451.5, 376.0, 240.0],
            &[-0.1, 0.02, 0.0, 0.0, 0.0],
        );
        let mut want = head(&[IMG, "ros2", "run", "direct_visual_lidar_calibration"]);
        want.extend(
            [
                "preprocess",
                "/data/dvlc/bags",
                "/data/dvlc/preprocessed",
                "--image_topic",
                "/image",
                "--points_topic",
                "/points",
                "--camera_model",
                "plumb_bob",
                "--camera_intrinsics",
                "450,451.5,376,240",
                "--camera_distortion_coeffs",
                "-0.1,0.02,0,0,0",
            ]
            .map(String::from),
        );
        assert_eq!(inv.args_lossy(), want);

        let mut want = head(&[IMG, "ros2", "run", "direct_visual_lidar_calibration"]);
        want.extend(
            [
                "calibrate",
                "/data/dvlc/preprocessed",
                "--background",
                "--auto_quit",
            ]
            .map(String::from),
        );
        assert_eq!(dvlc_calibrate(&work(), IMG).args_lossy(), want);
    }
}
