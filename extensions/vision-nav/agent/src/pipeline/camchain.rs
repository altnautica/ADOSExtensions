//! Camera-IMU calibration loading.
//!
//! The calibration wizard (Python helper) writes a Kalibr-style
//! `camchain.yaml` into the plugin's data dir; the agent only reads the
//! `cam0` timeshift the frame/IMU aligner needs.

use serde::Deserialize;

/// Load the camera-IMU timeshift from the persisted camchain.yaml, or
/// `None` when no calibration is on disk.
pub fn load_calibration_timeshift(data_dir: Option<&std::path::Path>) -> Option<f64> {
    let text = std::fs::read_to_string(calibration_path(data_dir)).ok()?;
    timeshift(&text)
}

/// The calibration file in the plugin's data dir, which the host delivers
/// with the launch values (`ctx.data_dir`).
fn calibration_path(data_dir: Option<&std::path::Path>) -> std::path::PathBuf {
    data_dir
        .unwrap_or(std::path::Path::new(
            "/var/ados/plugins/com.altnautica.vision-nav/data",
        ))
        .join("camchain.yaml")
}

#[derive(Deserialize)]
struct Cam {
    intrinsics: Option<Vec<f64>>,
    timeshift_cam_imu: Option<f64>,
}

/// Read `timeshift_cam_imu` (seconds) out of a camchain.yaml string.
/// Accepts both the `cam0:` wrapper and a bare block. `None` when the
/// file does not carry a calibrated camera, in which case the aligner
/// keeps its zero offset and the heartbeat reports the calibration as
/// not loaded.
pub fn timeshift(text: &str) -> Option<f64> {
    let cam: Cam = serde_yaml::from_str::<std::collections::BTreeMap<String, Cam>>(text)
        .ok()
        .and_then(|mut m| m.remove("cam0"))
        .or_else(|| serde_yaml::from_str::<Cam>(text).ok())?;

    let intr = cam.intrinsics?;
    if intr.len() < 4 {
        return None;
    }
    Some(cam.timeshift_cam_imu.unwrap_or(0.0))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn camchain_parses_cam0_wrapper() {
        let yaml = r#"
cam0:
  camera_model: pinhole
  intrinsics: [500.0, 501.0, 320.0, 240.0]
  distortion_model: radtan
  distortion_coeffs: [0.1, 0.0, 0.0, 0.0]
  resolution: [640, 480]
  T_cam_imu:
    - [1.0, 0.0, 0.0, 0.01]
    - [0.0, 1.0, 0.0, 0.02]
    - [0.0, 0.0, 1.0, 0.03]
    - [0.0, 0.0, 0.0, 1.0]
  timeshift_cam_imu: -0.005
"#;
        let ts = timeshift(yaml).unwrap();
        assert!((ts - (-0.005)).abs() < 1e-9);
    }

    #[test]
    fn camchain_parses_bare_block() {
        let yaml = r#"
camera_model: pinhole
intrinsics: [400.0, 400.0, 200.0, 150.0]
resolution: [400, 300]
"#;
        assert_eq!(timeshift(yaml).unwrap(), 0.0);
    }

    #[test]
    fn camchain_without_intrinsics_is_not_a_calibration() {
        let yaml = "cam0:\n  camera_model: pinhole\n";
        assert!(timeshift(yaml).is_none());
    }
}
