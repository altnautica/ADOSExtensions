//! Session → Kalibr bag-creator folder: `cam<i>/<t_ns>.png` (8-bit gray) per
//! rig camera and `imu0.csv` (`timestamp,omega_x,omega_y,omega_z,alpha_x,
//! alpha_y,alpha_z`, nanoseconds, body FLU) from the rig's first IMU. Times
//! are the session timebase. Kalibr splits file-name and CSV timestamps into
//! seconds and nanoseconds by position, so names are padded to ten digits.

use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::Path;

use ados_capture_contract::foxglove::{CompressedImage, RawImage};
use ados_capture_contract::v1::ImuBatch;
use ados_capture_contract::{SessionMessage, SessionReader};
use image::{DynamicImage, GrayImage, RgbImage};

use super::kalibr_yaml::{rig_cameras, rig_imu};
use super::visit;
use crate::recon::StageError;

/// One decoded camera frame, rows packed.
#[derive(Debug, Clone, PartialEq)]
pub enum Frame {
    Mono8(GrayImage),
    Rgb8(RgbImage),
}

impl Frame {
    pub fn to_gray(&self) -> GrayImage {
        match self {
            Frame::Mono8(g) => g.clone(),
            Frame::Rgb8(c) => DynamicImage::ImageRgb8(c.clone()).to_luma8(),
        }
    }
}

fn raw_frame(img: RawImage) -> Result<Frame, StageError> {
    let channels = match img.encoding.as_str() {
        "mono8" => 1,
        "rgb8" | "bgr8" => 3,
        other => {
            return Err(StageError::input(format!(
                "raw image encoding `{other}` is not supported (mono8, rgb8, bgr8)"
            )))
        }
    };
    let (w, h) = (img.width as usize, img.height as usize);
    let row = w * channels;
    let step = img.step as usize;
    if step < row || img.data.len() < step * h.saturating_sub(1) + row {
        return Err(StageError::input(format!(
            "raw image {w}x{h} {} has {} bytes at step {step}",
            img.encoding,
            img.data.len()
        )));
    }
    let mut packed = Vec::with_capacity(row * h);
    for y in 0..h {
        packed.extend_from_slice(&img.data[y * step..y * step + row]);
    }
    if img.encoding == "bgr8" {
        for px in packed.as_chunks_mut::<3>().0 {
            px.swap(0, 2);
        }
    }
    let bad = || StageError::input("raw image buffer size mismatch");
    Ok(if channels == 1 {
        Frame::Mono8(GrayImage::from_raw(img.width, img.height, packed).ok_or_else(bad)?)
    } else {
        Frame::Rgb8(RgbImage::from_raw(img.width, img.height, packed).ok_or_else(bad)?)
    })
}

/// Decode a `/cam/<id>/image` message (`foxglove.RawImage` or
/// `foxglove.CompressedImage`).
pub fn frame_from_message(m: &SessionMessage<'_>) -> Result<Frame, StageError> {
    match m.schema_name {
        Some("foxglove.RawImage") => raw_frame(m.decode::<RawImage>()?),
        Some("foxglove.CompressedImage") => {
            let c: CompressedImage = m.decode()?;
            let img = image::load_from_memory(&c.data)
                .map_err(|e| StageError::input(format!("{}: {e}", m.topic)))?;
            Ok(if img.color().has_color() {
                Frame::Rgb8(img.to_rgb8())
            } else {
                Frame::Mono8(img.to_luma8())
            })
        }
        other => Err(StageError::input(format!(
            "{} carries {other:?}, not an image",
            m.topic
        ))),
    }
}

/// What a Kalibr folder export wrote.
#[derive(Debug, Clone, PartialEq)]
pub struct KalibrExport {
    /// Frames written per `cam<i>`.
    pub frames: Vec<usize>,
    /// Rows in `imu0.csv` (0 and no file when the rig has no IMU).
    pub imu_samples: u64,
    /// Mean IMU rate, Hz.
    pub imu_rate_hz: Option<f64>,
}

/// Image file name for a frame at `t_ns`.
pub fn frame_name(t_ns: u64) -> String {
    format!("{t_ns:010}.png")
}

/// Write the Kalibr folder for `reader` under `out`.
pub fn export_kalibr_folder(
    reader: &SessionReader,
    out: &Path,
) -> Result<KalibrExport, StageError> {
    let rig = reader.rig();
    let cam_ids: Vec<String> = rig_cameras(rig)
        .into_iter()
        .map(|i| rig.sensors[i].id.clone())
        .collect();
    if cam_ids.is_empty() {
        return Err(StageError::input("the rig has no camera"));
    }
    for i in 0..cam_ids.len() {
        fs::create_dir_all(out.join(format!("cam{i}")))?;
    }
    let imu_topic = rig_imu(rig).map(|i| format!("/imu/{}", rig.sensors[i].id));
    let mut csv = match &imu_topic {
        Some(_) => {
            let mut w = BufWriter::new(File::create(out.join("imu0.csv"))?);
            writeln!(
                w,
                "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z"
            )?;
            Some(w)
        }
        None => None,
    };
    let mut frames = vec![0usize; cam_ids.len()];
    let (mut imu_samples, mut t_first, mut t_last) = (0u64, u64::MAX, 0u64);
    visit(reader, |m| {
        if let Some(id) = m
            .topic
            .strip_prefix("/cam/")
            .and_then(|t| t.strip_suffix("/image"))
        {
            let Some(i) = cam_ids.iter().position(|c| c == id) else {
                return Ok(());
            };
            let gray = frame_from_message(&m)?.to_gray();
            let path = out.join(format!("cam{i}")).join(frame_name(m.t_ns));
            gray.save_with_format(&path, image::ImageFormat::Png)
                .map_err(|e| StageError::io(format!("{}: {e}", path.display())))?;
            frames[i] += 1;
        } else if Some(m.topic) == imu_topic.as_deref() {
            let w = csv.as_mut().expect("csv opened with the topic");
            for s in m.decode::<ImuBatch>()?.samples {
                if s.gyro.len() != 3 || s.accel.len() != 3 {
                    continue;
                }
                writeln!(
                    w,
                    "{},{},{},{},{},{},{}",
                    s.t_ns, s.gyro[0], s.gyro[1], s.gyro[2], s.accel[0], s.accel[1], s.accel[2]
                )?;
                imu_samples += 1;
                t_first = t_first.min(s.t_ns);
                t_last = t_last.max(s.t_ns);
            }
        }
        Ok(())
    })?;
    if let Some(mut w) = csv {
        w.flush()?;
    }
    let imu_rate_hz = (imu_samples > 1 && t_last > t_first)
        .then(|| (imu_samples - 1) as f64 / ((t_last - t_first) as f64 * 1e-9));
    Ok(KalibrExport {
        frames,
        imu_samples,
        imu_rate_hz,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::calib::test_util::tiny_session;

    #[test]
    fn tiny_session_exports_two_frames_and_twenty_imu_rows() {
        let dir = tempfile::tempdir().unwrap();
        let reader = SessionReader::open(&tiny_session()).unwrap();
        let out = dir.path().join("kalibr");
        let e = export_kalibr_folder(&reader, &out).unwrap();
        assert_eq!(e.frames, vec![2]);
        assert_eq!(e.imu_samples, 20);
        assert!((e.imu_rate_hz.unwrap() - 1000.0).abs() < 1e-6);

        let mut names: Vec<String> = fs::read_dir(out.join("cam0"))
            .unwrap()
            .map(|d| d.unwrap().file_name().to_string_lossy().into_owned())
            .collect();
        names.sort();
        assert_eq!(names, vec!["1000000000.png", "1050000000.png"]);
        let img = image::open(out.join("cam0/1050000000.png")).unwrap();
        assert_eq!(img.color(), image::ColorType::L8);
        let img = img.to_luma8();
        assert_eq!(img.dimensions(), (64, 48));
        // The fixture's pixel (x, 0) of frame 1 is x*4 + 16.
        assert_eq!(img.get_pixel(10, 0).0[0], 56);

        let csv = fs::read_to_string(out.join("imu0.csv")).unwrap();
        let lines: Vec<&str> = csv.lines().collect();
        assert_eq!(lines.len(), 21);
        assert_eq!(
            lines[0],
            "timestamp,omega_x,omega_y,omega_z,alpha_x,alpha_y,alpha_z"
        );
        assert_eq!(lines[1], "1000000000,0.01,-0.02,0,0,0,9.80665");
        let last: Vec<f64> = lines[20].split(',').map(|v| v.parse().unwrap()).collect();
        assert_eq!(last[0], 1_019_000_000.0);
        assert!((last[3] - 0.057).abs() < 1e-12, "{}", lines[20]);
    }
}
