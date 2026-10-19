//! Capture values → the `ados.capture/1` MCAP messages.

use ados_capture_contract::foxglove::{
    CompressedImage, LocationFix, Pose, PoseInFrame, Quaternion, RawImage, Vector3,
};
use ados_capture_contract::v1::PoseCov;
use ados_protocol::framebus::FrameFormat;

use crate::pose::{health_str, se3, PoseEstimate};

/// Session image encoding (`atlas.capture.image_encoding`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Deserialize, serde::Serialize)]
#[serde(rename_all = "snake_case")]
pub enum ImageEncoding {
    /// Raw planes (`foxglove.RawImage`), lz4-compressed in the segment chunks.
    /// 4:2:0 video formats are stored as JPEG q95, which no raw encoding names.
    #[default]
    Raw,
    /// JPEG q95 (`foxglove.CompressedImage`); Bayer mosaics stay raw.
    Jpeg,
}

/// JPEG quality of recorded images.
pub const JPEG_QUALITY: u8 = 95;

pub fn timestamp(t_ns: u64) -> prost_types::Timestamp {
    prost_types::Timestamp {
        seconds: (t_ns / 1_000_000_000) as i64,
        nanos: (t_ns % 1_000_000_000) as i32,
    }
}

/// The foxglove raw encoding and row stride of a format, `None` for the 4:2:0
/// video formats.
pub fn raw_encoding(format: FrameFormat, width: u32) -> Option<(&'static str, u32)> {
    Some(match format {
        FrameFormat::Gray8 => ("mono8", width),
        FrameFormat::Gray16 => ("mono16", width * 2),
        FrameFormat::Rgb24 => ("rgb8", width * 3),
        FrameFormat::BayerRggb8 => ("bayer_rggb8", width),
        FrameFormat::BayerRggb16 => ("bayer_rggb16", width * 2),
        FrameFormat::Nv12 | FrameFormat::Yuv420p => return None,
    })
}

/// One recorded image.
#[derive(Debug, Clone, PartialEq)]
pub enum ImageMsg {
    Raw(RawImage),
    Compressed(CompressedImage),
}

/// Build the image message for a frame of `camera` at session time `t_ns`.
pub fn image(
    camera: &str,
    t_ns: u64,
    format: FrameFormat,
    width: u32,
    height: u32,
    pixels: Vec<u8>,
    encoding: ImageEncoding,
) -> anyhow::Result<ImageMsg> {
    let need = format.frame_bytes(width, height);
    if pixels.len() < need {
        anyhow::bail!(
            "{camera}: {} bytes, {format:?} {width}x{height} needs {need}",
            pixels.len()
        );
    }
    let raw = raw_encoding(format, width);
    let bayer = matches!(format, FrameFormat::BayerRggb8 | FrameFormat::BayerRggb16);
    match raw {
        Some((enc, step)) if encoding == ImageEncoding::Raw || bayer => {
            let mut data = pixels;
            data.truncate(need);
            Ok(ImageMsg::Raw(RawImage {
                timestamp: Some(timestamp(t_ns)),
                frame_id: camera.to_string(),
                width,
                height,
                encoding: enc.to_string(),
                step,
                data,
            }))
        }
        _ => {
            let data = crate::encode::encode_jpeg(width, height, format, &pixels, JPEG_QUALITY)?;
            Ok(ImageMsg::Compressed(CompressedImage {
                timestamp: Some(timestamp(t_ns)),
                frame_id: camera.to_string(),
                data,
                format: "jpeg".to_string(),
            }))
        }
    }
}

/// `/pose/<source>` and `/pose/<source>/cov` for an estimate at session time
/// `t_ns`.
pub fn pose(e: &PoseEstimate, t_ns: u64) -> (PoseInFrame, PoseCov) {
    let q = se3::quat_from_matrix(&e.r);
    (
        PoseInFrame {
            timestamp: Some(timestamp(t_ns)),
            frame_id: "world".to_string(),
            pose: Some(Pose {
                position: Some(Vector3 {
                    x: e.t[0],
                    y: e.t[1],
                    z: e.t[2],
                }),
                orientation: Some(Quaternion {
                    x: q[1],
                    y: q[2],
                    z: q[3],
                    w: q[0],
                }),
            }),
        },
        PoseCov {
            t_ns,
            cov: e.cov.to_vec(),
            health: health_str(e.health).to_string(),
        },
    )
}

/// `/gnss/<id>/fix` from a vehicle-state position (ellipsoidal height).
pub fn location_fix(
    gnss_id: &str,
    t_ns: u64,
    lat_deg: f64,
    lon_deg: f64,
    h_ellipsoid_m: f64,
    sigma_m: Option<f64>,
) -> LocationFix {
    let (cov, kind) = match sigma_m {
        Some(s) => {
            let v = s * s;
            (vec![v, 0.0, 0.0, 0.0, v, 0.0, 0.0, 0.0, v], 2)
        }
        None => (vec![0.0; 9], 0),
    };
    LocationFix {
        timestamp: Some(timestamp(t_ns)),
        frame_id: gnss_id.to_string(),
        latitude: lat_deg,
        longitude: lon_deg,
        altitude: h_ellipsoid_m,
        position_covariance: cov,
        position_covariance_type: kind,
        ..Default::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use world_engine_protocol::atlas::{PoseSource, VioHealth};

    #[test]
    fn formats_pick_their_message() {
        let g = image(
            "cam",
            5,
            FrameFormat::Gray8,
            4,
            2,
            vec![1; 8],
            ImageEncoding::Raw,
        )
        .unwrap();
        let ImageMsg::Raw(r) = g else { panic!() };
        assert_eq!((r.encoding.as_str(), r.step, r.data.len()), ("mono8", 4, 8));
        let j = image(
            "cam",
            5,
            FrameFormat::Gray8,
            4,
            2,
            vec![1; 8],
            ImageEncoding::Jpeg,
        )
        .unwrap();
        assert!(
            matches!(&j, ImageMsg::Compressed(c) if c.format == "jpeg" && c.data[..2] == [0xFF, 0xD8])
        );
        let b = image(
            "cam",
            5,
            FrameFormat::BayerRggb8,
            4,
            2,
            vec![1; 8],
            ImageEncoding::Jpeg,
        )
        .unwrap();
        assert!(matches!(&b, ImageMsg::Raw(r) if r.encoding == "bayer_rggb8"));
        let v = image(
            "cam",
            5,
            FrameFormat::Nv12,
            4,
            2,
            vec![1; 12],
            ImageEncoding::Raw,
        )
        .unwrap();
        assert!(matches!(v, ImageMsg::Compressed(_)));
        assert!(image(
            "cam",
            5,
            FrameFormat::Gray8,
            4,
            2,
            vec![1; 3],
            ImageEncoding::Raw
        )
        .is_err());
    }

    #[test]
    fn pose_messages_carry_the_quaternion_and_covariance() {
        let e = PoseEstimate {
            t_ns: 1,
            r: se3::IDENTITY,
            t: [1.0, 2.0, 3.0],
            cov: crate::pose::diagonal_cov(0.1, 0.01),
            source: PoseSource::Vio,
            health: VioHealth::Degraded,
        };
        let (p, c) = pose(&e, 1_500_000_000);
        let pose = p.pose.unwrap();
        assert_eq!(pose.orientation.unwrap().w, 1.0);
        assert_eq!(pose.position.unwrap().z, 3.0);
        assert_eq!(p.timestamp.unwrap().seconds, 1);
        assert_eq!(
            (c.t_ns, c.cov.len(), c.health.as_str()),
            (1_500_000_000, 36, "degraded")
        );
    }
}
