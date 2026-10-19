//! Reading an uploaded capture session for the pipeline: the pose track used
//! as the SfM prior, the camera frames, the LiDAR sweeps, and decoding of the
//! recorded image messages.

use ados_capture_contract::foxglove::{CompressedImage, PointCloud, PoseInFrame, RawImage};
use ados_capture_contract::{Manifest, Rig, RigSensorKind, SessionReader};

use super::geom::{self, Mat4};
use super::StageError;

/// Pose sources usable as the SfM prior, most preferred first. `truth` (a
/// simulator's ground truth) is used only when nothing else was recorded, and
/// the stage detail records which source ran.
pub const PRIOR_SOURCES: [&str; 4] = ["vio", "lidar", "fc_ekf", "truth"];

/// The longest gap a pose may be interpolated across.
const MAX_GAP_NS: u64 = 500_000_000;
/// The furthest a pose may be extrapolated past the track's ends.
const MAX_EXTRAPOLATION_NS: u64 = 50_000_000;

/// A recorded `T_world_body` track from one `/pose/<source>` channel.
#[derive(Debug, Clone, Default)]
pub struct PoseTrack {
    pub source: String,
    pub samples: Vec<(u64, Mat4)>,
}

impl PoseTrack {
    /// The body pose at `t_ns` (interpolated; `None` outside the track or across
    /// a gap).
    pub fn at(&self, t_ns: u64) -> Option<Mat4> {
        let s = &self.samples;
        let (first, last) = (s.first()?, s.last()?);
        if t_ns <= first.0 {
            return (first.0 - t_ns <= MAX_EXTRAPOLATION_NS).then_some(first.1);
        }
        if t_ns >= last.0 {
            return (t_ns - last.0 <= MAX_EXTRAPOLATION_NS).then_some(last.1);
        }
        let i = s.partition_point(|(t, _)| *t <= t_ns);
        let (a, b) = (&s[i - 1], &s[i]);
        if b.0 - a.0 > MAX_GAP_NS {
            return None;
        }
        let u = (t_ns - a.0) as f64 / (b.0 - a.0) as f64;
        Some(geom::interpolate(&a.1, &b.1, u))
    }
}

/// One recorded camera frame (not yet decoded).
#[derive(Debug, Clone, PartialEq)]
pub struct FrameRef {
    pub cam: String,
    pub t_ns: u64,
}

/// The first-pass index of a session.
#[derive(Debug, Clone)]
pub struct SessionIndex {
    pub manifest: Manifest,
    pub rig: Rig,
    pub poses: Option<PoseTrack>,
    pub frames: Vec<FrameRef>,
    pub lidar_sweeps: usize,
}

/// Decode a `foxglove.PoseInFrame` into `T_world_body`.
pub fn pose_matrix(p: &PoseInFrame) -> Option<Mat4> {
    let pose = p.pose.as_ref()?;
    let pos = pose.position.as_ref()?;
    let q = pose.orientation.as_ref()?;
    Some(geom::from_quat_trans(
        [q.w, q.x, q.y, q.z],
        [pos.x, pos.y, pos.z],
    ))
}

/// Pass one: every camera frame's time, every pose of every source (the
/// preferred source becomes the prior track), and the LiDAR sweep count.
pub fn index(reader: &SessionReader) -> Result<SessionIndex, StageError> {
    let cameras: Vec<String> = reader
        .rig()
        .sensors
        .iter()
        .filter(|s| matches!(s.kind, RigSensorKind::Camera) && s.camera.is_some())
        .map(|s| s.id.clone())
        .collect();
    let mut tracks: Vec<PoseTrack> = Vec::new();
    let mut frames = Vec::new();
    let mut lidar_sweeps = 0usize;
    reader.for_each_message(|m| {
        if let Some(rest) = m.topic.strip_prefix("/cam/") {
            if let Some(cam) = rest.strip_suffix("/image") {
                if cameras.iter().any(|c| c == cam) {
                    frames.push(FrameRef {
                        cam: cam.to_string(),
                        t_ns: m.t_ns,
                    });
                }
            }
        } else if let Some(source) = m.topic.strip_prefix("/pose/") {
            if !source.contains('/') {
                if let Some(t) = m.decode::<PoseInFrame>().ok().and_then(|p| pose_matrix(&p)) {
                    match tracks.iter_mut().find(|tr| tr.source == source) {
                        Some(tr) => tr.samples.push((m.t_ns, t)),
                        None => tracks.push(PoseTrack {
                            source: source.to_string(),
                            samples: vec![(m.t_ns, t)],
                        }),
                    }
                }
            }
        } else if m.topic.starts_with("/lidar/") && m.topic.ends_with("/points") {
            lidar_sweeps += 1;
        }
        Ok(())
    })?;
    for tr in &mut tracks {
        tr.samples.sort_by_key(|(t, _)| *t);
        tr.samples.dedup_by_key(|(t, _)| *t);
    }
    frames.sort_by(|a, b| a.t_ns.cmp(&b.t_ns).then_with(|| a.cam.cmp(&b.cam)));
    let poses = PRIOR_SOURCES
        .iter()
        .find_map(|src| {
            tracks
                .iter()
                .find(|t| t.source == *src && t.samples.len() >= 2)
        })
        .cloned();
    Ok(SessionIndex {
        manifest: reader.manifest().clone(),
        rig: reader.rig().clone(),
        poses,
        frames,
        lidar_sweeps,
    })
}

/// A decoded frame: 8-bit grey or RGB pixels.
#[derive(Debug, Clone)]
pub struct DecodedImage {
    pub width: u32,
    pub height: u32,
    /// 1 (grey) or 3 (RGB).
    pub channels: u8,
    pub data: Vec<u8>,
}

impl DecodedImage {
    /// Encode as PNG.
    pub fn write_png(&self, path: &std::path::Path) -> Result<(), StageError> {
        let color = if self.channels == 1 {
            image::ExtendedColorType::L8
        } else {
            image::ExtendedColorType::Rgb8
        };
        image::save_buffer(path, &self.data, self.width, self.height, color)
            .map_err(|e| StageError::io(format!("write {}: {e}", path.display())))
    }

    /// Luminance at quarter resolution (2x2 block averages of a half-res
    /// sample), for the blur score.
    fn quarter_luma(&self) -> (usize, usize, Vec<f32>) {
        let (w, h) = (self.width as usize, self.height as usize);
        let (qw, qh) = (w / 4, h / 4);
        let mut out = Vec::with_capacity(qw * qh);
        let c = self.channels as usize;
        for y in 0..qh {
            for x in 0..qw {
                let i = ((y * 4) * w + x * 4) * c;
                let v = if c == 1 {
                    self.data[i] as f32
                } else {
                    0.299 * self.data[i] as f32
                        + 0.587 * self.data[i + 1] as f32
                        + 0.114 * self.data[i + 2] as f32
                };
                out.push(v);
            }
        }
        (qw, qh, out)
    }

    /// Variance of the Laplacian on quarter-resolution luminance: higher is
    /// sharper.
    pub fn blur_score(&self) -> f64 {
        let (w, h, y) = self.quarter_luma();
        if w < 3 || h < 3 {
            return f64::MAX;
        }
        let mut vals = Vec::with_capacity((w - 2) * (h - 2));
        for r in 1..h - 1 {
            for c in 1..w - 1 {
                let i = r * w + c;
                vals.push((y[i - 1] + y[i + 1] + y[i - w] + y[i + w] - 4.0 * y[i]) as f64);
            }
        }
        let n = vals.len() as f64;
        let mean = vals.iter().sum::<f64>() / n;
        vals.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / n
    }
}

/// Decode a `foxglove.RawImage` (`mono8`, `mono16`, `rgb8`, `bgr8`,
/// `bayer_rggb8`).
pub fn decode_raw(img: &RawImage) -> Result<DecodedImage, StageError> {
    let (w, h) = (img.width as usize, img.height as usize);
    let step = img.step as usize;
    let row = |r: usize, len: usize| -> Result<&[u8], StageError> {
        img.data
            .get(r * step..r * step + len)
            .ok_or_else(|| StageError::input(format!("short {} image", img.encoding)))
    };
    let (channels, mut data) = match img.encoding.as_str() {
        "mono8" | "8UC1" => (1u8, Vec::with_capacity(w * h)),
        "mono16" | "16UC1" | "rgb8" | "bgr8" | "bayer_rggb8" => (
            if img.encoding.starts_with("mono") || img.encoding == "16UC1" {
                1
            } else {
                3
            },
            Vec::with_capacity(w * h * 3),
        ),
        other => {
            return Err(StageError::input(format!(
                "image encoding {other} is not supported"
            )))
        }
    };
    for r in 0..h {
        match img.encoding.as_str() {
            "mono8" | "8UC1" => data.extend_from_slice(row(r, w)?),
            "mono16" | "16UC1" => {
                // Big- or little-endian per ROS convention is little on these rigs.
                data.extend(row(r, w * 2)?.chunks_exact(2).map(|p| p[1]));
            }
            "rgb8" => data.extend_from_slice(row(r, w * 3)?),
            "bgr8" => data.extend(
                row(r, w * 3)?
                    .chunks_exact(3)
                    .flat_map(|p| [p[2], p[1], p[0]]),
            ),
            _ => {
                // RGGB: each 2x2 block gives one colour shared by its pixels.
                let top = row(r & !1, w)?;
                let bottom = row((r & !1) + 1, w).unwrap_or(top);
                for c in 0..w {
                    let x = c & !1;
                    let rr = top[x];
                    let g = ((top[(x + 1).min(w - 1)] as u16 + bottom[x] as u16) / 2) as u8;
                    let b = bottom[(x + 1).min(w - 1)];
                    data.extend_from_slice(&[rr, g, b]);
                }
            }
        }
    }
    Ok(DecodedImage {
        width: img.width,
        height: img.height,
        channels,
        data,
    })
}

/// Decode a `foxglove.CompressedImage` (`jpeg` / `png`).
pub fn decode_compressed(img: &CompressedImage) -> Result<DecodedImage, StageError> {
    let dynimg = image::load_from_memory(&img.data)
        .map_err(|e| StageError::input(format!("decode {} image: {e}", img.format)))?;
    Ok(match dynimg {
        image::DynamicImage::ImageLuma8(g) => DecodedImage {
            width: g.width(),
            height: g.height(),
            channels: 1,
            data: g.into_raw(),
        },
        other => {
            let rgb = other.to_rgb8();
            DecodedImage {
                width: rgb.width(),
                height: rgb.height(),
                channels: 3,
                data: rgb.into_raw(),
            }
        }
    })
}

/// Points of one `foxglove.PointCloud` in its own frame, each with its time
/// offset after the message time (`t_offset_ns`, 0 when absent).
pub fn decode_points(pc: &PointCloud) -> Result<Vec<([f64; 3], u64)>, StageError> {
    let field = |name: &str| pc.fields.iter().find(|f| f.name == name);
    let (Some(fx), Some(fy), Some(fz)) = (field("x"), field("y"), field("z")) else {
        return Err(StageError::input("point cloud without x/y/z fields"));
    };
    let ft = field("t_offset_ns");
    let stride = pc.point_stride as usize;
    if stride == 0 {
        return Err(StageError::input("point cloud with zero stride"));
    }
    // foxglove NumericType: FLOAT32 = 7, FLOAT64 = 8, UINT32 = 5.
    let read = |p: &[u8], f: &ados_capture_contract::foxglove::PackedElementField| -> Option<f64> {
        let o = f.offset as usize;
        match f.r#type {
            7 => p
                .get(o..o + 4)
                .map(|b| f32::from_le_bytes(b.try_into().unwrap()) as f64),
            8 => p
                .get(o..o + 8)
                .map(|b| f64::from_le_bytes(b.try_into().unwrap())),
            5 => p
                .get(o..o + 4)
                .map(|b| u32::from_le_bytes(b.try_into().unwrap()) as f64),
            _ => None,
        }
    };
    let mut out = Vec::with_capacity(pc.data.len() / stride);
    for p in pc.data.chunks_exact(stride) {
        let (Some(x), Some(y), Some(z)) = (read(p, fx), read(p, fy), read(p, fz)) else {
            return Err(StageError::input("point cloud field type is not supported"));
        };
        if !(x.is_finite() && y.is_finite() && z.is_finite()) {
            continue;
        }
        let t = ft.and_then(|f| read(p, f)).unwrap_or(0.0) as u64;
        out.push(([x, y, z], t));
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_track_interpolates_and_refuses_far_extrapolation() {
        let a = geom::from_quat_trans([1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let b = geom::from_quat_trans([1.0, 0.0, 0.0, 0.0], [1.0, 0.0, 0.0]);
        let tr = PoseTrack {
            source: "vio".into(),
            samples: vec![(1_000, a), (101_000_000, b)],
        };
        let mid = tr.at(50_500_500).unwrap();
        assert!((geom::translation(&mid)[0] - 0.5).abs() < 1e-6);
        assert!(tr.at(101_000_000 + 40_000_000).is_some());
        assert!(tr.at(101_000_000 + 60_000_000).is_none());
    }

    #[test]
    fn raw_images_decode_and_blur_ranks_sharp_above_flat() {
        let w = 64u32;
        let h = 48u32;
        let checker: Vec<u8> = (0..w * h)
            .map(|i| {
                if ((i % w) / 4 + (i / w) / 4) % 2 == 0 {
                    0
                } else {
                    255
                }
            })
            .collect();
        let sharp = decode_raw(&RawImage {
            timestamp: None,
            frame_id: "cam0".into(),
            width: w,
            height: h,
            encoding: "mono8".into(),
            step: w,
            data: checker,
        })
        .unwrap();
        let flat = DecodedImage {
            width: w,
            height: h,
            channels: 1,
            data: vec![128; (w * h) as usize],
        };
        assert!(sharp.blur_score() > flat.blur_score());
        let bgr = decode_raw(&RawImage {
            timestamp: None,
            frame_id: "c".into(),
            width: 2,
            height: 1,
            encoding: "bgr8".into(),
            step: 6,
            data: vec![1, 2, 3, 4, 5, 6],
        })
        .unwrap();
        assert_eq!(bgr.data, vec![3, 2, 1, 6, 5, 4]);
    }
}
