//! Luma extraction from the shared vision-bus frame formats.
//!
//! The shared bus delivers frames as `nv12`, `yuv420p`, or `rgb24`
//! (see `ados_protocol::framebus::FrameFormat`). The optical-flow
//! tracker and the VIO bridge both want an 8-bit single-channel
//! grayscale (luma) image. This module turns one resolved
//! [`Frame`](ados_sdk::vision::Frame) into a packed row-major
//! `GrayImage` without pulling in an image library.
//!
//! - `nv12` / `yuv420p` are planar: the luma plane is the first
//!   `width * height` bytes, copied directly.
//! - `rgb24` is packed 3 bytes/pixel: luma is the Rec. 601 weighted
//!   sum per pixel.
//!
//! A frame whose pixel buffer is shorter than the format requires
//! (a truncated capture) yields `None` so the tracker drops it rather
//! than reading past the buffer.

use ados_protocol::framebus::FrameFormat;
use ados_sdk::vision::Frame;

/// A packed 8-bit grayscale image, row-major, one byte per pixel.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GrayImage {
    pub width: u32,
    pub height: u32,
    pub data: Vec<u8>,
}

impl GrayImage {
    /// Luma of the pixel at `(x, y)`; out-of-range reads as 0.
    #[inline]
    pub fn at(&self, x: u32, y: u32) -> u8 {
        if x >= self.width || y >= self.height {
            return 0;
        }
        let idx = y as usize * self.width as usize + x as usize;
        self.data.get(idx).copied().unwrap_or(0)
    }

    /// Bilinear-sampled luma at a sub-pixel `(x, y)`, clamped to the
    /// image bounds. Used by the Lucas-Kanade warp step.
    #[inline]
    pub fn sample(&self, x: f32, y: f32) -> f32 {
        if self.width == 0 || self.height == 0 {
            return 0.0;
        }
        let xc = x.clamp(0.0, (self.width - 1) as f32);
        let yc = y.clamp(0.0, (self.height - 1) as f32);
        let x0 = xc.floor() as u32;
        let y0 = yc.floor() as u32;
        let x1 = (x0 + 1).min(self.width - 1);
        let y1 = (y0 + 1).min(self.height - 1);
        let fx = xc - x0 as f32;
        let fy = yc - y0 as f32;
        let p00 = self.at(x0, y0) as f32;
        let p10 = self.at(x1, y0) as f32;
        let p01 = self.at(x0, y1) as f32;
        let p11 = self.at(x1, y1) as f32;
        let top = p00 + (p10 - p00) * fx;
        let bot = p01 + (p11 - p01) * fx;
        top + (bot - top) * fy
    }
}

/// Convert a resolved shared-bus frame to a grayscale image, or `None`
/// when the pixel buffer is empty or too short for the declared format.
pub fn frame_to_gray(frame: &Frame) -> Option<GrayImage> {
    let w = frame.descriptor.width;
    let h = frame.descriptor.height;
    if w == 0 || h == 0 || frame.pixels.is_empty() {
        return None;
    }
    let px = w as usize * h as usize;
    match frame.descriptor.format {
        FrameFormat::Nv12 | FrameFormat::Yuv420p => {
            // The luma plane is the first width*height bytes.
            if frame.pixels.len() < px {
                return None;
            }
            Some(GrayImage {
                width: w,
                height: h,
                data: frame.pixels[..px].to_vec(),
            })
        }
        FrameFormat::Rgb24 => {
            if frame.pixels.len() < px * 3 {
                return None;
            }
            let mut data = Vec::with_capacity(px);
            for i in 0..px {
                let base = i * 3;
                let r = frame.pixels[base] as u32;
                let g = frame.pixels[base + 1] as u32;
                let b = frame.pixels[base + 2] as u32;
                // 0.299R + 0.587G + 0.114B in fixed point (sum 1000).
                data.push(((299 * r + 587 * g + 114 * b) / 1000) as u8);
            }
            Some(GrayImage {
                width: w,
                height: h,
                data,
            })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ados_protocol::framebus::FrameDescriptor;

    fn frame(format: FrameFormat, w: u32, h: u32, pixels: Vec<u8>) -> Frame {
        Frame {
            descriptor: FrameDescriptor {
                camera_id: "uvc-0".into(),
                frame_id: 1,
                ts_ms: 0,
                width: w,
                height: h,
                format,
                shm_name: "ados-vision-uvc-0".into(),
                slot: 0,
                seq: 1,
                byte_len: pixels.len() as u32,
            },
            pixels,
        }
    }

    #[test]
    fn planar_luma_is_the_first_plane() {
        // 2x2 yuv420p: 4 luma bytes then 2 chroma bytes.
        let f = frame(FrameFormat::Yuv420p, 2, 2, vec![10, 20, 30, 40, 128, 128]);
        let g = frame_to_gray(&f).unwrap();
        assert_eq!(g.data, vec![10, 20, 30, 40]);
        assert_eq!(g.at(1, 1), 40);
    }

    #[test]
    fn rgb24_luma_uses_rec601() {
        // One grey pixel r=g=b=200 -> luma 200; one pure-green pixel.
        let f = frame(FrameFormat::Rgb24, 2, 1, vec![200, 200, 200, 0, 255, 0]);
        let g = frame_to_gray(&f).unwrap();
        assert_eq!(g.at(0, 0), 200);
        // 0.587 * 255 ~= 149.
        assert_eq!(g.at(1, 0), 149);
    }

    #[test]
    fn truncated_buffer_is_none() {
        let f = frame(FrameFormat::Rgb24, 4, 4, vec![0u8; 10]);
        assert!(frame_to_gray(&f).is_none());
    }

    #[test]
    fn bilinear_sample_interpolates() {
        let g = GrayImage {
            width: 2,
            height: 1,
            data: vec![0, 100],
        };
        assert!((g.sample(0.5, 0.0) - 50.0).abs() < 1e-3);
    }
}
