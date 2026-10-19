//! Per-frame image measures on a quarter-resolution luma plane: sharpness
//! (variance of the Laplacian) and texture (FAST-9 corner count).

use ados_protocol::framebus::FrameFormat;

/// A quarter-resolution 8-bit luma plane (each pixel the mean of a 4x4 block).
#[derive(Debug, Clone, PartialEq)]
pub struct Luma {
    pub width: usize,
    pub height: usize,
    pub data: Vec<u8>,
}

impl Luma {
    fn at(&self, x: usize, y: usize) -> i32 {
        self.data[y * self.width + x] as i32
    }

    /// Mean intensity, 0..255.
    pub fn mean(&self) -> f64 {
        if self.data.is_empty() {
            return 0.0;
        }
        self.data.iter().map(|&v| v as f64).sum::<f64>() / self.data.len() as f64
    }
}

/// The luma of one full-resolution pixel; the caller has checked the buffer
/// length for the format.
fn luma_reader(
    format: FrameFormat,
    width: usize,
    pixels: &[u8],
) -> impl Fn(usize, usize) -> u32 + '_ {
    move |x, y| match format {
        FrameFormat::Gray8 | FrameFormat::BayerRggb8 | FrameFormat::Nv12 | FrameFormat::Yuv420p => {
            pixels[y * width + x] as u32
        }
        FrameFormat::Gray16 | FrameFormat::BayerRggb16 => pixels[(y * width + x) * 2 + 1] as u32,
        FrameFormat::Rgb24 => {
            let i = (y * width + x) * 3;
            (pixels[i] as u32 * 77 + pixels[i + 1] as u32 * 150 + pixels[i + 2] as u32 * 29) >> 8
        }
    }
}

/// Downsample a frame to quarter-resolution luma. A Bayer mosaic averaged
/// over 4x4 blocks is a valid luma proxy (each block holds every colour).
pub fn quarter_luma(format: FrameFormat, width: u32, height: u32, pixels: &[u8]) -> Option<Luma> {
    let (w, h) = (width as usize, height as usize);
    if pixels.len() < format.frame_bytes(width, height) || w < 4 || h < 4 {
        return None;
    }
    let (qw, qh) = (w / 4, h / 4);
    let read = luma_reader(format, w, pixels);
    let mut data = Vec::with_capacity(qw * qh);
    for qy in 0..qh {
        for qx in 0..qw {
            let mut sum = 0u32;
            for dy in 0..4 {
                for dx in 0..4 {
                    sum += read(qx * 4 + dx, qy * 4 + dy);
                }
            }
            data.push((sum / 16) as u8);
        }
    }
    Some(Luma {
        width: qw,
        height: qh,
        data,
    })
}

/// Variance of the 4-neighbour Laplacian: high for a sharp image, low for a
/// blurred or featureless one.
pub fn blur_score(l: &Luma) -> f64 {
    if l.width < 3 || l.height < 3 {
        return 0.0;
    }
    let mut n = 0.0;
    let (mut sum, mut sq) = (0.0f64, 0.0f64);
    for y in 1..l.height - 1 {
        for x in 1..l.width - 1 {
            let v = (l.at(x - 1, y) + l.at(x + 1, y) + l.at(x, y - 1) + l.at(x, y + 1)
                - 4 * l.at(x, y)) as f64;
            sum += v;
            sq += v * v;
            n += 1.0;
        }
    }
    let mean = sum / n;
    sq / n - mean * mean
}

/// The 16-pixel Bresenham circle of radius 3 used by FAST.
const CIRCLE: [(i32, i32); 16] = [
    (0, -3),
    (1, -3),
    (2, -2),
    (3, -1),
    (3, 0),
    (3, 1),
    (2, 2),
    (1, 3),
    (0, 3),
    (-1, 3),
    (-2, 2),
    (-3, 1),
    (-3, 0),
    (-3, -1),
    (-2, -3),
    (-1, -3),
];

/// FAST-9 corners with intensity threshold `t`, non-maximum suppressed on a
/// 3x3 neighbourhood by corner score.
pub fn fast_corners(l: &Luma, t: i32) -> usize {
    if l.width < 7 || l.height < 7 {
        return 0;
    }
    let (w, h) = (l.width, l.height);
    let mut score = vec![0i32; w * h];
    for y in 3..h - 3 {
        for x in 3..w - 3 {
            let c = l.at(x, y);
            let ring: [i32; 16] = std::array::from_fn(|i| {
                let (dx, dy) = CIRCLE[i];
                l.at((x as i32 + dx) as usize, (y as i32 + dy) as usize)
            });
            let arc = |bright: bool| {
                let mut run = 0;
                for i in 0..32 {
                    let v = ring[i % 16];
                    let hit = if bright { v > c + t } else { v < c - t };
                    run = if hit { run + 1 } else { 0 };
                    if run >= 9 {
                        return true;
                    }
                }
                false
            };
            if arc(true) || arc(false) {
                score[y * w + x] = ring.iter().map(|v| (v - c).abs()).sum();
            }
        }
    }
    let mut n = 0;
    for y in 3..h - 3 {
        for x in 3..w - 3 {
            let s = score[y * w + x];
            if s == 0 {
                continue;
            }
            let max = (y - 1..=y + 1)
                .flat_map(|yy| (x - 1..=x + 1).map(move |xx| (xx, yy)))
                .filter(|&(xx, yy)| (xx, yy) != (x, y))
                .all(|(xx, yy)| {
                    score[yy * w + xx] < s || (score[yy * w + xx] == s && (yy, xx) > (y, x))
                });
            if max {
                n += 1;
            }
        }
    }
    n
}

/// The FAST threshold capture uses.
pub const FAST_THRESHOLD: i32 = 20;

#[cfg(test)]
mod tests {
    use super::*;

    fn checker(w: u32, h: u32, cell: u32) -> Vec<u8> {
        (0..h)
            .flat_map(|y| {
                (0..w).map(move |x| {
                    if (x / cell + y / cell).is_multiple_of(2) {
                        230
                    } else {
                        20
                    }
                })
            })
            .collect()
    }

    fn box_blur(px: &[u8], w: usize, h: usize, r: usize) -> Vec<u8> {
        let mut out = px.to_vec();
        for y in r..h - r {
            for x in r..w - r {
                let mut s = 0u32;
                for yy in y - r..=y + r {
                    for xx in x - r..=x + r {
                        s += px[yy * w + xx] as u32;
                    }
                }
                out[y * w + x] = (s / ((2 * r + 1) * (2 * r + 1)) as u32) as u8;
            }
        }
        out
    }

    #[test]
    fn a_blurred_image_scores_lower_than_a_sharp_one() {
        let (w, h) = (320, 240);
        let sharp = checker(w, h, 24);
        let blurred = box_blur(&sharp, w as usize, h as usize, 8);
        let s = blur_score(&quarter_luma(FrameFormat::Gray8, w, h, &sharp).unwrap());
        let b = blur_score(&quarter_luma(FrameFormat::Gray8, w, h, &blurred).unwrap());
        assert!(s > 4.0 * b, "sharp {s} blurred {b}");
        let flat = vec![128u8; (w * h) as usize];
        assert_eq!(
            blur_score(&quarter_luma(FrameFormat::Gray8, w, h, &flat).unwrap()),
            0.0
        );
    }

    #[test]
    fn texture_counts_corners_and_a_plain_wall_has_none() {
        let (w, h) = (640, 480);
        let busy = quarter_luma(FrameFormat::Gray8, w, h, &checker(w, h, 32)).unwrap();
        let wall = quarter_luma(FrameFormat::Gray8, w, h, &vec![200u8; (w * h) as usize]).unwrap();
        let n = fast_corners(&busy, FAST_THRESHOLD);
        assert!(n >= 60, "{n}");
        assert_eq!(fast_corners(&wall, FAST_THRESHOLD), 0);
    }

    #[test]
    fn every_format_reduces_to_luma() {
        let (w, h) = (8, 8);
        let rgb = vec![100u8; 8 * 8 * 3];
        let l = quarter_luma(FrameFormat::Rgb24, w, h, &rgb).unwrap();
        assert_eq!((l.width, l.height), (2, 2));
        assert!((l.mean() - 100.0).abs() <= 1.0);
        let g16: Vec<u8> = (0..64).flat_map(|_| [0u8, 50u8]).collect();
        assert_eq!(
            quarter_luma(FrameFormat::Gray16, w, h, &g16)
                .unwrap()
                .mean(),
            50.0
        );
        assert!(quarter_luma(FrameFormat::Gray8, w, h, &[0u8; 10]).is_none());
    }
}
