//! Lucas-Kanade pyramidal optical-flow tracker.
//!
//! Two consecutive grayscale frames plus a `dt` produce image-plane
//! flow (tenths-of-a-pixel scale), angular flow rates (rad/s), and an
//! optional metric translational velocity when a height-above-ground
//! distance is supplied. The output shape matches the MAVLink
//! `OPTICAL_FLOW_RAD` message so the emit layer copies fields without
//! further math.
//!
//! The tracker is a clean-room Rust implementation of sparse pyramidal
//! Lucas-Kanade optical flow:
//!
//! 1. Shi-Tomasi cornerness picks up to [`MAX_FEATURES`] strong corners
//!    on a grid over the previous frame (one candidate per cell keeps
//!    the corners spatially spread, like a min-distance constraint).
//! 2. Each corner is tracked into the current frame with the iterative
//!    Lucas-Kanade equations over a small pyramid (coarse-to-fine), so
//!    displacements larger than one window are still recovered.
//! 3. The per-corner displacements are reduced to a robust median after
//!    a magnitude-outlier reject ([`MAX_PIXEL_DELTA`]).
//!
//! Gyro derotation and the metric-velocity branch follow the same
//! formulas the prior implementation used, so the wire values the FC
//! consumes are unchanged.

/// Pixel displacement larger than this is a tracking outlier and is
/// dropped from the median. Real downward flow at typical descent
/// rates stays well below this on a 640x480 frame.
pub const MAX_PIXEL_DELTA: f32 = 30.0;

/// `OPTICAL_FLOW.flow_x` / `flow_y` are tenths of a pixel per the
/// message comment, but the wider ecosystem treats the legacy scale as
/// 8x. This path follows the 8x scaling so the value lines up with what
/// firmware expects on the wire.
pub const DPI_SCALE: f32 = 8.0;

const MAX_FEATURES: usize = 200;
const DEFAULT_FOV_HORIZONTAL_DEG: f32 = 60.0;
const DEFAULT_FX_PIXELS: f32 = 500.0;
/// Half-window radius for the LK integration window (a 15x15 window).
const WIN_RADIUS: i32 = 7;
/// Pyramid levels (0 = single resolution). Three levels matches the
/// prior `maxLevel=3`.
const PYRAMID_LEVELS: usize = 3;
/// LK refinement iterations per pyramid level.
const LK_ITERS: usize = 20;
/// Cornerness grid cell side; one corner candidate per cell keeps the
/// features spread (a cheap stand-in for a min-distance constraint).
const CELL: u32 = 16;
/// Minimum Shi-Tomasi score (scaled min-eigenvalue of the gradient
/// structure tensor) for a grid cell to contribute a corner. Rejects
/// flat / textureless patches that carry no usable flow.
const CORNER_MIN_SCORE: f32 = 0.2;

/// A latest gyro reading in rad/s, body frame.
#[derive(Debug, Clone, Copy)]
pub struct GyroReading {
    pub xgyro: f32,
    pub ygyro: f32,
    pub zgyro: f32,
}

/// One optical-flow sample, ready for `OPTICAL_FLOW_RAD` emission.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OpticalFlowResult {
    pub flow_x_dpi: f32,
    pub flow_y_dpi: f32,
    pub flow_comp_m_x: f32,
    pub flow_comp_m_y: f32,
    pub flow_rate_x: f32,
    pub flow_rate_y: f32,
    pub flow_rate_z: f32,
    pub quality: i32,
    pub integration_time_us: i32,
}

use crate::framing::GrayImage;

/// Stateful pyramidal Lucas-Kanade flow estimator. Construct once per
/// pipeline; call [`process`](OpticalFlowLk::process) on every
/// consecutive frame pair.
pub struct OpticalFlowLk {
    max_features: usize,
    fov_horizontal_rad: f32,
    fx_pixels: f32,
    fov_vertical_rad: f32,
}

impl Default for OpticalFlowLk {
    fn default() -> Self {
        Self {
            max_features: MAX_FEATURES,
            fov_horizontal_rad: DEFAULT_FOV_HORIZONTAL_DEG.to_radians(),
            fx_pixels: DEFAULT_FX_PIXELS,
            fov_vertical_rad: 0.0,
        }
    }
}

impl OpticalFlowLk {
    pub fn new() -> Self {
        Self::default()
    }

    /// Compute one flow sample from a pair of consecutive frames.
    ///
    /// `dt_seconds` is the interval; a clamp guards divide-by-zero.
    /// `gyro`, when present, is subtracted from the visual flow rate.
    /// `distance_m`, when present and positive, yields a metric
    /// translational velocity.
    pub fn process(
        &mut self,
        prev: &GrayImage,
        curr: &GrayImage,
        dt_seconds: f32,
        gyro: Option<GyroReading>,
        distance_m: Option<f32>,
    ) -> OpticalFlowResult {
        let w = curr.width;
        let h = curr.height;
        let aspect = if w > 0 { h as f32 / w as f32 } else { 0.75 };
        self.fov_vertical_rad = self.fov_horizontal_rad * aspect;

        let dt = dt_seconds.max(1e-6);
        let integration_time_us = (dt * 1_000_000.0).round() as i32;

        let corners = good_features_to_track(prev, self.max_features);
        if corners.is_empty() {
            return empty_result(integration_time_us, gyro);
        }

        let mut deltas: Vec<(f32, f32)> = Vec::with_capacity(corners.len());
        for &(cx, cy) in &corners {
            if let Some((dx, dy)) = track_pyramidal(prev, curr, cx, cy) {
                deltas.push((dx, dy));
            }
        }
        if deltas.is_empty() {
            return empty_result(integration_time_us, gyro);
        }

        // Magnitude outlier reject.
        deltas.retain(|&(dx, dy)| (dx * dx + dy * dy).sqrt() <= MAX_PIXEL_DELTA);
        if deltas.is_empty() {
            return empty_result(integration_time_us, gyro);
        }

        let dx_pixels = median(deltas.iter().map(|d| d.0));
        let dy_pixels = median(deltas.iter().map(|d| d.1));
        let num_tracked = deltas.len() as i32;

        let flow_x_dpi = dx_pixels * DPI_SCALE;
        let flow_y_dpi = dy_pixels * DPI_SCALE;

        let mut flow_rate_x = (dx_pixels / self.fx_pixels).atan() / dt;
        let mut flow_rate_y = (dy_pixels / self.fx_pixels).atan() / dt;
        let mut flow_rate_z = 0.0;

        if let Some(g) = gyro {
            flow_rate_x -= g.xgyro * self.fov_vertical_rad / 2.0;
            flow_rate_y -= g.ygyro * self.fov_horizontal_rad / 2.0;
            flow_rate_z = g.zgyro;
        }

        let (flow_comp_m_x, flow_comp_m_y) = match distance_m {
            Some(d) if d > 0.0 => (flow_rate_x * d, flow_rate_y * d),
            _ => (0.0, 0.0),
        };

        let quality = (num_tracked * 5).clamp(0, 255);

        OpticalFlowResult {
            flow_x_dpi,
            flow_y_dpi,
            flow_comp_m_x,
            flow_comp_m_y,
            flow_rate_x,
            flow_rate_y,
            flow_rate_z,
            quality,
            integration_time_us,
        }
    }
}

/// A zero-flow result, carrying gyro Z if present. Returned when no
/// features track this frame pair.
fn empty_result(integration_time_us: i32, gyro: Option<GyroReading>) -> OpticalFlowResult {
    OpticalFlowResult {
        flow_x_dpi: 0.0,
        flow_y_dpi: 0.0,
        flow_comp_m_x: 0.0,
        flow_comp_m_y: 0.0,
        flow_rate_x: 0.0,
        flow_rate_y: 0.0,
        flow_rate_z: gyro.map(|g| g.zgyro).unwrap_or(0.0),
        quality: 0,
        integration_time_us,
    }
}

/// Median of a float iterator. Empty input returns 0.
fn median<I: Iterator<Item = f32>>(it: I) -> f32 {
    let mut v: Vec<f32> = it.collect();
    if v.is_empty() {
        return 0.0;
    }
    v.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let n = v.len();
    if n % 2 == 1 {
        v[n / 2]
    } else {
        0.5 * (v[n / 2 - 1] + v[n / 2])
    }
}

/// Shi-Tomasi cornerness over a grid: one strongest corner per
/// [`CELL`]x[`CELL`] cell, capped at `max`. The min-eigenvalue of the
/// 2x2 gradient structure tensor is the corner score.
fn good_features_to_track(img: &GrayImage, max: usize) -> Vec<(f32, f32)> {
    let w = img.width;
    let h = img.height;
    if w < 3 || h < 3 {
        return Vec::new();
    }
    let mut corners: Vec<(f32, f32)> = Vec::new();
    let mut y = 1u32;
    while y + CELL <= h.saturating_sub(1).max(1) {
        let mut x = 1u32;
        while x + CELL <= w.saturating_sub(1).max(1) {
            let mut best_score = 0.0f32;
            let mut best = None;
            for dy in 0..CELL {
                for dx in 0..CELL {
                    let px = x + dx;
                    let py = y + dy;
                    if px == 0 || py == 0 || px + 1 >= w || py + 1 >= h {
                        continue;
                    }
                    let score = shi_tomasi(img, px, py);
                    if score > best_score {
                        best_score = score;
                        best = Some((px as f32, py as f32));
                    }
                }
            }
            // Only keep cells with real 2D structure; a flat cell on a
            // textureless ground patch carries no usable flow. The
            // threshold is on the min-eigenvalue of the gradient
            // structure tensor (Shi-Tomasi), scaled into a small
            // range; a corner needs gradient energy in both directions.
            if best_score > CORNER_MIN_SCORE {
                if let Some(c) = best {
                    corners.push(c);
                }
            }
            x += CELL;
        }
        y += CELL;
    }
    if corners.len() > max {
        corners.truncate(max);
    }
    corners
}

/// Min-eigenvalue of the 3x3 gradient structure tensor at `(x, y)`
/// (Shi-Tomasi corner score). Scaled down so the threshold is in a
/// human-readable range.
fn shi_tomasi(img: &GrayImage, x: u32, y: u32) -> f32 {
    let mut ixx = 0.0f32;
    let mut iyy = 0.0f32;
    let mut ixy = 0.0f32;
    for wy in -1i32..=1 {
        for wx in -1i32..=1 {
            let px = (x as i32 + wx) as u32;
            let py = (y as i32 + wy) as u32;
            let (gx, gy) = sobel(img, px, py);
            ixx += gx * gx;
            iyy += gy * gy;
            ixy += gx * gy;
        }
    }
    // Min eigenvalue of [[ixx, ixy], [ixy, iyy]].
    let trace = ixx + iyy;
    let det = ixx * iyy - ixy * ixy;
    let disc = (trace * trace / 4.0 - det).max(0.0).sqrt();
    let lambda_min = trace / 2.0 - disc;
    lambda_min / 255.0
}

/// Central-difference gradient at `(x, y)`.
#[inline]
fn sobel(img: &GrayImage, x: u32, y: u32) -> (f32, f32) {
    let xm = x.saturating_sub(1);
    let xp = (x + 1).min(img.width.saturating_sub(1));
    let ym = y.saturating_sub(1);
    let yp = (y + 1).min(img.height.saturating_sub(1));
    let gx = (img.at(xp, y) as f32 - img.at(xm, y) as f32) * 0.5;
    let gy = (img.at(x, yp) as f32 - img.at(x, ym) as f32) * 0.5;
    (gx, gy)
}

/// Track a corner from `prev` to `curr` with coarse-to-fine pyramidal
/// Lucas-Kanade. Returns the `(dx, dy)` displacement, or `None` when
/// the structure tensor is degenerate (no trackable gradient).
fn track_pyramidal(
    prev: &GrayImage,
    curr: &GrayImage,
    cx: f32,
    cy: f32,
) -> Option<(f32, f32)> {
    // Build downsampled pyramids; level 0 is full resolution.
    let prev_pyr = build_pyramid(prev, PYRAMID_LEVELS);
    let curr_pyr = build_pyramid(curr, PYRAMID_LEVELS);

    let mut flow = (0.0f32, 0.0f32);
    for level in (0..prev_pyr.len()).rev() {
        let scale = (1u32 << level) as f32;
        let p = &prev_pyr[level];
        let c = &curr_pyr[level];
        let lx = cx / scale;
        let ly = cy / scale;
        // Carry the flow estimate from the coarser level (it halves
        // each step down, so double it as we go finer).
        flow = (flow.0 * 2.0, flow.1 * 2.0);
        if let Some(refined) = lk_refine(p, c, lx, ly, flow) {
            flow = refined;
        } else if level == 0 {
            return None;
        }
    }
    Some(flow)
}

/// Build a `levels`-deep half-resolution pyramid (level 0 = the input).
fn build_pyramid(img: &GrayImage, levels: usize) -> Vec<GrayImage> {
    let mut pyr = Vec::with_capacity(levels);
    pyr.push(img.clone());
    for _ in 1..levels {
        let prev = pyr.last().unwrap();
        if prev.width < 4 || prev.height < 4 {
            break;
        }
        pyr.push(downsample_half(prev));
    }
    pyr
}

/// 2x2 box-average downsample to half resolution.
fn downsample_half(img: &GrayImage) -> GrayImage {
    let nw = img.width / 2;
    let nh = img.height / 2;
    let mut data = vec![0u8; (nw * nh) as usize];
    for y in 0..nh {
        for x in 0..nw {
            let sx = x * 2;
            let sy = y * 2;
            let s = img.at(sx, sy) as u32
                + img.at(sx + 1, sy) as u32
                + img.at(sx, sy + 1) as u32
                + img.at(sx + 1, sy + 1) as u32;
            data[(y * nw + x) as usize] = (s / 4) as u8;
        }
    }
    GrayImage {
        width: nw,
        height: nh,
        data,
    }
}

/// Iterative Lucas-Kanade refinement of the `(dx, dy)` flow at a point.
///
/// Solves for the displacement `d` such that `prev(x) ~= curr(x + d)`
/// (the motion of the feature from `prev` into `curr`). Forward-additive
/// Gauss-Newton: the spatial gradient is taken on the *previous* image
/// (constant across iterations, so the 2x2 normal matrix `G` is built
/// once) and each iteration updates `d` by `G^-1 * b` with
/// `b = Σ ∇prev · (curr(x+d) - prev(x))`. Returns `None` when `G` is
/// near-singular (a textureless window).
fn lk_refine(
    prev: &GrayImage,
    curr: &GrayImage,
    px: f32,
    py: f32,
    init: (f32, f32),
) -> Option<(f32, f32)> {
    // Structure tensor over the window in the previous image.
    let mut gxx = 0.0f64;
    let mut gyy = 0.0f64;
    let mut gxy = 0.0f64;
    let mut grads: Vec<(f32, f32, f32)> = Vec::new(); // (gx, gy, prev_val)
    for wy in -WIN_RADIUS..=WIN_RADIUS {
        for wx in -WIN_RADIUS..=WIN_RADIUS {
            let sx = px + wx as f32;
            let sy = py + wy as f32;
            let gx = (prev.sample(sx + 1.0, sy) - prev.sample(sx - 1.0, sy)) * 0.5;
            let gy = (prev.sample(sx, sy + 1.0) - prev.sample(sx, sy - 1.0)) * 0.5;
            gxx += (gx * gx) as f64;
            gyy += (gy * gy) as f64;
            gxy += (gx * gy) as f64;
            grads.push((gx, gy, prev.sample(sx, sy)));
        }
    }
    let det = gxx * gyy - gxy * gxy;
    if det.abs() < 1e-3 {
        return None;
    }

    let mut flow = init;
    for _ in 0..LK_ITERS {
        let mut bx = 0.0f64;
        let mut by = 0.0f64;
        let mut idx = 0usize;
        for wy in -WIN_RADIUS..=WIN_RADIUS {
            for wx in -WIN_RADIUS..=WIN_RADIUS {
                let sx = px + wx as f32;
                let sy = py + wy as f32;
                let (gx, gy, prev_val) = grads[idx];
                idx += 1;
                let curr_val = curr.sample(sx + flow.0, sy + flow.1);
                // Residual: how much the warped current image still
                // differs from the previous patch. Minimizing it drives
                // `d` toward the true motion prev -> curr.
                let e = curr_val - prev_val;
                bx += (gx * e) as f64;
                by += (gy * e) as f64;
            }
        }
        // d = G^-1 * b; subtract because the step reduces the residual.
        let step_x = (gyy * bx - gxy * by) / det;
        let step_y = (gxx * by - gxy * bx) / det;
        flow.0 -= step_x as f32;
        flow.1 -= step_y as f32;
        if step_x.abs() < 0.01 && step_y.abs() < 0.01 {
            break;
        }
    }
    if flow.0.is_finite() && flow.1.is_finite() {
        Some(flow)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A deterministic integer hash -> [0, 255], for the value-noise
    /// test texture.
    fn hash2(x: i32, y: i32) -> u8 {
        let mut h = (x as u32).wrapping_mul(0x9E37_79B1) ^ (y as u32).wrapping_mul(0x85EB_CA77);
        h ^= h >> 15;
        h = h.wrapping_mul(0x2545_F491);
        h ^= h >> 13;
        (h & 0xFF) as u8
    }

    /// Build a textured image translated by `(shift_x, shift_y)`.
    ///
    /// The base pattern is bilinearly-upsampled value noise (a hash on a
    /// coarse grid, smoothed). That gives strong, well-defined 2D
    /// structure (real corners the Shi-Tomasi score picks up) while
    /// being *aperiodic*. Aperiodicity matters: a tiling checker pattern
    /// would let the tracker lock onto an aliased shift (e.g. -4 instead
    /// of +2), which is a property of the test image, not the tracker.
    fn textured(w: u32, h: u32, shift_x: i32, shift_y: i32) -> GrayImage {
        // 4 px per noise cell: fine enough for dense corners, coarse
        // enough that bilinear smoothing leaves clean gradients.
        const CELL_PX: i32 = 4;
        let mut data = vec![0u8; (w * h) as usize];
        for y in 0..h as i32 {
            for x in 0..w as i32 {
                let sx = x - shift_x;
                let sy = y - shift_y;
                let gx = sx.div_euclid(CELL_PX);
                let gy = sy.div_euclid(CELL_PX);
                let fx = (sx.rem_euclid(CELL_PX)) as f32 / CELL_PX as f32;
                let fy = (sy.rem_euclid(CELL_PX)) as f32 / CELL_PX as f32;
                let c00 = hash2(gx, gy) as f32;
                let c10 = hash2(gx + 1, gy) as f32;
                let c01 = hash2(gx, gy + 1) as f32;
                let c11 = hash2(gx + 1, gy + 1) as f32;
                let top = c00 + (c10 - c00) * fx;
                let bot = c01 + (c11 - c01) * fx;
                let v = top + (bot - top) * fy;
                data[(y as u32 * w + x as u32) as usize] = v.clamp(0.0, 255.0) as u8;
            }
        }
        GrayImage {
            width: w,
            height: h,
            data,
        }
    }

    #[test]
    fn detects_corners_on_textured_image() {
        let img = textured(64, 64, 0, 0);
        let corners = good_features_to_track(&img, 200);
        assert!(!corners.is_empty(), "textured image yields corners");
    }

    #[test]
    fn no_corners_on_flat_image() {
        let flat = GrayImage {
            width: 64,
            height: 64,
            data: vec![128u8; 64 * 64],
        };
        assert!(good_features_to_track(&flat, 200).is_empty());
    }

    #[test]
    fn tracks_a_known_translation() {
        // Curr is prev shifted right by 2 px and down by 1 px. The
        // tracked flow should recover roughly (+2, +1).
        let prev = textured(96, 96, 0, 0);
        let curr = textured(96, 96, 2, 1);
        let mut lk = OpticalFlowLk::new();
        let result = lk.process(&prev, &curr, 1.0 / 30.0, None, None);
        assert!(result.quality > 0, "some features tracked");
        // Recovered median displacement, in pixels (flow_x_dpi = dx*8).
        let dx = result.flow_x_dpi / DPI_SCALE;
        let dy = result.flow_y_dpi / DPI_SCALE;
        assert!((dx - 2.0).abs() < 1.0, "dx ~= 2, got {dx}");
        assert!((dy - 1.0).abs() < 1.0, "dy ~= 1, got {dy}");
    }

    #[test]
    fn empty_result_carries_gyro_z() {
        let r = empty_result(33333, Some(GyroReading { xgyro: 0.0, ygyro: 0.0, zgyro: 0.5 }));
        assert_eq!(r.quality, 0);
        assert_eq!(r.flow_rate_z, 0.5);
    }

    #[test]
    fn metric_velocity_scales_with_distance() {
        let prev = textured(96, 96, 0, 0);
        let curr = textured(96, 96, 3, 0);
        let mut lk = OpticalFlowLk::new();
        let near = lk.process(&prev, &curr, 1.0 / 30.0, None, Some(2.0));
        let far = lk.process(&prev, &curr, 1.0 / 30.0, None, Some(4.0));
        // flow_comp scales linearly with distance for the same flow.
        if near.flow_comp_m_x.abs() > 1e-6 {
            let ratio = far.flow_comp_m_x / near.flow_comp_m_x;
            assert!((ratio - 2.0).abs() < 0.1, "metric scales ~2x, got {ratio}");
        }
    }

    #[test]
    fn median_of_odd_and_even() {
        assert_eq!(median([1.0, 3.0, 2.0].into_iter()), 2.0);
        assert_eq!(median([1.0, 2.0, 3.0, 4.0].into_iter()), 2.5);
    }
}
