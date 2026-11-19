//! Tests for the Lucas-Kanade tracker in `flow.rs`.

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
    let r = empty_result(
        33333,
        0.033,
        Some(GyroReading {
            xgyro: 0.0,
            ygyro: 0.0,
            zgyro: 0.5,
        }),
    );
    assert_eq!(r.quality, 0);
    assert!((r.integrated.zgyro - 0.5 * 0.033).abs() < 1e-6);
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
fn optical_flow_rad_integrates_angle() {
    let dt = 0.033;
    let gyro = GyroReading {
        xgyro: 0.2,
        ygyro: -0.1,
        zgyro: 0.05,
    };
    let i = integrate_flow(3.0, 0.0, 400.0, dt, Some(gyro));
    assert!((i.x - (3.0f32 / 400.0).atan()).abs() < 1e-7);
    assert_eq!(i.y, 0.0);
    assert!((i.xgyro - 0.2 * dt).abs() < 1e-7);
    assert!((i.ygyro + 0.1 * dt).abs() < 1e-7);
    assert!((i.zgyro - 0.05 * dt).abs() < 1e-7);
}

#[test]
fn gyro_does_not_derotate_the_raw_rate() {
    let prev = textured(96, 96, 0, 0);
    let curr = textured(96, 96, 2, 0);
    let mut lk = OpticalFlowLk::new();
    let still = lk.process(&prev, &curr, 1.0 / 30.0, None, None);
    let gyro = GyroReading {
        xgyro: 1.0,
        ygyro: 1.0,
        zgyro: 1.0,
    };
    let turning = lk.process(&prev, &curr, 1.0 / 30.0, Some(gyro), None);
    assert_eq!(still.flow_rate_x, turning.flow_rate_x);
    assert_eq!(still.integrated.x, turning.integrated.x);
}

#[test]
fn wide_frames_track_at_half_resolution() {
    // A 4 px shift at 1280 wide is 2 px on the half-resolution copy;
    // the reported angle and dpi stay in full-resolution terms.
    let prev = textured(1280, 96, 0, 0);
    let curr = textured(1280, 96, 4, 0);
    let mut lk = OpticalFlowLk::new();
    let r = lk.process(&prev, &curr, 1.0 / 30.0, None, None);
    assert!(r.quality > 0);
    let dx = r.flow_x_dpi / DPI_SCALE;
    assert!((dx - 4.0).abs() < 1.5, "dx ~= 4, got {dx}");
    let expect = (4.0f32 / DEFAULT_FX_PIXELS).atan();
    assert!(
        (r.integrated.x - expect).abs() < 0.004,
        "{}",
        r.integrated.x
    );
}

#[test]
fn median_of_odd_and_even() {
    assert_eq!(median([1.0, 3.0, 2.0].into_iter()), 2.0);
    assert_eq!(median([1.0, 2.0, 3.0, 4.0].into_iter()), 2.5);
}
