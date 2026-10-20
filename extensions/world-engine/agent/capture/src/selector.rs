//! Keyframe selection: decide when the camera has moved enough to be worth a
//! new keyframe. One selector per camera; the selection is purely a function of
//! pose deltas and elapsed time against the last accepted keyframe, with the
//! translation baseline scaled by the scene depth when a LiDAR scan gives it.

use crate::config::SelectionParams;
use world_engine_protocol::atlas::Pose;

/// Per-camera keyframe gate. Holds the last accepted keyframe's pose and
/// timestamp; the next frame is accepted when it crosses any selection
/// threshold. A fresh selector (no last keyframe) accepts the first frame.
#[derive(Debug, Clone, Default)]
pub struct KeyframeSelector {
    last: Option<(Pose, i64)>,
}

impl KeyframeSelector {
    /// Decide whether `pose` at `ts_ms` should become a keyframe, given the
    /// thresholds in `params` and the median scene depth (`None` without a
    /// LiDAR scan). Returns `true` and records the pose as the new last
    /// keyframe when ANY of: this is the first frame (no last keyframe); the
    /// translation baseline from the last keyframe is `>=`
    /// [`SelectionParams::baseline_m`]; the rotation from the last keyframe is
    /// `>= min_rotation_rad`; or the time since the last keyframe is
    /// `>= max_interval_ms`.
    pub fn should_select(
        &mut self,
        pose: &Pose,
        ts_ms: i64,
        params: &SelectionParams,
        scene_depth_m: Option<f64>,
    ) -> bool {
        let select = self.peek_select(pose, ts_ms, params, scene_depth_m);
        if select {
            self.last = Some((pose.clone(), ts_ms));
        }
        select
    }

    /// Non-mutating peek: whether [`should_select`](Self::should_select) WOULD
    /// accept this pose at this time, without recording it. The capture service
    /// calls this to decide whether the (expensive) keyframe image encode is
    /// worth doing before committing the frame via `on_frame`; because
    /// `should_select` is defined in terms of this predicate, the peek and the
    /// commit can never disagree.
    pub fn peek_select(
        &self,
        pose: &Pose,
        ts_ms: i64,
        params: &SelectionParams,
        scene_depth_m: Option<f64>,
    ) -> bool {
        match &self.last {
            // The first frame of a (sub-)stream is always a keyframe.
            None => true,
            Some((last_pose, last_ts)) => {
                let translated =
                    translation_distance(&pose.t, &last_pose.t) >= params.baseline_m(scene_depth_m);
                let rotated = rotation_angle(&last_pose.r, &pose.r) >= params.min_rotation_rad;
                let elapsed = ts_ms.saturating_sub(*last_ts) >= params.max_interval_ms;
                translated || rotated || elapsed
            }
        }
    }

    /// Whether this selector has accepted any keyframe yet.
    pub fn has_keyframe(&self) -> bool {
        self.last.is_some()
    }
}

/// Euclidean distance between two 3-vectors (metres).
fn translation_distance(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    let dz = a[2] - b[2];
    (dx * dx + dy * dy + dz * dz).sqrt()
}

/// Geodesic angle (radians) between two row-major 3x3 rotation matrices:
/// `acos((trace(R1ᵀ · R2) − 1) / 2)`, with the cosine clamped to `[-1, 1]`
/// before `acos` so float drift can never feed `acos` an out-of-domain value.
pub fn rotation_angle(r1: &[f64; 9], r2: &[f64; 9]) -> f64 {
    // Diagonal of M = R1ᵀ · R2 in row-major storage. For an entry M[i][i] the
    // sum over k is (R1ᵀ)[i][k] · R2[k][i] = R1[k][i] · R2[k][i], i.e. column i
    // of each matrix dotted together. Indices: R[k][i] = r[k * 3 + i].
    let m00 = r1[0] * r2[0] + r1[3] * r2[3] + r1[6] * r2[6];
    let m11 = r1[1] * r2[1] + r1[4] * r2[4] + r1[7] * r2[7];
    let m22 = r1[2] * r2[2] + r1[5] * r2[5] + r1[8] * r2[8];
    let trace = m00 + m11 + m22;
    let cos_theta = ((trace - 1.0) / 2.0).clamp(-1.0, 1.0);
    cos_theta.acos()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const IDENTITY: [f64; 9] = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
    // 90-degree rotation about +Z (yaw), row-major.
    const YAW_90: [f64; 9] = [0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0];

    fn pose_at(t: [f64; 3], r: [f64; 9]) -> Pose {
        Pose { r, t, cov: None }
    }

    #[test]
    fn rotation_angle_identity_is_zero() {
        assert!(rotation_angle(&IDENTITY, &IDENTITY).abs() < 1e-12);
    }

    #[test]
    fn rotation_angle_ninety_degree_yaw() {
        let angle = rotation_angle(&IDENTITY, &YAW_90);
        assert!((angle - FRAC_PI_2).abs() < 1e-9, "got {angle}");
    }

    #[test]
    fn rotation_angle_uses_the_transpose() {
        // With R1 == R2 (a non-identity rotation) the angle is exactly 0. This
        // pins trace(R1^T * R2): a regression to trace(R1 * R2) would give 180
        // deg here (the identity-R1 tests above cannot see a transpose bug).
        assert!(rotation_angle(&YAW_90, &YAW_90).abs() < 1e-9);
    }

    #[test]
    fn first_frame_always_selects() {
        let mut sel = KeyframeSelector::default();
        assert!(!sel.has_keyframe());
        let selected = sel.should_select(&pose_at([0.0, 0.0, 0.0], IDENTITY), 0, &params(), None);
        assert!(selected);
        assert!(sel.has_keyframe());
    }

    #[test]
    fn peek_matches_should_select_and_does_not_mutate() {
        let mut sel = KeyframeSelector::default();
        let p = params();
        // A fresh selector: peek and commit both say select; peek alone records
        // nothing, so a second peek still says select.
        assert!(sel.peek_select(&pose_at([0.0, 0.0, 0.0], IDENTITY), 0, &p, None));
        assert!(!sel.has_keyframe(), "peek must not record");
        assert!(sel.peek_select(&pose_at([0.0, 0.0, 0.0], IDENTITY), 0, &p, None));
        // Commit, then a sub-threshold pose: peek and should_select agree (no).
        assert!(sel.should_select(&pose_at([0.0, 0.0, 0.0], IDENTITY), 0, &p, None));
        let near = pose_at([0.1, 0.0, 0.0], IDENTITY);
        assert!(!sel.peek_select(&near, 200, &p, None));
        assert!(!sel.should_select(&near, 200, &p, None));
        // A past-threshold pose: peek predicts the commit.
        let far = pose_at([0.6, 0.0, 0.0], IDENTITY);
        assert!(sel.peek_select(&far, 300, &p, None));
        assert!(sel.should_select(&far, 300, &p, None));
    }

    #[test]
    fn pure_translation_past_threshold_selects() {
        let mut sel = KeyframeSelector::default();
        let p = params();
        assert!(sel.should_select(&pose_at([0.0, 0.0, 0.0], IDENTITY), 0, &p, None));
        // 0.6 m baseline > 0.5 m threshold, well under the time interval.
        assert!(sel.should_select(&pose_at([0.6, 0.0, 0.0], IDENTITY), 100, &p, None));
    }

    #[test]
    fn pure_rotation_past_threshold_selects() {
        let mut sel = KeyframeSelector::default();
        let p = params();
        assert!(sel.should_select(&pose_at([0.0, 0.0, 0.0], IDENTITY), 0, &p, None));
        // 90 deg rotation, same position, under the time interval.
        assert!(sel.should_select(&pose_at([0.0, 0.0, 0.0], YAW_90), 100, &p, None));
    }

    #[test]
    fn small_move_under_all_thresholds_does_not_select() {
        let mut sel = KeyframeSelector::default();
        let p = params();
        assert!(sel.should_select(&pose_at([0.0, 0.0, 0.0], IDENTITY), 0, &p, None));
        // 0.1 m, no rotation, 200 ms — under translation (0.5), rotation, and
        // the 2000 ms interval.
        assert!(!sel.should_select(&pose_at([0.1, 0.0, 0.0], IDENTITY), 200, &p, None));
    }

    #[test]
    fn exceeding_max_interval_selects_even_when_stationary() {
        let mut sel = KeyframeSelector::default();
        let p = params();
        assert!(sel.should_select(&pose_at([0.0, 0.0, 0.0], IDENTITY), 0, &p, None));
        // Identical pose, but 2500 ms > 2000 ms interval.
        assert!(sel.should_select(&pose_at([0.0, 0.0, 0.0], IDENTITY), 2500, &p, None));
    }

    #[test]
    fn creep_does_not_select_baseline_is_the_last_keyframe() {
        // Two sub-threshold steps whose CUMULATIVE distance crosses 0.5 m while
        // neither single step does. The delta is measured from the last
        // KEYFRAME, not the last frame, so neither step selects (a baseline-
        // advances-on-every-frame bug would also fail to select, but a
        // baseline-from-keyframe with a per-frame-reset bug would wrongly hold;
        // the third step from the keyframe at 0.0 crosses and MUST select).
        let mut sel = KeyframeSelector::default();
        let p = params();
        assert!(sel.should_select(&pose_at([0.0, 0.0, 0.0], IDENTITY), 0, &p, None));
        assert!(!sel.should_select(&pose_at([0.3, 0.0, 0.0], IDENTITY), 100, &p, None));
        assert!(!sel.should_select(&pose_at([0.45, 0.0, 0.0], IDENTITY), 200, &p, None));
        // 0.6 m from the last keyframe (still at 0.0) > 0.5 m: selects.
        assert!(sel.should_select(&pose_at([0.6, 0.0, 0.0], IDENTITY), 300, &p, None));
    }

    #[test]
    fn baseline_scales_with_depth_and_clamps() {
        let p = params();
        // 0.08 x the median scene depth, clamped to [0.15, 0.50] m; the
        // configured baseline without a LiDAR scan.
        assert!((p.baseline_m(Some(2.5)) - 0.20).abs() < 1e-12);
        assert_eq!(p.baseline_m(Some(1.0)), 0.15);
        assert_eq!(p.baseline_m(Some(20.0)), 0.50);
        assert_eq!(p.baseline_m(None), p.min_translation_m);
        assert_eq!(p.baseline_m(Some(f64::NAN)), p.min_translation_m);
        // A house 2.5 m deep takes a keyframe at 0.25 m, where the 0.5 m
        // default would not.
        let mut sel = KeyframeSelector::default();
        assert!(sel.should_select(&pose_at([0.0, 0.0, 0.0], IDENTITY), 0, &p, Some(2.5)));
        assert!(!sel.should_select(&pose_at([0.15, 0.0, 0.0], IDENTITY), 100, &p, Some(2.5)));
        assert!(sel.peek_select(&pose_at([0.25, 0.0, 0.0], IDENTITY), 200, &p, Some(2.5)));
        assert!(!sel.peek_select(&pose_at([0.25, 0.0, 0.0], IDENTITY), 200, &p, None));
    }

    fn params() -> SelectionParams {
        SelectionParams::default()
    }
}
