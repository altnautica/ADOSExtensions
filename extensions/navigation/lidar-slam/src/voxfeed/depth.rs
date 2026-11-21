//! The depth camera's reader: opens the configured source (retrying while it
//! is unavailable), turns each frame into body points through the camera
//! mount and queues them with the body pose at the frame's time.

use std::sync::Arc;
use std::time::{Duration, Instant};

use ados_voxmap::Pose;

use super::feeder::Input;
use super::{Shared, StampedPose};
use crate::depth_source::{self, DepthSettings, DepthSource, DepthStatus};

/// How long one read waits for a frame (bounds the reader's stop latency).
const FRAME_WAIT: Duration = Duration::from_millis(200);
/// An unavailable source is opened again after this.
const REOPEN_AFTER: Duration = Duration::from_secs(5);
/// A frame further than this from the newest body pose is not placed: the
/// pose is extrapolated at most this far, ns.
pub const POSE_REACH_NS: u64 = 300_000_000;
/// A frame is thinned to about this many pixels, the map's measured budget
/// for one insertion.
const POINTS_PER_FRAME: u32 = 20_000;

pub(super) fn run(shared: Arc<Shared>, settings: DepthSettings, max_range_m: f32) {
    while !shared.queue.stopped() {
        match depth_source::open(&settings) {
            Ok(None) => return,
            Ok(Some(src)) => {
                tracing::info!(source = ?settings.source, "voxmap_depth_open");
                stream(&shared, src, max_range_m);
            }
            Err(e) => {
                tracing::warn!(source = ?settings.source, error = %e, "voxmap_depth_unavailable");
                shared.health.lock().depth = Some(DepthStatus::unavailable(settings.source, &e));
                let until = Instant::now() + REOPEN_AFTER;
                while Instant::now() < until && !shared.queue.stopped() {
                    std::thread::sleep(Duration::from_millis(100));
                }
            }
        }
    }
}

fn stream(shared: &Shared, mut src: Box<dyn DepthSource>, max_range_m: f32) {
    let mut status_at = Instant::now();
    while !shared.queue.stopped() {
        match src.next_frame(FRAME_WAIT) {
            Ok(Some(frame)) => {
                if !place(shared, &frame, max_range_m) {
                    shared.health.lock().depth_unposed += 1;
                }
                src.recycle(frame);
            }
            Ok(None) => {}
            Err(e) => {
                tracing::warn!(error = %e, "voxmap_depth_stream_failed");
                shared.health.lock().depth = Some(DepthStatus::unavailable(src.kind(), &e));
                return;
            }
        }
        if status_at.elapsed() >= Duration::from_secs(1) {
            status_at = Instant::now();
            let status = src.status(crate::timebase::now_ns());
            shared.health.lock().depth = Some(status);
        }
    }
}

/// Queue one frame's points with the body pose at its time; false when no
/// pose is near enough.
fn place(shared: &Shared, frame: &depth_source::DepthFrame, max_range_m: f32) -> bool {
    let poses = *shared.poses.lock();
    let Some(body) = pose_at(&poses, frame.stamp_ns) else {
        return false;
    };
    let mut points = shared.spare.lock().pop().unwrap_or_default();
    frame.to_points(stride_for(frame.w, frame.h), max_range_m, &mut points);
    shared.queue.push(Input::Depth {
        body,
        origin_body: frame.extrinsic.t,
        points,
    });
    true
}

/// The pixel stride that thins a `w` x `h` frame to about
/// [`POINTS_PER_FRAME`].
pub fn stride_for(w: u32, h: u32) -> usize {
    let px = f64::from(w) * f64::from(h);
    (px / f64::from(POINTS_PER_FRAME)).sqrt().ceil().max(1.0) as usize
}

/// The body pose at host monotonic `t_ns` from the two newest poses: the
/// newest one's attitude, its position carried along the velocity between
/// the two (back to the older one at most, forward [`POSE_REACH_NS`] at
/// most). `None` when the newest is further than that from `t_ns`.
fn pose_at(poses: &[Option<StampedPose>; 2], t_ns: u64) -> Option<Pose> {
    let p1 = poses[1]?;
    if t_ns.abs_diff(p1.t_ns) > POSE_REACH_NS {
        return None;
    }
    let mut pose = p1.pose;
    if let Some(p0) = poses[0].filter(|p0| p0.t_ns < p1.t_ns) {
        let span = (p1.t_ns - p0.t_ns) as f64;
        let dt = (t_ns as f64 - p1.t_ns as f64).max(-span);
        let s = (dt / span) as f32;
        for a in 0..3 {
            pose.t[a] += (p1.pose.t[a] - p0.pose.t[a]) * s;
        }
    }
    Some(pose)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(t_ns: u64, n: f32) -> Option<StampedPose> {
        Some(StampedPose {
            t_ns,
            pose: Pose::at([n, 0.0, -2.0]),
        })
    }

    #[test]
    fn the_pose_is_carried_to_the_frame_time() {
        let ms = 1_000_000;
        // 1 m per 100 ms north.
        let poses = [at(100 * ms, 0.0), at(200 * ms, 1.0)];
        let p = pose_at(&poses, 250 * ms).unwrap();
        assert!((p.t[0] - 1.5).abs() < 1e-5, "{p:?}");
        let p = pose_at(&poses, 150 * ms).unwrap();
        assert!((p.t[0] - 0.5).abs() < 1e-5, "{p:?}");
        // Never back past the older pose.
        let p = pose_at(&poses, 0).unwrap();
        assert!(p.t[0].abs() < 1e-5, "{p:?}");
        assert!(pose_at(&poses, 600 * ms).is_none());
        assert!(pose_at(&[None, None], 200 * ms).is_none());
    }

    #[test]
    fn strides_thin_frames_to_the_budget() {
        assert_eq!(stride_for(160, 120), 1);
        assert_eq!(stride_for(848, 480), 5);
        assert_eq!(stride_for(640, 480), 4);
    }
}
