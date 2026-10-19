//! Per-frame work: rates, pose feed, quality, coverage, recording, keyframes.

use ados_capture_contract::{RigSensorKind, Timebase};
use ados_protocol::framebus::TIMEBASE_SIM;
use world_engine_protocol::atlas::{
    ImageEncoding as KfEncoding, KeyframeImage, Pose, TimeAlignment,
};

use super::Engine;
use crate::frame_source::CapturedFrame;
use crate::pose::{se3, PoseEstimate};
use crate::quality;
use crate::recorder::Item;
use crate::recording::Phase;
use crate::session::{FrameInput, PoseInput};

/// IMU history attached to a keyframe when its camera has no earlier one.
const KF_IMU_WINDOW_NS: u64 = 200_000_000;

impl Engine {
    /// Session-timebase ns of host monotonic `host_ns`.
    pub(super) fn to_session(&self, host_ns: u64) -> u64 {
        self.sensors.lock().clock.to_session(host_ns)
    }

    fn session_timebase(&self) -> Option<Timebase> {
        self.session.as_ref().map(|s| s.timebase)
    }

    /// The frame's exposure instant on the session timebase, and the trigger
    /// record it joined (already on the session timebase).
    fn frame_time(
        &self,
        f: &CapturedFrame,
    ) -> (u64, Option<ados_capture_contract::v1::TriggerRecord>) {
        if self.session_timebase() == Some(Timebase::Sim) && f.timebase == TIMEBASE_SIM {
            if let Some(t) = f.ts_sensor_ns {
                let trig = self
                    .sensors
                    .lock()
                    .join_trigger(&f.camera_id, f.trigger_seq, f.ts_ns);
                return (
                    t,
                    trig.map(|r| ados_capture_contract::v1::TriggerRecord { t_mid_ns: t, ..r }),
                );
            }
        }
        let trig = self
            .sensors
            .lock()
            .join_trigger(&f.camera_id, f.trigger_seq, f.ts_ns);
        match trig {
            Some(r) => {
                let t = self.to_session(r.t_mid_ns);
                (
                    t,
                    Some(ados_capture_contract::v1::TriggerRecord { t_mid_ns: t, ..r }),
                )
            }
            None => (self.to_session(f.ts_ns), None),
        }
    }

    pub(super) async fn on_frame(&mut self, f: CapturedFrame) {
        let cam = self.cams.entry(f.camera_id.clone()).or_default();
        cam.width = f.width;
        cam.height = f.height;
        cam.frames += 1;
        if let Some(prev) = cam.last_ts_ns.filter(|p| f.ts_ns > *p) {
            let inst = 1e9 / (f.ts_ns - prev) as f64;
            cam.fps = if cam.fps == 0.0 {
                inst
            } else {
                0.9 * cam.fps + 0.1 * inst
            };
        }
        cam.last_ts_ns = Some(f.ts_ns);
        self.pose.feed_frame(&f);
        let host_t = f.ts_ns;
        if let Some(q) = quality::measure(f.format, f.width, f.height, &f.pixels) {
            self.quality.on_frame(&f.camera_id, host_t, q);
        }
        let est = self.pose.pose_at(host_t);
        let recording = self
            .session
            .as_ref()
            .is_some_and(|s| s.phase == Phase::Recording);
        if recording {
            if let Some(e) = &est {
                self.observe_coverage(&f.camera_id, e);
            }
        }
        if let Some(e) = &est {
            self.keyframe_path(&f, e).await;
        }
        if !recording {
            return;
        }
        // The image is stamped at its joined trigger's mid-exposure (the
        // trigger record itself is recorded from the bus).
        let (t, _) = self.frame_time(&f);
        let Some(s) = &self.session else { return };
        let camera = f.camera_id;
        if !s.recorder.submit(Item::Image {
            camera: camera.clone(),
            t_ns: t,
            format: f.format,
            width: f.width,
            height: f.height,
            pixels: f.pixels,
        }) {
            if let Some(c) = self.cams.get_mut(&camera) {
                c.dropped += 1;
            }
        }
    }

    fn observe_coverage(&mut self, camera: &str, e: &PoseEstimate) {
        let Some(rig) = &self.rig else { return };
        let Some(sensor) = rig
            .sensors
            .iter()
            .find(|s| s.id == camera && s.kind == RigSensorKind::Camera)
        else {
            return;
        };
        let (r_bc, t_bc) = se3::from_mat4(&sensor.t_body_sensor.0);
        let r_wc = se3::mat_mul(&e.r, &r_bc);
        let off = se3::mat_vec(&e.r, t_bc);
        let pos = [e.t[0] + off[0], e.t[1] + off[1], e.t[2] + off[2]];
        let (hh, vh) = sensor.camera.as_ref().map_or((0.6, 0.45), |c| {
            (
                (c.width as f64 / 2.0 / c.fx).atan(),
                (c.height as f64 / 2.0 / c.fy).atan(),
            )
        });
        self.coverage.observe_camera(&r_wc, pos, hh, vh);
        self.coverage.observe_body(&e.r, e.t);
    }

    /// Feed the keyframe selector; encode, publish and keep a keyframe when it
    /// fires.
    async fn keyframe_path(&mut self, f: &CapturedFrame, e: &PoseEstimate) {
        let pose = Pose {
            r: e.r,
            t: e.t,
            cov: None,
        };
        let anchor = self.session.as_ref().and_then(|s| s.anchor.clone());
        if !self.keyframes.would_select(&f.camera_id, &pose, f.ts_ms) {
            if let Some(pd) =
                self.keyframes
                    .on_pose_only(&f.camera_id, PoseInput { pose, anchor }, f.ts_ms)
            {
                self.publisher.publish_pose(&pd).await;
            }
            return;
        }
        let (w, h, fmt) = (f.width, f.height, f.format);
        let pixels = f.pixels.clone();
        let jpeg = match tokio::task::spawn_blocking(move || {
            crate::encode::encode_keyframe_jpeg(w, h, fmt, &pixels)
        })
        .await
        {
            Ok(Ok(j)) => j,
            Ok(Err(err)) => {
                tracing::warn!(camera = %f.camera_id, error = %err, "keyframe_encode_failed");
                return;
            }
            Err(err) => {
                tracing::error!(camera = %f.camera_id, error = %err, "keyframe_encode_task_failed");
                return;
            }
        };
        let (t_session, trig) = self.frame_time(f);
        let (timebase, sigma) = match &self.session {
            Some(s) => (s.timebase, self.sensors.lock().clock.lock().1),
            None => (Timebase::HostMonotonic, -1),
        };
        let cam = self.cams.get(&f.camera_id).cloned().unwrap_or_default();
        let from = cam
            .last_kf_ns
            .unwrap_or_else(|| f.ts_ns.saturating_sub(KF_IMU_WINDOW_NS));
        let mut imu_window = self.sensors.lock().imu_window(from, f.ts_ns);
        for s in &mut imu_window {
            s.t_ns = self.to_session(s.t_ns);
        }
        let input = FrameInput {
            image: KeyframeImage {
                encoding: KfEncoding::Jpeg,
                width: w,
                height: h,
                bytes: jpeg,
            },
            camera: self.camera_model(&f.camera_id, w, h),
            pose,
            pose_cov: e.cov.to_vec(),
            pose_source: e.source,
            time: TimeAlignment {
                t_exposure_mid_ns: t_session,
                timebase,
                sigma_ns: sigma,
                trigger_seq: trig.map(|r| r.seq),
            },
            anchor,
            imu_window,
        };
        let Some(out) = self.keyframes.on_frame(&f.camera_id, input, f.ts_ms) else {
            return;
        };
        if let Some(pd) = &out.pose {
            self.publisher.publish_pose(pd).await;
        }
        if let Some(kf) = out.keyframe {
            if let Some(c) = self.cams.get_mut(&f.camera_id) {
                c.last_kf_ns = Some(f.ts_ns);
            }
            self.kf_seq += 1;
            if let Some(s) = &self.session {
                s.recorder.submit(crate::recorder::Item::Keyframe {
                    file: format!("{:06}_{}.jpg", kf.kf_id, kf.camera_id),
                    jpeg: kf.image.bytes.clone(),
                });
            }
            let dropped = self.publisher.publish_keyframe(&kf).await;
            self.keyframes.note_keyframe_dropped(dropped);
        }
    }

    /// The camera model: the rig's when it has one, else the configured or
    /// derived intrinsics.
    fn camera_model(&self, camera: &str, w: u32, h: u32) -> ados_capture_contract::CameraModel {
        self.rig
            .as_ref()
            .and_then(|r| r.sensors.iter().find(|s| s.id == camera))
            .and_then(|s| s.camera.clone())
            .unwrap_or_else(|| self.cfg.intrinsics_for(camera, w, h))
    }
}
