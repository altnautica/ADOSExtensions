//! Per-frame work: rates, pose feed, quality, coverage, recording, keyframes.

use ados_capture_contract::{RigSensorKind, Timebase};
use ados_protocol::framebus::TIMEBASE_SIM;
use world_engine_protocol::atlas::{
    ImageEncoding as KfEncoding, KeyframeImage, Pose, TimeAlignment,
};

use super::Engine;
use crate::clock::ClockMap;
use crate::frame_source::CapturedFrame;
use crate::pose::{se3, PoseEstimate};
use crate::quality;
use crate::recorder::Item;
use crate::recording::Phase;
use crate::session::{FrameInput, PoseInput};

/// IMU history attached to a keyframe when its camera has no earlier one.
const KF_IMU_WINDOW_NS: u64 = 200_000_000;

impl Engine {
    /// Session-timebase ns of host monotonic `host_ns`, through the open
    /// session's own map (the identity for a `host_monotonic` session). With
    /// no session open the stamp stays on the host clock, which is what an
    /// idle keyframe is labelled with.
    pub(super) fn to_session(&self, host_ns: u64) -> u64 {
        match &self.session {
            Some(s) => crate::clock::session_time(s.timebase, &s.clock, host_ns),
            None => host_ns,
        }
    }

    fn session_timebase(&self) -> Option<Timebase> {
        self.session.as_ref().map(|s| s.timebase)
    }

    /// Whether the open session is recording (not paused).
    pub(super) fn is_recording(&self) -> bool {
        self.session
            .as_ref()
            .is_some_and(|s| s.phase == Phase::Recording)
    }

    /// The host-clock exposure instant of a frame stamped with simulator time
    /// (`ts_sensor_ns` on `TIMEBASE_SIM`, from the renderer's SEI), whose
    /// `ts_ns` is only its arrival over RTSP, 0.1–0.3 s late and jittery. See
    /// [`sim_exposure_host`].
    fn sim_exposure(&self, f: &CapturedFrame) -> SimStamp {
        let sensors = self.sensors.lock();
        sim_exposure_host(
            f.timebase,
            f.ts_sensor_ns,
            self.session.as_ref().map(|s| (s.timebase, &s.clock)),
            self.bus_timebase,
            &sensors.clock,
        )
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

    pub(super) async fn on_frame(&mut self, mut f: CapturedFrame) {
        // The pose provider, the quality and coverage clocks, the keyframe
        // selector and the fallback session stamp all want the exposure
        // instant, not the arrival.
        match self.sim_exposure(&f) {
            SimStamp::Mapped(t) => retime(&mut f, t),
            SimStamp::Unmapped => {
                self.sim_unmapped += 1;
                if self.sim_unmapped % 500 == 1 {
                    tracing::warn!(
                        frames = self.sim_unmapped,
                        bus_timebase = ?self.bus_timebase,
                        "sim_frame_stamp_unmapped_keeping_arrival"
                    );
                }
            }
            SimStamp::NotSim => {}
        }
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
        let recording = self.is_recording();
        self.quality.recording = recording;
        if let Some(q) = quality::measure(f.format, f.width, f.height, &f.pixels) {
            self.quality.on_frame(&f.camera_id, host_t, q);
        }
        let est = self.pose.pose_at(host_t);
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
            Some(s) => (s.timebase, s.clock.lock().1),
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

/// How a frame's simulator stamp maps onto the host clock.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum SimStamp {
    /// Not a sim-stamped frame.
    NotSim,
    /// The host-clock exposure instant.
    Mapped(u64),
    /// A sim stamp with no clock on `sim` to invert (or no sample yet): the
    /// frame keeps its arrival stamp.
    Unmapped,
}

/// The host instant of a `TIMEBASE_SIM` stamp. Only a clock whose timebase is
/// `sim` can be inverted: the open session's when it is a `sim` session, else
/// the bus clock when the clock service names `sim`. In a simulation the agent
/// clock runs on simulator time (its `sim` source), so that map's inverse is
/// the host instant the IMU samples are stamped on; any other clock (hub, GNSS,
/// host) is a different time scale and inverting it would misplace the frame.
pub(super) fn sim_exposure_host(
    frame_timebase: u8,
    ts_sensor_ns: Option<u64>,
    session: Option<(Timebase, &ClockMap)>,
    bus_timebase: Option<Timebase>,
    bus: &ClockMap,
) -> SimStamp {
    if frame_timebase != TIMEBASE_SIM {
        return SimStamp::NotSim;
    }
    let Some(t) = ts_sensor_ns else {
        return SimStamp::NotSim;
    };
    let map = match session {
        Some((Timebase::Sim, m)) => Some(m),
        _ if bus_timebase == Some(Timebase::Sim) => Some(bus),
        _ => None,
    };
    match map.and_then(|m| m.to_host(t)) {
        Some(h) => SimStamp::Mapped(h),
        None => SimStamp::Unmapped,
    }
}

/// Move a frame's host stamp to `t_ns`, shifting its unix-ms stamp (the
/// keyframe selector's clock) by the same amount so both name the exposure.
pub(super) fn retime(f: &mut CapturedFrame, t_ns: u64) {
    let shift_ns = t_ns as i128 - f.ts_ns as i128;
    f.ts_ms += (shift_ns.div_euclid(1_000_000)) as i64;
    f.ts_ns = t_ns;
}

#[cfg(test)]
mod tests {
    use super::*;
    use ados_capture_contract::v1::ClockSample;
    use ados_protocol::framebus::TIMEBASE_HOST;

    /// A map whose source reads `offset` ahead of the host clock.
    fn map(offset: u64) -> ClockMap {
        let mut m = ClockMap::new();
        for h in [1_000_000_000u64, 2_000_000_000] {
            m.observe(ClockSample {
                t_ns: h + offset,
                host_mono_ns: h,
                utc_ns: None,
                locked: true,
                sigma_ns: 1_000,
            });
        }
        m
    }

    #[test]
    fn sim_stamp_inverts_only_a_sim_clock() {
        let bus = map(5_000_000_000);
        let t = 7_000_000_000;
        // Bus clock on sim: inverted.
        assert_eq!(
            sim_exposure_host(TIMEBASE_SIM, Some(t), None, Some(Timebase::Sim), &bus),
            SimStamp::Mapped(2_000_000_000)
        );
        // Bus clock on the hub or unknown: arrival kept, flagged.
        for tb in [
            Some(Timebase::HubFreerun),
            Some(Timebase::HostMonotonic),
            None,
        ] {
            assert_eq!(
                sim_exposure_host(TIMEBASE_SIM, Some(t), None, tb, &bus),
                SimStamp::Unmapped
            );
        }
        // A non-sim session never borrows the bus clock unless it is on sim.
        let sess = map(1_000_000_000);
        assert_eq!(
            sim_exposure_host(
                TIMEBASE_SIM,
                Some(t),
                Some((Timebase::HubFreerun, &sess)),
                Some(Timebase::HubFreerun),
                &bus
            ),
            SimStamp::Unmapped
        );
        // A sim session uses its own map.
        assert_eq!(
            sim_exposure_host(
                TIMEBASE_SIM,
                Some(t),
                Some((Timebase::Sim, &sess)),
                None,
                &bus
            ),
            SimStamp::Mapped(6_000_000_000)
        );
        // Sim clock with no sample yet: unmapped.
        assert_eq!(
            sim_exposure_host(
                TIMEBASE_SIM,
                Some(t),
                None,
                Some(Timebase::Sim),
                &ClockMap::new()
            ),
            SimStamp::Unmapped
        );
        assert_eq!(
            sim_exposure_host(TIMEBASE_HOST, Some(t), None, Some(Timebase::Sim), &bus),
            SimStamp::NotSim
        );
    }

    #[test]
    fn retime_moves_the_unix_stamp_with_the_exposure() {
        let mut f = CapturedFrame {
            ts_ms: 1_700_000_000_250,
            ts_ns: 10_250_000_000,
            camera_id: "front_left".into(),
            ts_sensor_ns: Some(3_000_000_000),
            timebase: TIMEBASE_SIM,
            trigger_seq: None,
            exposure_ns: None,
            seq: 1,
            width: 2,
            height: 2,
            format: ados_protocol::framebus::FrameFormat::Gray8,
            pixels: vec![0; 4],
        };
        retime(&mut f, 10_000_000_000);
        assert_eq!(f.ts_ns, 10_000_000_000);
        assert_eq!(f.ts_ms, 1_700_000_000_000);
    }
}
