//! Control requests (socket and buttons): start, stop, pause, resume, mark,
//! exposure, and the state/coverage reads.

use ados_capture_contract::{SessionHealth, Timebase};
use serde_json::json;
use tokio::sync::oneshot;
use world_engine_protocol::atlas::CaptureState;
use world_engine_protocol::capture_ctl::{codes, CtlReply, CtlRequest, ExposureView, StartArgs};

use super::{Engine, ExposureDone};
use crate::camera_control::{apply_exposure, APPLY_TIMEOUT};
use crate::clock::ClockSidecar;
use crate::control::ControlMsg;
use crate::hud::HudAction;
use crate::quality::coverage::Coverage;
use crate::quality::flicker::settle_exposure;
use crate::recording::{ActiveSession, Phase, StartEnv};
use crate::session::CaptureSession;

/// Finalize and bag the keyframe session so the compute node's reconstruct
/// trigger (which fires on `Bagged`) sees it end. Returns whether it moved.
pub(crate) fn stop_and_bag(session: &mut CaptureSession) -> bool {
    match session.state() {
        CaptureState::Capturing | CaptureState::Paused => {
            session.finalize();
            session.mark_bagged();
            true
        }
        _ => false,
    }
}

impl Engine {
    /// Answer a control request. An exposure request is answered once the
    /// cameras have taken it (or [`APPLY_TIMEOUT`]), off the loop.
    pub(super) async fn handle(&mut self, msg: ControlMsg) {
        let ControlMsg { req, reply } = msg;
        if let CtlRequest::Exposure(e) = req {
            self.exposure = settle_exposure(&self.exposure, e, self.cfg.mains_hz);
            self.apply_exposure(Some(reply));
            return;
        }
        let _ = reply.send(self.answer(req).await);
    }

    async fn answer(&mut self, req: CtlRequest) -> CtlReply {
        match req {
            CtlRequest::Status => CtlReply::ok(self.keyframes.status()),
            CtlRequest::State => CtlReply::ok(self.state_view()),
            CtlRequest::Coverage => CtlReply::ok(self.coverage.view()),
            CtlRequest::Start(args) => match self.start(args).await {
                Ok(id) => CtlReply::ok(json!({ "session_id": id })),
                Err(e) => e,
            },
            CtlRequest::Stop => {
                if self.session.is_none() {
                    return CtlReply::err(codes::NOT_RECORDING, "no session is open");
                }
                self.begin_close();
                self.publish_status().await;
                CtlReply::ok(self.state_view())
            }
            CtlRequest::Pause | CtlRequest::Resume => {
                let pause = matches!(req, CtlRequest::Pause);
                let Some(s) = self.session.as_mut() else {
                    return CtlReply::err(codes::NOT_RECORDING, "no session is open");
                };
                if pause {
                    s.phase = Phase::Paused;
                    self.keyframes.pause();
                } else {
                    s.phase = Phase::Recording;
                    self.keyframes.resume();
                }
                self.publish_status().await;
                CtlReply::ok(self.state_view())
            }
            CtlRequest::Mark(m) => {
                if m.room.trim().is_empty() {
                    return CtlReply::err(codes::BAD_REQUEST, "room is empty");
                }
                match self.mark(m.room, Some(m.floor)) {
                    Some(segment) => CtlReply::ok(json!({ "segment": segment })),
                    None => CtlReply::err(codes::NOT_RECORDING, "no session is open"),
                }
            }
            // Answered in `handle` once applied; the current view otherwise.
            CtlRequest::Exposure(_) => CtlReply::ok(&self.exposure),
        }
    }

    pub(super) async fn on_hud(&mut self, action: HudAction) {
        match action {
            HudAction::ToggleRecording if self.session.is_some() => {
                self.begin_close();
                self.publish_status().await;
            }
            HudAction::ToggleRecording => {
                if let Err(e) = self.start(StartArgs::default()).await {
                    tracing::warn!(error = ?e.error, detail = ?e.detail, "button_start_refused");
                }
            }
            HudAction::Mark => {
                let n = self.session.as_ref().map_or(0, |s| s.segment()) + 1;
                self.mark(format!("Room {n}"), None);
            }
        }
    }

    fn mark(&mut self, room: String, floor: Option<i64>) -> Option<u32> {
        let t = self.to_session(crate::clock::mono_ns());
        let s = self.session.as_mut()?;
        let floor = floor.unwrap_or(s.floor);
        let seg = s.mark(room, floor, t);
        self.coverage.mark_room();
        Some(seg)
    }

    /// Apply the current exposure to every enabled capture camera on its own
    /// task; the result comes back through [`Self::exposure_done`].
    pub(super) fn apply_exposure(&mut self, reply: Option<oneshot::Sender<CtlReply>>) {
        self.exposure_seq += 1;
        let seq = self.exposure_seq;
        let want = self.exposure.clone();
        let ctl = self.camera.clone();
        let cameras: Vec<String> = self
            .cfg
            .capture
            .enabled_cameras()
            .map(|c| c.id.clone())
            .collect();
        let tx = self.exposure_tx.clone();
        tokio::spawn(async move {
            let view = match ctl {
                Some(ctl) => apply_exposure(ctl.as_ref(), &cameras, want, APPLY_TIMEOUT).await,
                None => ExposureView {
                    applied: false,
                    reason: Some("camera_control_unavailable".into()),
                    ..want
                },
            };
            let _ = tx.send(ExposureDone { seq, view, reply });
        });
    }

    /// Keep the latest apply's result for the state reads and answer its
    /// request; a superseded apply only answers.
    pub(super) fn exposure_done(&mut self, done: ExposureDone) {
        if done.seq == self.exposure_seq {
            self.exposure = done.view.clone();
        }
        if let Some(reply) = done.reply {
            let _ = reply.send(CtlReply::ok(&done.view));
        }
    }

    /// Admit and open a session.
    pub(super) async fn start(&mut self, args: StartArgs) -> Result<String, CtlReply> {
        if self.session.is_some() || self.closing.is_some() {
            return Err(CtlReply::err(codes::ALREADY_RECORDING, "a session is open"));
        }
        let rig = crate::rig_source::load_rig(&self.paths.rig).unwrap_or_else(|| {
            let cams = self.cams.clone();
            let imus = self.sensors.lock().imu_sources();
            crate::rig_source::derive_rig(
                &self.cfg,
                |id| {
                    cams.get(id)
                        .filter(|c| c.width > 0)
                        .map(|c| (c.width, c.height))
                },
                &imus,
            )
        });
        let clock = ClockSidecar::read(&self.paths.clock_sidecar);
        let (timebase, sigma) = match &clock {
            Some(c) => (c.timebase(), c.sigma_ns),
            None => (Timebase::HostMonotonic, -1),
        };
        let now = crate::clock::mono_ns();
        let (t0, utc) = {
            let s = self.sensors.lock();
            // A session never mixes timebases: a non-host one waits for its
            // first bus sample instead of starting on the host clock.
            if !crate::clock::timebase_ready(timebase, &s.clock) {
                return Err(CtlReply::err(
                    codes::CLOCK_NOT_READY,
                    format!(
                        "timebase {} has no clock sample yet",
                        crate::clock::timebase_str(timebase)
                    ),
                ));
            }
            let utc = s
                .clock
                .utc_at(now)
                .or_else(|| (timebase == Timebase::HostMonotonic).then(crate::clock::realtime_ns));
            (crate::clock::session_time(timebase, &s.clock, now), utc)
        };
        let env = StartEnv {
            device_id: &self.cfg.device_id,
            profile: &self.profile,
            rig: rig.clone(),
            timebase,
            sigma_ns: sigma,
            t0_ns: t0,
            utc_at_t0_ns: utc,
            gnss_anchor: self.vehicle.anchor(),
            sidecar: &self.paths.storage_sidecar,
            limits: self.cfg.segment.limits(),
            encoding: self.cfg.image_encoding,
            active_marker: self.paths.active_marker.clone(),
        };
        let s = ActiveSession::start(args, env, now)?;
        let id = s.session_id.clone();
        if let Some(fc) = &self.fc {
            fc.set_anchor(s.anchor.clone());
        }
        self.coverage = Coverage::new();
        self.quality.raised = Default::default();
        for c in self.cams.values_mut() {
            c.frames = 0;
            c.dropped = 0;
            c.last_kf_ns = None;
        }
        self.keyframes.start(id.clone());
        self.keyframes.set_anchored(true);
        self.keyframes.set_pose_tier(self.pose.source());
        self.rig = Some(rig);
        self.session = Some(s);
        self.sensors
            .lock()
            .clock
            .set_monotonic(timebase != Timebase::HostMonotonic);
        tracing::info!(session = %id, ?timebase, "capture_session_started");
        self.publish_status().await;
        Ok(id)
    }

    /// Close the open session on a blocking thread (checksums over a large
    /// session take a while); the state reads `closing` until it is done.
    pub(super) fn begin_close(&mut self) {
        let t_end = self.to_session(crate::clock::mono_ns());
        let Some(s) = self.session.take() else { return };
        self.sensors.lock().clock.set_monotonic(false);
        let flags = self.quality.raised;
        let stats = s.recorder.stats();
        let mut dropped_pct = self.dropped_pct();
        let bad: u64 = stats.bad_items.values().sum();
        if bad > 0 {
            tracing::warn!(bad_items = ?stats.bad_items, "capture_session_skipped_items");
        }
        if let Some(err) = &stats.error {
            let seen: u64 = self.cams.values().map(|c| c.frames).sum();
            let written: u64 = stats.images.values().sum();
            dropped_pct = dropped_pct.max(unwritten_pct(seen, written));
            tracing::error!(
                error = %err,
                frames_seen = seen,
                frames_written = written,
                lost_after_error = stats.lost_after_error,
                frames_dropped_pct = dropped_pct,
                "capture_session_truncated"
            );
        }
        let health = SessionHealth {
            blur: flags.blur,
            texture: flags.texture,
            flicker: flags.flicker,
            corridor: flags.corridor,
            frames_dropped_pct: dropped_pct,
        };
        stop_and_bag(&mut self.keyframes);
        self.closing = Some(tokio::task::spawn_blocking(move || s.finish(t_end, health)));
    }

    pub(super) async fn poll_closing(&mut self) {
        if !self.closing.as_ref().is_some_and(|h| h.is_finished()) {
            return;
        }
        if let Some(h) = self.closing.take() {
            match h.await {
                Ok(Ok(m)) => {
                    tracing::info!(session = %m.session_id, segments = m.segments.len(), "capture_session_closed")
                }
                Ok(Err(e)) => tracing::error!(error = %e, "capture_session_close_failed"),
                Err(e) => tracing::error!(error = %e, "capture_session_close_panicked"),
            }
        }
    }

    /// Dropped frames as a share of frames seen this session, percent.
    pub(super) fn dropped_pct(&self) -> f64 {
        let (seen, dropped) = self
            .cams
            .values()
            .fold((0u64, 0u64), |(s, d), c| (s + c.frames, d + c.dropped));
        if seen == 0 {
            0.0
        } else {
            (100.0 * dropped as f64 / seen as f64).min(100.0)
        }
    }
}

/// Frames seen but never written, percent of those seen (a session whose
/// writer failed is truncated: everything after the failure is missing).
pub(crate) fn unwritten_pct(seen: u64, written: u64) -> f64 {
    if seen == 0 {
        0.0
    } else {
        (100.0 * seen.saturating_sub(written) as f64 / seen as f64).min(100.0)
    }
}
