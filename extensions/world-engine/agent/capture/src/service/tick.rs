//! Sensor-bus records, the 10 Hz tick (pose drain, closing, record retry) and
//! the 1 Hz slow tick (health record, HUD, state and live-map publish).

use ados_capture_contract::v1::ImuBatch;
use world_engine_protocol::capture_ctl::StartArgs;

use world_engine_protocol::atlas::PoseDescriptor;

use super::{Engine, SLOW_TICKS};
use crate::quality::{ClockReading, Level, StorageReading};
use crate::recorder::{convert, Item};
use crate::recording::Phase;
use crate::sensor_bus::BusEvent;

/// Retry period of `record_on_start` when a start is refused.
const RECORD_RETRY_NS: u64 = 5_000_000_000;

impl Engine {
    pub(super) fn recorder(&self) -> Option<&crate::recorder::Recorder> {
        self.session
            .as_ref()
            .filter(|s| s.phase == Phase::Recording)
            .map(|s| &s.recorder)
    }

    pub(super) fn on_bus(&mut self, ev: BusEvent) {
        match &ev {
            BusEvent::Imu { src, batch } => {
                self.pose.feed_imu(src, batch);
                if let Some(s) = batch.samples.last() {
                    self.quality.on_gyro(crate::clock::mono_ns(), &s.gyro);
                }
            }
            // The open session's map follows its own timebase, paused or not.
            BusEvent::Clock(s) => {
                if let Some(session) = self.session.as_mut() {
                    session.clock.observe(*s);
                }
            }
            BusEvent::Gnss { .. } | BusEvent::Trigger { .. } => {}
        }
        if self.recorder().is_none() {
            return;
        }
        let item = match ev {
            BusEvent::Imu { src, mut batch } => {
                let Some(first) = batch.samples.first().map(|s| s.t_ns) else {
                    return;
                };
                self.map_imu(&mut batch);
                let t = self.to_session(first);
                Item::Imu {
                    src,
                    t_ns: t,
                    batch,
                }
            }
            BusEvent::Trigger { camera, mut rec } => {
                rec.t_mid_ns = self.to_session(rec.t_mid_ns);
                Item::Trigger { camera, rec }
            }
            BusEvent::Gnss { src, mut raw } => {
                raw.t_ns = self.to_session(raw.t_ns);
                Item::GnssRaw { src, raw }
            }
            // The sample is kept as published; its log time is the session
            // time of its host instant, so a sample on another timebase (the
            // host fallback, a source switch) cannot land outside the session.
            BusEvent::Clock(s) => Item::Clock {
                t_ns: self.to_session(s.host_mono_ns),
                sample: s,
            },
        };
        if let Some(r) = self.recorder() {
            r.submit(item);
        }
    }

    fn map_imu(&self, batch: &mut ImuBatch) {
        for s in &mut batch.samples {
            s.t_ns = self.to_session(s.t_ns);
        }
    }

    pub(super) async fn on_tick(&mut self) {
        let now = crate::clock::mono_ns();
        if let Some(pd) = self.drain_poses() {
            self.publisher.publish_pose(&pd).await;
        }
        self.poll_closing().await;
        if let Some(at) = self.record_retry_at.filter(|at| now >= *at) {
            self.record_retry_at = None;
            if self.session.is_none() && self.closing.is_none() {
                if let Err(e) = self.start(StartArgs::default()).await {
                    tracing::info!(error = ?e.error, detail = ?e.detail, "record_on_start_refused");
                    self.record_retry_at = Some(at + RECORD_RETRY_NS);
                }
            }
        }
        self.slow += 1;
        if self.slow < SLOW_TICKS {
            return;
        }
        self.slow = 0;
        self.refresh_readings();
        if let Some(router) = &self.router {
            for (cam, n) in router.take_drops() {
                self.cams.entry(cam).or_default().dropped += n;
            }
        }
        self.record_fix();
        let view = self.state_view();
        if let Some(r) = self.recorder() {
            let t = self.to_session(now);
            r.submit(Item::Health {
                t_ns: t,
                json: serde_json::json!({"health": view.health, "frames": view.frames}),
            });
            if let Some(err) = r.stats().error {
                tracing::error!(error = %err, "recording_failed_closing_session");
                self.begin_close();
            }
        }
        if let Some(tx) = &self.view_tx {
            let _ = tx.send(view);
        }
        self.publish_status().await;
        self.nav_slow_tick().await;
    }

    /// Hand every new pose estimate to the navigation lane and the recorder;
    /// the pose descriptor to publish when its interval is due.
    pub(super) fn drain_poses(&mut self) -> Option<PoseDescriptor> {
        let mut out = Vec::new();
        self.pose.drain_new(&mut out);
        let q = self.pose.quality();
        self.quality.recording = self.is_recording();
        self.quality
            .on_pose_quality(q.degeneracy_ratio, q.tracked_features);
        let now_ms = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_millis() as i64);
        let (fc_up, heading, open) = (self.fc_up(), self.fc_heading(), self.session.is_some());
        let due = out
            .last()
            .and_then(|e| self.nav.on_pose(e, fc_up, heading, now_ms, open));
        if out.is_empty() || self.recorder().is_none() {
            return due;
        }
        let source = crate::pose::source_str(self.pose.source());
        for e in out {
            let t = self.to_session(e.t_ns);
            let (pose, cov) = convert::pose(&e, t);
            if let Some(r) = self.recorder() {
                r.submit(Item::Pose {
                    source,
                    t_ns: t,
                    pose,
                    cov,
                });
            }
        }
        due
    }

    /// Record the newest vehicle-state GNSS fix, once.
    fn record_fix(&mut self) {
        let Some((seq, fix)) = self.vehicle.latest() else {
            return;
        };
        if seq == self.last_fix_seq {
            return;
        }
        self.last_fix_seq = seq;
        let t = self.to_session(fix.t_ns);
        let id = self
            .rig
            .as_ref()
            .map_or_else(|| "gnss0".to_string(), crate::rig_source::gnss_id);
        let sigma = fix
            .dop
            .map(|d| d * self.cfg.pose_prior.gnss_uere_m)
            .or(self.cfg.pose_prior.position_sigma_m);
        let msg = convert::location_fix(&id, t, fix.lat_deg, fix.lon_deg, fix.h_ellipsoid_m, sigma);
        if let Some(r) = self.recorder() {
            r.submit(Item::GnssFix {
                src: id,
                t_ns: t,
                fix: msg,
            });
        }
    }

    /// The storage and clock readings for the HUD chips.
    fn refresh_readings(&mut self) {
        use ados_config::capture_storage::CaptureState as S;
        self.quality.storage = match crate::storage::read_sidecar(&self.paths.storage_sidecar) {
            Some(sc) => StorageReading {
                free_pct: sc
                    .volume
                    .as_ref()
                    .filter(|v| v.total_bytes > 0)
                    .map(|v| 100.0 * v.free_bytes as f64 / v.total_bytes as f64),
                level: match sc.capture.state {
                    S::Ok => Level::Ok,
                    S::NearFull => Level::Warn,
                    S::Full => Level::Bad,
                },
            },
            None => StorageReading {
                free_pct: None,
                level: Level::Bad,
            },
        };
        self.bus_timebase =
            crate::clock::ClockSidecar::read(&self.paths.clock_sidecar).map(|c| c.timebase());
        let s = self.sensors.lock();
        let (locked, sigma) = s.clock.lock();
        self.quality.clock = ClockReading {
            connected: s.connected && s.clock.last().is_some(),
            locked,
            sigma_ns: sigma,
        };
    }

    pub(super) async fn shutdown(&mut self) {
        if self.session.is_some() {
            self.begin_close();
        }
        if let Some(c) = self.closing.take() {
            match c.handle.await {
                Ok(Ok(m)) => tracing::info!(session = %m.session_id, "capture_session_closed"),
                Ok(Err(e)) => tracing::error!(error = %e, "capture_session_close_failed"),
                Err(e) => tracing::error!(error = %e, "capture_session_close_panicked"),
            }
        }
        if crate::service::control::stop_and_bag(&mut self.keyframes) {
            self.publish_status().await;
            tokio::time::sleep(std::time::Duration::from_millis(150)).await;
        }
    }

    pub(super) async fn publish_status(&self) {
        self.publisher
            .publish_capture_state(&self.capture_status())
            .await;
    }

    /// The keyframe session's status, carrying the drone's live
    /// `atlas.reconstruct_steps` for the compute node.
    pub(super) fn capture_status(&self) -> world_engine_protocol::atlas::CaptureStatus {
        let mut status = self.keyframes.status();
        status.reconstruct_steps = self.reconstruct_steps.get();
        status
    }
}
