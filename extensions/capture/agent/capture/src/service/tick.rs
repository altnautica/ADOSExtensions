//! Sensor-bus records, the 10 Hz tick (pose drain, closing, record retry) and
//! the 1 Hz slow tick (health record, HUD, state publish, queue caps).

use ados_capture_contract::v1::ImuBatch;
use ados_mapping_protocol::capture_ctl::StartArgs;

use super::{Engine, KEYFRAME_SHARE, SCAN_SHARE, SLOW_TICKS};
use crate::quality::lidar_depth::LidarDepth;
use crate::quality::{ClockReading, Level, StorageReading};
use crate::recorder::{convert, Item};
use crate::recording::Phase;
use ados_mapping_pose::sensor_bus::BusEvent;

/// Retry period of `record_on_start` when a start is refused.
const RECORD_RETRY_NS: u64 = 5_000_000_000;

impl Engine {
    pub(super) fn recorder(&self) -> Option<&crate::recorder::Recorder> {
        self.session
            .as_ref()
            .filter(|s| s.phase == Phase::Recording)
            .map(|s| &s.recorder)
    }

    /// One whole LiDAR scan from Navigation: the depth summary for keyframe
    /// baselines and coverage (rotated into the world when the session's pose
    /// and the rig's mount are known), and, while recording, the session's
    /// point cloud.
    pub(super) fn on_scan(&mut self, scan: ados_mapping_pose::lidar_scan::LidarScan) {
        let mount = self
            .rig
            .as_ref()
            .and_then(|r| ados_mapping_pose::rig::lidar_sensor(r, &scan.sensor_id))
            .map(|(s, _)| s.t_body_sensor.0);
        let r_world_sensor = self.pose.pose_at(scan.t_ns).map(|e| match &mount {
            Some(m) => {
                ados_mapping_pose::se3::mat_mul(&e.r, &ados_mapping_pose::se3::from_mat4(m).0)
            }
            None => e.r,
        });
        self.lidar_depth = Some(LidarDepth::from_scan(&scan, r_world_sensor.as_ref()));
        if let Some(r) = self.recorder() {
            let t = self.to_session(scan.t_ns);
            let cloud = crate::lidar_record::scan_to_point_cloud(&scan, t, &scan.sensor_id);
            r.submit(Item::Lidar {
                src: scan.sensor_id,
                t_ns: t,
                cloud,
            });
        }
    }

    pub(super) fn on_bus(&mut self, ev: BusEvent) {
        match &ev {
            BusEvent::Imu { src, batch } => {
                self.pose.feed_imu(src, batch);
                if let Some(s) = batch.samples.last() {
                    self.quality
                        .on_gyro(ados_mapping_pose::clock::mono_ns(), &s.gyro);
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
        let now = ados_mapping_pose::clock::mono_ns();
        self.drain_poses();
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
                json: serde_json::json!({
                    "health": view.health,
                    "frames": view.frames,
                    "lidar_scans_incomplete": self.scans.as_ref().map_or(0, |s| s.incomplete(now)),
                }),
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
        if let Some(r) = self.recorder() {
            r.sidecars().set_caps(
                self.budget.share(KEYFRAME_SHARE),
                self.budget.share(SCAN_SHARE),
            );
        }
    }

    /// Hand every new pose estimate to the quality monitor and, while
    /// recording, the recorder.
    pub(super) fn drain_poses(&mut self) {
        let mut out = Vec::new();
        self.pose.drain_new(&mut out);
        let q = self.pose.quality();
        self.quality.recording = self.is_recording();
        self.quality
            .on_pose_quality(q.degeneracy_ratio, q.tracked_features);
        if out.is_empty() || self.recorder().is_none() {
            return;
        }
        let source = ados_mapping_pose::pose::source_str(self.pose.source());
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
        self.bus_timebase = ados_mapping_pose::clock::ClockSidecar::read(&self.paths.clock_sidecar)
            .map(|c| c.timebase());
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
    /// `capture.reconstruct_steps` for the compute node.
    pub(super) fn capture_status(&self) -> ados_mapping_protocol::atlas::CaptureStatus {
        let mut status = self.keyframes.status();
        status.reconstruct_steps = self.reconstruct_steps.get();
        status
    }
}
