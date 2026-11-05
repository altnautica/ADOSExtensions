//! The capture loop's side of the navigation lanes ([`crate::nav`]): LiDAR
//! scans, the pose interval, and the 1 Hz live-map publish.

use std::path::Path;

use ados_capture_contract::v1::ImuBatch;
use ados_scan2d::VelocitySample;

use super::Engine;
use crate::lidar_scan::LidarScan;
use crate::nav::{fc_imu, TickInputs};
use crate::pose::{se3, PoseEstimate, PoseProvider};
use crate::quality::lidar_depth::LidarDepth;
use crate::recorder::Item;

/// How often `nav_imu_feed` reports the FC IMU feed.
const NAV_FEED_LOG_EVERY: std::time::Duration = std::time::Duration::from_secs(10);

/// The FC IMU feed since the last `nav_imu_feed` log: samples handed to the
/// pose provider, scans, and the widest gap between a scan's last point and
/// the newest IMU sample fed before it.
pub(crate) struct NavFeedStats {
    imu_samples: u64,
    scans: u64,
    max_gap_ns: i64,
    since: std::time::Instant,
}

impl Default for NavFeedStats {
    fn default() -> Self {
        Self {
            imu_samples: 0,
            scans: 0,
            max_gap_ns: 0,
            since: std::time::Instant::now(),
        }
    }
}

impl Engine {
    /// The flight controller's up height (its EKF world is ENU), metres.
    pub(super) fn fc_up(&self) -> Option<f64> {
        self.nav_deps
            .fc_feed
            .as_ref()
            .and_then(|f| f.track().latest())
            .map(|e| e.t[2])
    }

    /// The flight controller's attitude yaw (its compass heading), radians
    /// clockwise from north: the map's north until the GNSS fit.
    pub(super) fn fc_heading(&self) -> Option<f64> {
        self.nav_deps
            .fc_feed
            .as_ref()
            .and_then(|f| f.attitude())
            .map(|(_, r_ned_frd)| crate::nav::frd_yaw(&r_ned_frd))
    }

    /// Pair each new provider estimate with the flight controller's EKF
    /// position (`LOCAL_POSITION_NED`) at the same instant for the map's GNSS
    /// alignment while that position was GNSS-referenced, and freeze the
    /// alignment at an estimate where it was not. Estimates the FC stream has
    /// not reached yet are skipped.
    pub(super) fn fold_gnss_alignment(&mut self, poses: &[PoseEstimate]) {
        let Some(fc) = self.nav_deps.fc_feed.as_ref() else {
            return;
        };
        for e in poses {
            let Some(ekf) = fc.ned_at(e.t_ns) else {
                continue;
            };
            if fc.gnss_steady_at(e.t_ns) {
                self.nav.on_gnss_ref(e, ekf);
            } else {
                self.nav.on_gnss_lost();
            }
        }
    }

    fn lidar_mount(&self, sensor: &str) -> [f64; 16] {
        self.nav_deps
            .lidar_mount
            .or_else(|| {
                self.rig
                    .as_ref()
                    .and_then(|r| r.sensors.iter().find(|s| s.id == sensor))
                    .map(|s| s.t_body_sensor.0)
            })
            .unwrap_or_else(|| se3::to_mat4(&se3::IDENTITY, [0.0; 3]))
    }

    /// One LiDAR scan: the pose provider and the shared scan lane now; the
    /// keyframe depth, the live map, the session record and the node stream
    /// once its pose is known ([`Self::place_scans`]).
    pub(super) async fn on_lidar(&mut self, scan: LidarScan) {
        // The FC's IMU up to now first, so the matcher has the rotation
        // across this scan before it sees the scan.
        self.feed_fc_imu();
        self.feed_fc_velocity();
        if self.cfg.lidar.imu == fc_imu::FC_IMU_ID {
            let end = scan.t_ns + u64::from(scan.t_offset_ns.iter().copied().max().unwrap_or(0));
            let f = &mut self.nav_feed;
            f.scans += 1;
            if let Some(n) = self.newest_fc_imu_ns {
                f.max_gap_ns = f.max_gap_ns.max(end as i64 - n as i64);
            }
        }
        self.pose.feed_lidar(&scan);
        let mount = self.lidar_mount(&scan.sensor_id);
        if let Some(d) = self.nav.body_scan(&scan, &mount, self.session.is_some()) {
            self.publisher.publish_scan(&d).await;
        }
        if let Some(oldest) = self.scan_wait.push(scan) {
            self.place_scan(oldest, None).await;
        }
        self.place_scans().await;
    }

    /// Place every waiting scan whose pose is settled (see
    /// [`crate::nav::scan_wait`]), oldest first.
    pub(super) async fn place_scans(&mut self) {
        loop {
            let now = crate::clock::mono_ns();
            let newest = self.pose.latest().map(|e| e.t_ns);
            let pose = &self.pose;
            let Some((scan, world)) = self.scan_wait.next(now, newest, |t| pose.pose_at(t)) else {
                return;
            };
            self.place_scan(scan, world).await;
        }
    }

    async fn place_scan(&mut self, scan: LidarScan, world: Option<PoseEstimate>) {
        let mount = self.lidar_mount(&scan.sensor_id);
        let r_world_sensor = world
            .as_ref()
            .map(|e| se3::mat_mul(&e.r, &se3::from_mat4(&mount).0));
        self.lidar_depth = Some(LidarDepth::from_scan(&scan, r_world_sensor.as_ref()));
        let (fc_up, open) = (self.fc_up(), self.session.is_some());
        let polar = self
            .nav
            .place_scan(&scan, &mount, world.as_ref(), fc_up, open);
        let t = self.to_session(scan.t_ns);
        if let Some(r) = self.recorder() {
            match (&polar, self.nav_deps.lidar_planar) {
                // A planar LiDAR records 3 bytes a bin instead of 17 a point.
                (Some(polar), true) => {
                    let mut rec = polar.clone();
                    rec.t_ns = t;
                    r.push_polar(&scan.sensor_id, &rec);
                }
                _ => {
                    let cloud = crate::lidar_record::scan_to_point_cloud(&scan, t, &scan.sensor_id);
                    r.submit(Item::Lidar {
                        src: scan.sensor_id.clone(),
                        t_ns: t,
                        cloud,
                    });
                }
            }
        }
        // While streaming, the node builds the full-resolution live map.
        if let (true, Some(polar)) = (self.nav.streams(), &polar) {
            self.publisher.publish_raw_scan(polar).await;
        }
    }

    /// With `atlas.lidar.imu = "fc"`, the FC's IMU stream for the pose
    /// provider (see [`crate::nav::fc_imu`]): every `RAW_IMU` sample since
    /// the last tick, with gravity in the FC's newest attitude as its
    /// specific force, or, until the FC streams `RAW_IMU`, the newest FC
    /// attitude turned into a sample.
    fn feed_fc_imu(&mut self) {
        if self.cfg.lidar.imu != fc_imu::FC_IMU_ID {
            return;
        }
        let Some(fc) = self.nav_deps.fc_feed.as_ref() else {
            return;
        };
        // The bridge works in a z-up world with an FLU body.
        let attitude = fc.attitude().map(|(t_ns, r_ned_frd)| {
            let r_enu_flu = se3::mat_mul(
                &crate::geodesy::R_ENU_NED,
                &se3::mat_mul(&r_ned_frd, &crate::geodesy::R_FRD_FLU),
            );
            (t_ns, r_enu_flu)
        });
        let raw = fc.drain_imu();
        if !raw.is_empty() {
            if !self.fc_raw_imu {
                tracing::info!("nav_imu_source_raw_imu");
                self.fc_raw_imu = true;
            }
            self.nav_feed.imu_samples += raw.len() as u64;
            if let Some(t) = raw.iter().map(|(t, _)| *t).max() {
                self.newest_fc_imu_ns = Some(self.newest_fc_imu_ns.map_or(t, |n| n.max(t)));
            }
            let samples = raw
                .iter()
                .map(|(t, s)| fc_imu::from_raw(*t, s, attitude))
                .collect();
            self.pose.feed_imu(fc_imu::FC_IMU_ID, &ImuBatch { samples });
            return;
        }
        if self.fc_raw_imu {
            return;
        }
        // The attitude alone: a GNSS-denied FC reports no EKF position (so no
        // pose estimate) until the external navigation it waits for arrives.
        let Some(cur) = attitude else {
            return;
        };
        if self.fc_imu_prev.is_some_and(|(t, _)| t == cur.0) {
            return;
        }
        if let Some(s) = fc_imu::sample(self.fc_imu_prev, cur) {
            let batch = ImuBatch { samples: vec![s] };
            self.pose.feed_imu(fc_imu::FC_IMU_ID, &batch);
            self.nav_feed.imu_samples += 1;
            self.newest_fc_imu_ns = Some(cur.0);
        }
        self.fc_imu_prev = Some(cur);
    }

    /// With `atlas.lidar.imu = "fc"`, the FC's ground velocity (its optical
    /// flow over its downward range, see [`crate::nav::fc_flow`]) for the
    /// pose provider, in the same body frame as the FC's IMU. After the IMU
    /// samples of the same tick, so the matcher levels it with an attitude
    /// that has caught up.
    fn feed_fc_velocity(&mut self) {
        if self.cfg.lidar.imu != fc_imu::FC_IMU_ID {
            return;
        }
        let Some(fc) = self.nav_deps.fc_feed.as_ref() else {
            return;
        };
        let samples: Vec<VelocitySample> = fc
            .drain_velocity()
            .into_iter()
            .map(|(t_ns, v)| VelocitySample { t_ns, v })
            .collect();
        if samples.is_empty() {
            return;
        }
        if !self.fc_flow {
            tracing::info!("nav_velocity_source_optical_flow");
            self.fc_flow = true;
        }
        self.pose.feed_velocity(fc_imu::FC_IMU_ID, &samples);
    }

    /// The pose interval: feed the FC's IMU (or attitude) and ground velocity
    /// when it stands in for an IMU, publish the newest pose when due, then
    /// place the scans the new poses settled.
    pub(super) async fn on_nav_tick(&mut self) {
        self.feed_fc_imu();
        self.feed_fc_velocity();
        if let Some(pd) = self.drain_poses() {
            self.publisher.publish_pose(&pd).await;
        }
        self.place_scans().await;
    }

    /// At most every [`NAV_FEED_LOG_EVERY`], with the FC as the IMU: the IMU
    /// rate fed to the pose provider and the widest scan-to-IMU gap.
    fn log_nav_feed(&mut self) {
        if self.cfg.lidar.imu != fc_imu::FC_IMU_ID {
            return;
        }
        let elapsed_s = self.nav_feed.since.elapsed().as_secs_f64();
        if elapsed_s < NAV_FEED_LOG_EVERY.as_secs_f64() {
            return;
        }
        let f = std::mem::take(&mut self.nav_feed);
        tracing::info!(
            imu_hz = f.imu_samples as f64 / elapsed_s,
            scans = f.scans,
            max_scan_imu_gap_ms = f.max_gap_ns as f64 / 1e6,
            "nav_imu_feed"
        );
    }

    /// The 1 Hz live-map publish: changed occupancy tiles, the sweep delta
    /// and the map status. The keyframe and scan-ring shares of the budget are
    /// applied to the open session's queues here.
    pub(super) async fn nav_slow_tick(&mut self) {
        self.log_nav_feed();
        let budget = self.nav.settings().budget;
        let recorder = self.session.as_ref().map(|s| &s.recorder);
        if let Some(r) = recorder {
            r.sidecars().set_caps(budget.keyframes(), budget.scans());
        }
        let inputs = TickInputs {
            session_id: recorder.map(|r| r.session_id()),
            evict_dir: recorder
                .map(|r| {
                    r.dir()
                        .join(crate::nav::SESSION_MAP_DIR)
                        .join(crate::nav::TILES_DIR)
                })
                .or_else(|| self.nav_deps.scratch_dir.clone()),
            rss_mb: crate::nav::read_vm_rss_mb(),
            keyframes_dropped: recorder.map_or(0, |r| r.sidecars().keyframes.dropped()),
        };
        let open = self.session.is_some();
        let out = self.nav.tick(&inputs, open);
        if let Some(o) = &out.occupancy {
            self.publisher.publish_live_occupancy(o).await;
        }
        if let Some(c) = &out.pointcloud {
            self.publisher.publish_live_pointcloud(c).await;
        }
        self.publisher.publish_map_status(&out.status).await;
    }

    /// Save the live map into a closing session's directory, before its
    /// checksums are taken.
    pub(super) fn save_map_into(&self, session_dir: &Path) {
        self.nav.save(session_dir);
    }
}
