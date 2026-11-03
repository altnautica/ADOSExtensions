//! The capture loop's side of the navigation lanes ([`crate::nav`]): LiDAR
//! scans, the pose interval, and the 1 Hz live-map publish.

use std::path::Path;

use ados_capture_contract::v1::ImuBatch;

use super::Engine;
use crate::lidar_scan::LidarScan;
use crate::nav::{fc_imu, TickInputs};
use crate::pose::{se3, PoseProvider};
use crate::quality::lidar_depth::LidarDepth;
use crate::recorder::Item;

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
    /// clockwise from north: the map's north.
    pub(super) fn fc_heading(&self) -> Option<f64> {
        self.nav_deps
            .fc_feed
            .as_ref()
            .and_then(|f| f.attitude())
            .map(|(_, r_ned_frd)| crate::nav::frd_yaw(&r_ned_frd))
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

    /// One LiDAR scan: the pose provider, the keyframe depth, the live map,
    /// the shared scan lane and the session record.
    pub(super) async fn on_lidar(&mut self, scan: LidarScan) {
        self.pose.feed_lidar(&scan);
        let mount = self.lidar_mount(&scan.sensor_id);
        let world = self.pose.pose_at(scan.t_ns);
        let r_world_sensor = world
            .as_ref()
            .map(|e| se3::mat_mul(&e.r, &se3::from_mat4(&mount).0));
        self.lidar_depth = Some(LidarDepth::from_scan(&scan, r_world_sensor.as_ref()));
        let (fc_up, open) = (self.fc_up(), self.session.is_some());
        let out = self
            .nav
            .on_scan(&scan, &mount, world.as_ref(), fc_up, open);
        if let Some(d) = &out.scan {
            self.publisher.publish_scan(d).await;
        }
        let t = self.to_session(scan.t_ns);
        if let Some(r) = self.recorder() {
            match (&out.polar, self.nav_deps.lidar_planar) {
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
        if let (true, Some(polar)) = (self.nav.streams(), &out.polar) {
            self.publisher.publish_raw_scan(polar).await;
        }
    }

    /// With `atlas.lidar.imu = "fc"`, the newest FC attitude as an IMU sample
    /// for the pose provider (see [`crate::nav::fc_imu`]).
    fn feed_fc_imu(&mut self) {
        if self.cfg.lidar.imu != fc_imu::FC_IMU_ID {
            return;
        }
        // The attitude alone: a GNSS-denied FC reports no EKF position (so no
        // pose estimate) until the external navigation it waits for arrives.
        let Some((t_ns, r_ned_frd)) = self.nav_deps.fc_feed.as_ref().and_then(|f| f.attitude())
        else {
            return;
        };
        if self.fc_imu_prev.is_some_and(|(t, _)| t == t_ns) {
            return;
        }
        // The bridge works in a z-up world with an FLU body.
        let r_enu_flu = se3::mat_mul(
            &crate::geodesy::R_ENU_NED,
            &se3::mat_mul(&r_ned_frd, &crate::geodesy::R_FRD_FLU),
        );
        let cur = (t_ns, r_enu_flu);
        if let Some(s) = fc_imu::sample(self.fc_imu_prev, cur) {
            let batch = ImuBatch { samples: vec![s] };
            self.pose.feed_imu(fc_imu::FC_IMU_ID, &batch);
        }
        self.fc_imu_prev = Some(cur);
    }

    /// The pose interval: feed the FC attitude when it stands in for an IMU,
    /// then publish the newest pose when due.
    pub(super) async fn on_nav_tick(&mut self) {
        self.feed_fc_imu();
        if let Some(pd) = self.drain_poses() {
            self.publisher.publish_pose(&pd).await;
        }
    }

    /// The 1 Hz live-map publish: changed occupancy tiles, the sweep delta
    /// and the map status. The keyframe and scan-ring shares of the budget are
    /// applied to the open session's queues here.
    pub(super) async fn nav_slow_tick(&mut self) {
        let budget = self.nav.settings().budget;
        let recorder = self.session.as_ref().map(|s| &s.recorder);
        if let Some(r) = recorder {
            r.sidecars().set_caps(budget.keyframes(), budget.scans());
        }
        let inputs = TickInputs {
            session_id: recorder.map(|r| r.session_id()),
            evict_dir: recorder
                .map(|r| r.dir().join(crate::nav::SESSION_MAP_DIR).join(crate::nav::TILES_DIR))
                .or_else(|| self.nav_deps.scratch_dir.clone()),
            queued_bytes: recorder.map_or(0, |r| r.sidecars().bytes()),
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
