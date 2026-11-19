//! The pose Navigation estimates, as a capture pose source.
//!
//! Navigation publishes every estimate its provider (VIO or LiDAR) produces
//! on `plugin.navigation.pose_track`, batched ([`PoseTrackBatch`]). Each
//! estimate lands in this source's [`PoseTrack`], so keyframe tagging
//! (`pose_at`) and pose recording (`drain_new`) read it exactly as they read
//! an in-process provider. With no batch for [`STALE_AFTER`] the health is
//! `Lost`, as an in-process provider whose input stopped.

use std::sync::Arc;
use std::time::{Duration, Instant};

use ados_mapping_pose::pose::{PoseProvider, PoseTrack};
use ados_mapping_pose::PoseEstimate;
use ados_mapping_protocol::atlas::{PoseSource, PoseTrackBatch, VioHealth, POSE_COV_LEN};
use parking_lot::Mutex;

/// No batch for this long: the pose is lost.
pub const STALE_AFTER: Duration = Duration::from_secs(1);

/// The provider Navigation runs, from the batches it publishes.
#[derive(Default)]
pub struct NavigationTrack {
    track: PoseTrack,
    /// The last batch's arrival and its provider.
    last: Mutex<Option<(Instant, PoseSource)>>,
}

impl NavigationTrack {
    pub fn new() -> Arc<Self> {
        Arc::new(Self::default())
    }

    /// Fold one batch received at `now`. An estimate whose covariance is
    /// not 6x6 is skipped.
    pub fn on_batch(&self, batch: PoseTrackBatch, now: Instant) {
        let mut health = None;
        for e in batch.estimates {
            let Ok(cov) = <[f64; POSE_COV_LEN]>::try_from(e.cov.as_slice()) else {
                continue;
            };
            health = Some(e.health);
            self.track.push(PoseEstimate {
                t_ns: e.t_ns,
                r: e.r,
                t: e.t,
                cov,
                source: batch.source,
                health: e.health,
                sigma_xy_m: e.sigma_xy_m,
                heading_ok: e.heading_ok,
            });
        }
        if let Some(h) = health {
            self.track.set_health(h);
        }
        *self.last.lock() = Some((now, batch.source));
    }

    /// A batch arrived within `within` of `now`.
    pub fn fresh(&self, now: Instant, within: Duration) -> bool {
        self.last
            .lock()
            .is_some_and(|(at, _)| now.saturating_duration_since(at) <= within)
    }

    /// The callback for `event.subscribe`: deliveries carry the batch under
    /// `payload`; one that does not decode is dropped.
    pub fn callback(self: &Arc<Self>) -> ados_sdk::EventCallback {
        let me = self.clone();
        Arc::new(move |args: rmpv::Value| {
            let Some(payload) = args.as_map().and_then(|m| {
                m.iter()
                    .find(|(k, _)| k.as_str() == Some("payload"))
                    .map(|(_, v)| v)
            }) else {
                return;
            };
            let mut bytes = Vec::new();
            if rmpv::encode::write_value(&mut bytes, payload).is_err() {
                return;
            }
            if let Ok(batch) = PoseTrackBatch::from_msgpack(&bytes) {
                me.on_batch(batch, Instant::now());
            }
        })
    }
}

impl PoseProvider for NavigationTrack {
    /// The provider of the last batch; `Lidar` before the first one.
    fn source(&self) -> PoseSource {
        self.last
            .lock()
            .map_or(PoseSource::Lidar, |(_, source)| source)
    }

    fn track(&self) -> &PoseTrack {
        &self.track
    }

    fn health(&self) -> VioHealth {
        if self.fresh(Instant::now(), STALE_AFTER) {
            self.track.health()
        } else {
            VioHealth::Lost
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ados_mapping_protocol::atlas::PoseTrackEntry;

    fn entry(t_ns: u64, x: f64) -> PoseTrackEntry {
        PoseTrackEntry {
            t_ns,
            r: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            t: [x, 0.0, 0.0],
            cov: vec![0.01; POSE_COV_LEN],
            health: VioHealth::Good,
            sigma_xy_m: None,
            heading_ok: true,
        }
    }

    #[test]
    fn batches_answer_pose_at_and_the_pose_is_lost_a_second_after_the_last() {
        let nav = NavigationTrack::new();
        let t0 = Instant::now();
        nav.on_batch(
            PoseTrackBatch {
                source: PoseSource::Vio,
                estimates: vec![entry(1_000, 0.0), entry(3_000, 2.0)],
            },
            t0,
        );
        let mid = nav.pose_at(2_000).expect("interpolated");
        assert!((mid.t[0] - 1.0).abs() < 1e-9);
        assert_eq!(mid.source, PoseSource::Vio);
        assert_eq!(nav.source(), PoseSource::Vio);
        assert!(nav.fresh(t0 + Duration::from_millis(900), STALE_AFTER));
        assert!(!nav.fresh(t0 + Duration::from_millis(1_100), STALE_AFTER));
        let mut drained = Vec::new();
        nav.drain_new(&mut drained);
        assert_eq!(drained.len(), 2, "both estimates reach the recorder");
    }

    #[test]
    fn without_a_batch_the_pose_is_lost() {
        assert_eq!(NavigationTrack::new().health(), VioHealth::Lost);
    }
}
