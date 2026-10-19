//! The `GET /atlas/capture/state` view.

use world_engine_protocol::atlas::VioHealth;
use world_engine_protocol::capture_ctl::{
    CameraFramesView, CaptureStateView, FramesView, PoseView, StorageView,
};

use super::Engine;
use crate::recording::Phase;

impl Engine {
    pub(crate) fn state_view(&mut self) -> CaptureStateView {
        let now = crate::clock::mono_ns();
        let state = match (&self.session, &self.closing) {
            (Some(s), _) if s.phase == Phase::Recording => "recording",
            (Some(_), _) => "paused",
            (None, Some(_)) => "closing",
            (None, None) => "idle",
        };
        let mut per_camera: Vec<CameraFramesView> = self
            .cams
            .iter()
            .map(|(id, c)| CameraFramesView {
                id: id.clone(),
                fps: (c.fps * 10.0).round() / 10.0,
                dropped: c.dropped,
            })
            .collect();
        per_camera.sort_by(|a, b| a.id.cmp(&b.id));
        let total = self.cams.values().map(|c| c.frames).sum();
        let health = self.pose.health();
        let latest = self.pose.latest();
        let fresh = latest
            .as_ref()
            .is_some_and(|e| now.saturating_sub(e.t_ns) <= crate::pose::MAX_EXTRAPOLATION_NS * 10);
        let pose = PoseView {
            source: if fresh {
                crate::pose::source_str(self.pose.source()).to_string()
            } else {
                "none".to_string()
            },
            health: match (fresh, health) {
                (false, _) | (_, VioHealth::Lost) => "lost",
                (_, VioHealth::Degraded) => "degraded",
                (_, VioHealth::Good) => "ok",
            }
            .to_string(),
            tracked_features: self.quality.tracked_features(),
            drift_hint_m: self.pose.quality().drift_hint_m,
        };
        let sc = crate::storage::read_sidecar(&self.paths.storage_sidecar);
        let free = sc
            .as_ref()
            .and_then(|s| s.volume.as_ref())
            .map(|v| v.free_bytes);
        let rate = self
            .session
            .as_ref()
            .map(|s| s.recorder.stats().write_bps)
            .filter(|r| *r > 0.0);
        let s = self.session.as_ref();
        CaptureStateView {
            state: state.to_string(),
            session_id: s.map(|s| s.session_id.clone()),
            started_at_ns: s.map(|s| s.started_mono_ns),
            duration_s: s.map(|s| now.saturating_sub(s.started_mono_ns) as f64 / 1e9),
            site: s.and_then(|s| s.site.clone()),
            floor: s.map_or(0, |s| s.floor),
            room: s.and_then(|s| s.room().map(str::to_string)),
            segment: s.map(|s| s.segment()),
            frames: FramesView {
                total,
                dropped_pct: (self.dropped_pct() * 10.0).round() / 10.0,
                per_camera,
            },
            pose,
            coverage: self.coverage.summary(),
            health: self.quality.health(),
            exposure: self.exposure,
            storage: StorageView {
                free_bytes: free,
                est_remaining_s: free.zip(rate).map(|(f, r)| (f as f64 / r).round()),
            },
        }
    }
}
