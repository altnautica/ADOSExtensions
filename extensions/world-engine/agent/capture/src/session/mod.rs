//! Capture session: the stateful core that turns a stream of pose-tagged
//! frames into the pose stream plus selected keyframes a compute node
//! reconstructs from.
//!
//! One session runs one flow regardless of camera count. Each enabled camera
//! gets its own [`KeyframeSelector`]; a frame always contributes to the ~10 Hz
//! pose stream and, when its camera's selector fires, a full keyframe. The
//! fusion keys off the enabled count downstream; the same code path serves one
//! camera or an all-sides rig.

use crate::config::CaptureConfig;
use crate::selector::KeyframeSelector;
use ados_capture_contract::v1::ImuSample;
use ados_capture_contract::{CameraModel, GeoAnchor};
use std::collections::{HashMap, VecDeque};
use std::time::{Duration, Instant};
use world_engine_protocol::atlas::{
    CaptureState, CaptureStatus, KeyframeEnvelope, KeyframeFlags, KeyframeImage, KeyframeTier,
    Pose, PoseDescriptor, PoseSource, TimeAlignment, VioHealth,
};

/// How many recent accepted frames the ingest rate is measured over.
const INGEST_WINDOW: usize = 64;
/// No accepted frame for this long reads as a 0 Hz ingest.
const INGEST_STALE: Duration = Duration::from_secs(2);

/// One camera frame handed to the session, already pose-tagged by the active
/// pose provider (`pose_source`); the session does not care which provider
/// filled it.
#[derive(Debug, Clone)]
pub struct FrameInput {
    pub image: KeyframeImage,
    pub camera: CameraModel,
    pub pose: Pose,
    /// Row-major 6x6 covariance of `pose`. Empty when the producer could state
    /// none, which makes the keyframe fail
    /// [`KeyframeEnvelope::validate`](world_engine_protocol::atlas::KeyframeEnvelope::validate)
    /// downstream rather than silently reconstructing without a prior sigma.
    pub pose_cov: Vec<f64>,
    pub pose_source: PoseSource,
    /// When the frame was exposed, on which timebase.
    pub time: TimeAlignment,
    pub anchor: Option<GeoAnchor>,
    pub imu_window: Vec<ImuSample>,
}

/// The pose side of one frame, with no image and no intrinsics. The
/// overwhelming majority of frames are not keyframes, and building a whole
/// [`FrameInput`] with an empty [`KeyframeImage`] and a resolved intrinsics
/// matrix purely to reach the pose stream was a per-frame allocation for
/// nothing.
#[derive(Debug, Clone)]
pub struct PoseInput {
    pub pose: Pose,
    pub anchor: Option<GeoAnchor>,
}

/// The result of ingesting one frame: a pose descriptor for the live pose
/// stream when the pose cadence is due, plus a keyframe when the camera's
/// selector fired.
#[derive(Debug, Clone)]
pub struct CaptureOutput {
    /// `None` when the ~10 Hz pose cadence is not due yet. The frame was still
    /// accepted and counted; only the publish is decimated.
    pub pose: Option<PoseDescriptor>,
    pub keyframe: Option<KeyframeEnvelope>,
}

/// The capture state machine for one session.
#[derive(Debug)]
pub struct CaptureSession {
    config: CaptureConfig,
    selectors: HashMap<String, KeyframeSelector>,
    state: CaptureState,
    session_id: String,
    kf_count: u64,
    vio_health: VioHealth,
    // Ingest-rate measurement over the frames actually accepted (enabled camera
    // + capturing): the timestamps of the most recent INGEST_WINDOW frames plus
    // the local instant the last one was accepted. A lifetime average would keep
    // reporting a healthy rate long after the camera died.
    recent_frames_ms: VecDeque<i64>,
    last_accept: Option<Instant>,
    // Whether the session's world frame is fixed; gates keyframe selection.
    frame_fixed: bool,
    // Honesty state surfaced on `status()`.
    anchored: bool,
    pose_tier: PoseSource,
    dropped_keyframes: u64,
    /// Selected frames refused this session because their pose sat too far from
    /// the exposure (see `MAX_PAIRING_RESIDUAL_MS`).
    pairing_rejected: u64,
    // Pose-publish decimation. ONE clock for the whole rig, not one per camera:
    // the rig has a single pose, so an N-camera rig must not publish N times the
    // documented rate onto the same bus that carries multi-MB keyframes.
    pose_interval_ms: i64,
    last_pose_pub_ms: Option<i64>,
}

impl CaptureSession {
    /// Build an idle session for the given config. No keyframes flow until
    /// [`start`](Self::start) moves it to capturing.
    pub fn new(config: CaptureConfig) -> Self {
        Self::with_pose_interval(config, 100)
    }

    /// Build an idle session that publishes at most one pose descriptor per
    /// `pose_interval_ms` across the whole rig.
    pub fn with_pose_interval(config: CaptureConfig, pose_interval_ms: i64) -> Self {
        Self {
            config,
            selectors: HashMap::new(),
            state: CaptureState::Idle,
            session_id: String::new(),
            kf_count: 0,
            vio_health: VioHealth::Good,
            recent_frames_ms: VecDeque::with_capacity(INGEST_WINDOW),
            last_accept: None,
            frame_fixed: false,
            anchored: false,
            pose_tier: PoseSource::FcEkf,
            dropped_keyframes: 0,
            pairing_rejected: 0,
            pose_interval_ms: pose_interval_ms.max(0),
            last_pose_pub_ms: None,
        }
    }

    /// Begin a capture session. Resets keyframe/ingest counters and the
    /// per-camera selectors so a new session never inherits the previous run's
    /// last-keyframe state, and moves to [`CaptureState::Capturing`].
    pub fn start(&mut self, session_id: String) {
        self.session_id = session_id;
        self.state = CaptureState::Capturing;
        self.kf_count = 0;
        self.recent_frames_ms.clear();
        self.last_accept = None;
        self.selectors.clear();
        self.dropped_keyframes = 0;
        self.pairing_rejected = 0;
        self.last_pose_pub_ms = None;
        // The world frame and anchor are per-session: a new session must not
        // inherit the previous run's latch, or its first keyframes would be
        // selected before this run's frame is fixed.
        self.frame_fixed = false;
        self.anchored = false;
        tracing::info!(session_id = %self.session_id, cameras = self.enabled_camera_count(), "atlas capture started");
    }

    /// Pause capture (drops out of [`CaptureState::Capturing`]); a no-op from
    /// any other state. Selectors are retained so resuming continues the same
    /// keyframe cadence.
    pub fn pause(&mut self) {
        if self.state == CaptureState::Capturing {
            self.state = CaptureState::Paused;
            tracing::info!(session_id = %self.session_id, "atlas capture paused");
        }
    }

    /// Resume a paused session; a no-op from any other state.
    pub fn resume(&mut self) {
        if self.state == CaptureState::Paused {
            self.state = CaptureState::Capturing;
            tracing::info!(session_id = %self.session_id, "atlas capture resumed");
        }
    }

    /// Begin finalizing the session (no more frames are accepted). Valid from
    /// capturing or paused; a no-op otherwise. Follow with
    /// [`mark_bagged`](Self::mark_bagged) once the bag is written.
    pub fn finalize(&mut self) {
        if matches!(self.state, CaptureState::Capturing | CaptureState::Paused) {
            self.state = CaptureState::Finalizing;
            tracing::info!(session_id = %self.session_id, keyframes = self.kf_count, "atlas capture finalizing");
        }
    }

    /// Mark the session's bag fully written. Valid only from
    /// [`CaptureState::Finalizing`]; a no-op otherwise.
    pub fn mark_bagged(&mut self) {
        if self.state == CaptureState::Finalizing {
            self.state = CaptureState::Bagged;
            tracing::info!(session_id = %self.session_id, "atlas capture bagged");
        }
    }

    /// Update the current VIO/SLAM health, surfaced on the capture status.
    pub fn set_vio_health(&mut self, health: VioHealth) {
        self.vio_health = health;
    }

    /// Record that the session's world frame is fixed. Keyframe selection is
    /// REFUSED until it is.
    ///
    /// The recorder fixes the frame at session start (local frame from the
    /// first pose, geo anchor if one existed then) and never re-bases it
    /// mid-session, so a session never hands the reconstructor keyframes in
    /// two frames of reference under one id. Whether a geo anchor exists is a
    /// separate fact, reported via [`set_anchored`](Self::set_anchored).
    pub fn set_frame_fixed(&mut self, fixed: bool) {
        self.frame_fixed = fixed;
    }

    /// Record whether the session has a geo anchor, surfaced on the capture
    /// status. Does not gate selection: an unanchored session still records
    /// in its local frame.
    pub fn set_anchored(&mut self, anchored: bool) {
        self.anchored = anchored;
    }

    /// Record which provider filled the pose being tagged, surfaced on the
    /// capture status so a provider switch is visible.
    pub fn set_pose_tier(&mut self, tier: PoseSource) {
        self.pose_tier = tier;
    }

    /// Record that a keyframe the capture path produced was not delivered (the
    /// bus pruned a slow subscriber). Surfaced on the capture status: without it
    /// the keyframe count claims frames the reconstruction never received.
    pub fn note_keyframe_dropped(&mut self, dropped: u64) {
        self.dropped_keyframes = self.dropped_keyframes.saturating_add(dropped);
    }

    /// Record a selected frame refused for its frame-to-pose gap; returns the
    /// session's running count.
    pub fn note_pairing_rejected(&mut self) -> u64 {
        self.pairing_rejected = self.pairing_rejected.saturating_add(1);
        self.pairing_rejected
    }

    /// Selected frames refused this session for their frame-to-pose gap.
    pub fn pairing_rejected(&self) -> u64 {
        self.pairing_rejected
    }

    /// Whether the session-wide keyframe cap has stopped selection.
    pub fn is_capped(&self) -> bool {
        self.at_keyframe_cap()
    }

    /// The current capture state.
    pub fn state(&self) -> CaptureState {
        self.state
    }

    /// Count of enabled cameras (1 to N). The fusion layer keys off this.
    pub fn enabled_camera_count(&self) -> u32 {
        self.config.enabled_camera_count()
    }

    /// Whether the session has reached its configured session-wide keyframe cap.
    /// A cap of `0` means unlimited. This is a SESSION-level gate: the per-camera
    /// selector is count-blind and would keep re-crossing its motion/time
    /// thresholds on a repeating flight path (an orbit revisits the same viewpoints
    /// every lap), so an uncapped session on a loop accumulates keyframes without
    /// bound. Both [`would_select`](Self::would_select) and
    /// [`on_frame`](Self::on_frame) consult this before the per-camera decision so
    /// the peek and the commit can never disagree.
    fn at_keyframe_cap(&self) -> bool {
        let cap = self.config.selection.max_keyframes;
        cap > 0 && self.kf_count >= cap
    }

    /// A snapshot of the capture status for the state topic.
    pub fn status(&self) -> CaptureStatus {
        CaptureStatus {
            session_id: self.session_id.clone(),
            state: self.state,
            keyframes: self.kf_count,
            vio_health: self.vio_health,
            camera_count: self.config.enabled_camera_count(),
            ingest_rate_hz: self.ingest_rate_hz(),
            capped: self.at_keyframe_cap(),
            anchored: self.anchored,
            pose_tier: self.pose_tier,
            dropped_keyframes: self.dropped_keyframes,
        }
    }

    /// Non-mutating peek: whether the next [`on_frame`](Self::on_frame) for this
    /// camera at this pose and time WOULD produce a keyframe. Mirrors
    /// `on_frame`'s gates (capturing + frame fixed + camera enabled + the per-camera
    /// selector), so the capture service can skip the expensive keyframe image
    /// encode when the frame would only contribute to the pose stream. Records
    /// nothing.
    pub fn would_select(&self, camera_id: &str, pose: &Pose, ts_ms: i64) -> bool {
        if self.state != CaptureState::Capturing {
            return false;
        }
        // No world frame yet: see `set_frame_fixed`.
        if !self.frame_fixed {
            return false;
        }
        // Session-wide keyframe cap reached: no camera selects again this session.
        if self.at_keyframe_cap() {
            return false;
        }
        let enabled = self
            .config
            .cameras
            .iter()
            .any(|c| c.id == camera_id && c.enabled);
        if !enabled {
            return false;
        }
        match self.selectors.get(camera_id) {
            Some(sel) => sel.peek_select(pose, ts_ms, &self.config.selection),
            // A camera with no selector yet: its first frame always selects.
            None => true,
        }
    }

    /// Ingest the POSE side of one frame — no image, no intrinsics, no keyframe.
    /// The path for the overwhelming majority of frames, which only feed the
    /// live pose stream.
    ///
    /// Returns `None` when the session is not capturing, the camera is not
    /// enabled, or the rig-wide pose cadence is not due yet.
    pub fn on_pose_only(
        &mut self,
        camera_id: &str,
        input: PoseInput,
        ts_ms: i64,
    ) -> Option<PoseDescriptor> {
        if !self.accept_frame(camera_id, ts_ms) {
            return None;
        }
        self.due_pose(input.pose, input.anchor, ts_ms)
    }

    /// Whether this frame is accepted for ingest, recording it in the rate
    /// measurement when it is. Shared by both ingest paths so they cannot
    /// disagree on what "accepted" means.
    fn accept_frame(&mut self, camera_id: &str, ts_ms: i64) -> bool {
        if self.state != CaptureState::Capturing {
            return false;
        }
        let enabled = self
            .config
            .cameras
            .iter()
            .any(|c| c.id == camera_id && c.enabled);
        if !enabled {
            return false;
        }
        // Record the ingest for the rate measurement (every accepted frame, not
        // only keyframes — the rate is the camera feed rate).
        if self.recent_frames_ms.len() == INGEST_WINDOW {
            self.recent_frames_ms.pop_front();
        }
        self.recent_frames_ms.push_back(ts_ms);
        self.last_accept = Some(Instant::now());
        true
    }

    /// A pose descriptor when the rig-wide cadence is due, else `None`.
    ///
    /// One clock for the whole rig: the rig has a single pose, so an N-camera
    /// set must not publish N descriptors per interval. A non-monotonic `ts_ms`
    /// (a source that re-stamps or replays) is treated as due rather than
    /// wedging the cadence forever.
    fn due_pose(
        &mut self,
        pose: Pose,
        anchor: Option<GeoAnchor>,
        ts_ms: i64,
    ) -> Option<PoseDescriptor> {
        let due = match self.last_pose_pub_ms {
            None => true,
            Some(last) => ts_ms < last || ts_ms - last >= self.pose_interval_ms,
        };
        if !due {
            return None;
        }
        self.last_pose_pub_ms = Some(ts_ms);
        Some(PoseDescriptor {
            pose,
            anchor,
            ts_ms,
        })
    }

    /// Ingest one frame for `camera_id`, image included. Returns `None` when the
    /// session is not capturing or the camera is not enabled. Otherwise it
    /// produces a pose descriptor when the rig-wide pose cadence is due and,
    /// when the camera's selector fires, a full keyframe (with
    /// `is_session_start` set on the first keyframe of the session).
    pub fn on_frame(
        &mut self,
        camera_id: &str,
        frame: FrameInput,
        ts_ms: i64,
    ) -> Option<CaptureOutput> {
        // Capture the role before the mutable accept, so an unknown or disabled
        // camera yields nothing and never touches the rate counters.
        let role = match self.config.cameras.iter().find(|c| c.id == camera_id) {
            Some(c) if c.enabled => c.role,
            _ => return None,
        };
        if !self.accept_frame(camera_id, ts_ms) {
            return None;
        }

        // Per-camera keyframe selection, gated by the frame latch and the
        // session-wide cap first so it mirrors `would_select`. Once at the cap we
        // short-circuit BEFORE touching the per-camera selector: the pose stream
        // and Capturing state continue (the live-capture model is preserved),
        // only keyframe selection stops.
        let selected = if !self.frame_fixed || self.at_keyframe_cap() {
            false
        } else {
            self.selectors
                .entry(camera_id.to_string())
                .or_default()
                .should_select(&frame.pose, ts_ms, &self.config.selection)
        };

        let keyframe = if selected {
            let is_first = self.kf_count == 0;
            let kf = KeyframeEnvelope {
                session_id: self.session_id.clone(),
                kf_id: self.kf_count,
                ts_unix_ms: ts_ms,
                camera_id: camera_id.to_string(),
                camera_role: role,
                tier: KeyframeTier::Full,
                image: frame.image,
                camera: frame.camera,
                pose: frame.pose.clone(),
                pose_cov: frame.pose_cov,
                pose_source: frame.pose_source,
                time: frame.time,
                anchor: frame.anchor.clone(),
                imu_window: frame.imu_window,
                flags: KeyframeFlags {
                    is_session_start: is_first,
                    ..KeyframeFlags::default()
                },
            };
            self.kf_count += 1;
            Some(kf)
        } else {
            None
        };

        let pose = self.due_pose(frame.pose, frame.anchor, ts_ms);
        Some(CaptureOutput { pose, keyframe })
    }

    /// Measured ingest rate (Hz) over the last [`INGEST_WINDOW`] accepted frames,
    /// from the span of their timestamps. Zero until two frames over a positive
    /// span have been seen, and zero once no frame has been accepted for
    /// [`INGEST_STALE`] — a camera that stopped delivering reads 0 Hz, not its
    /// last healthy rate.
    fn ingest_rate_hz(&self) -> f32 {
        if self
            .last_accept
            .is_none_or(|at| at.elapsed() > INGEST_STALE)
        {
            return 0.0;
        }
        match (self.recent_frames_ms.front(), self.recent_frames_ms.back()) {
            (Some(&first), Some(&last)) if self.recent_frames_ms.len() >= 2 && last > first => {
                let span_s = (last - first) as f64 / 1000.0;
                ((self.recent_frames_ms.len() - 1) as f64 / span_s) as f32
            }
            _ => 0.0,
        }
    }
}

#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_stream;
