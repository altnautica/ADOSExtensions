//! Capture quality and the six HUD health chips: `blur`, `texture`,
//! `flicker`, `corridor`, `storage`, `clock`.
//!
//! [`QualityMonitor`] folds each frame's quarter-resolution luma (sharpness,
//! FAST corners, mean intensity for flicker), the pose provider's degeneracy
//! ratio, the capture-storage reading and the clock lock, and renders
//! [`HealthView`]s with levels `ok|warn|bad`. Thresholds follow the indoor
//! capture rules: blur and texture only turn a chip after holding for
//! [`HOLD_NS`] (one bad frame while turning is not a problem).

pub mod coverage;
pub mod flicker;
pub mod image;

use std::collections::HashMap;

use ados_protocol::framebus::FrameFormat;
use world_engine_protocol::capture_ctl::HealthView;

use flicker::{FlickerDetector, FLICKER_BAD_PCT, FLICKER_WARN_PCT};

/// A blur or texture condition must hold this long to raise its chip.
pub const HOLD_NS: u64 = 1_000_000_000;
/// Variance-of-Laplacian (¼-res) under which a frame is blurred ("slow down").
pub const BLUR_WARN: f64 = 50.0;
pub const BLUR_BAD: f64 = 20.0;
/// FAST corners (¼-res) under which a frame is low-texture.
pub const TEXTURE_WARN: usize = 60;
pub const TEXTURE_BAD: usize = 20;
/// Degeneracy ratio (min/max eigenvalue) under which the pose is degenerate
/// along one axis (a corridor).
pub const CORRIDOR_WARN: f64 = 0.15;
pub const CORRIDOR_BAD: f64 = 0.05;
/// Clock sigma over which the clock chip warns, ns.
pub const CLOCK_WARN_SIGMA_NS: i64 = 1_000_000;
/// A camera with no frame for this long is left out of the chips.
pub const CAMERA_STALE_NS: u64 = 2_000_000_000;

/// One frame's measures.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FrameQuality {
    pub blur: f64,
    pub corners: usize,
    pub mean: f64,
}

/// Measure one frame; `None` for a buffer that does not match its format.
pub fn measure(
    format: FrameFormat,
    width: u32,
    height: u32,
    pixels: &[u8],
) -> Option<FrameQuality> {
    let l = image::quarter_luma(format, width, height, pixels)?;
    Some(FrameQuality {
        blur: image::blur_score(&l),
        corners: image::fast_corners(&l, image::FAST_THRESHOLD),
        mean: l.mean(),
    })
}

/// A condition that must hold for [`HOLD_NS`] before it counts.
#[derive(Debug, Default, Clone, Copy)]
struct Held {
    since: Option<u64>,
}

impl Held {
    fn update(&mut self, active: bool, t_ns: u64) -> bool {
        if !active {
            self.since = None;
            return false;
        }
        let since = *self.since.get_or_insert(t_ns);
        t_ns.saturating_sub(since) >= HOLD_NS
    }
}

#[derive(Debug, Default)]
struct CameraQuality {
    last: Option<FrameQuality>,
    /// Host ns of the last frame measured.
    last_t_ns: u64,
    blur_warn: Held,
    blur_bad: Held,
    tex_warn: Held,
    tex_bad: Held,
    flicker: FlickerDetector,
    levels: (Level, Level),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Default)]
pub enum Level {
    #[default]
    Ok,
    Warn,
    Bad,
}

impl Level {
    pub fn as_str(self) -> &'static str {
        match self {
            Level::Ok => "ok",
            Level::Warn => "warn",
            Level::Bad => "bad",
        }
    }
}

/// Storage as the HUD sees it.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct StorageReading {
    pub free_pct: Option<f64>,
    /// `near_full` → warn, `full` (or unknown) → bad.
    pub level: Level,
}

/// Clock as the HUD sees it.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct ClockReading {
    pub connected: bool,
    pub locked: bool,
    pub sigma_ns: i64,
}

#[derive(Debug, Default)]
pub struct QualityMonitor {
    cameras: HashMap<String, CameraQuality>,
    degeneracy: Option<f64>,
    tracked_features: Option<u32>,
    pub storage: StorageReading,
    pub clock: ClockReading,
    /// Raised at any point of the session (the manifest's health flags).
    pub raised: SessionFlags,
    /// Whether a session is recording; conditions seen while idle or paused
    /// show on the chips but raise no session flag.
    pub recording: bool,
}

/// The manifest's session health flags.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub struct SessionFlags {
    pub blur: bool,
    pub texture: bool,
    pub flicker: bool,
    pub corridor: bool,
}

impl QualityMonitor {
    pub fn new() -> Self {
        Self::default()
    }

    /// Fold one frame's measures at `t_ns`.
    pub fn on_frame(&mut self, camera: &str, t_ns: u64, q: FrameQuality) {
        let c = self.cameras.entry(camera.to_string()).or_default();
        c.last = Some(q);
        c.last_t_ns = t_ns;
        c.flicker.push(t_ns, q.mean);
        let blur = if c.blur_bad.update(q.blur < BLUR_BAD, t_ns) {
            Level::Bad
        } else if c.blur_warn.update(q.blur < BLUR_WARN, t_ns) {
            Level::Warn
        } else {
            Level::Ok
        };
        let tex = if c.tex_bad.update(q.corners < TEXTURE_BAD, t_ns) {
            Level::Bad
        } else if c.tex_warn.update(q.corners < TEXTURE_WARN, t_ns) {
            Level::Warn
        } else {
            Level::Ok
        };
        c.levels = (blur, tex);
        if self.recording {
            self.raised.blur |= blur != Level::Ok;
            self.raised.texture |= tex != Level::Ok;
        }
    }

    /// The pose provider's quality, for the corridor chip.
    pub fn on_pose_quality(&mut self, degeneracy: Option<f64>, tracked: Option<u32>) {
        self.degeneracy = degeneracy;
        self.tracked_features = tracked;
        if self.recording && degeneracy.is_some_and(|r| r < CORRIDOR_WARN) {
            self.raised.corridor = true;
        }
    }

    pub fn tracked_features(&self) -> Option<u32> {
        self.tracked_features
    }

    /// The six chips, in display order, at host `now_ns`. A chip covering
    /// cameras reports the worst camera that delivered a frame within
    /// [`CAMERA_STALE_NS`].
    pub fn health(&mut self, now_ns: u64) -> Vec<HealthView> {
        let live: Vec<&CameraQuality> = self
            .cameras
            .values()
            .filter(|c| now_ns.saturating_sub(c.last_t_ns) <= CAMERA_STALE_NS)
            .collect();
        let worst = |f: &dyn Fn(&CameraQuality) -> Level| {
            live.iter().map(|c| f(c)).max().unwrap_or(Level::Ok)
        };
        let blur_level = worst(&|c| c.levels.0);
        let tex_level = worst(&|c| c.levels.1);
        let min_of = |f: &dyn Fn(&FrameQuality) -> f64| {
            live.iter()
                .filter_map(|c| c.last.as_ref().map(f))
                .reduce(f64::min)
        };
        let blur_v = min_of(&|q| q.blur);
        let tex_v = min_of(&|q| q.corners as f64);
        let flicker_v = live
            .iter()
            .filter_map(|c| c.flicker.amplitude_pct())
            .reduce(f64::max);
        let flicker_level = match flicker_v {
            Some(v) if v >= FLICKER_BAD_PCT => Level::Bad,
            Some(v) if v >= FLICKER_WARN_PCT => Level::Warn,
            _ => Level::Ok,
        };
        if self.recording {
            self.raised.flicker |= flicker_level != Level::Ok;
        }
        let corridor_level = match self.degeneracy {
            Some(r) if r < CORRIDOR_BAD => Level::Bad,
            Some(r) if r < CORRIDOR_WARN => Level::Warn,
            _ => Level::Ok,
        };
        let c = self.clock;
        let clock_level = if !c.connected {
            Level::Bad
        } else if !c.locked || c.sigma_ns < 0 || c.sigma_ns > CLOCK_WARN_SIGMA_NS {
            Level::Warn
        } else {
            Level::Ok
        };
        let clock_v = (c.sigma_ns >= 0).then(|| c.sigma_ns as f64 / 1e6);
        let chip = |id: &str, level: Level, value: Option<f64>| HealthView {
            id: id.to_string(),
            level: level.as_str().to_string(),
            value: value.map(|v| (v * 100.0).round() / 100.0),
        };
        vec![
            chip("blur", blur_level, blur_v),
            chip("texture", tex_level, tex_v),
            chip("flicker", flicker_level, flicker_v),
            chip("corridor", corridor_level, self.degeneracy),
            chip("storage", self.storage.level, self.storage.free_pct),
            chip("clock", clock_level, clock_v),
        ]
    }
}

#[cfg(test)]
mod tests;
