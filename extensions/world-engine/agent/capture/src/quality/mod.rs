//! Capture quality and the six HUD health chips: `blur`, `texture`,
//! `flicker`, `corridor`, `storage`, `clock`.
//!
//! [`QualityMonitor`] folds each frame's quarter-resolution luma (sharpness,
//! FAST corners, mean intensity for flicker), the latest gyro rate, the pose
//! provider's degeneracy ratio, the capture-storage reading and the clock
//! lock, and renders [`HealthView`]s with levels `ok|warn|bad`.
//!
//! The blur chip is the worse of two measures: the motion blur the gyro rate
//! predicts over the exposure (`|ω| · t_exp · fx` pixels), and the frame's
//! Laplacian score relative to a running average of that camera's score (a
//! sharp but bare wall is not blur; a sudden drop is). Its value is that
//! relative score. Blur and texture only turn a chip after holding for
//! [`HOLD_NS`] (one bad frame while turning is not a problem).

pub mod coverage;
pub mod flicker;
pub mod image;
pub mod lidar_depth;

use std::collections::HashMap;

use ados_protocol::framebus::FrameFormat;
use world_engine_protocol::capture_ctl::HealthView;

use flicker::{FlickerDetector, FLICKER_BAD_PCT, FLICKER_WARN_PCT};

/// A blur or texture condition must hold this long to raise its chip.
pub const HOLD_NS: u64 = 1_000_000_000;
/// Predicted motion blur, pixels, over which the blur chip warns / is bad.
pub const MOTION_BLUR_WARN_PX: f64 = 1.0;
pub const MOTION_BLUR_BAD_PX: f64 = 2.5;
/// Laplacian score relative to the camera's running average under which
/// the blur chip warns / is bad.
pub const BLUR_REL_WARN: f64 = 0.5;
pub const BLUR_REL_BAD: f64 = 0.3;
/// Weight of each frame in the running average of its camera's score.
pub const BLUR_EMA_ALPHA: f64 = 0.01;
/// A gyro sample older than this predicts nothing.
pub const GYRO_STALE_NS: u64 = 500_000_000;
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

/// What turns an angular rate into image motion for one frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Optics {
    /// Focal length in this frame's pixels.
    pub fx_px: f64,
    /// Exposure time, seconds.
    pub exposure_s: f64,
}

/// The motion blur, pixels, of a rotation at `rate_rad_s` over the exposure.
pub fn predicted_blur_px(rate_rad_s: f64, optics: Optics) -> f64 {
    rate_rad_s * optics.exposure_s * optics.fx_px
}

/// The level of a measure with held bad and warn conditions.
fn held_level(bad: &mut Held, warn: &mut Held, is_bad: bool, is_warn: bool, t_ns: u64) -> Level {
    let (b, w) = (bad.update(is_bad, t_ns), warn.update(is_warn, t_ns));
    if b {
        Level::Bad
    } else if w {
        Level::Warn
    } else {
        Level::Ok
    }
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
    /// Running average of the Laplacian score.
    blur_ema: Option<f64>,
    /// The last frame's score relative to the running average before it.
    blur_rel: Option<f64>,
    blur_warn: Held,
    blur_bad: Held,
    motion_warn: Held,
    motion_bad: Held,
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
    /// Host ns and magnitude (rad/s) of the latest gyro sample.
    gyro: Option<(u64, f64)>,
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

    /// The latest gyro sample (rad/s, body axes), received at host `t_ns`.
    pub fn on_gyro(&mut self, t_ns: u64, gyro: &[f64]) {
        if let [x, y, z] = gyro {
            self.gyro = Some((t_ns, (x * x + y * y + z * z).sqrt()));
        }
    }

    /// The motion blur, pixels, a frame at host `t_ns` with `optics` gets
    /// from the latest gyro rate; `None` without a recent gyro sample.
    pub fn motion_blur_px(&self, t_ns: u64, optics: Optics) -> Option<f64> {
        let (t, rate) = self.gyro?;
        (t.abs_diff(t_ns) <= GYRO_STALE_NS).then(|| predicted_blur_px(rate, optics))
    }

    /// Fold one frame's measures at `t_ns`; `optics` (when known) turns the
    /// gyro rate into the frame's motion blur.
    pub fn on_frame(&mut self, camera: &str, t_ns: u64, q: FrameQuality, optics: Option<Optics>) {
        let motion = optics.and_then(|o| self.motion_blur_px(t_ns, o));
        let c = self.cameras.entry(camera.to_string()).or_default();
        c.last = Some(q);
        c.last_t_ns = t_ns;
        c.flicker.push(t_ns, q.mean);
        let rel = match c.blur_ema {
            Some(ema) if ema > 0.0 => q.blur / ema,
            _ => 1.0,
        };
        c.blur_ema = Some(
            c.blur_ema
                .map_or(q.blur, |e| e + BLUR_EMA_ALPHA * (q.blur - e)),
        );
        c.blur_rel = Some(rel);
        let laplacian = held_level(
            &mut c.blur_bad,
            &mut c.blur_warn,
            rel < BLUR_REL_BAD,
            rel < BLUR_REL_WARN,
            t_ns,
        );
        let px = motion.unwrap_or(0.0);
        let moving = held_level(
            &mut c.motion_bad,
            &mut c.motion_warn,
            px > MOTION_BLUR_BAD_PX,
            px > MOTION_BLUR_WARN_PX,
            t_ns,
        );
        let blur = laplacian.max(moving);
        let tex = held_level(
            &mut c.tex_bad,
            &mut c.tex_warn,
            q.corners < TEXTURE_BAD,
            q.corners < TEXTURE_WARN,
            t_ns,
        );
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
    /// [`CAMERA_STALE_NS`]; with no live camera, blur and texture have no
    /// value and read `warn` (nothing is measuring, so not `ok`).
    pub fn health(&mut self, now_ns: u64) -> Vec<HealthView> {
        let live: Vec<&CameraQuality> = self
            .cameras
            .values()
            .filter(|c| now_ns.saturating_sub(c.last_t_ns) <= CAMERA_STALE_NS)
            .collect();
        let worst = |f: &dyn Fn(&CameraQuality) -> Level| {
            live.iter().map(|c| f(c)).max().unwrap_or(Level::Warn)
        };
        let blur_level = worst(&|c| c.levels.0);
        let tex_level = worst(&|c| c.levels.1);
        let min_of = |f: &dyn Fn(&FrameQuality) -> f64| {
            live.iter()
                .filter_map(|c| c.last.as_ref().map(f))
                .reduce(f64::min)
        };
        let blur_v = live.iter().filter_map(|c| c.blur_rel).reduce(f64::min);
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
