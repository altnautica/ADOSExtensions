//! The vision engine's camera depth, read off the plugin bus: `vision.depth.v2`
//! frames ([`ados_protocol::vision_depth_frame`]), a 128 x 96 depth plane from
//! the engine's monocular model with a per-pixel σ.
//!
//! The supervisor subscribes to the topic (it needs `vision.depth.read`) and
//! the subscription's callback drops each delivery into the process-wide
//! [`mailbox`]; [`VisionSource`] takes them from there on the map's depth
//! thread. The mailbox holds only the newest delivery: a slow map loses the
//! older frame (counted in `drops`), never stalls the bus.
//!
//! **Scale.** The model ranks depth; the engine fits it to metres with
//! ranging hits and publishes only frames with a fit. A frame goes to the map
//! only when its fit is metric enough: at least one inlier, a finite positive
//! scale and a fit no older than `vision_max_fit_age_ms`. The rest are
//! counted in `unscaled` and left out. Pixels whose σ exceeds
//! [`MAX_REL_SIGMA`] of their depth are blanked, so an uncertain pixel never
//! carves free space or marks a wall.
//!
//! **Geometry.** The plane's intrinsics put pixel centres at `i + 0.5`; this
//! source shifts them to the integer-centre convention of [`DepthFrame`]. With
//! no rotation configured in `navigation.depth.extrinsic`, the camera's mount
//! is the frame's own `pitch_down_deg` (the engine's configured camera mount).

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, LazyLock};
use std::time::{Duration, Instant};

use ados_protocol::vision_depth_frame::DepthFrame as BusFrame;
use parking_lot::{Condvar, Mutex};

use super::{DepthFrame, DepthHealth, DepthSource, DepthSourceError, DepthSourceKind, Extrinsic};

/// A pixel whose σ is more than this fraction of its depth is blanked.
pub const MAX_REL_SIGMA: f32 = 0.3;

/// The newest undelivered bus payload, and whether the topic is subscribed.
#[derive(Default)]
pub struct Mailbox {
    slot: Mutex<Slot>,
    ready: Condvar,
    subscribed: AtomicBool,
}

#[derive(Default)]
struct Slot {
    payload: Option<rmpv::Value>,
    /// Deliveries overwritten before the source took them.
    overwritten: u64,
}

impl Mailbox {
    /// Keep `payload` as the newest delivery, replacing an untaken one.
    pub fn put(&self, payload: rmpv::Value) {
        let mut slot = self.slot.lock();
        if slot.payload.replace(payload).is_some() {
            slot.overwritten += 1;
        }
        drop(slot);
        self.ready.notify_one();
    }

    /// Take the newest delivery, waiting at most `timeout`; with the number
    /// of deliveries lost since the last take.
    fn take(&self, timeout: Duration) -> (Option<rmpv::Value>, u64) {
        let mut slot = self.slot.lock();
        if slot.payload.is_none() {
            let _ = self.ready.wait_for(&mut slot, timeout);
        }
        let lost = std::mem::take(&mut slot.overwritten);
        (slot.payload.take(), lost)
    }

    pub fn set_subscribed(&self, on: bool) {
        self.subscribed.store(on, Ordering::Relaxed);
    }

    pub fn subscribed(&self) -> bool {
        self.subscribed.load(Ordering::Relaxed)
    }

    /// The callback for `event.subscribe`: deliveries carry the frame under
    /// `payload`.
    pub fn callback(self: &Arc<Self>) -> ados_sdk::EventCallback {
        let mailbox = self.clone();
        Arc::new(move |args: rmpv::Value| {
            let payload = args.as_map().and_then(|m| {
                m.iter()
                    .find(|(k, _)| k.as_str() == Some("payload"))
                    .map(|(_, v)| v.clone())
            });
            if let Some(p) = payload {
                mailbox.put(p);
            }
        })
    }
}

/// The process's one mailbox for `vision.depth.v2`.
pub fn mailbox() -> &'static Arc<Mailbox> {
    static MAILBOX: LazyLock<Arc<Mailbox>> = LazyLock::new(Arc::default);
    &MAILBOX
}

/// How long one depth lease holds; renewed every supervisor poll (5 s), so a
/// stopped service lets it lapse within this even without a release.
pub const LEASE_HOLD: Duration = Duration::from_secs(15);

/// The engine runs its depth model on a camera only while a depth lease is
/// held there. This tracks the camera this service leases: each poll asks
/// [`Self::step`] what to release and what to renew.
#[derive(Debug, Default)]
pub struct DepthLease {
    held: Option<String>,
}

/// One poll's lease work.
#[derive(Debug, PartialEq, Eq)]
pub struct LeaseStep {
    /// Release this camera's lease (hold 0).
    pub release: Option<String>,
    /// Take or renew this camera's lease for [`LEASE_HOLD`].
    pub renew: Option<String>,
}

impl DepthLease {
    /// The work for a poll that wants `want` leased (`None`: no lease). A
    /// renewal that fails is still released later: a stray release is
    /// harmless, a stray lease keeps the model running.
    pub fn step(&mut self, want: Option<&str>) -> LeaseStep {
        let release = self.held.take_if(|c| Some(c.as_str()) != want);
        self.held = want.map(str::to_owned);
        LeaseStep {
            release,
            renew: self.held.clone(),
        }
    }

    /// The camera to release when the service stops.
    pub fn take(&mut self) -> Option<String> {
        self.held.take()
    }
}

#[derive(Debug, Clone)]
pub struct VisionConfig {
    /// Only frames from this camera; empty takes every camera.
    pub camera: String,
    /// A fit older than this (ms) does not scale a frame for the map.
    pub max_fit_age_ms: u32,
    /// Translation, and the rotation unless [`Self::frame_mount`].
    pub extrinsic: Extrinsic,
    /// Take the rotation from each frame's `pitch_down_deg`.
    pub frame_mount: bool,
}

/// Why a decoded bus frame is not handed to the map.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Refusal {
    OtherCamera,
    Unscaled,
}

pub struct VisionSource {
    mailbox: Arc<Mailbox>,
    cfg: VisionConfig,
    health: DepthHealth,
    seq: u64,
    buf: Vec<u8>,
}

impl VisionSource {
    /// Read from `mailbox`; unavailable until the supervisor's subscription
    /// to the topic has succeeded.
    pub fn open(mailbox: Arc<Mailbox>, cfg: VisionConfig) -> Result<Self, DepthSourceError> {
        if !mailbox.subscribed() {
            return Err(DepthSourceError::Unavailable(
                "vision.depth.v2 is not subscribed (vision.depth.read not granted, or no host yet)"
                    .into(),
            ));
        }
        Ok(Self {
            mailbox,
            cfg,
            health: DepthHealth::default(),
            seq: 0,
            buf: Vec::new(),
        })
    }

    /// One decoded bus frame as a map frame stamped `stamp_ns`, or why not.
    pub fn convert(
        cfg: &VisionConfig,
        f: &BusFrame,
        stamp_ns: u64,
        seq: u64,
    ) -> Result<DepthFrame, Refusal> {
        if !cfg.camera.is_empty() && f.camera_id != cfg.camera {
            return Err(Refusal::OtherCamera);
        }
        let fit = &f.fit;
        if fit.inliers == 0
            || !fit.scale.is_finite()
            || fit.scale <= 0.0
            || fit.held_ms > cfg.max_fit_age_ms
        {
            return Err(Refusal::Unscaled);
        }
        let depth_mm = f
            .depth_mm
            .iter()
            .zip(&f.sigma_mm)
            .map(|(&z, &s)| {
                if f32::from(s) > MAX_REL_SIGMA * f32::from(z) {
                    0
                } else {
                    z
                }
            })
            .collect();
        let extrinsic = if cfg.frame_mount {
            let pitch = -f64::from(f.pitch_down_deg);
            Extrinsic::from_rpy_deg(cfg.extrinsic.t, [0.0, pitch, 0.0])
        } else {
            cfg.extrinsic
        };
        Ok(DepthFrame {
            stamp_ns,
            source_ns: f.stamp_ns,
            seq,
            w: u32::from(f.w),
            h: u32::from(f.h),
            fx: f.fx,
            fy: f.fy,
            cx: f.cx - 0.5,
            cy: f.cy - 0.5,
            depth_mm,
            extrinsic,
        })
    }
}

impl DepthSource for VisionSource {
    fn kind(&self) -> DepthSourceKind {
        DepthSourceKind::Vision
    }

    fn next_frame(&mut self, timeout: Duration) -> Result<Option<DepthFrame>, DepthSourceError> {
        let deadline = Instant::now() + timeout;
        loop {
            let left = deadline.saturating_duration_since(Instant::now());
            let (payload, lost) = self.mailbox.take(left);
            self.health.drops += lost;
            let Some(payload) = payload else {
                return Ok(None);
            };
            self.buf.clear();
            let decoded = rmpv::encode::write_value(&mut self.buf, &payload)
                .ok()
                .and_then(|()| BusFrame::from_msgpack(&self.buf).ok());
            let Some(bus) = decoded else {
                self.health.rejected += 1;
                continue;
            };
            let stamp_ns = crate::timebase::now_ns();
            match Self::convert(&self.cfg, &bus, stamp_ns, self.seq) {
                Ok(frame) => {
                    self.seq += 1;
                    self.health.frames += 1;
                    self.health.last_stamp_ns = stamp_ns;
                    return Ok(Some(frame));
                }
                Err(Refusal::Unscaled) => self.health.unscaled += 1,
                Err(Refusal::OtherCamera) => {}
            }
            if left.is_zero() {
                return Ok(None);
            }
        }
    }

    fn health(&self) -> DepthHealth {
        self.health
    }
}

#[cfg(test)]
mod tests;
