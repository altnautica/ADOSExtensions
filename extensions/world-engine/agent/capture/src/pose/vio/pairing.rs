//! Stereo pairing: the left and right camera frames that belong together.
//!
//! Two frames pair when both carry a `trigger_seq` and the sequences are
//! equal, or, when either lacks one, when their host timestamps are within
//! [`PAIR_TOLERANCE_NS`]. Basalt tracks on 8-bit luma, so each frame is
//! reduced to Gray8 first ([`luma`]): Gray8 as is, the Y plane of NV12 and
//! YUV 4:2:0, BT.601 luma of RGB24, the high byte of Gray16. A Bayer mosaic
//! or a buffer that does not match its size is rejected. The newest unmatched
//! frame of each side is held until its partner arrives or a newer frame of
//! the same side replaces it. At [`VioScale::Half`] each luma plane is
//! halved in both directions (2×2 box average) before it is held, so Basalt's
//! optical flow does a quarter of the work.

use std::borrow::Cow;

use ados_protocol::framebus::FrameFormat;

use crate::frame_source::CapturedFrame;

/// Largest timestamp gap of an untriggered pair.
pub const PAIR_TOLERANCE_NS: u64 = 1_000_000;

/// The resolution VIO runs at (`atlas.vio.scale`): the frames as captured
/// (`1`), or halved in each direction (`0.5`) for a device without the CPU
/// for full-resolution optical flow. The Basalt calibration is scaled to
/// match ([`super::calib::scaled_rig`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Deserialize)]
#[serde(try_from = "f64")]
pub enum VioScale {
    #[default]
    Full,
    Half,
}

impl TryFrom<f64> for VioScale {
    type Error = String;

    fn try_from(v: f64) -> Result<Self, String> {
        if v == 1.0 {
            Ok(Self::Full)
        } else if v == 0.5 {
            Ok(Self::Half)
        } else {
            Err(format!("atlas.vio.scale must be 1 or 0.5, got {v}"))
        }
    }
}

impl VioScale {
    pub fn factor(self) -> f64 {
        match self {
            Self::Full => 1.0,
            Self::Half => 0.5,
        }
    }

    /// The image size VIO sees for a `width`×`height` frame.
    pub fn size(self, width: u32, height: u32) -> (u32, u32) {
        match self {
            Self::Full => (width, height),
            Self::Half => (width / 2, height / 2),
        }
    }
}

/// `gray` (`width`×`height`) halved in both directions by averaging each 2×2
/// block; an odd last row or column is dropped.
pub fn halve(gray: &[u8], width: u32, height: u32) -> Vec<u8> {
    let (w, ow, oh) = (width as usize, width as usize / 2, height as usize / 2);
    let mut out = Vec::with_capacity(ow * oh);
    for y in 0..oh {
        let top = &gray[2 * y * w..][..2 * ow];
        let bottom = &gray[(2 * y + 1) * w..][..2 * ow];
        out.extend(
            top.as_chunks::<2>()
                .0
                .iter()
                .zip(bottom.as_chunks::<2>().0)
                .map(|([a, b], [c, d])| {
                    ((u16::from(*a) + u16::from(*b) + u16::from(*c) + u16::from(*d) + 2) / 4) as u8
                }),
        );
    }
    out
}

/// The 8-bit luma plane of `f`, or `None` for a Bayer mosaic or a buffer
/// whose length does not match its format and size.
pub fn luma(f: &CapturedFrame) -> Option<Cow<'_, [u8]>> {
    let px = f.width as usize * f.height as usize;
    if px == 0 || f.pixels.len() != f.format.frame_bytes(f.width, f.height) {
        return None;
    }
    match f.format {
        FrameFormat::Gray8 => Some(Cow::Borrowed(f.pixels.as_slice())),
        FrameFormat::Nv12 | FrameFormat::Yuv420p => Some(Cow::Borrowed(&f.pixels[..px])),
        // BT.601 luma in 8.8 fixed point: 77 R + 150 G + 29 B (sum 256).
        FrameFormat::Rgb24 => Some(Cow::Owned(
            f.pixels
                .as_chunks::<3>()
                .0
                .iter()
                .map(|[r, g, b]| {
                    ((77 * u32::from(*r) + 150 * u32::from(*g) + 29 * u32::from(*b) + 128) >> 8)
                        as u8
                })
                .collect(),
        )),
        FrameFormat::Gray16 => Some(Cow::Owned(
            f.pixels
                .as_chunks::<2>()
                .0
                .iter()
                .map(|[_, hi]| *hi)
                .collect(),
        )),
        FrameFormat::BayerRggb8 | FrameFormat::BayerRggb16 => None,
    }
}

#[derive(Debug, Clone)]
struct Pending {
    ts_ns: u64,
    trigger_seq: Option<u64>,
    width: u32,
    height: u32,
    pixels: Vec<u8>,
}

impl Pending {
    fn of(f: &CapturedFrame, gray: Cow<'_, [u8]>) -> Self {
        Self {
            ts_ns: f.ts_ns,
            trigger_seq: f.trigger_seq,
            width: f.width,
            height: f.height,
            pixels: gray.into_owned(),
        }
    }

    fn matches(&self, f: &CapturedFrame) -> bool {
        if self.width != f.width || self.height != f.height {
            return false;
        }
        match (self.trigger_seq, f.trigger_seq) {
            (Some(a), Some(b)) => a == b,
            _ => self.ts_ns.abs_diff(f.ts_ns) <= PAIR_TOLERANCE_NS,
        }
    }
}

/// A matched pair; the pair time is the left frame's.
#[derive(Debug)]
pub struct StereoPair<'a> {
    pub t_ns: u64,
    pub width: u32,
    pub height: u32,
    pub left: Cow<'a, [u8]>,
    pub right: Cow<'a, [u8]>,
}

/// What [`StereoPairer::push`] did with a frame.
#[derive(Debug)]
pub enum PairOutcome<'a> {
    /// Not one of the two cameras.
    Ignored,
    /// A Bayer mosaic, or the pixel buffer does not match the size: dropped.
    Rejected,
    /// Held for its partner.
    Held,
    Paired(StereoPair<'a>),
}

#[derive(Debug)]
pub struct StereoPairer {
    left_id: String,
    right_id: String,
    scale: VioScale,
    left: Option<Pending>,
    right: Option<Pending>,
}

impl StereoPairer {
    pub fn new(left_id: impl Into<String>, right_id: impl Into<String>, scale: VioScale) -> Self {
        Self {
            left_id: left_id.into(),
            right_id: right_id.into(),
            scale,
            left: None,
            right: None,
        }
    }

    pub fn push<'a>(&mut self, f: &'a CapturedFrame) -> PairOutcome<'a> {
        let is_left = f.camera_id == self.left_id;
        if !is_left && f.camera_id != self.right_id {
            return PairOutcome::Ignored;
        }
        let Some(gray) = luma(f) else {
            return PairOutcome::Rejected;
        };
        let gray = match self.scale {
            VioScale::Full => gray,
            VioScale::Half => Cow::Owned(halve(&gray, f.width, f.height)),
        };
        let (width, height) = self.scale.size(f.width, f.height);
        let (mine, other) = if is_left {
            (&mut self.left, &mut self.right)
        } else {
            (&mut self.right, &mut self.left)
        };
        match other.take_if(|p| p.matches(f)) {
            Some(p) => {
                *mine = None;
                let (t_ns, left, right) = if is_left {
                    (f.ts_ns, gray, Cow::Owned(p.pixels))
                } else {
                    (p.ts_ns, Cow::Owned(p.pixels), gray)
                };
                PairOutcome::Paired(StereoPair {
                    t_ns,
                    width,
                    height,
                    left,
                    right,
                })
            }
            None => {
                *mine = Some(Pending::of(f, gray));
                PairOutcome::Held
            }
        }
    }

    /// Forget held frames (the consumer restarted).
    pub fn clear(&mut self) {
        self.left = None;
        self.right = None;
    }
}
