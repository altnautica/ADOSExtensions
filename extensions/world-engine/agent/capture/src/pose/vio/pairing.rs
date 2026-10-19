//! Stereo pairing: the left and right camera frames that belong together.
//!
//! Two frames pair when both carry a `trigger_seq` and the sequences are
//! equal, or, when either lacks one, when their host timestamps are within
//! [`PAIR_TOLERANCE_NS`]. Only Gray8 frames of equal size pair. The newest
//! unmatched frame of each side is held until its partner arrives or a newer
//! frame of the same side replaces it.

use std::borrow::Cow;

use ados_protocol::framebus::FrameFormat;

use crate::frame_source::CapturedFrame;

/// Largest timestamp gap of an untriggered pair.
pub const PAIR_TOLERANCE_NS: u64 = 1_000_000;

#[derive(Debug, Clone)]
struct Pending {
    ts_ns: u64,
    trigger_seq: Option<u64>,
    width: u32,
    height: u32,
    pixels: Vec<u8>,
}

impl Pending {
    fn of(f: &CapturedFrame) -> Self {
        Self {
            ts_ns: f.ts_ns,
            trigger_seq: f.trigger_seq,
            width: f.width,
            height: f.height,
            pixels: f.pixels.clone(),
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
    /// Not Gray8, or the pixel buffer does not match the size: dropped.
    Rejected,
    /// Held for its partner.
    Held,
    Paired(StereoPair<'a>),
}

#[derive(Debug)]
pub struct StereoPairer {
    left_id: String,
    right_id: String,
    left: Option<Pending>,
    right: Option<Pending>,
}

impl StereoPairer {
    pub fn new(left_id: impl Into<String>, right_id: impl Into<String>) -> Self {
        Self {
            left_id: left_id.into(),
            right_id: right_id.into(),
            left: None,
            right: None,
        }
    }

    pub fn push<'a>(&mut self, f: &'a CapturedFrame) -> PairOutcome<'a> {
        let is_left = f.camera_id == self.left_id;
        if !is_left && f.camera_id != self.right_id {
            return PairOutcome::Ignored;
        }
        let plane = f.width as usize * f.height as usize;
        if f.format != FrameFormat::Gray8 || f.pixels.len() != plane || plane == 0 {
            return PairOutcome::Rejected;
        }
        let (mine, other) = if is_left {
            (&mut self.left, &mut self.right)
        } else {
            (&mut self.right, &mut self.left)
        };
        match other.take_if(|p| p.matches(f)) {
            Some(p) => {
                *mine = None;
                let (t_ns, left, right) = if is_left {
                    (
                        f.ts_ns,
                        Cow::Borrowed(f.pixels.as_slice()),
                        Cow::Owned(p.pixels),
                    )
                } else {
                    (
                        p.ts_ns,
                        Cow::Owned(p.pixels),
                        Cow::Borrowed(f.pixels.as_slice()),
                    )
                };
                PairOutcome::Paired(StereoPair {
                    t_ns,
                    width: f.width,
                    height: f.height,
                    left,
                    right,
                })
            }
            None => {
                *mine = Some(Pending::of(f));
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
