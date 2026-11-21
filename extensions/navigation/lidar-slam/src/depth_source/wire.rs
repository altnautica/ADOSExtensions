//! The World Engine depth stream's wire format and frame reassembly.
//!
//! One frame is cut into datagrams of at most [`MAX_DATAGRAM`] bytes, each a
//! [`HEADER_LEN`]-byte little-endian header and one chunk of the payload:
//!
//! | offset | type | field |
//! |---|---|---|
//! | 0 | `[u8; 10]` | magic `ADOSDEPTH1` |
//! | 10 | u32 | `seq`: frame number |
//! | 14 | u64 | `sim_ns`: sim time of the frame, ns |
//! | 22 | u16 | `w`, px |
//! | 24 | u16 | `h`, px |
//! | 26 | f32 ×4 | `fx`, `fy`, `cx`, `cy`, px |
//! | 42 | u16 | `frag`: this datagram's index in the frame, from 0 |
//! | 44 | u16 | `frags`: datagrams in the frame |
//! | 46 | u32 | `offset`: byte offset of the chunk in the payload |
//! | 50 | bytes | the chunk |
//!
//! The payload is `w · h` u16 millimetres, row-major, 0 for no return; every
//! chunk holds whole pixels. A subscriber sends any datagram ([`SUBSCRIBE`]
//! by convention) to the stream's port at least every 2 s.
//!
//! [`Reassembler`] tolerates loss, duplicates and reordering: it keeps the
//! two newest incomplete frames, so a fragment overtaken by the next frame's
//! first datagrams still completes its frame; a frame still incomplete when
//! a third arrives, or when a newer one completes, is dropped.

/// Frame magic.
pub const MAGIC: &[u8; 10] = b"ADOSDEPTH1";
/// Header bytes before each chunk.
pub const HEADER_LEN: usize = 50;
/// Largest datagram: a 1500-byte MTU less the IPv4 and UDP headers.
pub const MAX_DATAGRAM: usize = 1472;
/// Payload bytes per datagram the World Engine sends (even).
pub const CHUNK: usize = (MAX_DATAGRAM - HEADER_LEN) & !1;
/// The subscription datagram.
pub const SUBSCRIBE: &[u8] = b"ADOSDEPTH1 SUB";
/// Largest frame accepted, px (1280 × 800).
pub const MAX_PIXELS: usize = 1280 * 800;
/// A fragment this many frames older than the newest one seen means the
/// stream restarted (a new sim run counts from 0 again).
pub const RESTART_GAP: u32 = 64;

/// One datagram's header.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Header {
    pub seq: u32,
    pub sim_ns: u64,
    pub w: u16,
    pub h: u16,
    pub fx: f32,
    pub fy: f32,
    pub cx: f32,
    pub cy: f32,
    pub frag: u16,
    pub frags: u16,
    pub offset: u32,
}

impl Header {
    /// Payload bytes of the whole frame.
    pub fn payload_len(&self) -> usize {
        usize::from(self.w) * usize::from(self.h) * 2
    }

    fn same_frame(&self, o: &Header) -> bool {
        (self.seq, self.sim_ns, self.w, self.h, self.frags) == (o.seq, o.sim_ns, o.w, o.h, o.frags)
            && [self.fx, self.fy, self.cx, self.cy] == [o.fx, o.fy, o.cx, o.cy]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, thiserror::Error)]
pub enum WireError {
    #[error("datagram of {0} bytes is shorter than the header")]
    Short(usize),
    #[error("not an ADOSDEPTH1 datagram")]
    Magic,
    #[error("frame size or intrinsics out of range")]
    Geometry,
    #[error("fragment index, offset or length out of range")]
    Fragment,
}

fn u16_at(d: &[u8], i: usize) -> u16 {
    u16::from_le_bytes([d[i], d[i + 1]])
}

fn u32_at(d: &[u8], i: usize) -> u32 {
    u32::from_le_bytes([d[i], d[i + 1], d[i + 2], d[i + 3]])
}

fn f32_at(d: &[u8], i: usize) -> f32 {
    f32::from_bits(u32_at(d, i))
}

/// Parse one datagram into its header and chunk.
pub fn parse(d: &[u8]) -> Result<(Header, &[u8]), WireError> {
    if d.len() < HEADER_LEN {
        return Err(WireError::Short(d.len()));
    }
    if &d[..10] != MAGIC {
        return Err(WireError::Magic);
    }
    let mut ns = [0u8; 8];
    ns.copy_from_slice(&d[14..22]);
    let h = Header {
        seq: u32_at(d, 10),
        sim_ns: u64::from_le_bytes(ns),
        w: u16_at(d, 22),
        h: u16_at(d, 24),
        fx: f32_at(d, 26),
        fy: f32_at(d, 30),
        cx: f32_at(d, 34),
        cy: f32_at(d, 38),
        frag: u16_at(d, 42),
        frags: u16_at(d, 44),
        offset: u32_at(d, 46),
    };
    let px = usize::from(h.w) * usize::from(h.h);
    let focal_ok = |f: f32| f.is_finite() && f > 0.0;
    if px == 0
        || px > MAX_PIXELS
        || !focal_ok(h.fx)
        || !focal_ok(h.fy)
        || !h.cx.is_finite()
        || !h.cy.is_finite()
    {
        return Err(WireError::Geometry);
    }
    let chunk = &d[HEADER_LEN..];
    let off = h.offset as usize;
    if h.frags == 0
        || h.frag >= h.frags
        || usize::from(h.frags) > px
        || chunk.is_empty()
        || !off.is_multiple_of(2)
        || !chunk.len().is_multiple_of(2)
        || off + chunk.len() > h.payload_len()
    {
        return Err(WireError::Fragment);
    }
    Ok((h, chunk))
}

/// Calls `send` with each datagram of a frame of `depth_mm`, built in `buf`,
/// as the World Engine cuts it (`head.frag`, `frags` and `offset` are
/// filled in). For tests and replays.
pub fn for_each_datagram(
    head: &Header,
    depth_mm: &[u16],
    buf: &mut Vec<u8>,
    mut send: impl FnMut(&[u8]),
) {
    let px_per_chunk = CHUNK / 2;
    let frags = depth_mm.len().div_ceil(px_per_chunk).max(1);
    for (i, px) in depth_mm.chunks(px_per_chunk).enumerate() {
        buf.clear();
        buf.extend_from_slice(MAGIC);
        buf.extend_from_slice(&head.seq.to_le_bytes());
        buf.extend_from_slice(&head.sim_ns.to_le_bytes());
        buf.extend_from_slice(&head.w.to_le_bytes());
        buf.extend_from_slice(&head.h.to_le_bytes());
        for v in [head.fx, head.fy, head.cx, head.cy] {
            buf.extend_from_slice(&v.to_le_bytes());
        }
        buf.extend_from_slice(&(i as u16).to_le_bytes());
        buf.extend_from_slice(&(frags as u16).to_le_bytes());
        buf.extend_from_slice(&((i * CHUNK) as u32).to_le_bytes());
        for &mm in px {
            buf.extend_from_slice(&mm.to_le_bytes());
        }
        send(buf);
    }
}

/// A frame reassembled from its datagrams.
#[derive(Debug, Clone, PartialEq)]
pub struct Assembled {
    /// The header of its first datagram to arrive (`frag`/`offset` are that
    /// datagram's).
    pub head: Header,
    pub depth_mm: Vec<u16>,
}

struct Partial {
    head: Header,
    depth_mm: Vec<u16>,
    got: Vec<bool>,
    got_n: u16,
    bytes: usize,
}

/// Counters since the reassembler was made.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ReassemblyStats {
    pub frames: u64,
    /// Frames dropped with fragments missing.
    pub incomplete: u64,
    /// Malformed or inconsistent datagrams.
    pub rejected: u64,
    /// Duplicates and fragments of frames already complete or dropped.
    pub stale: u64,
}

/// Rebuilds frames from datagrams; see the module docs for the rules.
#[derive(Default)]
pub struct Reassembler {
    slots: [Option<Partial>; 2],
    /// Newest seq that completed or was dropped.
    retired: Option<u32>,
    spare: Vec<u16>,
    pub stats: ReassemblyStats,
}

/// Whether `a` comes after `b` in wrapping frame order.
fn newer(a: u32, b: u32) -> bool {
    (a.wrapping_sub(b) as i32) > 0
}

impl Reassembler {
    /// Take one datagram; returns the frame it completes.
    pub fn push(&mut self, d: &[u8]) -> Option<Assembled> {
        let Ok((head, chunk)) = parse(d) else {
            self.stats.rejected += 1;
            return None;
        };
        if let Some(r) = self.retired {
            if !newer(head.seq, r) {
                if r.wrapping_sub(head.seq) < RESTART_GAP {
                    self.stats.stale += 1;
                    return None;
                }
                self.restart();
            }
        }
        let i = match self
            .slots
            .iter()
            .position(|s| s.as_ref().is_some_and(|p| p.head.seq == head.seq))
        {
            Some(i) => i,
            None if self
                .slots
                .iter()
                .all(|s| s.as_ref().is_some_and(|p| newer(p.head.seq, head.seq))) =>
            {
                // Older than both frames in flight: it would evict a newer one.
                self.stats.stale += 1;
                return None;
            }
            None => self.open_slot(&head),
        };
        let p = self.slots[i].as_mut()?;
        if !p.head.same_frame(&head) {
            self.stats.rejected += 1;
            return None;
        }
        let f = usize::from(head.frag);
        if p.got[f] {
            self.stats.stale += 1;
            return None;
        }
        let first = head.offset as usize / 2;
        for (dst, src) in p.depth_mm[first..first + chunk.len() / 2]
            .iter_mut()
            .zip(chunk.as_chunks::<2>().0)
        {
            *dst = u16::from_le_bytes(*src);
        }
        p.got[f] = true;
        p.got_n += 1;
        p.bytes += chunk.len();
        if p.got_n < p.head.frags {
            return None;
        }
        let done = self.slots[i].take()?;
        if done.bytes != done.head.payload_len() {
            // Every fragment arrived yet the chunks do not tile the payload.
            self.stats.rejected += 1;
            self.retire(done.head.seq);
            self.spare = done.depth_mm;
            return None;
        }
        self.retire(done.head.seq);
        self.stats.frames += 1;
        Some(Assembled {
            head: done.head,
            depth_mm: done.depth_mm,
        })
    }

    /// Give a frame buffer back for reuse.
    pub fn recycle(&mut self, buf: Vec<u16>) {
        if buf.capacity() > self.spare.capacity() {
            self.spare = buf;
        }
    }

    /// A slot for a frame not yet seen, evicting the older incomplete one
    /// when both are taken.
    fn open_slot(&mut self, head: &Header) -> usize {
        let i = match self.slots.iter().position(Option::is_none) {
            Some(i) => i,
            None => {
                let seq = |s: &Option<Partial>| s.as_ref().map_or(0, |p| p.head.seq);
                let older = usize::from(newer(seq(&self.slots[0]), seq(&self.slots[1])));
                if let Some(p) = self.slots[older].take() {
                    self.stats.incomplete += 1;
                    self.spare = p.depth_mm;
                }
                older
            }
        };
        let px = usize::from(head.w) * usize::from(head.h);
        let mut depth_mm = std::mem::take(&mut self.spare);
        depth_mm.clear();
        depth_mm.resize(px, 0);
        self.slots[i] = Some(Partial {
            head: *head,
            depth_mm,
            got: vec![false; usize::from(head.frags)],
            got_n: 0,
            bytes: 0,
        });
        i
    }

    /// `seq` is done: drop every incomplete frame before it.
    fn retire(&mut self, seq: u32) {
        for s in &mut self.slots {
            if s.as_ref().is_some_and(|p| !newer(p.head.seq, seq)) {
                if let Some(p) = s.take() {
                    self.stats.incomplete += 1;
                    if p.depth_mm.capacity() > self.spare.capacity() {
                        self.spare = p.depth_mm;
                    }
                }
            }
        }
        if self.retired.is_none_or(|r| newer(seq, r)) {
            self.retired = Some(seq);
        }
    }

    fn restart(&mut self) {
        for s in &mut self.slots {
            if s.take().is_some() {
                self.stats.incomplete += 1;
            }
        }
        self.retired = None;
    }
}
