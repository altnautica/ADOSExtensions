//! The live sweep cloud as `GET /navigation/sweep?since=<seq>` serves it.
//!
//! The lane keeps the newest [`HISTORY`] deltas the live map produced. A
//! client that names the seq it holds gets the deltas after it; a client
//! with no seq, with one older than the history or newer than the lane (the
//! lane restarted), or one from before the cloud was coarsened gets the
//! whole cloud with `reset = 1` and must drop what it holds.
//!
//! The body is a 24-byte little-endian header (`seq: u64`, `count: u32`,
//! `voxel_m: f32`, `reset: u32`, `reserved: u32`) then `count` `f32`
//! `x, y, z` triples in `map_ned`.

use std::collections::VecDeque;

use base64::Engine as _;

use ados_mapping_protocol::atlas::LivePointCloud;

/// Deltas kept.
pub const HISTORY: usize = 64;
/// Header bytes.
pub const HEADER_LEN: usize = 24;

struct Delta {
    seq: u64,
    points: Vec<u8>,
}

/// The newest deltas since the last reset.
#[derive(Default)]
pub struct SweepHistory {
    deltas: VecDeque<Delta>,
    /// The newest seq the lane produced.
    seq: u64,
    voxel_m: f32,
    /// The first seq at the current voxel edge: a client holding an earlier
    /// one holds points at another edge.
    reset_seq: u64,
}

/// What a request gets.
#[derive(Debug, PartialEq)]
pub enum SweepReply {
    /// The deltas after the client's seq, packed.
    Delta {
        seq: u64,
        voxel_m: f32,
        points: Vec<u8>,
    },
    /// The client must take the whole cloud.
    Whole,
}

impl SweepHistory {
    /// Fold one live-map delta. A changed voxel edge (the cloud was
    /// coarsened) drops every earlier delta: they are superseded.
    pub fn push(&mut self, c: &LivePointCloud) {
        if c.voxel_m != self.voxel_m {
            self.deltas.clear();
            self.voxel_m = c.voxel_m;
            self.reset_seq = c.seq;
        }
        let points = base64::engine::general_purpose::STANDARD
            .decode(&c.points_b64)
            .unwrap_or_default();
        if self.deltas.len() == HISTORY {
            self.deltas.pop_front();
        }
        self.deltas.push_back(Delta { seq: c.seq, points });
        self.seq = c.seq;
    }

    /// The newest seq.
    pub fn seq(&self) -> u64 {
        self.seq
    }

    /// The reply to a client holding `since`.
    pub fn reply(&self, since: Option<u64>) -> SweepReply {
        let Some(since) = since else {
            return SweepReply::Whole;
        };
        let oldest = self.deltas.front().map_or(self.seq + 1, |d| d.seq);
        // The client holds everything up to `since`: it must reach back at
        // least to the delta before the oldest kept.
        if since > self.seq || since + 1 < oldest || since < self.reset_seq {
            return SweepReply::Whole;
        }
        let mut points = Vec::new();
        for d in self.deltas.iter().filter(|d| d.seq > since) {
            points.extend_from_slice(&d.points);
        }
        SweepReply::Delta {
            seq: self.seq,
            voxel_m: self.voxel_m,
            points,
        }
    }
}

/// The response body: header then the packed triples.
pub fn encode(seq: u64, voxel_m: f32, reset: bool, points: &[u8]) -> Vec<u8> {
    let count = (points.len() / 12) as u32;
    let mut out = Vec::with_capacity(HEADER_LEN + count as usize * 12);
    out.extend_from_slice(&seq.to_le_bytes());
    out.extend_from_slice(&count.to_le_bytes());
    out.extend_from_slice(&voxel_m.to_le_bytes());
    out.extend_from_slice(&u32::from(reset).to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes());
    out.extend_from_slice(&points[..count as usize * 12]);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use ados_mapping_protocol::atlas::{MapPayloadKind, NavFrame};

    fn delta(seq: u64, voxel_m: f32, n: usize) -> LivePointCloud {
        let bytes: Vec<u8> = (0..n * 12).map(|i| (seq as usize + i) as u8).collect();
        LivePointCloud {
            kind: MapPayloadKind::Live,
            seq,
            frame: NavFrame::MapNed,
            voxel_m,
            count: n as u32,
            points_b64: base64::engine::general_purpose::STANDARD.encode(bytes),
        }
    }

    #[test]
    fn a_client_gets_the_deltas_after_its_seq() {
        let mut h = SweepHistory::default();
        for s in 1..=5 {
            h.push(&delta(s, 0.05, 2));
        }
        match h.reply(Some(3)) {
            SweepReply::Delta {
                seq,
                voxel_m,
                points,
            } => {
                assert_eq!((seq, voxel_m, points.len()), (5, 0.05, 2 * 2 * 12));
            }
            other => panic!("{other:?}"),
        }
        assert_eq!(
            h.reply(Some(5)),
            SweepReply::Delta {
                seq: 5,
                voxel_m: 0.05,
                points: Vec::new()
            },
            "up to date: an empty delta"
        );
    }

    #[test]
    fn no_seq_an_aged_out_seq_a_future_seq_or_a_coarsening_needs_the_whole_cloud() {
        let mut h = SweepHistory::default();
        for s in 1..=(HISTORY as u64 + 10) {
            h.push(&delta(s, 0.05, 1));
        }
        assert_eq!(h.reply(None), SweepReply::Whole);
        assert_eq!(
            h.reply(Some(5)),
            SweepReply::Whole,
            "older than the history"
        );
        assert!(
            matches!(h.reply(Some(10)), SweepReply::Delta { .. }),
            "the seq before the oldest kept"
        );
        assert_eq!(
            h.reply(Some(1_000)),
            SweepReply::Whole,
            "the lane restarted"
        );
        h.push(&delta(HISTORY as u64 + 11, 0.10, 1));
        assert_eq!(
            h.reply(Some(HISTORY as u64 + 10)),
            SweepReply::Whole,
            "coarsened since"
        );
        assert!(matches!(
            h.reply(Some(HISTORY as u64 + 11)),
            SweepReply::Delta { .. }
        ));
    }

    #[test]
    fn the_body_has_the_documented_header() {
        let pts: Vec<u8> = [1.0f32, 2.0, 3.0]
            .iter()
            .flat_map(|f| f.to_le_bytes())
            .collect();
        let b = encode(7, 0.05, true, &pts);
        assert_eq!(b.len(), HEADER_LEN + 12);
        assert_eq!(u64::from_le_bytes(b[0..8].try_into().unwrap()), 7);
        assert_eq!(u32::from_le_bytes(b[8..12].try_into().unwrap()), 1);
        assert_eq!(f32::from_le_bytes(b[12..16].try_into().unwrap()), 0.05);
        assert_eq!(u32::from_le_bytes(b[16..20].try_into().unwrap()), 1);
        assert_eq!(f32::from_le_bytes(b[32..36].try_into().unwrap()), 3.0);
    }
}
