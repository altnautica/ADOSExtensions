//! Publishing the map over the agent's voxel-map seam.

use ados_protocol::voxmap::{VoxTier, VoxmapError, VoxmapMeta, VoxmapWriter};

use crate::insert::Pose;
use crate::VoxMap;

impl VoxMap {
    /// The header this map publishes with body pose `body` and `tier`.
    pub fn seam_meta(&self, body: &Pose, tier: VoxTier) -> VoxmapMeta {
        let [tn, te, td] = body.t;
        let [qw, qx, qy, qz] = body.q;
        VoxmapMeta {
            stamp_ns: self.stamp_ns,
            res_m: self.res,
            origin_cell: self.origin,
            dims: self.dims(),
            map_from_body: [tn, te, td, qw, qx, qy, qz],
            tier,
        }
    }

    /// Publish occupancy and distance through `writer` (created for this
    /// map's [`dims`](VoxMap::dims)), unrolling the ring into window order,
    /// x fastest. Returns the publication's guard value.
    pub fn to_seam(
        &self,
        writer: &mut VoxmapWriter,
        body: &Pose,
        tier: VoxTier,
    ) -> Result<u64, VoxmapError> {
        let meta = self.seam_meta(body, tier);
        writer.publish(&meta, |occ, esdf| {
            self.unroll_planes(occ, esdf);
        })
    }

    /// Write the class plane into `occ` and the decimetre distance plane into
    /// `esdf`, both in window order.
    pub(crate) fn unroll_planes(&self, occ: &mut [u8], esdf: &mut [u8]) {
        let [nx, ny, nz] = self.n.map(|v| v as usize);
        let omx = self.om[0] as usize;
        let head = nx - omx;
        let mut out = 0;
        for wz in 0..nz as i32 {
            for wy in 0..ny as i32 {
                let r = self.ring_of_window([0, wy, wz]);
                let row = self.idx_of_ring([0, r[1], r[2]]);
                // Window x 0.. starts at ring x `omx` and wraps to ring x 0.
                let (tail, front) = (row + omx..row + nx, row..row + omx);
                occ[out..out + head].copy_from_slice(&self.state[tail.clone()]);
                occ[out + head..out + nx].copy_from_slice(&self.state[front.clone()]);
                esdf[out..out + head].copy_from_slice(&self.esdf.dm[tail]);
                esdf[out + head..out + nx].copy_from_slice(&self.esdf.dm[front]);
                out += nx;
            }
        }
        debug_assert_eq!(out, nx * ny * nz);
    }
}
