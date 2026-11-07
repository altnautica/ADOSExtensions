//! Live map builders shared by the drone's capture service and the compute
//! node: [`OccupancyGrid`] (5 cm tiles, ray traced, evicted by distance and
//! budget) and [`SweepCloud`] (a bounded voxel-centroid cloud), plus
//! [`LiveMap`], which feeds both from posed polar scans and emits the live
//! occupancy and point-cloud payloads.

pub mod occupancy;
pub mod sweep;

pub use occupancy::OccupancyGrid;
pub use sweep::SweepCloud;

use std::path::Path;

use world_engine_protocol::atlas::nav::OCCUPANCY_TILE_CELLS;
use world_engine_protocol::atlas::{
    LiveOccupancy, LivePointCloud, MapPayloadKind, NavFrame, PolarScan, VioHealth,
};

/// Every this many ticks (5 s at the capture's 1 Hz) the occupancy payload
/// also carries every tile within [`OCCUPANCY_RESEND_RADIUS_M`] of the
/// vehicle, changed or not, so a subscriber that started late or missed a
/// payload holds the map around the vehicle within that time. Subscribers
/// overwrite tiles, so a repeat is harmless.
pub const OCCUPANCY_RESEND_TICKS: u64 = 5;
/// The radius of that resend, metres.
pub const OCCUPANCY_RESEND_RADIUS_M: f32 = 20.0;

/// The occupancy grid and sweep cloud fed together.
pub struct LiveMap {
    pub grid: OccupancyGrid,
    pub sweep: SweepCloud,
    occ_seq: u64,
    cloud_seq: u64,
    last_voxel_m: f32,
    ticks: u64,
    /// The latest sensor position, map_ned x, y.
    pub vehicle: [f32; 2],
}

impl LiveMap {
    pub fn new(voxel_m: f32, sweep_cap_bytes: usize) -> Self {
        Self {
            grid: OccupancyGrid::new(),
            sweep: SweepCloud::new(voxel_m, sweep_cap_bytes),
            occ_seq: 0,
            cloud_seq: 0,
            last_voxel_m: voxel_m,
            ticks: 0,
            vehicle: [0.0, 0.0],
        }
    }

    /// Paint one posed scan; a lost pose paints nothing.
    pub fn insert(&mut self, scan: &PolarScan) {
        self.vehicle = [scan.t[0], scan.t[1]];
        if scan.health == VioHealth::Lost {
            return;
        }
        for p in scan.world_points() {
            self.grid.insert_ray(self.vehicle, [p[0], p[1]]);
        }
        self.sweep.insert(scan.world_points());
    }

    /// Evict far / over-budget tiles (to `dir` when given), then the changed
    /// tiles (every [`OCCUPANCY_RESEND_TICKS`]th tick with the tiles near the
    /// vehicle) and the new sweep points as live payloads, each `None` when
    /// nothing changed.
    pub fn tick(
        &mut self,
        occupancy_cap_bytes: usize,
        evict_dir: Option<&Path>,
    ) -> (Option<LiveOccupancy>, Option<LivePointCloud>) {
        self.grid
            .evict(self.vehicle, occupancy_cap_bytes, evict_dir);
        self.ticks += 1;
        if self.ticks.is_multiple_of(OCCUPANCY_RESEND_TICKS) {
            let [n, e] = self.vehicle;
            self.grid.resend_near(n, e, OCCUPANCY_RESEND_RADIUS_M);
        }
        let tiles = self.grid.take_changed();
        let occ = (!tiles.is_empty()).then(|| {
            self.occ_seq += 1;
            LiveOccupancy {
                kind: MapPayloadKind::Live,
                seq: self.occ_seq,
                frame: NavFrame::MapNed,
                res_m: occupancy::RES_M,
                tile_cells: OCCUPANCY_TILE_CELLS,
                tiles,
            }
        });
        let (count, points_b64) = self.sweep.take_delta();
        let coarsened = self.sweep.voxel_m() != self.last_voxel_m;
        self.last_voxel_m = self.sweep.voxel_m();
        let cloud = (count > 0 || coarsened).then(|| {
            self.cloud_seq += 1;
            LivePointCloud {
                kind: MapPayloadKind::Live,
                seq: self.cloud_seq,
                frame: NavFrame::MapNed,
                voxel_m: self.sweep.voxel_m(),
                count,
                points_b64,
            }
        });
        (occ, cloud)
    }

    /// RAM the grid and cloud hold, bytes.
    pub fn bytes(&self) -> usize {
        self.grid.bytes() + self.sweep.bytes()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn an_unchanged_tile_near_the_vehicle_is_resent_every_fifth_tick() {
        let mut m = LiveMap::new(0.05, 1 << 20);
        m.grid.insert_ray([0.01, 0.01], [1.0, 0.01]);
        m.grid.insert_ray([100.0, 0.0], [100.5, 0.0]);
        let keys = |o: Option<LiveOccupancy>| {
            o.map(|o| o.tiles.iter().map(|t| (t.ix, t.iy)).collect::<Vec<_>>())
        };
        assert_eq!(
            keys(m.tick(usize::MAX, None).0),
            Some(vec![(0, 0), (31, 0)])
        );
        for tick in 2..=4 {
            assert_eq!(keys(m.tick(usize::MAX, None).0), None, "tick {tick}");
        }
        let fifth = m.tick(usize::MAX, None).0.expect("the 5th tick resends");
        assert_eq!(fifth.seq, 2);
        assert_eq!(keys(Some(fifth)), Some(vec![(0, 0)]), "only the near tile");
        assert_eq!(keys(m.tick(usize::MAX, None).0), None);
    }
}
