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

use ados_mapping_protocol::capture::nav::OCCUPANCY_TILE_CELLS;
use ados_mapping_protocol::capture::{
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
/// The top of the band a drone sweeps over its scan plane, metres: the
/// airframe's top (0.03 m) plus 0.3 m of vertical clearance, the band the
/// ADOS pilot's obstacle brake keeps walls in. Returns over it are not
/// walls at flight height.
pub const BAND_TOP_M: f32 = 0.33;

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

    /// Paint one posed scan; a lost pose paints nothing. A return levelled
    /// more than [`BAND_TOP_M`] over the sensor (the scan's pose carries the
    /// tilt) met something over the drone, a canopy's underside a leaning
    /// scan reached: it paints its cell neither occupied nor the cells along
    /// its beam free, since the beam left the flight band before it. The
    /// sweep cloud keeps every return.
    pub fn insert(&mut self, scan: &PolarScan) {
        self.vehicle = [scan.t[0], scan.t[1]];
        if scan.health == VioHealth::Lost {
            return;
        }
        let up = scan.t[2];
        for p in scan.world_points() {
            if up - p[2] <= BAND_TOP_M {
                self.grid.insert_ray(self.vehicle, [p[0], p[1]]);
            }
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

    /// A scan pitched 10° nose up under a canopy: the beam ahead meets the
    /// leaves 0.5 m over the sensor, 2.8 m out, and paints nothing; the
    /// one behind, down into the band, paints its wall and the free cells
    /// before it.
    #[test]
    fn a_return_over_the_flight_band_paints_no_wall_and_no_free_space() {
        // World-from-sensor: 10° about y, the forward axis rising.
        let half = 10f32.to_radians() / 2.0;
        let scan = PolarScan {
            t_ns: 1,
            angle_start_rad: 0.0,
            angle_inc_rad: std::f32::consts::PI,
            t: [0.0, 0.0, -4.0],
            q: [half.cos(), 0.0, half.sin(), 0.0],
            health: VioHealth::Good,
            range_mm: vec![2843, 2000],
            quality: vec![0, 0],
        };
        let pts: Vec<_> = scan.world_points().collect();
        // Height over the sensor (NED: up is the smaller down).
        assert!(-4.0 - pts[0][2] > BAND_TOP_M, "{pts:?}");
        assert!(-4.0 - pts[1][2] < 0.0, "{pts:?}");
        let mut m = LiveMap::new(0.05, 1 << 20);
        for _ in 0..5 {
            m.insert(&scan);
        }
        let unknown = ados_mapping_protocol::capture::nav::OCCUPANCY_UNKNOWN;
        assert_eq!(m.grid.value_at(pts[0][0], pts[0][1]), unknown, "no wall");
        assert_eq!(m.grid.value_at(1.0, 0.0), unknown, "no free space under it");
        assert_ne!(
            m.grid.value_at(pts[1][0], pts[1][1]),
            unknown,
            "a wall behind"
        );
        // Near the vehicle: the beam's y drifts a hair off zero (sin π), so
        // its far half runs along the next row of cells.
        assert_ne!(m.grid.value_at(-0.5, 0.0), unknown, "free before it");
    }

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
