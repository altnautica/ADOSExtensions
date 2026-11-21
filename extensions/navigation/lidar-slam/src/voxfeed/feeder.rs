//! The map side of the feed, free of threads and clocks: each input is
//! inserted as it arrives, and [`Feeder::frame`] decays the map, updates the
//! distance field and publishes it with the tier that fed it last.

use ados_livemap::BAND_TOP_M;
use ados_mapping_pose::lidar_scan::LidarScan;
use ados_protocol::voxmap::{VoxmapError, VoxmapWriter};
use ados_voxmap::{ConfigError, Pose, VoxMap, VoxMapConfig, VoxTier};

use super::convert::{blocked, body_from_sensor, flu_to_frd};

/// A 3D LiDAR cloud is thinned to about this many returns per insertion:
/// the map's frame budget is measured at 20 000 returns of a 10 Hz scan.
pub const CLOUD_POINTS_PER_SCAN: usize = 20_000;
/// A 3D LiDAR return farther than this is not inserted, metres.
pub const LIDAR3D_MAX_RANGE_M: f32 = 40.0;
/// A planar LiDAR return farther than this is not inserted, metres.
pub const PLANAR_MAX_RANGE_M: f32 = 30.0;
/// LiDAR returns nearer the sensor than this are the airframe (props, legs,
/// the mount), never obstacles, metres.
pub const SELF_RANGE_M: f32 = 0.4;
/// The tier is what fed the map within this long, ADOS ns.
pub const TIER_WINDOW_NS: u64 = 1_000_000_000;

/// One input of the map.
pub enum Input {
    /// A posed LiDAR scan. `sensor` is the sensor's pose in the map frame
    /// (north, east, down) with its forward-right-down axes; `scan` holds the
    /// sensor's own forward-left-up points.
    Lidar {
        scan: LidarScan,
        sensor: Pose,
        /// The sensor's mount on the body, forward-right-down axes.
        mount: Pose,
        /// The rig's blocked sectors, sensor azimuth degrees.
        blocked: Vec<[f64; 2]>,
        planar: bool,
    },
    /// A depth frame turned into body-frame points.
    Depth {
        body: Pose,
        /// The camera's position on the body (forward, right, down).
        origin_body: [f32; 3],
        points: Vec<[f32; 3]>,
    },
}

/// Which tiers fed the map, by the ADOS time each last did.
#[derive(Debug, Clone, Copy, Default)]
struct Seen {
    lidar3d: Option<u64>,
    depth: Option<u64>,
    planar: Option<u64>,
}

pub struct Feeder {
    map: VoxMap,
    depth_max_range_m: f32,
    /// The newest body pose, for the published header.
    body: Option<Pose>,
    seen: Seen,
    cloud: Vec<[f32; 3]>,
    ranges: Vec<f32>,
    angles: Vec<f32>,
}

impl Feeder {
    pub fn new(cfg: VoxMapConfig, depth_max_range_m: f32) -> Result<Self, ConfigError> {
        Ok(Self {
            map: VoxMap::new(cfg)?,
            depth_max_range_m,
            body: None,
            seen: Seen::default(),
            cloud: Vec::with_capacity(CLOUD_POINTS_PER_SCAN),
            ranges: Vec::new(),
            angles: Vec::new(),
        })
    }

    pub fn map(&self) -> &VoxMap {
        &self.map
    }

    /// Insert one input at ADOS time `now_ns`. Returns a depth input's point
    /// buffer for reuse.
    pub fn ingest(&mut self, now_ns: u64, input: Input) -> Option<Vec<[f32; 3]>> {
        match input {
            Input::Lidar {
                scan,
                sensor,
                mount,
                blocked,
                planar,
            } => {
                let body = body_from_sensor(&sensor, &mount);
                self.map.recentre(body.t);
                let ok = if planar {
                    self.insert_planar(now_ns, &scan, &sensor, &blocked)
                } else {
                    self.insert_3d(now_ns, &scan, &sensor, &blocked)
                };
                if ok {
                    self.body = Some(body);
                    let at = Some(now_ns);
                    if planar {
                        self.seen.planar = at;
                    } else {
                        self.seen.lidar3d = at;
                    }
                }
                None
            }
            Input::Depth {
                body,
                origin_body,
                mut points,
            } => {
                for p in points.iter_mut() {
                    let w = body.rotate(*p);
                    *p = [w[0] + body.t[0], w[1] + body.t[1], w[2] + body.t[2]];
                }
                let o = body.rotate(origin_body);
                let origin = [o[0] + body.t[0], o[1] + body.t[1], o[2] + body.t[2]];
                self.map.recentre(body.t);
                if self
                    .map
                    .insert_cloud(now_ns, origin, &points, self.depth_max_range_m)
                {
                    self.body = Some(body);
                    self.seen.depth = Some(now_ns);
                }
                Some(points)
            }
        }
    }

    fn insert_3d(
        &mut self,
        now_ns: u64,
        scan: &LidarScan,
        sensor: &Pose,
        sectors: &[[f64; 2]],
    ) -> bool {
        let stride = scan.points.len().div_ceil(CLOUD_POINTS_PER_SCAN).max(1);
        self.cloud.clear();
        for p in scan.points.iter().step_by(stride) {
            let r2 = p[0] * p[0] + p[1] * p[1] + p[2] * p[2];
            if !r2.is_finite() || r2 < SELF_RANGE_M * SELF_RANGE_M || blocked(p, sectors) {
                continue;
            }
            let v = sensor.rotate(flu_to_frd(*p));
            self.cloud
                .push([v[0] + sensor.t[0], v[1] + sensor.t[1], v[2] + sensor.t[2]]);
        }
        self.map
            .insert_cloud(now_ns, sensor.t, &self.cloud, LIDAR3D_MAX_RANGE_M)
    }

    /// The planar scan placed at its true 3D position from the sensor's
    /// attitude. Only returns within the flight band are trusted: one more
    /// than [`BAND_TOP_M`] over the sensor met something over the drone (a
    /// crown's underside a leaning scan reached), which a single slice
    /// cannot place, so it neither marks its voxel nor clears its beam.
    fn insert_planar(
        &mut self,
        now_ns: u64,
        scan: &LidarScan,
        sensor: &Pose,
        sectors: &[[f64; 2]],
    ) -> bool {
        self.ranges.clear();
        self.angles.clear();
        for p in &scan.points {
            let range = p[0].hypot(p[1]);
            if !range.is_finite() || range < SELF_RANGE_M || blocked(p, sectors) {
                continue;
            }
            self.ranges.push(range);
            // Sensor forward-right: clockwise from forward.
            self.angles.push((-p[1]).atan2(p[0]));
        }
        let up = sensor.t[2];
        self.map.insert_planar_scan(
            now_ns,
            sensor,
            &self.ranges,
            &self.angles,
            PLANAR_MAX_RANGE_M,
            |r| up - r.point[2] <= BAND_TOP_M,
        )
    }

    /// The sensing tier behind the map at `now_ns`: the best one that fed it
    /// within [`TIER_WINDOW_NS`].
    pub fn tier(&self, now_ns: u64) -> VoxTier {
        let recent = |t: Option<u64>| t.is_some_and(|t| now_ns.saturating_sub(t) <= TIER_WINDOW_NS);
        if recent(self.seen.lidar3d) {
            VoxTier::Lidar3d
        } else if recent(self.seen.depth) {
            VoxTier::Depth
        } else if recent(self.seen.planar) {
            VoxTier::Planar
        } else {
            VoxTier::Unknown
        }
    }

    /// One frame at ADOS time `now_ns`: decay, the distance field, and the
    /// publication through `writer` once a pose has placed the map. Returns
    /// whether it published.
    pub fn frame(
        &mut self,
        now_ns: u64,
        writer: Option<&mut VoxmapWriter>,
    ) -> Result<bool, VoxmapError> {
        self.map.decay(now_ns);
        self.map.tick_esdf();
        match (writer, self.body) {
            (Some(w), Some(body)) => {
                self.map.to_seam(w, &body, self.tier(now_ns))?;
                Ok(true)
            }
            _ => Ok(false),
        }
    }
}
