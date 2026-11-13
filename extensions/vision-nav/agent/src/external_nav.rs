//! External navigation: a shared pose topic to `ODOMETRY` (#331).
//!
//! The pose descriptor carries `R` (row-major 3x3) and `t` of the body
//! FRD frame in the map frame, the tracker `health`, its `frame` and, for
//! `map_ned`, its `north` (where the map's north comes from). `map_ned`
//! (north-aligned, so its axes are those of the FC's `LOCAL_POSITION_NED`)
//! is used as is; `map_flu` (the planar SLAM world: body FLU at the first
//! scan) is converted with `x = x, y = -y, z = -z, yaw = -yaw`. Sent as
//! `MAV_FRAME_LOCAL_FRD`: `VISO_TYPE` 1 (MAVLink) aligns only the
//! external-navigation position to the EKF's, not its yaw, so the pose frame
//! must already be north-aligned, which `map_ned` is. The EKF may take its
//! yaw from the pose (`EK3_SRCn_YAW` 6) only when that north does not come
//! from the flight controller's own heading ([`MapNorth`]).
//!
//! `ODOMETRY` goes out on component 197 with `frame_id = MAV_FRAME_LOCAL_FRD`
//! and `child_frame_id = MAV_FRAME_BODY_FRD`, the only pair ArduPilot
//! accepts (`GCS_MAVLINK::handle_odometry`). x, y and yaw come from the
//! pose; z and vz from the flight controller's own `LOCAL_POSITION_NED`
//! (the planar pose has no height); roll and pitch from its `ATTITUDE`.
//! Horizontal velocity is the pose difference over the last 100 ms,
//! rotated into the body frame, because ArduPilot rotates
//! `vx, vy, vz` by `q` into NED.
//!
//! `time_usec` is the instant the pose describes (its `ts_ms`, a LiDAR
//! pose's first return) on the flight controller's clock, and the velocity
//! is differenced over those instants, so the stamps no longer carry the
//! varying delay of the scan, the matcher and the topic. ArduPilot's jitter
//! correction maps the stamps onto its clock by the least delay seen;
//! `VISO_DELAY_MS` must cover that least delay (for a 10 Hz LiDAR, at least
//! the 100 ms revolution) for the EKF to fuse each pose at its own time.
//!
//! Covariance: ArduPilot reduces the pose covariance to one error,
//! `sqrt(c[0] + c[6] + c[11])` for position and `sqrt(c[15] + c[18] +
//! c[20])` for angle. z echoes the flight controller's own estimate (its
//! height source stays the barometer or rangefinder), so it carries the
//! horizontal variance: a large z variance would swamp that sum and make
//! the horizontal fix look metres wide.

use std::collections::VecDeque;

use rmpv::Value;

use crate::fc_nav::FcNavSnapshot;
use crate::mavlink_emit::encode_v2;
use crate::nav_config::as_f64;

/// `MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`.
pub const COMPONENT_EXTNAV: u8 = 197;
pub const ODOMETRY_ID: u32 = 331;
pub const ODOMETRY_CRC_EXTRA: u8 = 91;
/// Base fields (230 bytes) plus `reset_counter`, `estimator_type`, `quality`.
pub const ODOMETRY_LEN: usize = 233;
pub const MAV_FRAME_LOCAL_FRD: u8 = 20;
pub const MAV_FRAME_BODY_FRD: u8 = 12;
pub const MAV_ESTIMATOR_TYPE_LIDAR: u8 = 7;

/// A pose this far from where the last velocity predicts it (or, with no
/// velocity to predict from after a gap, from the last pose) bumps
/// `reset_counter`: a relocalisation, not motion.
pub const RESET_JUMP_M: f64 = 0.5;
/// The longest gap over which the last velocity predicts the next pose.
pub const RESET_PREDICT_NS: i64 = 500_000_000;
/// Velocity is the pose difference over this window.
pub const VELOCITY_WINDOW_NS: i64 = 100_000_000;
/// Velocity needs at least this much time between the two samples.
const MIN_VELOCITY_DT_NS: i64 = 20_000_000;
/// A pose stamped older than this is taken as this old.
pub const MAX_POSE_AGE_NS: i64 = 500_000_000;

/// Tracker health as the pose topic reports it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ExtnavHealth {
    Good,
    Degraded,
    Lost,
}

impl ExtnavHealth {
    pub fn as_str(self) -> &'static str {
        match self {
            ExtnavHealth::Good => "good",
            ExtnavHealth::Degraded => "degraded",
            ExtnavHealth::Lost => "lost",
        }
    }

    fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "good" => ExtnavHealth::Good,
            "degraded" => ExtnavHealth::Degraded,
            "lost" => ExtnavHealth::Lost,
            _ => return None,
        })
    }

    /// `(σ_xy m, σ_yaw rad, ODOMETRY quality %)`.
    fn noise(self) -> (f32, f32, i8) {
        match self {
            ExtnavHealth::Good => (0.05, 0.02, 100),
            ExtnavHealth::Degraded => (0.30, 0.10, 50),
            ExtnavHealth::Lost => (f32::NAN, f32::NAN, -1),
        }
    }
}

/// Where a `map_ned` pose's north comes from, as the pose topic reports it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapNorth {
    /// The flight controller's heading until take-off: it echoes the FC's own
    /// yaw, so it cannot be the EKF's yaw source.
    FcHeading,
    /// Latched once from a declared take-off heading.
    Declared,
    /// Fitted against the EKF's GNSS-referenced track.
    Gnss,
}

impl MapNorth {
    fn parse(s: &str) -> Option<Self> {
        Some(match s {
            "fc_heading" => MapNorth::FcHeading,
            "declared" => MapNorth::Declared,
            "gnss" => MapNorth::Gnss,
            _ => return None,
        })
    }

    /// Whether the EKF may take its yaw from a map with this north.
    pub fn holds_yaw(north: Option<Self>) -> bool {
        matches!(north, Some(MapNorth::Declared | MapNorth::Gnss))
    }
}

/// One pose in map FRD.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PoseSample {
    pub ts_ms: i64,
    pub r: [f64; 9],
    pub t: [f64; 3],
    pub health: ExtnavHealth,
    /// `None` when the publisher does not say (an older one, or `map_flu`).
    pub north: Option<MapNorth>,
}

impl PoseSample {
    /// Heading of body +x in the map, radians (FRD: clockwise from +x).
    pub fn yaw(&self) -> f64 {
        self.r[3].atan2(self.r[0])
    }
}

/// Monotonic time of the instant a pose describes, from its unix stamp
/// `ts_ms`: the age the stamp gives at `now_unix_ms` (read with `now_ns`)
/// back from `now_ns`. A LiDAR pose describes its scan's first return, a
/// revolution or more before it arrives, and `ODOMETRY` must carry that
/// time for the flight controller to fuse it where it belongs. A pose is
/// never later than its receipt `recv_ns` nor older than [`MAX_POSE_AGE_NS`]
/// before it; an unstamped pose (`ts_ms` 0) is taken at its receipt.
pub fn measured_at_ns(ts_ms: i64, now_unix_ms: i64, now_ns: i64, recv_ns: i64) -> i64 {
    if ts_ms <= 0 {
        return recv_ns;
    }
    let t = now_ns - (now_unix_ms - ts_ms).saturating_mul(1_000_000);
    t.clamp(recv_ns - MAX_POSE_AGE_NS, recv_ns)
}

/// Why a pose payload was not used.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PoseError {
    Malformed(&'static str),
    UnknownFrame(String),
    UnknownHealth(String),
}

impl PoseError {
    /// Stable key for health `degradedReasons`.
    pub fn reason(&self) -> String {
        match self {
            PoseError::Malformed(what) => format!("pose_malformed:{what}"),
            PoseError::UnknownFrame(f) => format!("pose_frame_unsupported:{f}"),
            PoseError::UnknownHealth(h) => format!("pose_health_unknown:{h}"),
        }
    }
}

fn get<'a>(v: &'a Value, key: &str) -> Option<&'a Value> {
    match v {
        Value::Map(e) => e
            .iter()
            .find(|(k, _)| k.as_str() == Some(key))
            .map(|(_, val)| val),
        _ => None,
    }
}

fn floats<const N: usize>(v: Option<&Value>, what: &'static str) -> Result<[f64; N], PoseError> {
    let Some(Value::Array(a)) = v else {
        return Err(PoseError::Malformed(what));
    };
    if a.len() != N {
        return Err(PoseError::Malformed(what));
    }
    let mut out = [0.0; N];
    for (slot, item) in out.iter_mut().zip(a) {
        *slot = as_f64(item)
            .filter(|x| x.is_finite())
            .ok_or(PoseError::Malformed(what))?;
    }
    Ok(out)
}

/// Parse an `event.subscribe` delivery (`{topic, payload, ..}`).
pub fn parse_pose_event(args: &Value) -> Result<PoseSample, PoseError> {
    let payload = get(args, "payload").ok_or(PoseError::Malformed("payload"))?;
    parse_pose_payload(payload)
}

/// Parse a pose descriptor and bring it into map FRD.
pub fn parse_pose_payload(p: &Value) -> Result<PoseSample, PoseError> {
    let pose = get(p, "pose").ok_or(PoseError::Malformed("pose"))?;
    let r = floats::<9>(get(pose, "R"), "R")?;
    let t = floats::<3>(get(pose, "t"), "t")?;
    let health = match get(p, "health").and_then(Value::as_str) {
        Some(h) => ExtnavHealth::parse(h).ok_or_else(|| PoseError::UnknownHealth(h.into()))?,
        None => return Err(PoseError::Malformed("health")),
    };
    let ts_ms = get(p, "ts_ms").and_then(Value::as_i64).unwrap_or(0);
    let (r, t, north) = match get(p, "frame").and_then(Value::as_str) {
        Some("map_ned") => (
            r,
            t,
            get(p, "north").and_then(Value::as_str).and_then(MapNorth::parse),
        ),
        Some("map_flu") => {
            let (r, t) = flu_to_frd(&r, &t);
            (r, t, None)
        }
        Some(other) => return Err(PoseError::UnknownFrame(other.into())),
        None => return Err(PoseError::UnknownFrame("missing".into())),
    };
    Ok(PoseSample {
        ts_ms,
        r,
        t,
        health,
        north,
    })
}

/// FLU world / FLU body to FRD world / FRD body: `S·R·S`, `S·t` with
/// `S = diag(1, -1, -1)`.
pub fn flu_to_frd(r: &[f64; 9], t: &[f64; 3]) -> ([f64; 9], [f64; 3]) {
    let s = [1.0, -1.0, -1.0];
    let mut out = [0.0; 9];
    for i in 0..3 {
        for j in 0..3 {
            out[i * 3 + j] = s[i] * r[i * 3 + j] * s[j];
        }
    }
    (out, [t[0], -t[1], -t[2]])
}

/// Quaternion `[w, x, y, z]` of ZYX Euler angles (body to NED).
pub fn quat_from_euler(roll: f64, pitch: f64, yaw: f64) -> [f64; 4] {
    let (sr, cr) = (roll / 2.0).sin_cos();
    let (sp, cp) = (pitch / 2.0).sin_cos();
    let (sy, cy) = (yaw / 2.0).sin_cos();
    [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]
}

/// Rotate a world (NED) vector into the body frame: `Cᵀ·v` with
/// `C = Rz(yaw)·Ry(pitch)·Rx(roll)`.
pub fn world_to_body(v: [f64; 3], roll: f64, pitch: f64, yaw: f64) -> [f64; 3] {
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    // Rows of C (body -> world).
    let c = [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ];
    let mut out = [0.0; 3];
    for (j, o) in out.iter_mut().enumerate() {
        *o = (0..3).map(|i| c[i][j] * v[i]).sum();
    }
    out
}

/// The `ODOMETRY` fields, extensions included.
#[derive(Debug, Clone, PartialEq)]
pub struct Odometry {
    pub time_usec: u64,
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub q: [f32; 4],
    pub vx: f32,
    pub vy: f32,
    pub vz: f32,
    pub rollspeed: f32,
    pub pitchspeed: f32,
    pub yawspeed: f32,
    pub pose_covariance: [f32; 21],
    pub velocity_covariance: [f32; 21],
    pub reset_counter: u8,
    pub quality: i8,
}

impl Odometry {
    /// MAVLink 2 frame on component 197.
    pub fn encode(&self, sequence: u8) -> Vec<u8> {
        let mut p = Vec::with_capacity(ODOMETRY_LEN);
        p.extend_from_slice(&self.time_usec.to_le_bytes());
        let scalars = [self.x, self.y, self.z];
        let vel = [
            self.vx,
            self.vy,
            self.vz,
            self.rollspeed,
            self.pitchspeed,
            self.yawspeed,
        ];
        for f in scalars
            .iter()
            .chain(&self.q)
            .chain(&vel)
            .chain(&self.pose_covariance)
            .chain(&self.velocity_covariance)
        {
            p.extend_from_slice(&f.to_le_bytes());
        }
        p.push(MAV_FRAME_LOCAL_FRD);
        p.push(MAV_FRAME_BODY_FRD);
        p.push(self.reset_counter);
        p.push(MAV_ESTIMATOR_TYPE_LIDAR);
        p.push(self.quality as u8);
        encode_v2(
            COMPONENT_EXTNAV,
            sequence,
            ODOMETRY_ID,
            ODOMETRY_CRC_EXTRA,
            p,
        )
    }
}

/// Upper-triangle 6x6 covariance with the given diagonal.
fn diag_cov(d: [f32; 6]) -> [f32; 21] {
    // Row-major upper triangle: diagonal entries sit at 0, 6, 11, 15, 18, 20.
    const DIAG: [usize; 6] = [0, 6, 11, 15, 18, 20];
    let mut c = [0.0; 21];
    for (i, v) in DIAG.iter().zip(d) {
        c[*i] = v;
    }
    c
}

/// One pose-derived state, the inputs of one `ODOMETRY`.
#[derive(Debug, Clone, Copy)]
struct Fix {
    x: f64,
    y: f64,
    yaw: f64,
    vn: f64,
    ve: f64,
    vel_var: f32,
    health: ExtnavHealth,
}

/// Turns pose samples into `ODOMETRY`, tracking the velocity window and
/// the reset counter. Exactly one message per measured pose: the planar
/// tracker publishes about once per scan (~10 Hz), and nothing is
/// interpolated or extrapolated between poses, so every message the EKF
/// fuses is a real measurement with its own covariance.
#[derive(Default)]
pub struct OdometryBuilder {
    history: VecDeque<(i64, f64, f64)>,
    last_xy: Option<(f64, f64)>,
    last_t_ns: Option<i64>,
    /// `(vn, ve)` of the last built fix; cleared on a reset and on lost.
    last_v: Option<(f64, f64)>,
    /// The last pose started a segment (the first pose, or a reset): the
    /// next one has no velocity to be predicted from and sets it.
    segment_start: bool,
    reset_counter: u8,
}

impl OdometryBuilder {
    pub fn reset_counter(&self) -> u8 {
        self.reset_counter
    }

    /// Build the message for a pose measured at `t_ns` (monotonic, see
    /// [`measured_at_ns`]; `time_usec` is the same instant on the flight
    /// controller's clock), its velocity differenced over measurement
    /// times. `None` while the tracker is lost, and for the first sample
    /// after a gap or a reset (no velocity yet).
    pub fn build(
        &mut self,
        time_usec: u64,
        t_ns: i64,
        pose: &PoseSample,
        fc: &FcNavSnapshot,
    ) -> Option<Odometry> {
        if pose.health == ExtnavHealth::Lost {
            self.history.clear();
            self.last_v = None;
            // After a loss the next pose is compared with the last one held.
            self.segment_start = false;
            return None;
        }
        let (x, y) = (pose.t[0], pose.t[1]);
        if self.is_jump(x, y, t_ns) {
            self.reset_counter = self.reset_counter.wrapping_add(1);
            self.history.clear();
            self.last_v = None;
            self.segment_start = true;
        } else {
            self.segment_start = self.last_xy.is_none();
        }
        self.last_xy = Some((x, y));
        self.last_t_ns = Some(t_ns);
        let cutoff = t_ns - VELOCITY_WINDOW_NS;
        while self.history.len() > 1 && self.history[1].0 <= cutoff {
            self.history.pop_front();
        }
        let oldest = self.history.front().copied();
        self.history.push_back((t_ns, x, y));
        let (t0, x0, y0) = oldest?;
        let dt_ns = t_ns - t0;
        if dt_ns < MIN_VELOCITY_DT_NS {
            return None;
        }
        let dt = dt_ns as f64 / 1e9;
        let (sxy, _, _) = pose.health.noise();
        let fix = Fix {
            x,
            y,
            yaw: pose.yaw(),
            vn: (x - x0) / dt,
            ve: (y - y0) / dt,
            // Difference of two positions, each with variance σ², over dt.
            vel_var: (2.0 * f64::from(sxy * sxy) / (dt * dt)) as f32,
            health: pose.health,
        };
        self.last_v = Some((fix.vn, fix.ve));
        Some(self.compose(time_usec, &fix, fc))
    }

    /// Whether the pose at `(x, y, t_ns)` is a relocalisation. Within
    /// [`RESET_PREDICT_NS`] of the last pose it is measured from where the
    /// last velocity predicts, so steady flight at any speed is motion; the
    /// pose right after a segment start has no velocity yet and is taken as
    /// motion; after a longer gap or a loss it is measured from the last
    /// pose.
    fn is_jump(&self, x: f64, y: f64, t_ns: i64) -> bool {
        let (Some((lx, ly)), Some(lt)) = (self.last_xy, self.last_t_ns) else {
            return false;
        };
        let dt_ns = t_ns - lt;
        let within = dt_ns > 0 && dt_ns <= RESET_PREDICT_NS;
        let (px, py) = match self.last_v {
            Some((vn, ve)) if within => {
                let dt = dt_ns as f64 / 1e9;
                (lx + vn * dt, ly + ve * dt)
            }
            None if within && self.segment_start => return false,
            _ => (lx, ly),
        };
        (x - px).hypot(y - py) > RESET_JUMP_M
    }

    fn compose(&self, time_usec: u64, fix: &Fix, fc: &FcNavSnapshot) -> Odometry {
        let (x, y) = (fix.x, fix.y);
        let (roll, pitch, rates) = match fc.attitude {
            Some(a) => (
                f64::from(a.roll),
                f64::from(a.pitch),
                [a.rollspeed, a.pitchspeed, a.yawspeed],
            ),
            None => (0.0, 0.0, [0.0; 3]),
        };
        let (z, vd) = fc
            .local
            .map_or((0.0, 0.0), |l| (f64::from(l.z), f64::from(l.vz)));
        let body_v = world_to_body([fix.vn, fix.ve, vd], roll, pitch, fix.yaw);
        let q = quat_from_euler(roll, pitch, fix.yaw);
        let (sxy, syaw, quality) = fix.health.noise();
        let (vxy, vyaw) = (sxy * sxy, syaw * syaw);
        let vel_var = fix.vel_var;
        Odometry {
            time_usec,
            x: x as f32,
            y: y as f32,
            z: z as f32,
            q: q.map(|c| c as f32),
            vx: body_v[0] as f32,
            vy: body_v[1] as f32,
            vz: body_v[2] as f32,
            rollspeed: rates[0],
            pitchspeed: rates[1],
            yawspeed: rates[2],
            pose_covariance: diag_cov([vxy, vxy, vxy, vyaw, vyaw, vyaw]),
            velocity_covariance: diag_cov([vel_var, vel_var, vel_var, vyaw, vyaw, vyaw]),
            reset_counter: self.reset_counter,
            quality,
        }
    }
}

#[cfg(test)]
#[path = "external_nav_tests.rs"]
mod tests;
