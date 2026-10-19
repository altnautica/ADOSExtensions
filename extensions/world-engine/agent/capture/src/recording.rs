//! One open capture session: admission, the manifest it starts with, the
//! operator's site/floor/rooms (MARK), and closing it.

use std::path::Path;
use std::time::SystemTime;

use ados_capture_contract::{
    session_id, ContractId, GeoAnchor, GeoAnchorDatum, GeoAnchorSource, Manifest, ManifestProfile,
    Operator, Rig, RigRef, Room, SessionHealth, Timebase,
};
use world_engine_protocol::capture_ctl::{codes, CtlReply, OperatorAnchor, StartArgs};

use crate::recorder::convert::ImageEncoding;
use crate::recorder::{Recorder, RecorderConfig};
use crate::storage;

/// Recording or paused (a closed session is no session).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Phase {
    Recording,
    Paused,
}

/// Everything a new session is started from.
pub struct StartEnv<'a> {
    pub device_id: &'a str,
    pub profile: &'a str,
    pub rig: Rig,
    pub timebase: Timebase,
    pub sigma_ns: i64,
    /// Session-timebase ns now.
    pub t0_ns: u64,
    pub utc_at_t0_ns: Option<i64>,
    /// The GNSS-mean anchor, when one has latched.
    pub gnss_anchor: Option<GeoAnchor>,
    pub sidecar: &'a Path,
    pub limits: ados_capture_contract::SegmentLimits,
    pub encoding: ImageEncoding,
    pub active_marker: Option<std::path::PathBuf>,
}

pub struct ActiveSession {
    pub recorder: Recorder,
    pub phase: Phase,
    pub session_id: String,
    /// Host monotonic ns the session started.
    pub started_mono_ns: u64,
    pub t0_ns: u64,
    pub timebase: Timebase,
    pub anchor: Option<GeoAnchor>,
    pub site: Option<String>,
    pub floor: i64,
    rooms: Vec<Room>,
    /// The open room's name and start.
    current: Option<(String, i64, u64)>,
    pub operator_floor: Option<i64>,
}

fn refuse(code: &str, detail: impl Into<String>) -> CtlReply {
    CtlReply::err(code, detail)
}

/// The anchor an operator typed (address-level position, no GNSS indoors).
pub fn operator_anchor(a: &OperatorAnchor) -> Result<GeoAnchor, CtlReply> {
    let ok = a.lat_deg.is_finite()
        && a.lon_deg.is_finite()
        && a.h_ellipsoid_m.is_finite()
        && (-90.0..=90.0).contains(&a.lat_deg)
        && (-180.0..=180.0).contains(&a.lon_deg);
    if !ok {
        return Err(refuse(codes::BAD_REQUEST, "operator_anchor out of range"));
    }
    Ok(GeoAnchor {
        lat_deg: a.lat_deg,
        lon_deg: a.lon_deg,
        h_ellipsoid_m: a.h_ellipsoid_m,
        geoid_undulation_m: crate::geo::geoid_undulation_m(a.lat_deg, a.lon_deg),
        datum: GeoAnchorDatum::Wgs84,
        source: GeoAnchorSource::Operator,
    })
}

/// A device id reduced to the session-id-safe characters.
pub fn device_segment(id: &str) -> String {
    let s: String = id
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' || c == '_' {
                c
            } else {
                '-'
            }
        })
        .collect();
    let s = s.trim_matches('-').to_string();
    if s.is_empty() {
        "device".into()
    } else {
        s
    }
}

fn nonce() -> u16 {
    let n = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .map(|d| d.subsec_nanos())
        .unwrap_or(0);
    (n ^ (n >> 16) ^ std::process::id()) as u16
}

impl ActiveSession {
    /// Admit and open a session. Refusals carry the control-socket code.
    pub fn start(args: StartArgs, env: StartEnv<'_>, now_mono_ns: u64) -> Result<Self, CtlReply> {
        let site = args.site.filter(|s| !s.trim().is_empty());
        let room = args.room.filter(|s| !s.trim().is_empty());
        let anchor = match &args.operator_anchor {
            Some(a) => Some(operator_anchor(a)?),
            None => env.gnss_anchor.clone(),
        };
        let sidecar = storage::admit(env.sidecar)
            .map_err(|r| refuse(codes::STORAGE_FULL, format!("{r:?}")))?;
        let root = storage::local_root_for(Path::new(&sidecar.capture_root)).ok_or_else(|| {
            refuse(
                codes::STORAGE_FULL,
                "the capture root moved since this service started; it must be restarted",
            )
        })?;
        if env.rig.sensors.is_empty() {
            return Err(refuse(
                codes::NO_RIG,
                "no rig document and no camera seen yet",
            ));
        }
        let unix_s = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let id = session_id(&device_segment(env.device_id), unix_s as i64, nonce());
        let floor = args.floor.unwrap_or(0);
        let manifest = Manifest {
            contract: ContractId::AdosCapture1,
            session_id: id.clone(),
            device_id: env.device_id.to_string(),
            profile: if env.profile == "visor" {
                ManifestProfile::Visor
            } else {
                ManifestProfile::Drone
            },
            timebase: env.timebase,
            t0_ns: env.t0_ns,
            utc_at_t0_ns: env.utc_at_t0_ns,
            sigma_ns: env.sigma_ns.max(-1),
            anchor: anchor.clone(),
            rig: RigRef {
                rig_id: env.rig.rig_id.clone(),
                calib_version: env.rig.calib_version.clone(),
            },
            operator: Operator {
                site: site.clone(),
                floor: args.floor.map(|f| f as i32),
                rooms: vec![],
            },
            segments: vec![],
            health: SessionHealth {
                blur: false,
                texture: false,
                flicker: false,
                corridor: false,
                frames_dropped_pct: 0.0,
            },
        };
        let recorder = Recorder::start(RecorderConfig {
            root,
            manifest,
            rig: env.rig,
            limits: env.limits,
            encoding: env.encoding,
            active_marker: env.active_marker,
        })
        .map_err(|e| refuse(codes::RECORDER, e.to_string()))?;
        Ok(Self {
            recorder,
            phase: Phase::Recording,
            session_id: id,
            started_mono_ns: now_mono_ns,
            t0_ns: env.t0_ns,
            timebase: env.timebase,
            anchor,
            site,
            floor,
            rooms: Vec::new(),
            current: room.map(|r| (r, floor, env.t0_ns)),
            operator_floor: args.floor,
        })
    }

    /// MARK: close the open room at `t_ns` and open `name` on `floor`.
    /// Returns the new segment number (1-based count of rooms so far).
    pub fn mark(&mut self, name: String, floor: i64, t_ns: u64) -> u32 {
        self.close_room(t_ns);
        self.floor = floor;
        self.current = Some((name, floor, t_ns));
        (self.rooms.len() + 1) as u32
    }

    fn close_room(&mut self, t_ns: u64) {
        if let Some((name, floor, start)) = self.current.take() {
            self.rooms.push(Room {
                name,
                floor: floor as i32,
                t_start_ns: start,
                t_end_ns: t_ns.max(start),
            });
        }
    }

    pub fn room(&self) -> Option<&str> {
        self.current.as_ref().map(|(n, _, _)| n.as_str())
    }

    /// The current segment number: rooms closed plus the open one.
    pub fn segment(&self) -> u32 {
        (self.rooms.len() + usize::from(self.current.is_some())) as u32
    }

    /// Close: the open room ends at `t_end_ns`, the manifest is written.
    pub fn finish(mut self, t_end_ns: u64, health: SessionHealth) -> anyhow::Result<Manifest> {
        self.close_room(t_end_ns);
        let operator = Operator {
            site: self.site.clone(),
            floor: self.operator_floor.map(|f| f as i32),
            rooms: std::mem::take(&mut self.rooms),
        };
        self.recorder.finish(operator, health)
    }
}

#[cfg(test)]
mod tests;
