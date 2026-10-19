//! Sensor-bus ingest (seam J, `/run/ados/sensors.sock`).
//!
//! One task subscribes to the `clock`, `imu`, `gnss` and `trigger` kinds and
//! keeps a [`SensorState`]: a 10 s IMU ring per stream, recent trigger
//! records per camera, the clock map and the latest GNSS bytes. Every frame is
//! also forwarded as a [`BusEvent`] to the capture loop, which records it.
//!
//! The socket is bound into the plugin's sandbox only with the
//! `sensor.stream.read` grant; without it (or without the clock service) the
//! connect fails, the state reports `connected: false`, and capture runs on
//! frames alone. Every `t_ns` here is host monotonic, as on the bus.

use std::collections::{HashMap, VecDeque};
use std::path::PathBuf;
use std::sync::Arc;
use std::time::Duration;

use ados_capture_contract::v1::{ClockSample, GnssRaw, ImuBatch, ImuSample, TriggerRecord};
use ados_protocol::shutdown::Shutdown;
use ados_sensorbus::{BusFrame, Kind, Subscriber};
use parking_lot::Mutex;
use prost::Message;
use tokio::sync::mpsc;

use crate::clock::ClockMap;

/// IMU history kept per stream.
pub const IMU_RING_NS: u64 = 10_000_000_000;
/// Trigger history kept per camera.
pub const TRIGGER_RING_NS: u64 = 2_000_000_000;
/// A frame without a trigger sequence joins the trigger within this of its
/// stamp.
pub const TRIGGER_JOIN_TOL_NS: u64 = 5_000_000;
const RECONNECT: Duration = Duration::from_secs(1);

/// One decoded bus frame for the capture loop.
#[derive(Debug, Clone, PartialEq)]
pub enum BusEvent {
    Imu { src: String, batch: ImuBatch },
    Gnss { src: String, raw: GnssRaw },
    Trigger { camera: String, rec: TriggerRecord },
    Clock(ClockSample),
}

/// What the bus has delivered recently.
#[derive(Debug, Default)]
pub struct SensorState {
    pub connected: bool,
    imu: HashMap<String, VecDeque<ImuSample>>,
    triggers: HashMap<String, VecDeque<TriggerRecord>>,
    pub clock: ClockMap,
    pub frames: u64,
    pub decode_errors: u64,
    /// Events folded into the state but not forwarded to the capture loop
    /// because its queue was full (the loop did not record them).
    pub dropped_forward: u64,
}

impl SensorState {
    /// Fold one decoded event in.
    pub fn apply(&mut self, ev: &BusEvent) {
        self.frames += 1;
        match ev {
            BusEvent::Imu { src, batch } => {
                let ring = self.imu.entry(src.clone()).or_default();
                for s in &batch.samples {
                    if ring.back().is_none_or(|b| s.t_ns > b.t_ns) {
                        ring.push_back(s.clone());
                    }
                }
                if let Some(last) = ring.back().map(|s| s.t_ns) {
                    let horizon = last.saturating_sub(IMU_RING_NS);
                    while ring.front().is_some_and(|s| s.t_ns < horizon) {
                        ring.pop_front();
                    }
                }
            }
            BusEvent::Trigger { camera, rec } => {
                let ring = self.triggers.entry(camera.clone()).or_default();
                ring.push_back(*rec);
                let horizon = rec.t_mid_ns.saturating_sub(TRIGGER_RING_NS);
                while ring.front().is_some_and(|r| r.t_mid_ns < horizon) {
                    ring.pop_front();
                }
            }
            BusEvent::Clock(s) => {
                self.clock.observe(*s);
            }
            BusEvent::Gnss { .. } => {}
        }
    }

    /// IMU samples of every stream with `from_ns < t_ns <= to_ns`, time
    /// ordered (the keyframe IMU window).
    pub fn imu_window(&self, from_ns: u64, to_ns: u64) -> Vec<ImuSample> {
        let mut out: Vec<ImuSample> = self
            .imu
            .values()
            .flat_map(|r| {
                r.iter()
                    .filter(|s| s.t_ns > from_ns && s.t_ns <= to_ns)
                    .cloned()
            })
            .collect();
        out.sort_by_key(|s| s.t_ns);
        out
    }

    /// The IMU stream ids seen, sorted.
    pub fn imu_sources(&self) -> Vec<String> {
        let mut ids: Vec<String> = self.imu.keys().cloned().collect();
        ids.sort();
        ids
    }

    /// Samples held for one stream (tests and the HUD rate readout).
    pub fn imu_len(&self, src: &str) -> usize {
        self.imu.get(src).map_or(0, VecDeque::len)
    }

    /// The trigger that exposed a frame of `camera`: by sequence when the
    /// frame carries one, else the nearest within [`TRIGGER_JOIN_TOL_NS`].
    pub fn join_trigger(&self, camera: &str, seq: Option<u64>, t_ns: u64) -> Option<TriggerRecord> {
        let ring = self.triggers.get(camera)?;
        match seq {
            Some(seq) => ring.iter().rev().find(|r| r.seq == seq).copied(),
            None => ring
                .iter()
                .filter(|r| r.t_mid_ns.abs_diff(t_ns) <= TRIGGER_JOIN_TOL_NS)
                .min_by_key(|r| r.t_mid_ns.abs_diff(t_ns))
                .copied(),
        }
    }
}

/// Decode one bus frame into an event; `None` for a kind capture ignores.
pub fn decode(frame: &BusFrame) -> Result<Option<BusEvent>, prost::DecodeError> {
    Ok(Some(match frame.kind {
        Kind::Imu => BusEvent::Imu {
            src: frame.src.clone(),
            batch: ImuBatch::decode(frame.payload.as_slice())?,
        },
        Kind::Gnss => BusEvent::Gnss {
            src: frame.src.clone(),
            raw: GnssRaw::decode(frame.payload.as_slice())?,
        },
        Kind::Trigger => BusEvent::Trigger {
            camera: frame.src.clone(),
            rec: TriggerRecord::decode(frame.payload.as_slice())?,
        },
        Kind::Clock => BusEvent::Clock(ClockSample::decode(frame.payload.as_slice())?),
        _ => return Ok(None),
    }))
}

/// The kinds capture subscribes to.
pub const KINDS: [Kind; 4] = [Kind::Clock, Kind::Imu, Kind::Gnss, Kind::Trigger];

/// Run the ingest until `cancel`: (re)connect to `path`, fold every frame
/// into `state`, and forward it on `tx` (dropped when the loop is behind;
/// the state still has it).
pub async fn run_ingest(
    path: PathBuf,
    state: Arc<Mutex<SensorState>>,
    tx: mpsc::Sender<BusEvent>,
    cancel: Shutdown,
) {
    loop {
        let sub = tokio::select! {
            _ = cancel.wait() => return,
            s = Subscriber::connect(&path, &KINDS) => s,
        };
        let mut sub = match sub {
            Ok(s) => s,
            Err(_) => {
                state.lock().connected = false;
                tokio::select! {
                    _ = cancel.wait() => return,
                    _ = tokio::time::sleep(RECONNECT) => continue,
                }
            }
        };
        state.lock().connected = true;
        tracing::info!(path = %path.display(), "sensor_bus_subscribed");
        loop {
            let frame = tokio::select! {
                _ = cancel.wait() => return,
                f = sub.recv() => f,
            };
            match frame {
                Ok(frame) => match decode(&frame) {
                    Ok(Some(ev)) => {
                        state.lock().apply(&ev);
                        if tx.try_send(ev).is_err() {
                            state.lock().dropped_forward += 1;
                        }
                    }
                    Ok(None) => {}
                    Err(_) => state.lock().decode_errors += 1,
                },
                Err(e) if e.kind() == std::io::ErrorKind::InvalidData => {
                    state.lock().decode_errors += 1;
                }
                Err(e) => {
                    tracing::warn!(error = %e, "sensor_bus_closed");
                    state.lock().connected = false;
                    break;
                }
            }
        }
    }
}

#[cfg(test)]
mod tests;
