//! The runtime pipeline + plugin lifecycle.
//!
//! On start the plugin:
//!
//! 1. merges the static config with the live per-drone keys
//!    ([`crate::live_config`]) and starts the config watcher, which
//!    restarts the process when that config changes,
//! 2. registers MAVLink component 198 (kind `vio`, the kind the host
//!    reserves 197 and 198 for),
//! 3. subscribes to the FC streams the modes consume
//!    ([`subscriptions`]),
//! 4. starts the TIMESYNC, companion-HEARTBEAT, health and stream-rate
//!    ticks ([`ticks`]),
//! 5. runs the estimator on every frame pair from the shared vision frame
//!    bus and emits the flow sample on component 198.
//!
//! The frame callback runs on the IPC reader task and must not block, so
//! it forwards each grayscale frame down a channel; a worker task owns the
//! per-frame loop. A 2 s degraded/failed streak flips the companion to
//! CRITICAL, and so does a 2 s gap with no frames at all.
//!
//! The config is fixed for the lifetime of the process: a change exits
//! the process and the unit restarts it on the new config.

pub mod camchain;
mod status;
pub mod subscriptions;
pub mod ticks;

use std::collections::{BTreeMap, BTreeSet};
use std::sync::atomic::{AtomicI64, AtomicU64};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use ados_sdk::context::PluginContext;
use ados_sdk::{ClientError, Plugin};
use async_trait::async_trait;
use rmpv::Value;
use tokio::sync::{mpsc, Notify};

use crate::clock_align::ClockAlign;
use crate::config::{Mode, Topology, VisionNavConfig};
use crate::estimator::{Estimator, RangeReading, StepInputs};
use crate::estimators::{
    available_estimators, NullEstimator, OpticalFlowDegradedEstimator, OpticalFlowEstimator,
};
use crate::flow::GyroReading;
use crate::framing::{frame_to_gray, GrayImage};
use crate::health::{CompanionState, HealthSnapshot};
use crate::imu::{ImuBuffer, TimeAligner};
use crate::live_config;
use crate::mavlink_emit::{monotonic_ns, ComponentRouter, COMPONENT_OF};
use crate::pre_arm::PreArmGate;
use crate::rangefinder::{Rangefinder, RelayDistanceSensor, TfLunaUart};
use crate::scale::ScaleLadder;
use status::{mark_frames_stale, publish_health, update_companion, TickInputs};
use subscriptions::{FcTarget, Sinks};

const SENSOR_ID: u8 = 0;
const DEGRADED_GRACE_NS: i64 = 2_000_000_000;
/// No frame for this long marks the estimator failed and the companion
/// critical.
pub const FRAME_STALE_NS: i64 = DEGRADED_GRACE_NS;

/// The plugin entry point loaded by the host.
pub struct VisionNavPlugin {
    static_config: BTreeMap<String, Value>,
    tasks: Vec<tokio::task::JoinHandle<()>>,
    health: Arc<Mutex<HealthSnapshot>>,
}

#[async_trait]
impl Plugin for VisionNavPlugin {
    fn new() -> Self {
        Self {
            static_config: BTreeMap::new(),
            tasks: Vec::new(),
            health: Arc::new(Mutex::new(HealthSnapshot::default())),
        }
    }

    async fn on_configure(
        &mut self,
        _ctx: &PluginContext,
        config: &BTreeMap<String, Value>,
    ) -> Result<(), ClientError> {
        self.static_config = config.clone();
        Ok(())
    }

    async fn on_start(&mut self, ctx: &PluginContext) -> Result<(), ClientError> {
        let live = live_config::read_live(ctx).await.unwrap_or_default();
        let merged = live_config::merge(&self.static_config, &live);
        let cfg = VisionNavConfig::from_map(&merged).unwrap_or_else(|e| {
            eprintln!("vision-nav: config invalid, staying disengaged: {e}");
            VisionNavConfig::disengaged()
        });
        self.tasks.push(live_config::spawn_config_watch(
            ctx,
            self.static_config.clone(),
            merged,
        ));

        if let Err(e) = ctx
            .mavlink
            .register_component(COMPONENT_OF as i64, "vio")
            .await
        {
            eprintln!("vision-nav: register component {COMPONENT_OF} failed: {e}");
        }

        let sinks = Sinks {
            imu: Arc::new(ImuBuffer::default()),
            relay: Arc::new(RelayDistanceSensor::new()),
            clock: Arc::new(ClockAlign::new()),
            ladder: Arc::new(ScaleLadder::new(cfg.outdoor)),
            fc_target: Arc::new(FcTarget::default()),
            fc_params: Arc::new(Mutex::new(BTreeSet::new())),
        };
        let emit_errors = Arc::new(AtomicU64::new(0));
        let router = Arc::new(ComponentRouter::new(
            SENSOR_ID,
            cfg.firmware.firmware,
            sinks.clock.clone(),
            emit_errors.clone(),
        ));

        {
            let mut h = self.health.lock().expect("health lock");
            h.mode = Some(cfg.effective_mode().as_str().to_string());
            h.available_estimators = available_estimators()
                .iter()
                .map(|s| s.to_string())
                .collect();
            h.companion_state = Some(CompanionState::Inactive);
            h.recommended_camera_id = Some(cfg.camera.device_path.clone());
            h.imu_source = Some(crate::imu::SOURCE_ID_MAVLINK_RAW_IMU.to_string());
            if cfg.rangefinder.topology != Topology::None {
                h.rangefinder_topology = Some(cfg.rangefinder.topology.as_str().to_string());
            }
        }

        let denied = subscriptions::subscribe_all(ctx, &sinks).await;
        self.health.lock().expect("health lock").degraded_reasons = denied;

        let companion = Arc::new(AtomicI64::new(CompanionState::Inactive as i64));
        let publish_now = Arc::new(Notify::new());
        self.tasks.push(ticks::spawn_timesync(
            ctx,
            sinks.clock.clone(),
            router.clone(),
        ));
        self.tasks.push(ticks::spawn_heartbeat(
            ctx,
            companion.clone(),
            router.clone(),
        ));
        self.tasks.push(ticks::spawn_stream_intervals(
            ctx,
            sinks.fc_target.clone(),
            router.clone(),
        ));
        self.tasks.push(ticks::spawn_health(
            ctx,
            ticks::HealthInputs {
                health: self.health.clone(),
                emit_errors,
                fc_params: sinks.fc_params.clone(),
                engaged: cfg.effective_mode() != Mode::Off,
                publish_now: publish_now.clone(),
            },
        ));

        let (tx, rx) = mpsc::channel::<GrayFramePair>(8);
        let worker = Worker {
            ctx: ctx.clone(),
            cfg: cfg.clone(),
            sinks,
            router,
            companion,
            health: self.health.clone(),
            publish_now,
        };
        self.tasks.push(tokio::spawn(worker.run(rx)));
        subscribe_frames(ctx, tx).await?;

        eprintln!(
            "vision-nav: started (mode={})",
            cfg.effective_mode().as_str()
        );
        Ok(())
    }

    async fn on_stop(&mut self, _ctx: &PluginContext) -> Result<(), ClientError> {
        for t in self.tasks.drain(..) {
            t.abort();
        }
        Ok(())
    }
}

/// A grayscale frame plus the agent monotonic ingest time.
struct GrayFramePair {
    gray: GrayImage,
    ts_ns: i64,
}

async fn subscribe_frames(
    ctx: &PluginContext,
    tx: mpsc::Sender<GrayFramePair>,
) -> Result<(), ClientError> {
    let on_frame = Arc::new(move |frame: ados_sdk::vision::Frame| {
        let Some(gray) = frame_to_gray(&frame) else {
            return;
        };
        // Latest-wins: drop the frame if the worker is behind; the reader
        // task must never block on the channel.
        let _ = tx.try_send(GrayFramePair {
            gray,
            ts_ns: monotonic_ns(),
        });
    });
    ctx.vision.subscribe_frames(None, on_frame).await
}

/// The per-frame loop.
struct Worker {
    ctx: PluginContext,
    cfg: VisionNavConfig,
    sinks: Sinks,
    router: Arc<ComponentRouter>,
    companion: Arc<AtomicI64>,
    health: Arc<Mutex<HealthSnapshot>>,
    /// Asks the health task to publish now, on a companion transition.
    publish_now: Arc<Notify>,
}

impl Worker {
    async fn run(self, mut rx: mpsc::Receiver<GrayFramePair>) {
        let Worker {
            ctx,
            cfg,
            sinks,
            router,
            companion,
            health,
            publish_now,
        } = self;
        let gate = PreArmGate::with_flow_quality_gate(cfg.flow_quality_min);
        let mut aligner = TimeAligner::new(0.0, 60);
        let mut intrinsics_loaded = false;
        if let Some(timeshift) = camchain::load_calibration_timeshift(ctx.data_dir.as_deref()) {
            aligner.set_timeshift(timeshift);
            intrinsics_loaded = true;
        }
        let mut estimator = build_estimator(&cfg, sinks.ladder.clone());
        let mut rangefinder = build_rangefinder(&cfg, sinks.relay.clone());

        let mut prev_gray: Option<GrayImage> = None;
        let mut prev_ts_ns: Option<i64> = None;
        let mut degraded_streak_start: Option<i64> = None;
        // Set while frames are missing; the first output after the gap
        // publishes the recovery at once.
        let mut frames_lost = false;
        let stale = Duration::from_nanos(FRAME_STALE_NS as u64);

        loop {
            let pair = match tokio::time::timeout(stale, rx.recv()).await {
                Ok(Some(pair)) => pair,
                Ok(None) => break,
                Err(_) => {
                    if cfg.effective_mode() != Mode::Off {
                        mark_frames_stale(&health, &gate, &cfg, &companion);
                        if !frames_lost {
                            publish_now.notify_one();
                        }
                        frames_lost = true;
                    }
                    // Never difference the first frame after a gap.
                    prev_gray = None;
                    prev_ts_ns = None;
                    degraded_streak_start = None;
                    continue;
                }
            };
            let ts_ns = pair.ts_ns;
            let (Some(prev), Some(pts)) = (prev_gray.as_ref(), prev_ts_ns) else {
                prev_gray = Some(pair.gray);
                prev_ts_ns = Some(ts_ns);
                continue;
            };
            let dt = ((ts_ns - pts) as f32 / 1e9).max(1e-6);

            let gyro = sinks.imu.aligned(ts_ns, &mut aligner).map(|a| GyroReading {
                xgyro: a.imu_sample.xgyro,
                ygyro: a.imu_sample.ygyro,
                zgyro: a.imu_sample.zgyro,
            });
            let reading = rangefinder.read();
            let inputs = StepInputs {
                prev_gray: Some(prev),
                curr_gray: Some(&pair.gray),
                dt_seconds: dt,
                gyro,
                range_reading: reading.map(|r| RangeReading {
                    distance_m: r.distance_m,
                    quality: r.quality,
                }),
            };
            let output = estimator.step(&inputs);

            if let Some(out) = &output {
                router.emit(&ctx, out).await;
                // Co-emit DISTANCE_SENSOR when a companion rangefinder is
                // wired.
                if let (Some(r), Topology::Companion) = (reading, cfg.rangefinder.topology) {
                    router
                        .emit_distance_sensor(
                            &ctx,
                            r.distance_m,
                            rangefinder.min_range_m(),
                            rangefinder.max_range_m(),
                            r.quality,
                        )
                        .await;
                }
                update_companion(&companion, out.state, ts_ns, &mut degraded_streak_start);
            }

            publish_health(
                &health,
                &gate,
                &cfg,
                output.as_ref(),
                &companion,
                TickInputs {
                    sync_offset_ms: aligner.mean_residual_ms(),
                    imu_rate_hz: sinks.imu.rate_hz(),
                    intrinsics_loaded,
                    dt,
                    rangefinder: rangefinder.as_ref(),
                    reading_m: reading.map(|r| r.distance_m),
                },
            );
            if frames_lost && output.is_some() {
                frames_lost = false;
                publish_now.notify_one();
            }

            prev_gray = Some(pair.gray);
            prev_ts_ns = Some(ts_ns);
        }
    }
}

/// Build the estimator for a config mode.
pub fn build_estimator(cfg: &VisionNavConfig, ladder: Arc<ScaleLadder>) -> Box<dyn Estimator> {
    match cfg.effective_mode() {
        Mode::Off => Box::new(NullEstimator),
        Mode::OpticalFlow => Box::new(OpticalFlowEstimator::new(cfg.flow_quality_min)),
        Mode::OpticalFlowDegraded => Box::new(OpticalFlowDegradedEstimator::new(
            cfg.flow_quality_min,
            Some(ladder),
        )),
    }
}

fn build_rangefinder(
    cfg: &VisionNavConfig,
    relay: Arc<RelayDistanceSensor>,
) -> Box<dyn Rangefinder> {
    match (cfg.rangefinder.topology, cfg.rangefinder.driver.as_str()) {
        (Topology::None, _) => Box::new(NoRangefinder),
        (Topology::Companion, "tfluna_uart") => {
            let dev = cfg
                .rangefinder
                .device
                .clone()
                .unwrap_or_else(|| "/dev/ttyUSB0".into());
            Box::new(TfLunaUart::spawn(dev, cfg.rangefinder.baud))
        }
        _ => Box::new(ArcRelay(relay)),
    }
}

/// A rangefinder that never reports (no rangefinder wired).
struct NoRangefinder;
impl Rangefinder for NoRangefinder {
    fn read(&mut self) -> Option<crate::rangefinder::RangeReading> {
        None
    }
    fn min_range_m(&self) -> f32 {
        0.0
    }
    fn max_range_m(&self) -> f32 {
        0.0
    }
    fn name(&self) -> &'static str {
        "none"
    }
}

/// Reads the shared FC relay (written by the DISTANCE_SENSOR
/// subscription) through the `&mut`-taking [`Rangefinder`] trait.
struct ArcRelay(Arc<RelayDistanceSensor>);
impl Rangefinder for ArcRelay {
    fn read(&mut self) -> Option<crate::rangefinder::RangeReading> {
        self.0.read_shared()
    }
    fn min_range_m(&self) -> f32 {
        Rangefinder::min_range_m(self.0.as_ref())
    }
    fn max_range_m(&self) -> f32 {
        Rangefinder::max_range_m(self.0.as_ref())
    }
    fn name(&self) -> &'static str {
        "fc_relay"
    }
}

#[cfg(test)]
#[path = "tests.rs"]
mod tests;
