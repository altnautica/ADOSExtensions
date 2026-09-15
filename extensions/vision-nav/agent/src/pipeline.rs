//! The runtime pipeline + plugin lifecycle.
//!
//! On start the plugin:
//!
//! 1. registers MAVLink component 198 (peripheral),
//! 2. subscribes to the FC message streams the modes consume (RAW_IMU,
//!    DISTANCE_SENSOR, TIMESYNC, GLOBAL_POSITION_INT, VFR_HUD,
//!    GPS_RAW_INT),
//! 3. subscribes to the shared vision frame bus (`ctx.vision`),
//! 4. starts the TIMESYNC + companion-HEARTBEAT + health ticks,
//! 5. runs the estimator on every frame pair and emits the flow sample
//!    on component 198.
//!
//! The frame subscription callback runs on the IPC reader task and must
//! not block, so it forwards each grayscale-converted frame down a
//! channel; a worker task owns the per-frame loop (estimator step,
//! emit, health, pre-arm). The degradation ladder lives in the worker:
//! a 2 s degraded/failed streak flips the companion to CRITICAL.
//!
//! The mode is fixed for the lifetime of a worker. A config change
//! arrives through `on_configure`, which the host follows with a stop /
//! start of the plugin, so the worker is rebuilt with the new mode.

use std::collections::BTreeMap;
use std::sync::atomic::{AtomicI64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::Duration;

use ados_sdk::context::PluginContext;
use ados_sdk::{ClientError, Plugin};
use async_trait::async_trait;
use rmpv::Value;
use tokio::sync::mpsc;

use crate::clock_align::ClockAlign;
use crate::config::{Mode, Topology, VisionNavConfig};
use crate::estimator::{
    Estimator, EstimatorOutput, EstimatorState, RangeReading, StepInputs,
};
use crate::estimators::{
    available_estimators, NullEstimator, OpticalFlowDegradedEstimator, OpticalFlowEstimator,
};
use crate::flow::GyroReading;
use crate::framing::{frame_to_gray, GrayImage};
use crate::health::{companion_system_status, CompanionState, HealthSnapshot};
use crate::imu::{ImuBuffer, TimeAligner};
use crate::mavlink_emit::{
    build_timesync, monotonic_ns, of_frame, ComponentRouter, COMPONENT_OF,
};
use crate::pre_arm::{PreArmGate, PreArmInputs};
use crate::rangefinder::{
    I2cRangefinder, Rangefinder, RelayDistanceSensor, TfLunaUart,
};
use crate::scale::ScaleLadder;

const SENSOR_ID: u8 = 0;
const DEGRADED_GRACE_NS: i64 = 2_000_000_000;

/// The plugin entry point loaded by the host.
pub struct VisionNavPlugin {
    config: VisionNavConfig,
    shutdown: Arc<tokio::sync::Notify>,
    tasks: Vec<tokio::task::JoinHandle<()>>,
    health: Arc<Mutex<HealthSnapshot>>,
}

#[async_trait]
impl Plugin for VisionNavPlugin {
    fn new() -> Self {
        Self {
            config: VisionNavConfig::default(),
            shutdown: Arc::new(tokio::sync::Notify::new()),
            tasks: Vec::new(),
            health: Arc::new(Mutex::new(HealthSnapshot::default())),
        }
    }

    async fn on_configure(
        &mut self,
        _ctx: &PluginContext,
        config: &BTreeMap<String, Value>,
    ) -> Result<(), ClientError> {
        match VisionNavConfig::from_map(config) {
            Ok(cfg) => self.config = cfg,
            Err(e) => {
                // A config that fails validation disengages vision
                // navigation rather than falling back to a mode the
                // operator did not pick; the host re-invokes
                // on_configure on a fix.
                eprintln!("vision-nav: config invalid, staying disengaged: {e}");
                self.config = VisionNavConfig::disengaged();
            }
        }
        Ok(())
    }

    async fn on_start(&mut self, ctx: &PluginContext) -> Result<(), ClientError> {
        let cfg = self.config.clone();

        // Register the MAVLink component so subscribers see the
        // peripheral on the bus before the first emit. Idempotent.
        let _ = ctx
            .mavlink
            .register_component(COMPONENT_OF as i64, "peripheral")
            .await;

        let clock = Arc::new(ClockAlign::new());
        let imu = Arc::new(ImuBuffer::default());
        let relay = Arc::new(RelayDistanceSensor::new());
        let ladder = Arc::new(ScaleLadder::new(false));

        // Seed the static health fields.
        {
            let mut h = self.health.lock().expect("health lock");
            h.mode = Some(cfg.effective_mode().as_str().to_string());
            h.available_estimators = available_estimators().iter().map(|s| s.to_string()).collect();
            h.companion_state = Some(CompanionState::Inactive);
            h.recommended_camera_id = Some(cfg.camera.device_path.clone());
            h.imu_source = Some(crate::imu::SOURCE_ID_MAVLINK_RAW_IMU.to_string());
            if cfg.rangefinder.topology != Topology::None {
                h.rangefinder_topology = Some(cfg.rangefinder.topology.as_str().to_string());
            }
        }

        // ---- MAVLink subscriptions -----------------------------------
        self.subscribe_raw_imu(ctx, imu.clone()).await;
        self.subscribe_distance_sensor(ctx, relay.clone()).await;
        self.subscribe_timesync(ctx, clock.clone()).await;
        self.subscribe_scale_sources(ctx, ladder.clone()).await;

        // ---- TIMESYNC tick -------------------------------------------
        self.spawn_timesync_tick(ctx, clock.clone());

        // ---- companion HEARTBEAT tick --------------------------------
        let companion = Arc::new(AtomicI64::new(CompanionState::Inactive as i64));
        self.spawn_heartbeat_tick(ctx, companion.clone());

        // ---- health publish tick -------------------------------------
        // The engage Skill's read-back: engaged when the estimator is active
        // and a mode other than Off is configured.
        let engaged = cfg.active && cfg.mode != Mode::Off;
        self.spawn_health_tick(ctx, engaged);

        // ---- the per-frame worker + frame subscription ---------------
        let (tx, rx) = mpsc::channel::<GrayFramePair>(8);
        self.spawn_worker(
            ctx,
            rx,
            cfg.clone(),
            clock.clone(),
            imu.clone(),
            relay.clone(),
            ladder.clone(),
            companion.clone(),
        );
        self.subscribe_frames(ctx, tx).await?;

        eprintln!("vision-nav: started (mode={})", cfg.effective_mode().as_str());
        Ok(())
    }

    async fn on_stop(&mut self, _ctx: &PluginContext) -> Result<(), ClientError> {
        self.shutdown.notify_waiters();
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

impl VisionNavPlugin {
    // ------------------------------------------------------------------
    // Frame subscription -> worker
    // ------------------------------------------------------------------

    async fn subscribe_frames(
        &self,
        ctx: &PluginContext,
        tx: mpsc::Sender<GrayFramePair>,
    ) -> Result<(), ClientError> {
        let on_frame = Arc::new(move |frame: ados_sdk::vision::Frame| {
            let Some(gray) = frame_to_gray(&frame) else {
                return;
            };
            let pair = GrayFramePair {
                gray,
                ts_ns: monotonic_ns(),
            };
            // Drop the frame if the worker is behind (latest-wins; the
            // reader task must never block on the channel).
            let _ = tx.try_send(pair);
        });
        ctx.vision.subscribe_frames(None, on_frame).await
    }

    #[allow(clippy::too_many_arguments)]
    fn spawn_worker(
        &mut self,
        ctx: &PluginContext,
        mut rx: mpsc::Receiver<GrayFramePair>,
        cfg: VisionNavConfig,
        clock: Arc<ClockAlign>,
        imu: Arc<ImuBuffer>,
        relay: Arc<RelayDistanceSensor>,
        ladder: Arc<ScaleLadder>,
        companion: Arc<AtomicI64>,
    ) {
        let ctx = ctx.clone();
        let health = self.health.clone();
        let task = tokio::spawn(async move {
            let router = ComponentRouter::new(SENSOR_ID, clock.clone());
            let gate = PreArmGate::with_flow_quality_gate(cfg.flow_quality_min);
            let mut aligner = TimeAligner::new(0.0, 60);
            // A persisted calibration supplies the static camera-IMU
            // timeshift the aligner needs to pair a frame with the gyro
            // sample that was taken at the same instant.
            let mut intrinsics_loaded = false;
            if let Some(timeshift) = load_calibration_timeshift() {
                aligner.set_timeshift(timeshift);
                intrinsics_loaded = true;
            }

            let mut estimator: Box<dyn Estimator> = build_estimator(&cfg, ladder.clone());

            let mut prev_gray: Option<GrayImage> = None;
            let mut prev_ts_ns: Option<i64> = None;
            let mut degraded_streak_start: Option<i64> = None;
            let mut rangefinder: Box<dyn Rangefinder> = build_rangefinder(&cfg, relay.clone());

            while let Some(pair) = rx.recv().await {
                let ts_ns = pair.ts_ns;
                let (Some(prev), Some(pts)) = (prev_gray.as_ref(), prev_ts_ns) else {
                    prev_gray = Some(pair.gray);
                    prev_ts_ns = Some(ts_ns);
                    continue;
                };
                let dt = ((ts_ns - pts) as f32 / 1e9).max(1e-6);

                let imu_recent = imu.recent();
                let aligned = aligner.lookup(ts_ns, &imu_recent);
                let imu_sample = aligned.map(|a| a.imu_sample);
                let gyro = imu_sample.map(|s| GyroReading {
                    xgyro: s.xgyro,
                    ygyro: s.ygyro,
                    zgyro: s.zgyro,
                });
                let reading = rangefinder.read();
                let range_reading = reading.map(|r| RangeReading {
                    distance_m: r.distance_m,
                    quality: r.quality,
                });

                let inputs = StepInputs {
                    prev_gray: Some(prev),
                    curr_gray: Some(&pair.gray),
                    dt_seconds: dt,
                    gyro,
                    range_reading,
                };

                let output = estimator.step(&inputs);

                if let Some(out) = &output {
                    router.emit(&ctx, out).await;
                    // Co-emit DISTANCE_SENSOR on comp 198 when a
                    // companion rangefinder is wired.
                    if let Some(r) = reading {
                        if cfg.rangefinder.topology == Topology::Companion {
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
                    }
                    update_companion(&companion, out.state, ts_ns, &mut degraded_streak_start);
                }

                // Pre-arm + health every tick (even on a None sample so
                // the GCS sees an honest state).
                publish_health(
                    &health,
                    &gate,
                    &cfg,
                    output.as_ref(),
                    &companion,
                    aligner.mean_residual_ms(),
                    imu.rate_hz(),
                    intrinsics_loaded,
                    dt,
                    reading.map(|r| r.distance_m),
                );

                prev_gray = Some(pair.gray);
                prev_ts_ns = Some(ts_ns);
            }
        });
        self.tasks.push(task);
    }

    // ------------------------------------------------------------------
    // MAVLink subscriptions
    // ------------------------------------------------------------------

    async fn subscribe_raw_imu(&self, ctx: &PluginContext, imu: Arc<ImuBuffer>) {
        let cb = Arc::new(move |args: Value| {
            if let Some(frame) = decode_mavlink_payload(&args) {
                let g = |k: &str| field_f32(&frame, k).unwrap_or(0.0);
                imu.record_raw_imu(
                    monotonic_ns(),
                    g("xgyro"),
                    g("ygyro"),
                    g("zgyro"),
                    g("xacc"),
                    g("yacc"),
                    g("zacc"),
                );
            }
        });
        let _ = ctx.mavlink.subscribe("RAW_IMU", cb).await;
    }

    async fn subscribe_distance_sensor(&self, ctx: &PluginContext, relay: Arc<RelayDistanceSensor>) {
        let cb = Arc::new(move |args: Value| {
            if let Some(frame) = decode_mavlink_payload(&args) {
                let current = field_i64(&frame, "current_distance");
                if let Some(c) = current {
                    relay.on_distance(
                        c,
                        field_i64(&frame, "min_distance"),
                        field_i64(&frame, "max_distance"),
                        field_i64(&frame, "covariance").unwrap_or(0),
                        monotonic_ns(),
                    );
                }
            }
        });
        let _ = ctx.mavlink.subscribe("DISTANCE_SENSOR", cb).await;
    }

    async fn subscribe_timesync(&self, ctx: &PluginContext, clock: Arc<ClockAlign>) {
        let cb = Arc::new(move |args: Value| {
            if let Some(frame) = decode_mavlink_payload(&args) {
                if let (Some(tc1), Some(ts1)) =
                    (field_i64(&frame, "tc1"), field_i64(&frame, "ts1"))
                {
                    clock.handle_response(tc1, ts1);
                }
            }
        });
        let _ = ctx.mavlink.subscribe("TIMESYNC", cb).await;
    }

    async fn subscribe_scale_sources(&self, ctx: &PluginContext, ladder: Arc<ScaleLadder>) {
        let l1 = ladder.clone();
        let gp = Arc::new(move |args: Value| {
            if let Some(frame) = decode_mavlink_payload(&args) {
                if let Some(rel) = field_i64(&frame, "relative_alt") {
                    l1.on_global_position(rel, monotonic_ns());
                }
            }
        });
        let _ = ctx.mavlink.subscribe("GLOBAL_POSITION_INT", gp).await;

        let l2 = ladder.clone();
        let vfr = Arc::new(move |args: Value| {
            if let Some(frame) = decode_mavlink_payload(&args) {
                if let Some(alt) = field_f32(&frame, "alt") {
                    l2.on_vfr_hud(alt, monotonic_ns());
                }
            }
        });
        let _ = ctx.mavlink.subscribe("VFR_HUD", vfr).await;

        let l3 = ladder.clone();
        let gps = Arc::new(move |args: Value| {
            if let Some(frame) = decode_mavlink_payload(&args) {
                if let (Some(alt), Some(fix), Some(eph)) = (
                    field_i64(&frame, "alt"),
                    field_i64(&frame, "fix_type"),
                    field_i64(&frame, "eph"),
                ) {
                    l3.on_gps_raw(alt, fix as i32, eph as i32, monotonic_ns());
                }
            }
        });
        let _ = ctx.mavlink.subscribe("GPS_RAW_INT", gps).await;
    }

    // ------------------------------------------------------------------
    // Ticks
    // ------------------------------------------------------------------

    fn spawn_timesync_tick(&mut self, ctx: &PluginContext, clock: Arc<ClockAlign>) {
        let ctx = ctx.clone();
        let task = tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_secs(1));
            loop {
                interval.tick().await;
                let ts1 = monotonic_ns();
                clock.mark_outgoing(ts1);
                let msg = build_timesync(ts1);
                if let Some(frame) = of_frame(&msg) {
                    let _ = ctx.mavlink.send(&frame, Some(COMPONENT_OF as i64)).await;
                }
            }
        });
        self.tasks.push(task);
    }

    fn spawn_heartbeat_tick(&mut self, ctx: &PluginContext, companion: Arc<AtomicI64>) {
        let ctx = ctx.clone();
        let task = tokio::spawn(async move {
            let mut interval = tokio::time::interval(Duration::from_secs(1));
            loop {
                interval.tick().await;
                let state = companion_state_from_i64(companion.load(Ordering::Relaxed));
                let status = companion_system_status(state);
                let msg = build_companion_heartbeat(status);
                if let Some(frame) = of_frame(&msg) {
                    let _ = ctx.mavlink.send(&frame, Some(COMPONENT_OF as i64)).await;
                }
            }
        });
        self.tasks.push(task);
    }

    fn spawn_health_tick(&mut self, ctx: &PluginContext, engaged: bool) {
        let ctx = ctx.clone();
        let health = self.health.clone();
        let task = tokio::spawn(async move {
            let engage_state = if engaged { "active" } else { "idle" };
            let mut interval = tokio::time::interval(Duration::from_secs(1));
            loop {
                interval.tick().await;
                let snapshot = health.lock().expect("health lock").to_value();
                let _ = ctx.telemetry.extend("navigation", snapshot).await;
                // The engage Skill reads this state event to reflect whether
                // vision navigation is actually engaged, so a disengaged or
                // off-mode plugin reports idle instead of a permanent idle
                // bar that never tracks the running state.
                let _ = ctx
                    .events
                    .publish(
                        "navigation.engage",
                        Value::Map(vec![(
                            Value::from("state"),
                            Value::from(engage_state),
                        )]),
                    )
                    .await;
            }
        });
        self.tasks.push(task);
    }
}

// ---------------------------------------------------------------------------
// Estimator + rangefinder construction
// ---------------------------------------------------------------------------

/// Build the estimator for a config mode.
pub fn build_estimator(cfg: &VisionNavConfig, ladder: Arc<ScaleLadder>) -> Box<dyn Estimator> {
    // The engage Skill can disengage the estimator (active=false); the
    // effective mode is then Off, so the null estimator runs and nothing
    // is emitted.
    match cfg.effective_mode() {
        Mode::Off => Box::new(NullEstimator),
        Mode::OpticalFlow => Box::new(OpticalFlowEstimator::new(cfg.flow_quality_min)),
        Mode::OpticalFlowDegraded => Box::new(OpticalFlowDegradedEstimator::new(
            cfg.flow_quality_min,
            Some(ladder),
        )),
    }
}

fn build_rangefinder(cfg: &VisionNavConfig, relay: Arc<RelayDistanceSensor>) -> Box<dyn Rangefinder> {
    match cfg.rangefinder.topology {
        Topology::None => Box::new(NoRangefinder),
        Topology::Fc => Box::new(ArcRelay(relay)),
        Topology::Companion => match cfg.rangefinder.driver.as_str() {
            "tfluna_uart" => {
                let dev = cfg.rangefinder.device.clone().unwrap_or_else(|| "/dev/ttyUSB0".into());
                let mut d = TfLunaUart::new(dev, cfg.rangefinder.baud.unwrap_or(115200));
                d.open();
                Box::new(d)
            }
            "garmin_lidarlite_i2c" => Box::new(I2cRangefinder::garmin_lidarlite(
                i2c_bus(cfg.rangefinder.device.as_deref()),
            )),
            "vl53l1x_i2c" => Box::new(I2cRangefinder::vl53l1x(i2c_bus(
                cfg.rangefinder.device.as_deref(),
            ))),
            // fc_relay or unknown -> relay.
            _ => Box::new(ArcRelay(relay)),
        },
    }
}

/// A rangefinder that never reports (mode does not use one).
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

/// Wraps the shared relay `Arc` so the FC-relay rangefinder (whose
/// inner state the MAVLink `DISTANCE_SENSOR` subscription writes) can be
/// read through the `&mut`-taking [`Rangefinder`] trait. `read_shared`
/// only locks the inner mutex, so a shared `Arc` clone is enough.
struct ArcRelay(Arc<RelayDistanceSensor>);
impl Rangefinder for ArcRelay {
    fn read(&mut self) -> Option<crate::rangefinder::RangeReading> {
        self.0.read_shared()
    }
    fn min_range_m(&self) -> f32 {
        self.0.min_range_m()
    }
    fn max_range_m(&self) -> f32 {
        self.0.max_range_m()
    }
    fn name(&self) -> &'static str {
        "fc_relay"
    }
}

fn i2c_bus(device: Option<&str>) -> u32 {
    match device {
        None => 1,
        Some(s) if s.chars().all(|c| c.is_ascii_digit()) => s.parse().unwrap_or(1),
        Some(s) if s.contains("i2c-") => {
            s.rsplit("i2c-").next().and_then(|t| t.parse().ok()).unwrap_or(1)
        }
        Some(_) => 1,
    }
}

// ---------------------------------------------------------------------------
// Calibration loading
// ---------------------------------------------------------------------------

/// Load the camera-IMU timeshift from the persisted camchain.yaml, or
/// `None` when no calibration is on disk. The calibration wizard
/// (Python helper) produces this file; the Rust agent only reads it.
fn load_calibration_timeshift() -> Option<f64> {
    let text = std::fs::read_to_string(calibration_path()).ok()?;
    camchain::timeshift(&text)
}

fn calibration_path() -> std::path::PathBuf {
    let data_dir = std::env::var("ADOS_PLUGIN_DATA_DIR")
        .unwrap_or_else(|_| "/var/ados/plugins/com.altnautica.vision-nav/data".to_string());
    std::path::PathBuf::from(data_dir).join("camchain.yaml")
}

/// Kalibr `cam0` camchain.yaml parsing (the subset the agent needs).
pub mod camchain {
    use serde::Deserialize;

    #[derive(Deserialize)]
    struct Cam {
        intrinsics: Option<Vec<f64>>,
        timeshift_cam_imu: Option<f64>,
    }

    /// Read `timeshift_cam_imu` (seconds) out of a camchain.yaml string.
    /// Accepts both the `cam0:` wrapper and a bare block. `None` when
    /// the file does not carry a calibrated camera, in which case the
    /// aligner keeps its zero offset and the heartbeat reports the
    /// calibration as not loaded.
    pub fn timeshift(text: &str) -> Option<f64> {
        // Try the `cam0` wrapper first, then a bare block.
        let cam: Cam = serde_yaml::from_str::<std::collections::BTreeMap<String, Cam>>(text)
            .ok()
            .and_then(|mut m| m.remove("cam0"))
            .or_else(|| serde_yaml::from_str::<Cam>(text).ok())?;

        let intr = cam.intrinsics?;
        if intr.len() < 4 {
            return None;
        }
        Some(cam.timeshift_cam_imu.unwrap_or(0.0))
    }
}

// ---------------------------------------------------------------------------
// Companion state machine + health publish
// ---------------------------------------------------------------------------

fn update_companion(
    companion: &Arc<AtomicI64>,
    state: EstimatorState,
    ts_ns: i64,
    degraded_streak_start: &mut Option<i64>,
) {
    match state {
        EstimatorState::Converged => {
            *degraded_streak_start = None;
            companion.store(CompanionState::Active as i64, Ordering::Relaxed);
        }
        EstimatorState::Degraded | EstimatorState::Failed => {
            match *degraded_streak_start {
                None => *degraded_streak_start = Some(ts_ns),
                Some(start) if ts_ns - start >= DEGRADED_GRACE_NS => {
                    companion.store(CompanionState::Critical as i64, Ordering::Relaxed);
                }
                _ => {}
            }
        }
        // init / converging / off: leave the companion where it was.
        _ => *degraded_streak_start = None,
    }
}

#[allow(clippy::too_many_arguments)]
fn publish_health(
    health: &Arc<Mutex<HealthSnapshot>>,
    gate: &PreArmGate,
    cfg: &VisionNavConfig,
    output: Option<&EstimatorOutput>,
    companion: &Arc<AtomicI64>,
    sync_offset_ms: Option<f32>,
    imu_rate_hz: Option<f32>,
    intrinsics_loaded: bool,
    dt: f32,
    distance_m: Option<f32>,
) {
    let companion_state = companion_state_from_i64(companion.load(Ordering::Relaxed));
    let estimator_state = output.map(|o| o.state).unwrap_or(EstimatorState::Off);
    let topology = match cfg.rangefinder.topology {
        Topology::None => None,
        t => Some(t.as_str().to_string()),
    };
    let flow_scale_source = output.and_then(|o| o.flow_scale_source).map(|s| s.as_str().to_string());

    let inputs = PreArmInputs {
        mode: cfg.effective_mode(),
        companion_state,
        flow_quality: output.and_then(|o| o.flow_quality),
        flow_scale_source: flow_scale_source.clone(),
        rangefinder_topology: topology.clone(),
    };
    let report = gate.evaluate(&inputs);

    let mut h = health.lock().expect("health lock");
    h.companion_state = Some(companion_state);
    h.estimator_state = Some(estimator_state.as_str().to_string());
    h.camera_intrinsics_loaded = intrinsics_loaded;
    h.camera_imu_sync_offset_ms = sync_offset_ms;
    h.imu_rate_hz = imu_rate_hz;
    h.flow_scale_source = flow_scale_source;
    h.flow_distance_m = distance_m;
    h.pre_arm_report = Some(report.to_value());
    if dt > 0.0 {
        h.flow_rate_hz = Some(1.0 / dt);
    }
    if let Some(q) = output.and_then(|o| o.flow_quality) {
        h.flow_quality = Some(q);
    }
}

fn companion_state_from_i64(v: i64) -> CompanionState {
    match v {
        x if x == CompanionState::Active as i64 => CompanionState::Active,
        x if x == CompanionState::Critical as i64 => CompanionState::Critical,
        x if x == CompanionState::Terminating as i64 => CompanionState::Terminating,
        _ => CompanionState::Inactive,
    }
}

/// Build the companion HEARTBEAT (#0) with the given system_status.
fn build_companion_heartbeat(system_status: u8) -> ados_protocol::mavlink::MavMessage {
    use ados_protocol::mavlink::ardupilotmega::{
        HEARTBEAT_DATA, MavAutopilot, MavModeFlag, MavType,
    };
    use ados_protocol::mavlink::MavMessage;
    MavMessage::HEARTBEAT(HEARTBEAT_DATA {
        custom_mode: 0,
        mavtype: MavType::MAV_TYPE_GENERIC,
        autopilot: MavAutopilot::MAV_AUTOPILOT_INVALID,
        base_mode: MavModeFlag::empty(),
        system_status: system_status_to_enum(system_status),
        mavlink_version: 3,
    })
}

fn system_status_to_enum(status: u8) -> ados_protocol::mavlink::ardupilotmega::MavState {
    use ados_protocol::mavlink::ardupilotmega::MavState;
    match status {
        4 => MavState::MAV_STATE_ACTIVE,
        6 => MavState::MAV_STATE_CRITICAL,
        8 => MavState::MAV_STATE_FLIGHT_TERMINATION,
        _ => MavState::MAV_STATE_STANDBY,
    }
}

// ---------------------------------------------------------------------------
// MAVLink delivery payload decoding (the host forwards decoded fields)
// ---------------------------------------------------------------------------

/// The host delivers a MAVLink subscription as an args map carrying the
/// decoded fields. Different hosts nest the fields under `frame`,
/// `fields`, or `payload`, or place them at the top level. This pulls
/// whichever map carries the message fields.
fn decode_mavlink_payload(args: &Value) -> Option<Value> {
    for key in ["fields", "payload", "frame"] {
        if let Some(v) = map_get(args, key) {
            if matches!(v, Value::Map(_)) {
                return Some(v.clone());
            }
        }
    }
    // Top-level args may already be the field map.
    if matches!(args, Value::Map(_)) {
        return Some(args.clone());
    }
    None
}

fn map_get(args: &Value, key: &str) -> Option<Value> {
    match args {
        Value::Map(e) => e
            .iter()
            .find(|(k, _)| k.as_str() == Some(key))
            .map(|(_, v)| v.clone()),
        _ => None,
    }
}

fn field_f32(frame: &Value, key: &str) -> Option<f32> {
    let v = map_get(frame, key)?;
    v.as_f64().or_else(|| v.as_i64().map(|i| i as f64)).map(|f| f as f32)
}

fn field_i64(frame: &Value, key: &str) -> Option<i64> {
    let v = map_get(frame, key)?;
    v.as_i64().or_else(|| v.as_u64().map(|u| u as i64)).or_else(|| v.as_f64().map(|f| f as i64))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn companion_state_machine_promotes_and_demotes() {
        let companion = Arc::new(AtomicI64::new(CompanionState::Inactive as i64));
        let mut streak = None;
        // Converged -> active immediately.
        update_companion(&companion, EstimatorState::Converged, 0, &mut streak);
        assert_eq!(
            companion_state_from_i64(companion.load(Ordering::Relaxed)),
            CompanionState::Active
        );
        // A single degraded sample does not flip to critical.
        update_companion(&companion, EstimatorState::Degraded, 1_000_000_000, &mut streak);
        assert_eq!(
            companion_state_from_i64(companion.load(Ordering::Relaxed)),
            CompanionState::Active
        );
        // A 2 s degraded streak flips to critical.
        update_companion(
            &companion,
            EstimatorState::Degraded,
            1_000_000_000 + DEGRADED_GRACE_NS,
            &mut streak,
        );
        assert_eq!(
            companion_state_from_i64(companion.load(Ordering::Relaxed)),
            CompanionState::Critical
        );
    }

    #[test]
    fn camchain_parses_cam0_wrapper() {
        let yaml = r#"
cam0:
  camera_model: pinhole
  intrinsics: [500.0, 501.0, 320.0, 240.0]
  distortion_model: radtan
  distortion_coeffs: [0.1, 0.0, 0.0, 0.0]
  resolution: [640, 480]
  T_cam_imu:
    - [1.0, 0.0, 0.0, 0.01]
    - [0.0, 1.0, 0.0, 0.02]
    - [0.0, 0.0, 1.0, 0.03]
    - [0.0, 0.0, 0.0, 1.0]
  timeshift_cam_imu: -0.005
"#;
        // The aligner consumes the timeshift; the other blocks are
        // tolerated and ignored.
        let ts = camchain::timeshift(yaml).unwrap();
        assert!((ts - (-0.005)).abs() < 1e-9);
    }

    #[test]
    fn camchain_parses_bare_block() {
        let yaml = r#"
camera_model: pinhole
intrinsics: [400.0, 400.0, 200.0, 150.0]
resolution: [400, 300]
"#;
        // A calibration with no timeshift key aligns at zero offset.
        assert_eq!(camchain::timeshift(yaml).unwrap(), 0.0);
    }

    #[test]
    fn camchain_without_intrinsics_is_not_a_calibration() {
        // No intrinsics block: the file is not a usable calibration, so
        // the heartbeat must report the calibration as absent rather
        // than aligning on a zero offset it never measured.
        let yaml = "cam0:\n  camera_model: pinhole\n";
        assert!(camchain::timeshift(yaml).is_none());
    }

    #[test]
    fn decode_payload_finds_nested_fields() {
        let args = Value::Map(vec![
            (Value::from("msg_name"), Value::from("RAW_IMU")),
            (
                Value::from("fields"),
                Value::Map(vec![(Value::from("xgyro"), Value::from(1000i64))]),
            ),
        ]);
        let frame = decode_mavlink_payload(&args).unwrap();
        assert_eq!(field_i64(&frame, "xgyro"), Some(1000));
    }

    #[test]
    fn i2c_bus_parses_forms() {
        assert_eq!(i2c_bus(Some("1")), 1);
        assert_eq!(i2c_bus(Some("/dev/i2c-3")), 3);
        assert_eq!(i2c_bus(None), 1);
    }
}
