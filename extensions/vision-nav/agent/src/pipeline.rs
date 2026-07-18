//! The runtime pipeline + plugin lifecycle.
//!
//! On start the plugin:
//!
//! 1. registers MAVLink components 198 (peripheral) and 197 (VIO),
//! 2. subscribes to the FC message streams the modes consume (RAW_IMU,
//!    DISTANCE_SENSOR, TIMESYNC, GLOBAL_POSITION_INT, VFR_HUD,
//!    GPS_RAW_INT),
//! 3. subscribes to the shared vision frame bus (`ctx.vision`),
//! 4. starts the TIMESYNC + companion-HEARTBEAT + health ticks,
//! 5. runs the estimator on every frame pair and routes the MAVLink
//!    emission to the matching component.
//!
//! The frame subscription callback runs on the IPC reader task and must
//! not block, so it forwards each grayscale-converted frame down a
//! channel; a worker task owns the per-frame loop (estimator step,
//! emit, health, pre-arm). The degradation ladder lives in the worker:
//! a 2 s degraded/failed streak flips the companion to CRITICAL.
//!
//! An in-flight mode change (operator `set_mode` event) rebuilds the
//! estimator + scale source and swaps the worker's estimator on the
//! next tick.

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
use crate::config::{Firmware, Mode, Topology, VisionNavConfig};
use crate::estimator::{Estimator, EstimatorOutput, EstimatorState, OutputMode, RangeReading, StepInputs, VioFrame};
use crate::estimators::{
    available_estimators, HybridEstimator, NullEstimator, OpticalFlowDegradedEstimator,
    OpticalFlowEstimator, VioEstimator,
};
use crate::flow::GyroReading;
use crate::framing::{frame_to_gray, GrayImage};
use crate::health::{companion_system_status, CompanionState, HealthSnapshot};
use crate::imu::{ImuBuffer, TimeAligner};
use crate::mavlink_emit::{
    build_timesync, monotonic_ns, of_frame, ComponentRouter, COMPONENT_OF, COMPONENT_VIO,
};
use crate::pre_arm::{PreArmGate, PreArmInputs};
use crate::rangefinder::{
    I2cRangefinder, Rangefinder, RelayDistanceSensor, TfLunaUart,
};
use crate::scale::ScaleLadder;
use crate::vio::{EngineConfig, VioEngine};

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
                // Bad config keeps the previous (or default) config and
                // logs; the host re-invokes on_configure on a fix.
                eprintln!("vision-nav: config invalid: {e}");
            }
        }
        Ok(())
    }

    async fn on_start(&mut self, ctx: &PluginContext) -> Result<(), ClientError> {
        let cfg = self.config.clone();

        // Register the MAVLink components so subscribers see the
        // peripherals on the bus before the first emit. Idempotent.
        let _ = ctx
            .mavlink
            .register_component(COMPONENT_OF as i64, "peripheral")
            .await;
        let _ = ctx
            .mavlink
            .register_component(COMPONENT_VIO as i64, "vio")
            .await;
        let _ = ctx.vision.register_vio_component().await;

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

/// A grayscale frame plus the raw frame bytes (for the VIO SHM bridge)
/// and the agent monotonic ingest time.
struct GrayFramePair {
    gray: GrayImage,
    vio_frame: VioFrame,
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
            // VIO bridge wants the grayscale plane (the C++ shim is
            // monocular and reads GRAY8). The shared luma plane is the
            // same bytes for both the OF tracker and the SHM ring.
            let vio_frame = VioFrame {
                ts_us: (frame.descriptor.ts_ms.max(0) as u64) * 1000,
                width: gray.width,
                height: gray.height,
                stride: gray.width,
                pixel_format: crate::vio::FRAME_FORMAT_GRAY8,
                pixels: gray.data.clone(),
            };
            let pair = GrayFramePair {
                gray,
                vio_frame,
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
        let install_dir = install_dir();
        let task = tokio::spawn(async move {
            let router = ComponentRouter::new(SENSOR_ID, clock.clone());
            let gate = PreArmGate::with_flow_quality_gate(cfg.flow_quality_min);
            let mut aligner = TimeAligner::new(0.0, 60);
            // Try to load a persisted calibration's timeshift so VIO can
            // pre-arm without re-uploading on every boot.
            let mut intrinsics_loaded = false;
            if let Some((_engine_cfg, timeshift)) = load_calibration(&install_dir) {
                aligner.set_timeshift(timeshift);
                intrinsics_loaded = true;
            }

            let mut estimator: Box<dyn Estimator> =
                build_estimator(&cfg, ladder.clone(), &install_dir);
            estimator.configure();

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
                    curr_vio_frame: Some(&pair.vio_frame),
                    dt_seconds: dt,
                    gyro,
                    imu_sample,
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

            estimator.shutdown();
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
                // vision navigation is engaged (not a false-idle bar, Rule 44).
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

/// Build the estimator for a config mode. VIO modes that would run on
/// iNav (a config-level rejection bypassed by a hand-edited config)
/// fall back to the null estimator, matching the prior belt-and-braces
/// guard.
pub fn build_estimator(
    cfg: &VisionNavConfig,
    ladder: Arc<ScaleLadder>,
    install_dir: &str,
) -> Box<dyn Estimator> {
    // The engage Skill can disengage the estimator (active=false); the
    // effective mode is then Off, so the null estimator runs and no pose is
    // emitted.
    let mode = cfg.effective_mode();
    if cfg.firmware.firmware == Firmware::Inav
        && matches!(
            mode,
            Mode::VioOpenvins | Mode::VioVinsFusion | Mode::HybridOfPlusVio
        )
    {
        eprintln!("vision-nav: VIO not supported on iNav; falling back to off");
        return Box::new(NullEstimator);
    }
    match mode {
        Mode::Off => Box::new(NullEstimator),
        Mode::OpticalFlow => Box::new(OpticalFlowEstimator::new(cfg.flow_quality_min)),
        Mode::OpticalFlowDegraded => Box::new(OpticalFlowDegradedEstimator::new(
            cfg.flow_quality_min,
            Some(ladder),
        )),
        Mode::VioOpenvins => build_vio_estimator(cfg, "vio_openvins", install_dir),
        Mode::VioVinsFusion => build_vio_estimator(cfg, "vio_vins_fusion", install_dir),
        Mode::HybridOfPlusVio => {
            let of = OpticalFlowEstimator::new(cfg.flow_quality_min);
            let vio = build_vio_estimator(cfg, "vio_openvins", install_dir);
            Box::new(HybridEstimator::new(of, vio))
        }
    }
}

/// Build a VIO estimator. When the vendor binary is missing on disk the
/// estimator is still constructed but its engine will fail to start
/// (logged), leaving it in a no-emit state — fail safe. The pre-arm gate
/// independently refuses to arm a VIO mode without calibration.
fn build_vio_estimator(_cfg: &VisionNavConfig, id: &'static str, install_dir: &str) -> Box<dyn Estimator> {
    let engine = match id {
        "vio_vins_fusion" => VioEngine::vins_fusion(
            "/run/ados/plugins/vision-nav-vio.sock",
            "/ados_vio_frames",
        ),
        _ => VioEngine::openvins(
            "/run/ados/plugins/vision-nav-vio.sock",
            "/ados_vio_frames",
        ),
    };
    let engine_cfg = load_calibration(install_dir)
        .map(|(c, _)| c)
        .unwrap_or_else(default_engine_config);
    let leaked_id: &'static str = if id == "vio_vins_fusion" {
        "vio_vins_fusion"
    } else {
        "vio_openvins"
    };
    Box::new(VioEstimator::new(
        leaked_id,
        engine,
        engine_cfg,
        install_dir.to_string(),
    ))
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

/// Load the persisted camchain.yaml: returns `(EngineConfig, timeshift)`
/// or `None` when no calibration is on disk. The calibration wizard
/// (Python helper) produces this file; the Rust agent only reads it.
fn load_calibration(install_dir: &str) -> Option<(EngineConfig, f64)> {
    let path = calibration_path(install_dir);
    let text = std::fs::read_to_string(path).ok()?;
    crate::pipeline::camchain::parse(&text)
}

fn calibration_path(_install_dir: &str) -> std::path::PathBuf {
    let data_dir = std::env::var("ADOS_PLUGIN_DATA_DIR")
        .unwrap_or_else(|_| "/var/ados/plugins/com.altnautica.vision-nav/data".to_string());
    std::path::PathBuf::from(data_dir).join("camchain.yaml")
}

fn default_engine_config() -> EngineConfig {
    // Placeholder; the vendor binary rejects it for missing intrinsics,
    // which surfaces as a start failure (fail safe).
    EngineConfig {
        camera_model: "pinhole".into(),
        fx: 500.0,
        fy: 500.0,
        cx: 320.0,
        cy: 240.0,
        width: 640,
        height: 480,
        distortion_model: "none".into(),
        distortion_coeffs: vec![],
        t_cam_imu: vec![
            1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        ],
        timeshift_cam_imu_s: 0.0,
        imu_rate_hz: 100.0,
        camera_rate_hz: 30.0,
    }
}

/// Kalibr `cam0` camchain.yaml parsing (the subset the agent needs).
pub mod camchain {
    use super::EngineConfig;
    use serde::Deserialize;

    #[derive(Deserialize)]
    struct Cam {
        camera_model: Option<String>,
        intrinsics: Option<Vec<f64>>,
        distortion_model: Option<String>,
        distortion_coeffs: Option<Vec<f64>>,
        resolution: Option<Vec<u32>>,
        #[serde(rename = "T_cam_imu")]
        t_cam_imu: Option<Vec<Vec<f64>>>,
        timeshift_cam_imu: Option<f64>,
    }

    /// Parse a camchain.yaml string. Accepts both the `cam0:` wrapper
    /// and a bare block. Returns `(EngineConfig, timeshift_s)` or `None`
    /// when the required fields are missing.
    pub fn parse(text: &str) -> Option<(EngineConfig, f64)> {
        // Try the `cam0` wrapper first, then a bare block.
        let cam: Cam = serde_yaml::from_str::<std::collections::BTreeMap<String, Cam>>(text)
            .ok()
            .and_then(|mut m| m.remove("cam0"))
            .or_else(|| serde_yaml::from_str::<Cam>(text).ok())?;

        let intr = cam.intrinsics?;
        if intr.len() < 4 {
            return None;
        }
        let res = cam.resolution.unwrap_or_else(|| vec![640, 480]);
        let timeshift = cam.timeshift_cam_imu.unwrap_or(0.0);
        let t_flat: Vec<f64> = cam
            .t_cam_imu
            .map(|rows| rows.into_iter().flatten().collect())
            .unwrap_or_else(|| {
                vec![
                    1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                ]
            });
        let cfg = EngineConfig {
            camera_model: cam.camera_model.unwrap_or_else(|| "pinhole".into()),
            fx: intr[0],
            fy: intr[1],
            cx: intr[2],
            cy: intr[3],
            width: *res.first().unwrap_or(&640),
            height: *res.get(1).unwrap_or(&480),
            distortion_model: cam.distortion_model.unwrap_or_else(|| "none".into()),
            distortion_coeffs: cam.distortion_coeffs.unwrap_or_default(),
            t_cam_imu: t_flat,
            timeshift_cam_imu_s: timeshift,
            imu_rate_hz: 100.0,
            camera_rate_hz: 30.0,
        };
        Some((cfg, timeshift))
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
        estimator_state,
        flow_quality: output.and_then(|o| o.flow_quality),
        flow_scale_source: flow_scale_source.clone(),
        rangefinder_topology: topology.clone(),
        intrinsics_loaded,
        extrinsics_loaded: intrinsics_loaded,
        sync_offset_ms,
        feature_count: output.and_then(|o| o.feature_count),
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
    if let Some(o) = output {
        if let Some(q) = o.flow_quality {
            h.flow_quality = Some(q);
        }
        if o.output_mode == OutputMode::Vio {
            h.vio_state = Some(estimator_state.as_str().to_string());
            h.vio_quality = o.feature_count;
            h.feature_count = o.feature_count;
            if let Some(rc) = o.reset_counter {
                h.vio_reset_counter = rc;
            }
        }
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

fn install_dir() -> String {
    std::env::var("ADOS_PLUGIN_INSTALL_DIR").unwrap_or_else(|_| ".".to_string())
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
        let (cfg, ts) = camchain::parse(yaml).unwrap();
        assert!((cfg.fx - 500.0).abs() < 1e-9);
        assert_eq!(cfg.width, 640);
        assert_eq!(cfg.distortion_model, "radtan");
        assert_eq!(cfg.t_cam_imu.len(), 16);
        assert!((ts - (-0.005)).abs() < 1e-9);
    }

    #[test]
    fn camchain_parses_bare_block() {
        let yaml = r#"
camera_model: pinhole
intrinsics: [400.0, 400.0, 200.0, 150.0]
resolution: [400, 300]
"#;
        let (cfg, ts) = camchain::parse(yaml).unwrap();
        assert!((cfg.fx - 400.0).abs() < 1e-9);
        assert_eq!(cfg.height, 300);
        assert_eq!(ts, 0.0);
    }

    #[test]
    fn camchain_missing_intrinsics_is_none() {
        let yaml = "cam0:\n  camera_model: pinhole\n";
        assert!(camchain::parse(yaml).is_none());
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
