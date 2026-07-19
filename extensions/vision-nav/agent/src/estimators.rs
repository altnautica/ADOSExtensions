//! The six estimator modes + the registry.
//!
//! Each mode is an [`Estimator`]. A config-mode flip selects a
//! different estimator without bespoke routing:
//!
//! * `off` -> [`NullEstimator`]
//! * `optical_flow` -> [`OpticalFlowEstimator`]
//! * `optical_flow_degraded` -> [`OpticalFlowDegradedEstimator`]
//! * `vio_openvins` / `vio_vins_fusion` -> [`VioEstimator`]
//! * `hybrid_of_plus_vio` -> [`HybridEstimator`]

use crate::estimator::{
    Estimator, EstimatorOutput, EstimatorState, OutputMode, ScaleSourceLabel, StepInputs,
};
use crate::flow::OpticalFlowLk;
use crate::mavlink_emit::monotonic_ns;
use crate::scale::{ScaleLadder, ScaleRung};
use crate::vio::{EngineConfig, PoseMessage, VioEngine};

const DEFAULT_QUALITY_GATE: i32 = 50;

/// The list of estimator keys this build can run (surfaced on the
/// heartbeat so the GCS mode picker only shows modes the agent can
/// instantiate).
pub fn available_estimators() -> Vec<&'static str> {
    let mut v = vec![
        "off",
        "optical_flow",
        "optical_flow_degraded",
        "vio_openvins",
        "vio_vins_fusion",
        "hybrid_of_plus_vio",
    ];
    v.sort_unstable();
    v
}

// ---------------------------------------------------------------------------
// Off
// ---------------------------------------------------------------------------

/// Off mode: never emits MAVLink, reports `state=off`. Keeps the plugin
/// loaded with all sensors discovered but the EKF feed silent.
pub struct NullEstimator;

impl Estimator for NullEstimator {
    fn estimator_id(&self) -> &'static str {
        "off"
    }
    fn output_mode(&self) -> OutputMode {
        OutputMode::None
    }
    fn step(&mut self, _inputs: &StepInputs<'_>) -> Option<EstimatorOutput> {
        Some(EstimatorOutput::none(monotonic_ns() / 1000))
    }
}

// ---------------------------------------------------------------------------
// Optical flow (with rangefinder)
// ---------------------------------------------------------------------------

/// Optical-flow estimator. Wraps the Lucas-Kanade tracker behind the
/// estimator contract.
///
/// State mapping: `init` until the first sample at/above the gate
/// (then `converged`); once converged, a later sub-gate sample is
/// `degraded`.
pub struct OpticalFlowEstimator {
    processor: OpticalFlowLk,
    quality_gate: i32,
    state: EstimatorState,
    seen_converged: bool,
}

impl OpticalFlowEstimator {
    pub fn new(quality_gate: i32) -> Self {
        Self {
            processor: OpticalFlowLk::new(),
            quality_gate,
            state: EstimatorState::Init,
            seen_converged: false,
        }
    }

    pub fn state(&self) -> EstimatorState {
        self.state
    }

    /// Run the tracker and build the OF sample. Shared by the plain and
    /// the degraded estimators (the latter applies a scale multiplier
    /// and a synthetic distance before calling this).
    fn run(
        &mut self,
        inputs: &StepInputs<'_>,
        distance_m: Option<f32>,
        scale_source: Option<ScaleSourceLabel>,
    ) -> Option<EstimatorOutput> {
        let (prev, curr) = (inputs.prev_gray?, inputs.curr_gray?);
        let result = self.processor.process(
            prev,
            curr,
            inputs.dt_seconds,
            inputs.gyro,
            distance_m,
        );
        let quality = result.quality;
        if quality >= self.quality_gate {
            self.state = EstimatorState::Converged;
            self.seen_converged = true;
        } else if self.seen_converged {
            self.state = EstimatorState::Degraded;
        } else {
            self.state = EstimatorState::Init;
        }
        Some(EstimatorOutput {
            timestamp_us: result.integration_time_us as i64,
            output_mode: OutputMode::OpticalFlow,
            state: self.state,
            flow_rate_x: Some(result.flow_rate_x),
            flow_rate_y: Some(result.flow_rate_y),
            flow_rate_z: Some(result.flow_rate_z),
            flow_quality: Some(quality),
            flow_distance_m: distance_m,
            flow_scale_source: scale_source,
            integration_time_us: Some(result.integration_time_us as i64),
            pose: None,
            velocity: None,
            covariance: None,
            feature_count: None,
            reset_counter: None,
            extras_of: None,
        })
    }
}

impl Estimator for OpticalFlowEstimator {
    fn estimator_id(&self) -> &'static str {
        "optical_flow"
    }
    fn output_mode(&self) -> OutputMode {
        OutputMode::OpticalFlow
    }
    fn step(&mut self, inputs: &StepInputs<'_>) -> Option<EstimatorOutput> {
        let distance = inputs.range_reading.map(|r| r.distance_m);
        let label = distance.map(|_| ScaleSourceLabel::Rangefinder);
        self.run(inputs, distance, label)
    }
}

// ---------------------------------------------------------------------------
// Optical flow degraded (rangefinder-free)
// ---------------------------------------------------------------------------

/// Rangefinder-free OF. The same tracker, but scale comes from the
/// [`ScaleLadder`] and the raw quality is multiplied by the rung's
/// quality factor so the EKF auto-de-weights degraded rungs. Sitting on
/// the static rung is `degraded` regardless of raw quality.
pub struct OpticalFlowDegradedEstimator {
    inner: OpticalFlowEstimator,
    ladder: Option<std::sync::Arc<ScaleLadder>>,
    quality_gate: i32,
    state: EstimatorState,
    seen_converged: bool,
}

impl OpticalFlowDegradedEstimator {
    pub fn new(quality_gate: i32, ladder: Option<std::sync::Arc<ScaleLadder>>) -> Self {
        Self {
            inner: OpticalFlowEstimator::new(quality_gate),
            ladder,
            quality_gate,
            state: EstimatorState::Init,
            seen_converged: false,
        }
    }

    pub fn state(&self) -> EstimatorState {
        self.state
    }
}

impl Estimator for OpticalFlowDegradedEstimator {
    fn estimator_id(&self) -> &'static str {
        "optical_flow_degraded"
    }
    fn output_mode(&self) -> OutputMode {
        OutputMode::OpticalFlow
    }

    fn step(&mut self, inputs: &StepInputs<'_>) -> Option<EstimatorOutput> {
        let pick = self.ladder.as_ref().map(|l| l.pick(monotonic_ns()));
        let distance = pick.map(|p| p.distance_m);
        let label = pick.map(|p| match p.source {
            ScaleRung::Baro => ScaleSourceLabel::Baro,
            ScaleRung::Gps => ScaleSourceLabel::Gps,
            // The static rung is reported as baro (the physical fallback
            // expectation); the quality multiplier flags it degraded.
            ScaleRung::Static => ScaleSourceLabel::Baro,
        });

        let mut sample = self.inner.run(inputs, distance, label)?;

        // Apply the rung's quality multiplier; recompute the visible
        // state from the scaled quality so the operator's "converged?"
        // reflects the signal-to-noise the EKF will actually see.
        let on_static = matches!(pick.map(|p| p.source), Some(ScaleRung::Static));
        let mult = match pick {
            Some(p) => p.quality_multiplier,
            None => 0.2, // no healthy rung: penalize like static
        };
        let raw = sample.flow_quality.unwrap_or(0);
        let scaled = ((raw as f32 * mult).round() as i32).clamp(0, 255);
        sample.flow_quality = Some(scaled);
        sample.flow_distance_m = distance;
        if pick.is_none() {
            sample.flow_scale_source = None;
        }

        if scaled >= self.quality_gate {
            if on_static {
                self.state = EstimatorState::Degraded;
            } else {
                self.state = EstimatorState::Converged;
                self.seen_converged = true;
            }
        } else if self.seen_converged || on_static {
            self.state = EstimatorState::Degraded;
        } else {
            self.state = EstimatorState::Init;
        }
        sample.state = self.state;
        Some(sample)
    }
}

// ---------------------------------------------------------------------------
// VIO (vendor-binary backed)
// ---------------------------------------------------------------------------

/// Heartbeat watchdog for the vendor binary. Returns true from
/// [`check`](HeartbeatWatcher::check) when the engine has been silent
/// past the grace AND the restart cooldown has elapsed.
pub struct HeartbeatWatcher {
    cooldown_ns: i64,
    last_restart_ns: Option<i64>,
}

impl HeartbeatWatcher {
    /// `grace_s` lives on the [`VioEngine`] itself (it owns the `alive`
    /// timestamp); the watcher only enforces the restart cooldown so a
    /// bad config cannot spawn an unbounded restart storm.
    pub fn new(_grace_s: f64, cooldown_s: f64) -> Self {
        Self {
            cooldown_ns: (cooldown_s * 1e9) as i64,
            last_restart_ns: None,
        }
    }

    /// Decide whether a restart should fire now. `alive` is the engine's
    /// liveness at `now_ns` (from `VioEngine::is_alive`).
    pub fn should_restart(&mut self, alive: bool, now_ns: i64) -> bool {
        if alive {
            return false;
        }
        if let Some(last) = self.last_restart_ns {
            if now_ns - last < self.cooldown_ns {
                return false;
            }
        }
        self.last_restart_ns = Some(now_ns);
        true
    }
}

/// VIO estimator: delegates to a [`VioEngine`] vendor binary. Pose
/// returns become `vio`-path samples (position + velocity + covariance
/// + feature count + reset counter).
pub struct VioEstimator {
    id: &'static str,
    engine: VioEngine,
    config: EngineConfig,
    install_dir: String,
    watchdog: HeartbeatWatcher,
    state: EstimatorState,
    started: bool,
}

impl VioEstimator {
    pub fn new(
        id: &'static str,
        engine: VioEngine,
        config: EngineConfig,
        install_dir: String,
    ) -> Self {
        Self {
            id,
            engine,
            config,
            install_dir,
            watchdog: HeartbeatWatcher::new(2.0, 5.0),
            state: EstimatorState::Off,
            started: false,
        }
    }

    pub fn state(&self) -> EstimatorState {
        self.state
    }

    fn pose_to_output(&self, pose: &PoseMessage) -> EstimatorOutput {
        let (roll, pitch, yaw) = quat_to_euler(pose.orientation_quat);
        EstimatorOutput {
            timestamp_us: pose.ts_us as i64,
            output_mode: OutputMode::Vio,
            state: EstimatorState::from_vendor(&pose.state),
            flow_rate_x: None,
            flow_rate_y: None,
            flow_rate_z: None,
            flow_quality: None,
            flow_distance_m: None,
            flow_scale_source: None,
            integration_time_us: None,
            pose: Some((
                pose.position.0,
                pose.position.1,
                pose.position.2,
                roll,
                pitch,
                yaw,
            )),
            velocity: Some(pose.velocity),
            covariance: if pose.covariance.is_empty() {
                None
            } else {
                Some(pose.covariance.clone())
            },
            feature_count: Some(pose.feature_count),
            reset_counter: Some(pose.reset_counter),
            extras_of: None,
        }
    }
}

impl Estimator for VioEstimator {
    fn estimator_id(&self) -> &'static str {
        self.id
    }
    fn output_mode(&self) -> OutputMode {
        OutputMode::Vio
    }

    fn configure(&mut self) {
        if self.started {
            return;
        }
        // The spawn authorizer is a no-op here: the pipeline performs the
        // host-side process.spawn authorization before constructing this
        // estimator (see pipeline::build_vio_estimator). If the binary is
        // missing or the start fails, the engine stays not-started and
        // step() returns None forever, which fails safe (no wrong pose).
        let authorize = Box::new(|_b: &str, _a: &[String]| Ok(()));
        match self.engine.start(&self.config, &self.install_dir, authorize) {
            Ok(()) => {
                self.started = true;
                self.state = EstimatorState::Init;
            }
            Err(e) => {
                eprintln!("vision-nav: vio engine start failed: {e}");
                self.state = EstimatorState::Failed;
            }
        }
    }

    fn shutdown(&mut self) {
        self.engine.stop();
        self.state = EstimatorState::Off;
        self.started = false;
    }

    fn step(&mut self, inputs: &StepInputs<'_>) -> Option<EstimatorOutput> {
        if !self.engine.started() {
            return None;
        }
        // Forward the time-aligned IMU sample.
        if let Some(s) = inputs.imu_sample {
            let _ = self.engine.send_imu(
                (s.ts_ns / 1000) as u64,
                (s.xgyro, s.ygyro, s.zgyro),
                (s.xacc, s.yacc, s.zacc),
            );
        }
        // Bridge the shared frame into the SHM ring.
        if let Some(f) = inputs.curr_vio_frame {
            let _ = self.engine.send_frame(
                f.ts_us,
                f.width,
                f.height,
                f.stride,
                f.pixel_format,
                &f.pixels,
            );
        }
        self.engine.poll();

        let now = monotonic_ns();
        if self.watchdog.should_restart(self.engine.is_alive(now), now) {
            // Tear the engine down and try to bring it back up. A
            // restart tick produces no pose (fail safe: no stale pose).
            self.engine.stop();
            self.started = false;
            self.state = EstimatorState::Init;
            let authorize = Box::new(|_b: &str, _a: &[String]| Ok(()));
            match self
                .engine
                .start(&self.config, &self.install_dir, authorize)
            {
                Ok(()) => self.started = true,
                Err(e) => {
                    eprintln!("vision-nav: vio engine restart failed: {e}");
                    self.state = EstimatorState::Failed;
                }
            }
            return None;
        }

        let poses = self.engine.drain_poses();
        let pose = poses.last()?;
        self.state = EstimatorState::from_vendor(&pose.state);
        Some(self.pose_to_output(pose))
    }
}

// ---------------------------------------------------------------------------
// Hybrid (OF + VIO side by side)
// ---------------------------------------------------------------------------

/// Hybrid mode: runs an OF child + a VIO child. The primary sample
/// carries the VIO pose; the OF sample rides in `extras_of` so one tick
/// emits on both MAVLink components. The combined state is the worse of
/// the two children.
pub struct HybridEstimator {
    of: OpticalFlowEstimator,
    vio: Box<dyn Estimator>,
    state: EstimatorState,
}

impl HybridEstimator {
    pub fn new(of: OpticalFlowEstimator, vio: Box<dyn Estimator>) -> Self {
        Self {
            of,
            vio,
            state: EstimatorState::Off,
        }
    }
}

impl Estimator for HybridEstimator {
    fn estimator_id(&self) -> &'static str {
        "hybrid_of_plus_vio"
    }
    fn output_mode(&self) -> OutputMode {
        OutputMode::Vio
    }

    fn configure(&mut self) {
        self.vio.configure();
    }
    fn shutdown(&mut self) {
        self.of.shutdown();
        self.vio.shutdown();
    }

    fn step(&mut self, inputs: &StepInputs<'_>) -> Option<EstimatorOutput> {
        let of_out = self.of.step(inputs);
        let vio_out = self.vio.step(inputs);

        let of_state = of_out.as_ref().map(|o| o.state).unwrap_or(EstimatorState::Off);
        let vio_state = vio_out
            .as_ref()
            .map(|o| o.state)
            .unwrap_or(EstimatorState::Off);
        self.state = worse_state(of_state, vio_state);

        if let Some(mut v) = vio_out {
            v.state = self.state;
            v.extras_of = of_out.map(Box::new);
            return Some(v);
        }
        // No VIO sample yet; emit the OF half alone this tick.
        if let Some(mut o) = of_out {
            o.state = self.state;
            return Some(o);
        }
        None
    }
}

/// State priority: lower is worse. The hybrid combined state is the
/// worse of the two children.
fn worse_state(a: EstimatorState, b: EstimatorState) -> EstimatorState {
    fn prio(s: EstimatorState) -> u8 {
        match s {
            EstimatorState::Failed => 0,
            EstimatorState::Degraded => 1,
            EstimatorState::Off => 2,
            EstimatorState::Init => 3,
            EstimatorState::Converging => 4,
            EstimatorState::Converged => 5,
        }
    }
    if prio(a) <= prio(b) {
        a
    } else {
        b
    }
}

/// Quaternion `(w, x, y, z)` -> Euler `(roll, pitch, yaw)`, aerospace
/// ZYX.
fn quat_to_euler(q: (f32, f32, f32, f32)) -> (f32, f32, f32) {
    let (w, x, y, z) = q;
    let sinr_cosp = 2.0 * (w * x + y * z);
    let cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    let roll = sinr_cosp.atan2(cosr_cosp);
    let sinp = 2.0 * (w * y - z * x);
    let pitch = if sinp.abs() >= 1.0 {
        std::f32::consts::FRAC_PI_2.copysign(sinp)
    } else {
        sinp.asin()
    };
    let siny_cosp = 2.0 * (w * z + x * y);
    let cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    let yaw = siny_cosp.atan2(cosy_cosp);
    (roll, pitch, yaw)
}

/// Default quality gate.
pub fn default_quality_gate() -> i32 {
    DEFAULT_QUALITY_GATE
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::estimator::RangeReading;
    use crate::framing::GrayImage;

    fn textured(w: u32, h: u32, sx: i32, sy: i32) -> GrayImage {
        let mut data = vec![0u8; (w * h) as usize];
        for y in 0..h as i32 {
            for x in 0..w as i32 {
                let px = x - sx;
                let py = y - sy;
                data[(y as u32 * w + x as u32) as usize] = (((px * 37 + py * 17) & 0x3f) * 4) as u8;
            }
        }
        GrayImage {
            width: w,
            height: h,
            data,
        }
    }

    fn step_inputs<'a>(
        prev: &'a GrayImage,
        curr: &'a GrayImage,
        range: Option<RangeReading>,
    ) -> StepInputs<'a> {
        StepInputs {
            prev_gray: Some(prev),
            curr_gray: Some(curr),
            curr_vio_frame: None,
            dt_seconds: 1.0 / 30.0,
            gyro: None,
            imu_sample: None,
            range_reading: range,
        }
    }

    #[test]
    fn registry_lists_all_six_modes() {
        let modes = available_estimators();
        assert_eq!(modes.len(), 6);
        assert!(modes.contains(&"hybrid_of_plus_vio"));
        assert!(modes.contains(&"off"));
    }

    #[test]
    fn null_estimator_emits_none_shape() {
        let mut e = NullEstimator;
        let prev = textured(32, 32, 0, 0);
        let curr = textured(32, 32, 1, 0);
        let out = e.step(&step_inputs(&prev, &curr, None)).unwrap();
        assert_eq!(out.output_mode, OutputMode::None);
        assert_eq!(out.state, EstimatorState::Off);
    }

    #[test]
    fn optical_flow_converges_on_good_tracking() {
        let mut e = OpticalFlowEstimator::new(default_quality_gate());
        let prev = textured(128, 128, 0, 0);
        let curr = textured(128, 128, 2, 1);
        let out = e
            .step(&step_inputs(&prev, &curr, Some(RangeReading { distance_m: 1.5, quality: 90 })))
            .unwrap();
        assert_eq!(out.output_mode, OutputMode::OpticalFlow);
        assert_eq!(out.flow_scale_source, Some(ScaleSourceLabel::Rangefinder));
        // A richly-textured 128x128 frame yields many tracked features.
        assert!(out.flow_quality.unwrap() >= default_quality_gate());
        assert_eq!(out.state, EstimatorState::Converged);
    }

    #[test]
    fn degraded_static_rung_is_degraded_state() {
        use std::sync::Arc;
        let ladder = Arc::new(ScaleLadder::new(false)); // no messages -> static rung
        let mut e = OpticalFlowDegradedEstimator::new(default_quality_gate(), Some(ladder));
        let prev = textured(128, 128, 0, 0);
        let curr = textured(128, 128, 2, 0);
        let out = e.step(&step_inputs(&prev, &curr, None)).unwrap();
        assert_eq!(out.output_mode, OutputMode::OpticalFlow);
        // Static rung -> degraded regardless of raw quality.
        assert_eq!(out.state, EstimatorState::Degraded);
        // Quality is multiplied by the 0.2 static factor.
        assert!(out.flow_quality.unwrap() <= (255.0 * 0.2) as i32 + 1);
    }

    #[test]
    fn degraded_baro_rung_can_converge() {
        use std::sync::Arc;
        let ladder = Arc::new(ScaleLadder::new(false));
        ladder.on_global_position(2000, monotonic_ns()); // fresh relative_alt
        let mut e = OpticalFlowDegradedEstimator::new(10, Some(ladder));
        let prev = textured(160, 160, 0, 0);
        let curr = textured(160, 160, 2, 1);
        let out = e.step(&step_inputs(&prev, &curr, None)).unwrap();
        assert_eq!(out.flow_scale_source, Some(ScaleSourceLabel::Baro));
        // A low gate + a non-static rung lets it converge.
        if out.flow_quality.unwrap() >= 10 {
            assert_eq!(out.state, EstimatorState::Converged);
        }
    }

    #[test]
    fn watchdog_respects_cooldown() {
        let mut w = HeartbeatWatcher::new(2.0, 5.0);
        // Alive -> never restarts.
        assert!(!w.should_restart(true, 0));
        // Dead -> restarts once.
        assert!(w.should_restart(false, 1_000_000_000));
        // Dead again within cooldown -> suppressed.
        assert!(!w.should_restart(false, 2_000_000_000));
        // Dead after cooldown -> restarts again.
        assert!(w.should_restart(false, 7_000_000_000));
    }

    #[test]
    fn worse_state_picks_the_weaker_child() {
        assert_eq!(
            worse_state(EstimatorState::Converged, EstimatorState::Degraded),
            EstimatorState::Degraded
        );
        assert_eq!(
            worse_state(EstimatorState::Init, EstimatorState::Converged),
            EstimatorState::Init
        );
        assert_eq!(
            worse_state(EstimatorState::Failed, EstimatorState::Converged),
            EstimatorState::Failed
        );
    }

    #[test]
    fn quat_euler_identity_is_zero() {
        let (r, p, y) = quat_to_euler((1.0, 0.0, 0.0, 0.0));
        assert!(r.abs() < 1e-6 && p.abs() < 1e-6 && y.abs() < 1e-6);
    }
}
