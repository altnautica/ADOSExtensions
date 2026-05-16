# Changelog

All notable changes to the Vision Navigation extension.

## [Unreleased]

### Added

- **In-app calibration wizard.** Tapping Calibrate on the sensors
  card now opens a seven-step guided flow inside the Vision Nav tab:
  target check (with the bundled AprilGrid PDF), live camera preview
  with tag-corner overlay, per-frame quality-gated capture (sharpness
  + tag count + tag-area span + exposure), IMU motion segment with
  live gyro and accel sparklines, submit, wait with substep progress
  driven by agent heartbeat events, and a verify-and-compare result
  page that diffs the new intrinsics against any previously-loaded
  calibration. Apply persists the camchain to the plugin data
  directory and applies the new timeshift to the live time aligner
  on the next tick.
- **Pose coverage map.** Captured frames are scored for pose
  diversity (tilt + rotation buckets); a 5x5 heatmap surfaces which
  view zones still need coverage before the wizard advances.
- **Agent-side calibration runner.** New
  `altnautica_vision_nav.calibration.runner` module decodes the
  captured frame bundle, runs `cv2.aruco` AprilTag detection,
  `cv2.calibrateCamera` for monocular pinhole + radial-tangential
  intrinsics, and a golden-section search over the candidate
  timeshift band to fit the joint camera-IMU offset against the
  recorded IMU window. Substeps publish progress events back to the
  wizard live (`tag_detection`, `intrinsics_solve`,
  `extrinsics_solve`, `timeshift_solve`, `complete`).
- **Two new event topics.**
  `com.altnautica.vision-nav.start_calibration` carries the captured
  frame bundle + the IMU recording window from the wizard to the
  agent. `com.altnautica.vision-nav.calibration_progress` and
  `com.altnautica.vision-nav.calibration_complete` are the agent's
  return path with substep progress and the final result.
- **Calibration quality scorer + pose clusterer.** New TS modules
  under `gcs/src/calibration/` produce the per-frame GOOD / OK /
  DROP verdicts and the pose-diversity buckets the wizard renders.
  Pure functions so the demo-mode harness can drive them with
  synthetic signals.

### Changed

- `SensorsCard` Calibrate CTA now opens the in-app wizard instead
  of the YAML file picker. The Kalibr YAML upload path is still
  honoured for operators with an existing `camchain.yaml` (the
  agent's `upload_calibration` event accepts the same wire shape it
  always has).
- Agent dependency bumped from `opencv-python-headless` to
  `opencv-contrib-python-headless` to bring in the aruco module the
  calibration runner needs.

## [vision-nav-v0.1.x]

The plugin's estimator framework is now modular. Optical flow, the
rangefinder-free degraded mode, and two visual-inertial odometry
engines (OpenVINS and VINS-Fusion) all plug in behind one
`BaseEstimator` ABC. A hybrid mode runs an OF estimator and a VIO
estimator side-by-side. The VIO engines spawn vendor binaries via
the plugin host's subprocess sandbox; the Python plumbing, GCS
surfaces, pre-arm gate, and component router ship in this release.
Vendor-binary builds and on-rig validation are pending.

### Added

- **Estimator framework.** `BaseEstimator` ABC + `EstimatorOutput`
  dataclass + `ESTIMATOR_REGISTRY` that maps each config mode to a
  concrete estimator class. Adding a new estimator is a single file
  plus a registry entry.
- **`optical_flow_degraded` mode.** Runs the same Lucas-Kanade
  tracker but pulls scale from a four-rung baro/GPS/static fallback
  ladder when no rangefinder is wired. Per-rung quality multipliers
  (0.7 / 0.6 / 0.4 / 0.2) so the EKF auto-de-weights degraded scale
  sources.
- **`vio_openvins` and `vio_vins_fusion` modes.** Monocular visual-
  inertial odometry. The Python shim spawns a vendor binary
  (`ados_openvins_shim` or `ados_vins_fusion_shim`) via the plugin
  host's `process.spawn` allowlist; frames flow through a shared-
  memory ring and IMU + pose messages flow through a Unix-domain
  socket encoded as length-prefixed msgpack. Heartbeat watchdog
  restarts the binary on silence.
- **`hybrid_of_plus_vio` mode.** Runs an OF child + a VIO child on
  independent cameras. The component router emits on both MAVLink
  components in parallel; the combined state is the worse of the
  two child estimators.
- **Full IMU subscription.** The new `imu/` package replaces the
  gyro-only tap with a full gyro+accel reader. Prefers
  `SCALED_IMU2` (100 Hz) when the FC publishes it; falls back to
  `RAW_IMU`. A `TimeAligner` pairs each camera frame with the
  closest IMU sample and tracks the rolling residual offset.
- **Camera-IMU time-sync drift bands.** Green (≤10 ms residual),
  yellow (10 to 30 ms), red (>30 ms). VIO pre-arm refuses to arm
  when the band is red; the GCS sensors card colour-codes the live
  residual.
- **Calibration loaders.** Kalibr-compatible `camchain.yaml` loaders
  for both intrinsics (pinhole + radtan / equidistant / none) and
  camera-IMU extrinsics (SE(3) transform + scalar time offset).
  Both layouts accepted (`cam0` wrapper or bare block); multi-camera
  files accepted with the extra cameras ignored.
- **`VISION_POSITION_ESTIMATE` MAVLink emission.** The component
  router emits message 102 on MAVLink component 197
  (`MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`) at 30 Hz when a VIO mode
  is active. ArduPilot's EKF3 fuses these when `EK3_SRC1_*` are set
  to `ExternalNav`.
- **Pre-arm gate.** Mode-aware arm-readiness evaluator. For OF
  modes: companion active + flow quality ≥ 50 + rangefinder or
  scale source. For VIO modes: companion active + estimator
  converged + intrinsics + extrinsics + sync offset ≤ 30 ms +
  feature count ≥ 20. Hybrid runs both check sets. Heartbeat
  surfaces the full per-check report so the GCS pre-arm card
  renders the live status.
- **GCS mode picker.** The Navigation tab now hosts a segmented
  mode picker filtered against the agent's `availableEstimators`.
  Operator selects the mode at runtime; the heartbeat updates on
  the next tick.
- **GCS sensors card.** Camera row (device + resolution + FPS +
  Calibrate CTA), IMU row (source + rate + sync-offset pill),
  rangefinder row (driver + last reading + freshness).
- **GCS estimator card.** Engine + state pill + flow quality + VIO
  rows (features, drift, resets) + sync offset trend.
- **GCS telemetry charts.** Inline-SVG sparklines for flow quality,
  sync offset, feature count, and drift over a rolling 60-second
  window.
- **GCS fallback banner.** Appears when the estimator is degraded
  or failed. Names the most likely cause (low flow quality, sync
  drift, missing scale source, low feature count) and suggests a
  concrete next action.
- **Mode-aware fleet pill.** The Mission Control drone-card pill
  reads the active mode and renders the right short label: "OF",
  "OF*" (degraded), "VIO", or "Hybrid". Falls back to "GPS-denied"
  for older agents that predate the mode field.
- **Heartbeat schema additive fields.** `mode`,
  `availableEstimators`, `estimatorState`, `flowScaleSource`,
  `imuSource`, `imuRateHz`, `cameraImuSyncOffsetMs`,
  `cameraIntrinsicsLoaded`, `estimatorFeatureCount`,
  `estimatorDriftEstimateM`, `preArmReport`. All optional; older
  GCS instances ignore them without breaking.

### Changed

- The MAVLink `EkfSourceSwitcher` now strict-gates the VIO button
  on `vioSupported`. The button is hidden, not greyed, when no VIO
  engine is wired so the operator never sees an option that
  silently fails.
- The `mode` config field grew from `off | optical_flow` to all six
  modes.
- `max_ram_mb` raised to 512 (was 256) to accommodate the VIO
  engines when active.
- The plugin now declares `contains_vendor_binary: true` with a
  `vendor_attribution` block covering OpenVINS and VINS-Fusion per
  GPL §6.

### Documentation

Four new pages on `docs.altnautica.com`:

- [Modes](https://docs.altnautica.com/drone-agent/vision-nav-modes)
  — per-mode breakdown + pre-arm matrix + fallback ladder.
- [Calibration](https://docs.altnautica.com/drone-agent/vision-nav-calibration)
  — Kalibr YAML format + capture workflow + time-sync drift bands.
- [Troubleshooting](https://docs.altnautica.com/drone-agent/vision-nav-troubleshooting)
  — decision trees for the four common failure families.
- [Architecture](https://docs.altnautica.com/drone-agent/vision-nav-architecture)
  — module map + EstimatorOutput contract + IPC protocol + "adding
  a new estimator" recipe.

### Known limitations

- Vendor binaries (`ados_openvins_shim`, `ados_vins_fusion_shim`)
  ship from a separate CI job. Without them the plugin runs the
  OF modes; selecting a `vio_*` mode without the binary surfaces a
  clear error.
- On-rig validation is pending. Behaviour under real flight loads
  will be characterised in a published matrix before the modes
  are marked validated. The documentation does not claim
  bench-validation for any case until the matrix lands.

## [0.1.0] - 2026-05-16

Initial release. Optical flow capture and MAVLink emission for
ArduPilot and PX4 GPS-denied flight.

### Added

- **Optical flow capture pipeline.** Reads frames from a USB UVC or
  CSI downward camera at 30 Hz, downsamples to 320x240 grayscale, and
  runs a pyramidal Lucas-Kanade tracker against the previous frame.
  Produces an angular flow vector per frame.
- **Rangefinder drivers.** Four drivers ship in this release:
  - `tf-luna` over UART. Benewake-family LiDAR, 0.2 to 8 m range.
  - `lidar-lite-v3` over I2C. Garmin LIDAR-Lite v3, 0.05 to 40 m.
  - `vl53l1x` over I2C. ST VL53L1X time-of-flight, 0.04 to 4 m.
  - `mavlink-relay`. Reads the FC's `DISTANCE_SENSOR` MAVLink message
    when the rangefinder is wired to the flight controller instead
    of the companion.
- **MAVLink `OPTICAL_FLOW_RAD` emission.** The plugin emits
  `OPTICAL_FLOW_RAD` (msg id 106) to the FC at 10 Hz with the
  body-frame radian convention both ArduPilot and PX4 expect. Emits
  carry a per-sample quality score (0 to 255) derived from the
  tracker's feature-confidence aggregate.
- **MAVLink component id 198.** The plugin registers itself as a
  peripheral on MAVLink component 198 (the optical-flow companion
  convention). The agent's MAVLink router accepts the registration
  via the `mavlink.component.vio` capability.
- **Time synchronization.** The plugin subscribes to the FC's
  `TIMESYNC` exchange and produces accurate timestamps on every
  emitted `OPTICAL_FLOW_RAD` so the EKF's delay-buffer fusion stays
  consistent across companion and FC clocks.
- **Pre-arm helper.** A one-shot helper batches the FC parameter
  writes for vision-only flight: `FLOW_TYPE`, `EK3_SRC1_VELXY`,
  `EK3_FLOW_DELAY`, `EK3_FLOW_QUAL_MIN`, `RNGFND1_*` on ArduPilot;
  `EKF2_OF_CTRL`, `EKF2_OF_DELAY`, `EKF2_HGT_REF`, `EKF2_OF_POS_*`
  on PX4. Verifies each write read back correctly and reports
  partial-success cleanly.
- **`SET_GPS_GLOBAL_ORIGIN` auto-dispatch.** When the EKF reports
  "waiting for home" and no GPS source is configured, the plugin
  dispatches `SET_GPS_GLOBAL_ORIGIN` so the local position frame
  initializes without operator intervention.
- **GCS Navigation tab.** A per-drone Navigation tab on the Mission
  Control drone detail panel renders live optical flow rate, flow
  quality, rangefinder reading and health, the active EKF source
  set, and a four-card arm-readiness summary.
- **EKF source-set switcher (ArduPilot).** The Navigation tab exposes
  a runtime switch between EKF source sets via
  `MAV_CMD_SET_EKF_SOURCE_SET` (command id 42007). PX4 firmwares show
  the switcher disabled with a tooltip pointing at the parameter
  path.
- **Health cards.** Four arm-readiness cards (camera, rangefinder,
  EKF position, FC armable) light up green when each check passes.
  Operators can read the panel from a glance to know whether
  vision-only flight is safe to arm.
- **Heartbeat extras.** The plugin appends `vision.flowQuality`,
  `vision.flowRateRad`, `vision.rangefinderHealth`, and
  `vision.activeSourceSet` to the agent heartbeat so the fleet view
  can show vision health without polling the per-drone tab.

### Supported platforms

- ADOS Drone Agent 0.13 or newer.
- ADOS Mission Control 0.19 or newer.
- ArduPilot 4.5 or newer (Copter, Plane, Rover).
- PX4 1.14 or newer.
- Boards: Radxa ROCK 5C Lite, Radxa CM4 (RK3588S2), Rockchip RK3576,
  Raspberry Pi 5, Raspberry Pi CM5, Raspberry Pi CM4, Raspberry Pi
  Zero 2 W.

### Known limitations

- Visual inertial odometry (VIO) is not in this release. Optical
  flow only.
- Stereo cameras are unsupported.
- The plugin requires a rangefinder; fully visual-only operation is
  not supported.
- Production-flight validation has not yet been performed. Behaviour
  under real flight loads will be characterised in a published
  validation matrix before the next release.
