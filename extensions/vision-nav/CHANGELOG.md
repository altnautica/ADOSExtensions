# Changelog

All notable changes to the Vision Navigation extension.

## [0.8.0]

### Changed

- External navigation: the `viso_delay` pre-arm row says when poses are
  older than the 250 ms ArduPilot's EKF3 can place in time. EKF3 fuses every
  pose at its arrival minus `VISO_DELAY_MS`, and that delay is capped at
  250 ms, so older poses are fused late by the difference: at 2 m/s a
  450 ms pose against a 135 ms delay puts the fix 0.6 m behind, which the
  position loop turns into a slow sway. The row asks for a faster pose
  pipeline instead of a delay the flight controller cannot take. The
  `VISO_DELAY_MS` manifest note and the module docs now describe it as the
  typical age at arrival, not the least.
- External navigation fuses the optical-flow sensor. With `FLOW_TYPE` set,
  the `fc_params` pre-arm row recommends flow velocity in the external-nav
  source set (`EK3_SRC1_VELXY 5`) without holding the arm: SLAM-only
  velocity stays a valid choice where the floor is too dark for flow. Flow
  velocity without a flow sensor blocks, and so does a GNSS-denied
  fallback set on flow (`EK3_SRC2_VELXY 5`) with `FLOW_TYPE` 0. EKF3 fuses
  flow beside the external position in one set; the flow velocity is
  measured rather than differenced from late poses, so the EKF holds
  velocity between poses. In a SITL loop with poses as late as a loaded
  companion sends them (about 5 Hz, 150 to 900 ms old), flow velocity cut
  the velocity tracking error from 0.84 to 0.50 m/s RMS and the worst
  cross-track from 2.7 to 1.2 m. `FLOW_TYPE` is read with the other
  parameters, the manifest asks for `EK3_SRC1_VELXY 5` and a downward
  rangefinder (`RNGFND1_TYPE` 10 for a MAVLink flow-and-range module,
  otherwise the flight controller's own rangefinder type), and the
  parameter card lists the rangefinder.
- External navigation: the `ODOMETRY` position covariance follows the
  pose's own horizontal uncertainty (`sigma_xy_m` on the pose topic) where
  it is larger than the health's σ, and `quality` drops to 30 while it is
  over 1 m. A planar scan matcher that rides its motion prior unmatched (a
  climb, a featureless stretch) reports a σ that grows with the distance
  the prior carries it, so the EKF trusts that pose less instead of fusing
  it as good to 5 cm. A pose without `sigma_xy_m` keeps the health's σ.

## [0.7.5]

### Changed

- External navigation: the pre-arm blocks `EK3_SRCn_VELXY` 6 on the
  external-nav source set. ArduPilot ignores the `ODOMETRY` velocity
  covariance and fuses the velocity as good to `VISO_VEL_M_NSE` (0.1 m/s by
  default), while the velocity sent is a difference of poses that a planar
  lidar SLAM leaves about 1 m/s off in fast manoeuvres; over-trusted, it
  tipped the EKF's attitude until its variance failsafe landed the drone.
  Set 0 (position only). The parameter is now read with the others.
- External navigation: `EK3_SRCn_YAW` 6 (yaw from the pose) passes the
  pre-arm when the pose says its map's north is declared or fitted to GNSS
  (the pose's `north`), and still blocks when that north follows the flight
  controller's own heading, where it would only echo the EKF's yaw back.
  Before a pose arrives the check waits.

## [0.7.4]

### Fixed

- For navigation, nav.state and engage, the last report delivered again
  after an older one was dropped (a failed poll falling back to a slower
  source) is still ignored, so a feed that has stopped reads stale 2000 ms
  after its last report arrived instead of staying live.

## [0.7.3]

### Fixed

- An older navigation block, nav state or engage state that arrives after
  a newer one (the host falling back to a slower source after a failed
  poll) no longer replaces the newer value, keeps the feed live or adds a
  trend sample.

## [0.7.2]

### Fixed

- A live navigation block, nav state or engage state no longer reads
  stale when the drone clock runs behind or a poll is slow. Only the
  first value after the tab opens or the feed resumes is aged by the
  drone's timestamp; while values keep arriving, each is aged from its
  arrival.

## [0.7.1]

### Fixed

- An old navigation block, nav state or engage state no longer shows as
  live when the tab opens. The tab ages each value from the time the
  drone recorded it, so a value already two seconds old reads "No data"
  with its age from the first paint.
- The same report delivered twice no longer adds a second point to the
  trend charts or makes the value look newer.
- The rangefinder pill turns red when pre-arm blocks the rangefinder,
  for example a reading outside its configured range, instead of
  showing "Healthy" because a reading is present.
- "Mission Control did not grant navigation telemetry" now appears only
  when the permission is refused. Any other subscription failure says
  "Could not subscribe to navigation telemetry."

## [0.7.0]

### Added

- The Vision Nav tab shows when vision navigation is disengaged: with the
  Engage skill off, the Mode card says so and that Off is the effective
  mode, and points at the Engage skill (Shift+N).
- A Diagnostics row in the Sensors card shows the rangefinder error, the
  count of failed MAVLink sends, and each input the plugin could not read
  (for example "Cannot read RAW_IMU from the flight controller"), in words.
- The ArduPilot parameter card lists the external navigation parameters
  (`VISO_TYPE`, `VISO_DELAY_MS`, `EK3_SRC1_YAW`, `AHRS_OPTIONS`,
  `AHRS_ORIGIN_*`, `COMPASS_AUTODEC`, `EK3_MAG_EF_LIM`, `ARMING_SKIPCHK`) and
  the obstacle feed parameter (`PRX1_TYPE`), so every parameter the manifest
  requires is on the card.

### Changed

- Values in the tab count as live for 2 s after they arrive. After that the
  cards dim, a line says how long the agent has been silent, and the
  companion computer, estimator, pre-arm and external-nav pills read
  "No data · N s" instead of the last state. The age updates every second.
- Before the first navigation data the pre-arm card says it is waiting for
  the agent, and when Mission Control does not grant navigation telemetry it
  says that, instead of reporting the companion as down.
- The rangefinder pill shows the rangefinder's health (healthy, no reading,
  error) rather than where it is wired; the wiring moved to the row's
  sublabel and the scale source to its own column.
- The navigation feed card uses the same words as the cockpit and the TUI:
  "Set 1", "Odometry 10 Hz", and GPS fix names including Static and PPP
  (an unknown fix type reads "Unknown").
- The IMU and scale sources read as words ("Flight controller IMU
  (RAW_IMU)", "Barometer"), and the pre-arm details from the agent are plain
  sentences ("Companion critical.", "Companion rangefinder, 1.25 m.",
  "Scale from the barometer.").
- The status pill says "Companion computer".
- The mode list is a read-only list with each mode's description shown
  under it, instead of button-like tiles with the description in a tooltip.
- The guided tour is a dialog: focus moves to Next, Escape skips it, and
  Replay tour, Skip and Next are at least 48 px for touch.
- Every string in the tab comes from `locales/en.json`.

### Fixed

- In Mission Control, where the panel cannot use browser storage, the tour
  no longer covers the tab on every visit; it opens from Replay tour.
- The trend charts add one sample per received update, so a steady flow
  quality draws a line instead of "no data", and a single sample shows as
  the current value.
- The pre-arm card's fallback for older agents passes flow quality at
  exactly 50, as the agent does.

## [0.6.7]

### Added

- Pre-arm `fc_params` also reads `EK3_MAG_EF_LIM` and blocks when
  `COMPASS_AUTODEC` is on and `EK3_MAG_EF_LIM` is 0. With declination
  learning on, the compass library sets its declination the first time the
  EKF has a position, after the yaw alignment. EKF3 ignores it while it
  uses its own table field at the origin (`EK3_MAG_EF_LIM` above 0, the
  default 50); at 0 the late step reaches the yaw. In SITL with the origin
  set at boot and a position from external navigation, the declination
  stepped 0 to 12.8° about 2 s after the yaw alignment: with the default
  limit the gyro bias stayed within 0.04 °/s, with the limit at 0 it
  reached −0.56 °/s. The manifest lists `EK3_MAG_EF_LIM` among the
  required parameters.

### Fixed

- The Mission Control panel no longer goes blank when the agent reports no
  flow reading: the flow card treated a null rate as a number.

## [0.6.6]

### Added

- The navigation feed card shows the pose age (`navigation.poseAgeMs`, the
  median age of a pose when its `ODOMETRY` is sent) beside the `ODOMETRY`
  rate, so the operator can compare it with `VISO_DELAY_MS` without waiting
  for the `viso_delay` pre-arm row.

## [0.6.5]

### Fixed

- The EKF origin is no longer sent late. ArduPilot aligns its yaw at boot
  with the declination it has then; an origin that arrives afterwards loads
  the site's declination and steps the earth field, which with
  `COMPASS_AUTODEC` on becomes a z-gyro bias that spins the vehicle, and
  ArduPilot accepts an origin while armed. `SET_GPS_GLOBAL_ORIGIN` now goes
  out only when the flight controller has reported none 5 s after its boot
  (a heartbeat after 3 s of silence is a new boot, which also forgets the
  old origin), the vehicle is disarmed, and GPS is not primary. The wait
  counts from the flight controller's boot, not the plugin's start.
- A pose step is judged against where the last velocity predicts the pose,
  not against the last pose. At more than 5 m/s every 10 Hz pose moved
  over the 0.5 m reset threshold, so each one bumped `reset_counter`,
  restarted the velocity window and no `ODOMETRY` went out.
- With `gps_primary`, a GPS that keeps its 3D fix but reports a horizontal
  accuracy (`GPS_RAW_INT.h_acc`) worse than 5 m counts as lost, so a jammed
  or degraded receiver hands over to the external-nav source set.
- `external_nav.origin_lla` is required only without `gps_primary`.

### Added

- Pre-arm `ekf_origin` names how to get an origin (`AHRS_OPTIONS` bit 4
  with `AHRS_ORIGIN_LAT/LON/ALT`, or waiting for the GPS), blocks when the
  origin the plugin sent arrived after a compass-yaw alignment with
  `COMPASS_AUTODEC` on, and blocks when a boot-time `AHRS_ORIGIN` is more
  than 1 km from `origin_lla`.
- Pre-arm `fc_params`: the plugin reads `VISO_TYPE`, `VISO_DELAY_MS`,
  `COMPASS_AUTODEC`, `AHRS_OPTIONS`, `AHRS_ORIGIN_LAT/LON` and the source-set
  parameters (`PARAM_REQUEST_READ` every 10 s while any is missing, again
  after a flight-controller reboot) and blocks on a `VISO_TYPE` other than
  1, an external-nav set whose `POSXY` is not 6 or whose `YAW` is 6 (north
  from the SLAM map), or a fallback set that is not GPS or optical flow.
- Pre-arm `viso_delay` and telemetry `navigation.poseAgeMs`: the median age
  of a pose when its `ODOMETRY` is sent, and a pending row while
  `VISO_DELAY_MS` is more than 50 ms from it.
- Required flight-controller parameters `AHRS_OPTIONS`, `AHRS_ORIGIN_*`,
  `COMPASS_AUTODEC` (before ArduPilot 4.7) and `ARMING_SKIPCHK`; the
  external-nav set's yaw is the compass (`EK3_SRC1_YAW 1`).
- GCS labels for the two new pre-arm rows.

## [0.6.4]

### Added

- The flight controller is asked for its own `OPTICAL_FLOW` at 20 Hz, beside
  `RAW_IMU` and `DISTANCE_SENSOR`, at start and every 30 s. Its flow sensor's
  rate (body rate taken out) times the downward range is the ground velocity
  the world-engine lidar SLAM (2.1.8) follows along a corridor its scans
  leave unconstrained. An FC without a flow sensor sends none.

## [0.6.3]

### Fixed

- `ODOMETRY` carries the time the pose describes, not the time it arrived.
  `time_usec` was the pose's receipt on the flight controller's clock and
  the velocity was differenced over receipt times, while a planar lidar
  pose describes its scan's first return, about 0.2 s before it arrives.
  The EKF fused a position, velocity and yaw that lagged the vehicle through
  every turn. Both now come from the pose's own stamp (`ts_ms`), taken at
  receipt when the pose has none and never more than 0.5 s back. ArduPilot
  maps `time_usec` onto its clock by the least delay it has seen, so
  `VISO_DELAY_MS` (now listed with the required parameters) should cover
  that least delay.

## [0.6.2]

### Fixed

- The flight controller is found from its HEARTBEAT as well as its RAW_IMU.
  With an FC that is not system 1 (a second SITL instance, or a board set to
  another system id) the plugin kept the 1/1 default: it asked the wrong
  system to stream RAW_IMU, so the address was never learned, and it dropped
  every COMMAND_ACK, resending the EKF source-set switch without end.

## [0.6.1]

### Fixed

- Optical flow reads frames from one camera, `camera.camera_id`, instead of
  every camera on the vision bus; with two cameras captured the frames of
  both reached the flow estimator. Empty keeps the old any-camera behaviour.
  `recommendedCameraId`, the Sensors card's camera row, reports that camera
  when it is set, and the device path otherwise.

## [0.6.0]

### Added

- **External navigation mode** (`external_nav`, ArduPilot): the pose another
  plugin publishes (by default the World Engine extension's planar lidar
  SLAM) goes to the flight controller as `ODOMETRY`, exactly one message per
  SLAM pose, with nothing interpolated between poses.
- **EKF source-set switching** (`MAV_CMD_SET_EKF_SOURCE_SET`): the external
  navigation set while the pose tracks and the fallback set when it is lost,
  or GPS first with external navigation taking over when the 3D fix is lost.
- **The EKF origin** through `SET_GPS_GLOBAL_ORIGIN` from
  `external_nav.origin_lla`, sent when the flight controller reports none.
- **`OBSTACLE_DISTANCE`** from the shared planar scan: 72 sectors of 5° at
  up to 10 Hz, for ArduPilot's proximity and avoidance.
- **The `nav.state` feed and its GCS card**: active source set, external
  navigation health, odometry and obstacle rates, nearest obstacle and GPS
  fix.

### Fixed

- A MAVLink send the host reports as not sent (`sent: false`, or no router)
  now counts in health `emitErrors` instead of passing as delivered.
- The Vision Nav tab keeps `emitErrors`, `rangefinderError`,
  `degradedReasons` and `fcParams` from the health block; the PX4 parameter
  card read `fcParams` and never received it.

## [0.5.0]

### Fixed

- **The agent now reads the per-drone config.** Mode, engage state,
  rangefinder, firmware, quality gate and outdoor mode come from the live
  per-drone store. The agent polls it once a second and restarts on a change,
  so turning the Engage Skill off stops emission within about a second. With
  no stored config the plugin defaults to mode `off` and emits nothing.
- **ArduPilot and iNav now receive optical flow.** They get `OPTICAL_FLOW`
  (#100) with the raw angular rate in `flow_rate_x/y`; PX4 keeps
  `OPTICAL_FLOW_RAD` (#106). The firmware in the plugin config picks the
  message.
- **PX4 flow scaling.** `OPTICAL_FLOW_RAD` now carries the angle swept over the
  interval (it carried a rate, about 30 times too large) and the gyro rotation
  over the interval in `integrated_x/y/zgyro`. The flow is no longer
  derotated by a field-of-view factor.
- An unknown distance is sent as -1 instead of 0 m.
- MAVLink subscriptions now decode the raw frames the host delivers, so the
  IMU, rangefinder relay, TIMESYNC and scale-ladder inputs receive data.
- **No frames for 2 s** marks the estimator failed and the companion critical
  instead of freezing the last state.
- **The rangefinder pre-arm check needs a fresh reading** inside the sensor's
  min-max range; a configured topology alone no longer passes.
- **The static scale fallback is reported as `static`**, not `baro`, and blocks
  the degraded mode's scale-source check.
- Health `flowQuality` clears when a tick has no sample instead of repeating
  the last value.
- Each outbound frame carries an incrementing sequence number.
- **Component 198 registers with kind `vio`.** The agent host reserves
  components 197 and 198 for kind `vio`, so the `peripheral` registration was
  refused and no flow frame reached the flight controller. The manifest now
  requests `mavlink.component.vio` instead of `mavlink.component.peripheral`
  and declares `component_kind: vio`.
- Losing or regaining camera frames publishes the health block at once
  instead of on the next 1 s tick, so the estimator `failed` state shows about
  2 s after the last frame.
- Health `imuRateHz` is the inverse of the smoothed sample interval. It
  averaged the per-pair rate before, so `RAW_IMU` frames arriving in bursts
  read as tens of kHz.

### Added

- `outdoor` config key: allows the GPS altitude rung, measured from the first
  good 3D fix (never AMSL).
- `active` and `flow_quality_min` in the config form; the schema defaults now
  match the agent's (rangefinder `fc` / `fc_relay`).
- The agent asks the flight controller for `RAW_IMU` at 100 Hz and
  `DISTANCE_SENSOR` at 20 Hz, at start and every 30 s.
- The TF-Luna reader runs on its own task with raw 8N1 at the configured baud,
  retries opening every 2 s, and reports an open failure in health
  `rangefinderError` (the pre-arm check then blocks).
- Health: `emitErrors` (failed sends), `degradedReasons` (for example a refused
  MAVLink subscription), and `fcParams`.
- The PX4 parameter card shows `EKF2_OF_CTRL` and `EKF2_AID_MASK` only when the
  flight controller reports them.
- Frames wider than 640 px are tracked at half resolution, and the image
  pyramid is built once per frame pair.

### Changed

- The ground station reads the firmware from `firmware.type` in the plugin
  config, so the PX4 and iNav parameter cards appear.
- Copy: firmware is selected in the config (not auto-detected), the tour calls
  the pre-arm checklist advisory, and the fallback banner suggests landing or
  the RC source-set switch.
- Supported boards: `cm4`, `cm5`, `rock-5c-lite`, `rk3588s2`, `rk3576`, `rpi5`.

### Removed

- The Pi Zero 2 W: its 512 MB of RAM equals the plugin's reservation.
- The `garmin_lidarlite_i2c` and `vl53l1x_i2c` drivers, which never produced a
  reading, and the `hardware.i2c` permission. Wire an I2C rangefinder to the
  flight controller and use `fc_relay`.
- The `pre_arm` origin block and `firmware.ekf_source_set_index`, which had no
  effect. Stored configs that carry them still load.
- The always-empty auto-detect heartbeat fields (`suggestedMode`,
  `suggestedModeReason`, `detectedCameraCount`, `detectedRangefinderDriver`).
- The direct-I2C BMI088 IMU source listed in an earlier entry was never present
  in the Rust agent and is not offered; the IMU comes from MAVLink `RAW_IMU`.

## [0.4.1]

### Removed

- **The in-app calibration wizard and the IMU bias Recalibrate button.** Both
  sent requests (`vision-nav.start_calibration`, `vision-nav.upload_calibration`,
  `vision-nav.recalibrate_imu_biases`) that the ground station refuses as
  unknown methods and that the agent half never implemented, so the wizard's
  final step always failed. The camera row now reports whether the agent loaded
  a calibration: a Kalibr `camchain.yaml` in the plugin's data directory, read
  at start-up. The two calibration telemetry grants go with the wizard.
- **The EKF source-set switcher.** The ground station does not let a plugin send
  `MAV_CMD_SET_EKF_SOURCE_SET`, so every switch was refused, and the card then
  showed the new source set as active while the flight controller stayed on the
  old one. Switch source sets with the flight controller's own RC option or the
  ground station's flight-controller tools.
- The calibration helper's wizard runner, whose only caller was the in-app
  wizard. The helper is now the `camchain.yaml` loaders and validators, with the
  dependencies they actually import.

### Changed

- The tour no longer describes a mode picker, a calibration wizard or the
  source-set switcher.
- The agent half builds against the current agent SDK revision.

## [0.4.0]

### Removed

- **The visual-inertial odometry modes are withdrawn.** `vio_openvins`,
  `vio_vins_fusion`, and `hybrid_of_plus_vio` are gone from the config schema,
  the agent's estimator registry, the GCS mode picker, and the heartbeat's
  advertised estimator list. The extension ships optical-flow navigation only:
  `off`, `optical_flow`, and `optical_flow_degraded`. The estimators behind
  those modes were never implemented — the shipped code was a scaffold that
  started no estimator and produced no pose — so the modes advertised a
  capability the build could not deliver. A config naming a withdrawn mode is
  now rejected and leaves vision navigation disengaged rather than falling back
  to a mode the operator did not select.
- With the modes go their whole surface: the vendored C++ estimator sources and
  their build files, the agent-side subprocess bridge (shared-memory frame ring
  + msgpack control channel + restart watchdog), MAVLink component 197 and the
  `VISION_POSITION_ESTIMATE` emission, the `secondary_camera` config field, the
  camera-intrinsics / extrinsics / sync-offset / feature-count pre-arm checks,
  and the `vioSupported` / `vioState` / `vioResetCounter` / `vioQuality` /
  `estimatorFeatureCount` / `estimatorDriftEstimateM` heartbeat fields. The
  vision-pose EKF source set is no longer offered in the source-set switcher,
  and the external-vision firmware parameters are no longer listed.
- Manifest grants reconciled against call sites: `process.spawn`,
  `estimator.pose.inject`, `mavlink.component.vio`, `event.subscribe`, and
  `ui.slot.notification-channel` are dropped, along with the vendor-binary
  attribution block, the subprocess allowlist, and the notification channel that
  had no publisher. `telemetry_fields` now lists the keys the agent actually
  emits.
- The release workflow no longer builds or stages vendor binaries, and the
  conformance gate that passed on the scaffold's own error string is gone.

### Changed

- The agent SDK revision CI builds against is pinned in the `ADOS_AGENT_REV`
  file at the repository root instead of tracking a moving branch, so a release
  tag resolves to one fixed SDK commit.
- Version sites (manifest, extension package, GCS package, agent crate, and the
  registered plugin version) now agree; the GCS bundle previously registered
  `0.3.2` while the manifest declared `0.3.3`.
- The GCS locale bundle is read from the shipped `locales/en.json` instead of a
  hand-maintained copy inside `plugin.tsx`.
- The mode card is read-only. It lists the modes the agent advertises as
  runnable and marks the active one; the mode is set through the plugin's
  per-drone configuration (`mode` in `config-schema.json`). The card used to
  offer clickable mode buttons that called an agent method the plugin never
  implemented, so every click ended in an error line. It also no longer renders
  a default mode set before the agent has advertised one.
- The calibration wizard's two telemetry subscriptions
  (`vision-nav.calibration_progress`, `vision-nav.calibration_complete`) are
  now declared in the manifest. The host takes the full dotted topic as the
  capability string, so both subscriptions were being denied.

## [0.3.3]

- Release-pipeline completion: the release now cross-compiles the Rust agent
  binary (`bin/vision-nav`) for the static aarch64 target and stages it into the
  archive, so the published `.adosplug` is a complete, installable extension
  rather than a half-archive. Signing and the checksum sidecar now match the
  other first-party extensions.
- Stamped the frame-descriptor wire version in the framing tests and tidied the
  scale-source mapping. No runtime behavior change.

## [0.3.2]

- Added an Engage cockpit Skill that arms and disarms vision navigation. The
  Skill toggles a per-drone `active` flag the agent reads live; when
  disengaged the effective mode is off, so the null estimator runs and no pose
  is emitted and the flight controller falls back to its own sources. The
  toggle confirms before it engages, and the agent publishes an engage
  read-back so the Skill Bar reflects the true engaged state.
- Dropped the unused `ui.slot.video-overlay` capability. The extension
  contributes a node-detail tab, a notification, and now the Engage Skill; it
  never rendered a video overlay.

- Fixed the live telemetry subscription: the panel requested the
  `navigation` telemetry topic but the manifest granted only the bare
  `telemetry.subscribe` permission, so the host denied the per-topic
  subscription and the Vision Nav panel never received a snapshot. The
  permission is now the correct `telemetry.subscribe.navigation`.
- Declared the node-detail tab slot explicitly and raised the compatibility
  floors to the current platform (the plugin uses the shared vision frame bus
  and per-node tabs, which the old floors predated).

## [0.3.0]

The agent half is rewritten from Python to a compiled **Rust** binary.
The estimator output, the MAVLink wire surface, the six-mode
degradation ladder, the scale ladder, the pre-arm gate, the TIMESYNC
clock alignment, and the heartbeat block are all preserved; the GCS
half, the config schema, and the locales are unchanged.

### Changed

- **Agent runtime is now `rust`.** `manifest.yaml` declares
  `agent.runtime: rust` and `agent.entrypoint: bin/vision-nav`; the
  plugin host execs the compiled binary directly.
- **The plugin no longer opens a camera.** It subscribes to the shared
  vision frame bus (`ctx.vision.subscribe_frames`) instead of capturing
  from a V4L2 / libcamera device. The `sensor.camera.register`,
  `hardware.usb.uvc`, `hardware.camera.csi`, and `sensor.depth.register`
  permissions are dropped; `vision.frame.read` is added. The agent's own
  SHM-ring frame write is removed.
- **VIO frame input is bridged, not captured.** For the VIO modes the
  shared-bus frames are copied into the vendored C++ estimator's
  shared-memory ring (the C++ shim still reads frames from its ring).
- **Optical flow is a clean-room Rust Lucas-Kanade tracker** (sparse
  pyramidal LK with Shi-Tomasi corner selection). The output contract
  (8x DPI scaling, gyro derotation, metric velocity, quality) matches
  the prior implementation so the wire values the FC consumes are
  unchanged.

### Added

- Rust crate at `agent/` with modular files: `config`, `framing`,
  `flow`, `estimator`, `estimators`, `scale`, `rangefinder`, `imu`,
  `clock_align`, `mavlink_emit`, `pre_arm`, `health`, `vio`, `pipeline`.
- 72 Rust unit tests covering the optical-flow math, the scale-ladder
  rung selection + staleness gates, the pre-arm gate per mode, the
  6-mode estimator behaviour, the VIO control-channel codec + the SHM
  frame-ring header layout, the TIMESYNC offset state machine, the IMU
  time aligner + drift bands, and the camchain.yaml parser.

### Kept

- The vendored **OpenVINS** and **VINS-Fusion** C++ shims and their
  `vendor_attribution` block (required for GPL). The Rust plugin spawns
  them through `process.spawn` and speaks the same SHM-ring +
  length-prefixed-msgpack-over-UDS contract.
- The one-time offline calibration wizard as a **Python helper** at
  `agent/calibration-helper/` (AprilTag detection, intrinsics solve,
  golden-section timeshift fit). Out of the hot path; the Rust agent
  reads the `camchain.yaml` it produces.

### Known limitations

- **TODO: Rust-native VIO** to remove the C++ shims. The two VIO modes
  still spawn the vendored C++ estimators as subprocesses.
- The I2C rangefinder drivers (`garmin_lidarlite_i2c`, `vl53l1x_i2c`)
  are documented stubs that fail safe (return no reading) until the
  plugin SDK exposes an I2C facade; wire an I2C rangefinder to the FC
  and use the `fc_relay` topology instead. The UART TF-Luna driver and
  the FC relay are fully implemented.

## [0.2.8] — 2026-05-23

Re-sign release with the first-party publisher id. Catches package.json + agent pyproject.toml up to the manifest version. No functional changes.

## [0.2.1] — 2026-05-16

Vision Navigation gains a first-class **camera-orientation** field on its
config schema, **iNav 7.0+** firmware support for optical flow, and a
clearer rejection path for **Betaflight**.

### Added

- `CameraConfig.orientation` literal on the plugin schema:
  `forward`, `downward`, `side`, `auto`. VIO modes accept any
  orientation; optical-flow modes require `downward` (or `auto`).
- `secondary_camera` field on the plugin schema for `hybrid_of_plus_vio`,
  with validation that the two cameras have opposed orientations and
  distinct device paths.
- `inav` added to the `FirmwareConfig.type` literal. Optical-flow modes
  emit `OPTICAL_FLOW_RAD` over MAVLink rx, which iNav 7.0+ consumes
  when `opflow_hardware = MAVLINK`.
- New GCS component `InavVisionParams` listing the iNav-specific
  parameters operators need to set (`opflow_hardware`,
  `nav_use_optflow_for_poshold`, `rangefinder_hardware`,
  `nav_rangefinder_for_terrain`, and a few neighbours).
- New GCS component `BetaflightUnsupported` explaining why Betaflight
  cannot run optical flow or VIO (no position estimator inside the FC).
- New plugin tests: 11 cases for the iNav firmware branch and 20 cases
  for camera orientation + hybrid dual-camera validation. Total agent
  test count goes from 186 to 217. New GCS tests cover the iNav params
  panel and the Betaflight unsupported banner.
- `derive_suggested_mode()` now returns `recommended_orientation` and
  takes `has_downward_camera` + `prefers_over_ground` inputs. The
  over-ground intent flag steers VIO suggestions toward downward
  cameras for agriculture, survey, SAR, and pipeline patrol suites.
- HAL board profile schema accepts an additive `cameras:` block with
  `orientation`. The Rock 5C Lite profile carries explicit
  `front=forward` and `down=downward` entries for the dev rig.

### Changed

- Validator rejects `vio_*` and `hybrid_of_plus_vio` modes when
  `firmware.type == "inav"` with a clear message. iNav's external
  position-injection EKF integration is not VIO-grade in the 7.x
  series; the plugin disables VIO modes at config time.
- Validator rejects `optical_flow` and `optical_flow_degraded` when
  the camera orientation is explicitly `forward` or `side`. `auto`
  passes; the wizard resolves it.
- Plugin pre-arm helper skips the synthetic-origin push on iNav
  (iNav's position-hold runs from optical flow directly, no global
  EKF origin required).
- Plugin estimator factory returns `NullEstimator` when asked for a
  VIO mode on iNav, as a belt-and-suspenders guard for the validator.
- Mode descriptions in the GCS `ModeCard` rewritten to reflect that
  VIO is orientation-flexible (forward for indoor / corridor,
  downward for over-ground).

## [0.2.0] — 2026-05-16

This release flips Vision Navigation from "scaffolded but runtime-dormant"
to runtime-complete on the OF modes and runtime-ready on the VIO modes
pending the vendor-binary upstream integration. The plugin auto-detects
its hardware on start, ships a guided in-app calibration wizard, and
publishes an enriched heartbeat the cloud relay now accepts.

### Added

- **Public documentation refresh across five Mintlify pages.** The
  calibration page (`Documentation/drone-agent/vision-nav-calibration.mdx`)
  was rewritten to make the in-app wizard the primary path. The YAML
  upload route stays documented as the "advanced" alternative for
  operators with an existing Kalibr `camchain.yaml`. The walk-through
  covers the seven-step flow, frame-quality troubleshooting, pose
  coverage troubleshooting, IMU motion troubleshooting, and verify
  diagnostics interpretation.
- **New page `vision-nav-calibration-math.mdx`** for the technically
  curious. Covers why intrinsics + extrinsics + timeshift are three
  separate things, the math the wizard runs (`cv2.calibrateCamera`,
  per-frame PnP, golden-section timeshift fit), why AprilGrid beats
  a chessboard, and the role of pose diversity. Registered in
  `Documentation/docs.json` in the right reading order.
- **Overview + getting-started + architecture pages updated.** The
  overview surfaces the hardware auto-detect behaviour (camera, IMU,
  rangefinder probes + suggested mode + auto-flip), softens the
  "VIO in development" framing to match reality (Python plumbing,
  GCS surfaces, auto-detect, and build pipeline ship; the
  upstream-integration step is the open work). Getting-started step 4
  points at the wizard with a 1-paragraph summary. The architecture
  page documents the new `calibration/runner.py`, the `autodetect/`
  subpackage, and the `DirectI2cImu` source under the module map +
  a dedicated calibration-runner section.

- **Vendor build scaffold for the two VIO binaries.** New
  `extensions/vision-nav/vendor/openvins/` and
  `extensions/vision-nav/vendor/vins-fusion/` trees host the
  CMakeLists, the C++ adapter sources (`main.cpp` + `ipc_channel.{hpp,cpp}`
  + `shm_ring.{hpp,cpp}`), per-binary `BUILD.md` recipes (native +
  cross-compile), and GPL-3.0 LICENSE pointers to the upstream
  projects. The adapter sources speak the existing shim IPC wire
  format documented at
  `agent/src/altnautica_vision_nav/shim/ipc.py` (length-prefixed
  msgpack over UDS + a fixed-capacity SHM frame ring). The upstream
  estimator integrations are TODOs at the binary's `run()` entry
  point; the binaries currently exit with a "vendor binary not yet
  built" message until the on-rig build phase lands.
- **CI workflow** `.github/workflows/vision-nav-vendor-binaries.yml`.
  Triggers on `vision-nav-v*` tags and on `workflow_dispatch`.
  Matrix builds `aarch64-unknown-linux-gnu` + `x86_64-unknown-linux-gnu`
  for both binaries with native cross-compile (rockchip BSP variant
  added once the sysroot drops are mirrored). Runs the conformance
  flag on the x86_64 build, signs each tarball with the existing
  `ADOS_SIGNING_KEY` Ed25519 secret, and attaches the signed
  artefacts to the GitHub release.

- **Direct-I2C BMI088 IMU source.** New
  `altnautica_vision_nav.imu.direct_i2c.DirectI2cImu` reads the
  Bosch BMI088 accel + gyro dies directly off an I2C bus at ~400 Hz,
  bypassing the FC's MAVLink stream rate cap (typically 50-200 Hz).
  The auto-detect probe walks every `/dev/i2c-*` bus checking the
  accel and gyro chip-id registers; when both match, the plugin
  instantiates the direct source instead of `MavlinkRawImu`. Stationary
  bias calibration is operator-triggered from the GCS sensors card.
- **`Recalibrate` button on the IMU row** of the sensors card.
  Renders only when the active source is a direct-bus IMU; tapping
  publishes `vision-nav.recalibrate_imu_biases`. The agent's direct
  source averages 200 stationary samples and overwrites its bias
  offsets.
- **Cloud relay navigation schema** extended with 13 optional fields
  the plugin's enriched heartbeat has been emitting since the
  estimator framework landed (mode, availableEstimators, estimatorState,
  estimator{Feature,Drift}*, flowScaleSource, imuSource, imuRateHz,
  cameraImuSyncOffsetMs, cameraIntrinsicsLoaded, preArmReport, plus
  the auto-detect summary). Without this schema update the cloud
  relay was silently dropping every enriched field at validation
  time. Applies to both `ADOSMissionControl/convex/schema.ts` and
  `website/convex/schema.ts`; `npx convex deploy` required from a
  dev machine before the new fields land in the live database.

- **Hardware auto-detect on start-up.** New
  `altnautica_vision_nav.autodetect` package probes the host profile
  (refusing to enable on ground-station-profile hosts), enumerates
  `/dev/video*` cameras with a CSI hint, scans I2C buses for
  LIDAR-Lite (0x62) and VL53L1X (0x29), probes UART for TF-Luna at
  115200, and derives a `suggestedMode` from the detected hardware.
  The plugin merges detected defaults into config when the operator
  has left a field unset (operator-set values always win).
- **Auto-flip from `optical_flow` to `optical_flow_degraded`** when
  no rangefinder is detected, with a log line explaining the flip.
- **Four new heartbeat fields**: `suggestedMode`, `suggestedModeReason`,
  `detectedCameraCount`, `detectedRangefinderDriver`. The GCS
  ModeCard renders a "Suggested" badge on the recommended mode when
  it does not match the active selection.
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
- **Rangefinder drivers.** Two drivers ship in this release:
  - `tfluna_uart`. Benewake TF-Luna on a companion serial port, 0.2 to
    8 m range.
  - `fc_relay`. Reads the FC's `DISTANCE_SENSOR` MAVLink message when
    the rangefinder is wired to the flight controller instead of the
    companion.
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
