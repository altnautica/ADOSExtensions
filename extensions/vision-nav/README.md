# Vision Navigation

GPS-denied navigation for ADOS drones. The plugin ships four
selectable modes: optical flow with a rangefinder, optical flow
without one, external navigation from a shared pose topic, and off.

The agent half is a compiled **Rust** binary. It does not open a
camera: it subscribes to the shared vision frame bus, pairs each frame
with the closest IMU sample, runs the selected estimator, and emits the
flow message for the configured firmware (`OPTICAL_FLOW` for ArduPilot
and iNav, `OPTICAL_FLOW_RAD` for PX4; plus `DISTANCE_SENSOR` when the
rangefinder is companion-wired) that the flight controller fuses into
its estimator. It asks the flight controller for `RAW_IMU` at 100 Hz,
`DISTANCE_SENSOR` at 20 Hz and its own `OPTICAL_FLOW` at 20 Hz. External
navigation reads that flow for its `fc_flow` pre-arm row, and the
world-engine lidar SLAM follows its ground velocity along a corridor its
scans leave unconstrained. The GCS half mounts a node-detail tab with the mode
list, sensors card, estimator card, telemetry charts, pre-arm status,
and a fallback banner that fires when the estimator goes degraded or
fails.

Camera calibration is a Kalibr-style `camchain.yaml` in the plugin's
data directory, produced offline with Kalibr. The Rust agent reads the
camera-IMU timeshift out of that file at start-up and uses it to align
frames with IMU samples; `agent/calibration-helper/` validates a file
before it goes onto a vehicle.

## Modes

| Mode | Estimator | Camera | IMU | Rangefinder | MAVLink |
|---|---|---|---|---|---|
| `off` | None | optional | optional | optional | none |
| `optical_flow` | Lucas-Kanade | downward | gyro | required | comp 198 |
| `optical_flow_degraded` | Lucas-Kanade | downward | gyro | optional | comp 198 |
| `external_nav` | external pose topic | none | FC `ATTITUDE` | none | comp 197 |

`optical_flow_degraded` is the rangefinder-free path. Scale comes
from a ladder: baro from `GLOBAL_POSITION_INT.relative_alt`, raw baro
from `VFR_HUD.alt`, then (only with `outdoor: true`) GPS height above
the first good 3D fix, then a 1.5 m static fallback. Each rung carries
a quality multiplier so the EKF de-weights degraded scale sources. The
static rung blocks the pre-arm scale check; the pre-arm gate is
advisory and does not stop the flight controller from arming.

## External navigation

`external_nav` (ArduPilot) takes the pose another installed plugin
publishes on a shared topic (default `plugin.world-engine.pose`, the
World Engine extension's planar lidar SLAM) and sends it to the flight
controller as `ODOMETRY` (#331) on component 197, at up to
`external_nav.rate_hz` (30 Hz):

- `frame_id` `MAV_FRAME_LOCAL_FRD`, `child_frame_id` `MAV_FRAME_BODY_FRD`.
  The pose arrives in `map_ned` (north-aligned, the axes of the flight
  controller's `LOCAL_POSITION_NED`; a `map_flu` pose is converted with
  `y = -y`, `z = -z`, `yaw = -yaw`). `VISO_TYPE 1` aligns only the
  external position to the EKF's, not its yaw, so the EKF yaw stays on
  the compass (`EK3_SRC1_YAW 1`) unless the pose's map holds its own north
  (the pose's `north` is `declared`, from the World Engine extension's
  `atlas.takeoff_heading_deg`, or `gnss`); then `EK3_SRC1_YAW 6` takes yaw
  from the pose. The external-nav set takes no velocity from the pose
  (never `EK3_SRC1_VELXY 6`: ArduPilot ignores the `ODOMETRY` velocity
  covariance); with an optical-flow sensor (`FLOW_TYPE` set) it takes
  the flow's velocity (`EK3_SRC1_VELXY 5`), measured rather than
  differenced from late poses, and without one none (`0`). EKF3 fuses
  flow beside the external position in one set
  (`AP_NavEKF3_OptFlowFusion.cpp` `SelectFlowFusion`).
- x, y and yaw come from the pose; z and vertical speed from the
  flight controller's `LOCAL_POSITION_NED`, roll and pitch from its
  `ATTITUDE`. Velocity is the pose difference over the last 100 ms, in
  the body frame.
- Covariance: position σ 0.05 m and yaw σ 0.02 rad while tracking is
  good, 0.30 m and 0.1 rad while degraded; z carries the horizontal
  variance because ArduPilot folds the three position variances into
  one error. A pose that reports its own horizontal uncertainty
  (`sigma_xy_m`, the World Engine planar scan matcher's while it rides
  its motion prior unmatched) widens the position σ to it where it is
  larger. `quality` is 100 (good) or 50 (degraded), and 30 while that
  σ is over 1 m.
- `reset_counter` steps when a pose lands more than 0.5 m from where the
  last velocity predicts it (from the last pose after a gap over 0.5 s or
  a loss), so steady flight at any speed is motion, not a reset.
- Exactly one `ODOMETRY` per measured pose: the tracker publishes
  about once per scan (~10 Hz), and nothing is interpolated or
  extrapolated between poses, so every fused message is a real
  measurement. `rate_hz` only caps the rate. Nothing is sent while
  tracking is lost or the pose is older than 0.5 s.
- `time_usec` is the instant the pose describes (its `ts_ms`, a lidar
  pose's first return) on the flight controller's clock, and the velocity
  is differenced over those instants. Set `VISO_DELAY_MS` to the pose's
  minimum age when it reaches the flight controller, because ArduPilot's
  jitter correction absorbs a constant stamp offset and fuses each pose
  `VISO_DELAY_MS` before its corrected arrival.

EKF source sets (`MAV_CMD_SET_EKF_SOURCE_SET`, resent up to three times
until accepted):

- `gps_primary: false`: `extnav_set` while the pose tracks; lost for
  1 s switches to `fallback_set` (the flight controller's optical-flow
  set); tracked (good or degraded) for 3 s switches back.
- `gps_primary: true`: `fallback_set` (GPS) while the GPS is usable (a 3D
  fix whose reported horizontal accuracy, when sent, is within 5 m); no
  usable GPS for 2 s with a good pose switches to `extnav_set`; usable
  for 10 s switches back.

### EKF origin

ArduPilot aligns the EKF yaw at boot with the declination it has then. An
origin that arrives later loads the site's declination and steps the
earth field; with `COMPASS_AUTODEC` on, the step becomes a gyro bias and
the vehicle spins. So without GPS the flight controller should set the
origin itself at boot: `AHRS_OPTIONS` bit 4 with `AHRS_ORIGIN_LAT/LON/ALT`
at the take-off site (ArduPilot 4.7 and later), or before 4.7
`COMPASS_AUTODEC 0` with the site's `COMPASS_DEC`.

The plugin asks for the origin every 5 s until the flight controller
reports one. It sends `SET_GPS_GLOBAL_ORIGIN` from
`external_nav.origin_lla` only as a last resort: when the flight
controller has none 5 s after it boots (a heartbeat after 3 s of silence
is a new boot), is disarmed, and GPS is not primary.

### Pre-arm checks

- `extnav_source`: blocks with "External nav source missing" when the
  pose topic is not published by an installed plugin or reading it is
  not granted.
- `extnav_health`: good passes, degraded waits, lost blocks.
- `ekf_origin`: waits until the flight controller reports an origin;
  blocks when the origin the plugin sent arrived after a compass-yaw
  alignment with `COMPASS_AUTODEC` on, or when a boot-time `AHRS_ORIGIN`
  is more than 1 km from `origin_lla`.
- `fc_params`: reads `VISO_TYPE`, `VISO_DELAY_MS`, `COMPASS_AUTODEC`,
  `AHRS_OPTIONS`, `AHRS_ORIGIN_LAT/LON`, `EK3_MAG_EF_LIM` and the
  source-set parameters (every 10 s while any is missing), then blocks on
  `VISO_TYPE` other than 1, an external-nav set whose `POSXY` is not 6,
  whose `VELXY` is 6 (the pose-difference velocity fused as good to
  `VISO_VEL_M_NSE`), or whose `YAW` is 6 while the pose's map north
  follows the flight controller's heading (it waits for a pose to tell),
  a fallback set that is not
  GPS (`POSXY 3`, with `gps_primary`) or optical flow (`VELXY 5`,
  without), flow velocity in either set with `FLOW_TYPE` 0, or
  `COMPASS_AUTODEC` on with `EK3_MAG_EF_LIM` 0 (the compass
  library's declination update, made when the EKF first has a position,
  would then step the EKF yaw). With a flow sensor configured and the
  external-nav set's `VELXY` not 5, it passes with a note recommending
  flow velocity: SLAM-only velocity stays a valid choice on dark floors.
- `viso_delay`: waits while `VISO_DELAY_MS` is more than 50 ms from the
  median age of a pose when its `ODOMETRY` is sent (telemetry
  `navigation.poseAgeMs`), and while poses are older than the 250 ms
  EKF3 can place in time, which no `VISO_DELAY_MS` covers.
- `fc_flow`, when `FLOW_TYPE` is set and the external-nav set (or,
  without `gps_primary`, the fallback set) takes its velocity from
  optical flow (`VELXY 5`): the flight controller's own `OPTICAL_FLOW`
  arrives. ArduPilot sends none while its flow sensor is unhealthy (no
  sample for 0.5 s), so no flow for 1 s blocks as a lost sensor, and so
  does none at all 5 s after the flight controller is first heard. A
  quality of 0 (no light or texture, or out of the sensor's range) waits:
  EKF3 fuses no such sample. The row reads `Quality q/255` when fresh.

## Obstacle feed

With `obstacle.enabled`, the plugin turns the shared planar scan
(default `plugin.world-engine.scan`, body FRD, clockwise from forward)
into `OBSTACLE_DISTANCE` (#330) at up to 10 Hz on component 197: 72
sectors of 5°, `MAV_FRAME_BODY_FRD`, the nearest return per sector in
cm (5-1200), `max_distance + 1` for a sector with no return. Set
`PRX1_TYPE 2` and `AVOID_ENABLE` on the flight controller.

## `nav.state`

The agent publishes a `nav.state` event (5 Hz in `external_nav`, 1 Hz
otherwise):

```json
{"mode": "external_nav", "active_set": 1, "extnav_health": "good",
 "odom_rate_hz": 30.0, "last_reset_counter": 0, "obstacle_rate_hz": 10.0,
 "min_obstacle_m": 3.2, "gps_fix": 0}
```

Fields a mode does not produce are `null`. The Vision Nav tab shows it
as the navigation feed card.

## Public docs

Public docs (full mode breakdown, per-mode pre-arm matrix, fallback
ladder, calibration walkthrough, troubleshooting decision trees,
architecture reference) live at:

- [Vision Navigation Overview](https://docs.altnautica.com/drone-agent/vision-nav-overview)
- [Modes](https://docs.altnautica.com/drone-agent/vision-nav-modes)
- [Calibration](https://docs.altnautica.com/drone-agent/vision-nav-calibration)
- [Troubleshooting](https://docs.altnautica.com/drone-agent/vision-nav-troubleshooting)
- [Architecture](https://docs.altnautica.com/drone-agent/vision-nav-architecture)

## Supported firmware

- ArduPilot Copter, Plane, Rover 4.5 and newer
- PX4 1.14 and newer
- iNav 7.0 and newer

The firmware is selected in the plugin config (`firmware.type`); it
picks the flow message the plugin emits.

## Supported cameras

- USB UVC global-shutter modules (the Arducam B0332 OV9281 mono module
  is a known-good pick)
- USB UVC rolling-shutter modules
- CSI modules exposed through V4L2

## Supported rangefinders

When a rangefinder is wired, the OF modes consume:

- TF-Luna over a companion serial port (raw 8N1 at the configured baud)
- `fc_relay`: the FC owns the rangefinder and the plugin reads its
  `DISTANCE_SENSOR` stream. An I2C rangefinder is wired to the FC and
  read this way.

The `optical_flow` pre-arm check needs a fresh reading inside the
sensor's min-max range, not just a configured topology.

## Calibration

Camera intrinsics and the static camera-IMU time offset come from a
Kalibr-compatible `camchain.yaml`, which the plugin accepts directly.
Both layouts work (`cam0` wrapper or bare block); the loader reads
only `cam0`. Multi-camera files (`cam1`, `cam2`, ...) are accepted
with the additional cameras ignored. The agent uses the
`timeshift_cam_imu` value to align each frame with the gyro sample
taken at the same instant; the heartbeat reports the calibration as
absent when no usable file is on disk.

See the
[Calibration page](https://docs.altnautica.com/drone-agent/vision-nav-calibration)
for the full walkthrough.

## Building from a clone

The Rust agent depends on the plugin author SDK (`ados-sdk`,
`ados-protocol`) through a workspace path dependency, so the sibling
agent repository must be checked out next to this one:

```text
<root>/ADOSDroneAgent      # provides crates/ados-sdk, crates/ados-protocol
<root>/ADOSExtensions      # this repository
```

CI clones `ADOSDroneAgent` and checks out the commit recorded in the
`ADOS_AGENT_REV` file at this repository's root, so a release tag
always builds against one fixed SDK revision. For local development,
check out the same revision (or `main` if you are tracking SDK work):

```sh
git -C ../ADOSDroneAgent checkout "$(cat ADOS_AGENT_REV)"
```

GCS half:

```sh
pnpm install
pnpm --filter ./extensions/vision-nav/gcs build
pnpm --filter ./extensions/vision-nav/gcs test
```

Agent (Rust) build + tests, from the repo root:

```sh
cargo build -p vision-nav
cargo test -p vision-nav
```

Calibration helper (Python) tests:

```sh
cd extensions/vision-nav/agent/calibration-helper
python -m pip install -e .
python -m pytest -q
```

The shipped binary is statically linked against musl so one build runs
on every supported board. The `aarch64-unknown-linux-musl` target and
`musl-tools` (for the musl-gcc linker) are prerequisites:

```sh
rustup target add aarch64-unknown-linux-musl
sudo apt-get install -y musl-tools     # Debian / Ubuntu
```

To produce a `.adosplug` (cross-compiles the Rust binary to the
aarch64 musl target and stages it at the manifest entrypoint):

```sh
./scripts/pack-rust.sh vision-nav
```

## Surfaces contributed

| Slot | Purpose |
|------|---------|
| `node.detail.tab` | "Vision Nav" configuration tab on the node detail panel |
| `flight.skill` | "Engage" cockpit toggle that engages / disengages the estimator |

The node-detail tab hosts:

- Mode list (the modes the agent advertises as runnable, active one
  marked; the mode is set in the per-drone settings)
- Sensors card (camera + IMU + rangefinder rows with the calibration
  and sync-offset pills)
- Estimator card (engine + state + flow quality + sync offset)
- Telemetry charts (inline-SVG sparklines for flow quality and sync
  offset)
- Flow health card (live OF metrics)
- Pre-arm status (mode-aware check rows)
- Navigation feed (the live `nav.state`: active EKF source set,
  external-nav tracking, ODOMETRY and obstacle rates, nearest obstacle,
  GPS fix)
- Fallback banner (fires when the estimator is degraded or failed
  with reason plus suggested next action)

## Permissions

Agent:

- `vision.frame.read` (reads normalized frames from the shared vision
  frame bus; the plugin does not open a camera, so the camera / UVC /
  CSI / depth permissions are not requested)
- `hardware.uart` (the companion-wired TF-Luna)
- `telemetry.extend`, `event.publish`
- `event.subscribe`, `telemetry.read` (the shared pose and scan topics)
- `mavlink.read`, `mavlink.write`
- `estimator.pose.inject` (`ODOMETRY` and `SET_GPS_GLOBAL_ORIGIN` reach
  the flight controller's state estimator; the host requires it for both)
- `mavlink.component.vio` (the host reserves components 197 and 198
  for kind `vio`)

GCS:

- `ui.slot.node-detail-tab`, `ui.slot.flight-skill`
- `telemetry.subscribe.navigation`

Risk band: high. The plugin writes MAVLink and feeds a velocity source
the flight controller's estimator fuses for position hold, so a bad
flow feed moves the aircraft. Grant only on drones where vision
navigation is required.

## Configuration

See `config-schema.json` for the per-drone configuration form. Key
fields:

- `mode`: one of `off` (default), `optical_flow`,
  `optical_flow_degraded`, `external_nav`.
- `active`: written by the Engage Skill; `false` idles vision
  navigation.
- `flow_quality_min`: flow quality gate, 0-255 (default 50).
- `outdoor`: allow the GPS altitude rung (default `false`).
- `camera`: recommended device path, bus type (`uvc` or `csi`),
  orientation, resolution, and frame rate.
- `rangefinder`: topology (`companion`, `fc` (default), `none`), driver
  (`fc_relay` (default) or `tfluna_uart`), optional serial device,
  baud.
- `firmware.type`: `ardupilot`, `px4`, or `inav`.
- `external_nav`: `pose_topic`, `origin_lla` (`[lat, lon, alt_m]`,
  required in `external_nav` unless `gps_primary`), `rate_hz`,
  `extnav_set`, `fallback_set`, `gps_primary`.
- `obstacle`: `enabled` (default `false`), `scan_topic`, `sectors`,
  `rate_hz`.

The agent reads these keys live from the per-drone store once a second
and restarts itself when any of them changes.

## MAVLink components

- Component 198, `sub_id` 1: kind `vio`. Emits `OPTICAL_FLOW` or
  `OPTICAL_FLOW_RAD` (by firmware), the `MAV_CMD_SET_MESSAGE_INTERVAL`
  stream requests,
  `DISTANCE_SENSOR` (when relaying a companion-wired rangefinder),
  `TIMESYNC`, and the 1 Hz companion `HEARTBEAT`.
- Component 197, `sub_id` 2: kind `vio`, registered only in
  `external_nav` or with the obstacle feed on. Emits `ODOMETRY`,
  `OBSTACLE_DISTANCE`, and the `MAV_CMD_SET_EKF_SOURCE_SET` /
  `MAV_CMD_REQUEST_MESSAGE` commands and `SET_GPS_GLOBAL_ORIGIN`.

## License

GPL-3.0-or-later. See the repository `LICENSE` file.
