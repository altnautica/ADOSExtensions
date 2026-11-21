# Navigation

GPS-denied navigation for ADOS drones and Visors. The plugin ships four
selectable modes: optical flow with a rangefinder, optical flow
without one, external navigation from a shared pose topic, and off. Its
`lidar-slam` service runs a LiDAR or VIO pose of its own and builds the
live occupancy map from it.

The agent half is two compiled **Rust** processes. The entrypoint
(`navigation`) does not open a camera: it subscribes to the shared vision
frame bus, pairs each frame with the closest IMU sample, runs the selected
estimator, and emits the flow message for the configured firmware
(`OPTICAL_FLOW` for ArduPilot and iNav, `OPTICAL_FLOW_RAD` for PX4; plus
`DISTANCE_SENSOR` when the rangefinder is companion-wired) that the flight
controller fuses into its estimator. It asks the flight controller for
`RAW_IMU` at 100 Hz, `DISTANCE_SENSOR` at 20 Hz and its own `OPTICAL_FLOW`
at 20 Hz. External navigation reads that flow for its `fc_flow` pre-arm
row, and the planar scan matcher follows its ground velocity along a
corridor its scans leave unconstrained. The GCS half mounts a cockpit panel
and an Agent page with the mode list, sensors card, estimator card,
telemetry charts, pre-arm status, and a fallback banner that fires when the
estimator goes degraded or fails.

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
publishes on a shared topic (default `plugin.navigation.pose`, this
extension's own `lidar-slam` service) and sends it to the flight
controller as `ODOMETRY` (#331) on component 197, at up to
`external_nav.rate_hz` (30 Hz):

- `frame_id` `MAV_FRAME_LOCAL_FRD`, `child_frame_id` `MAV_FRAME_BODY_FRD`.
  The pose arrives in `map_ned` (north-aligned, the axes of the flight
  controller's `LOCAL_POSITION_NED`; a `map_flu` pose is converted with
  `y = -y`, `z = -z`, `yaw = -yaw`). `VISO_TYPE 1` aligns only the
  external position to the EKF's, not its yaw, so the EKF yaw stays on
  the compass (`EK3_SRC1_YAW 1`) unless the pose's map holds its own north
  (the pose's `north` is `declared`, from
  `navigation.takeoff_heading_deg`, or `gnss`); then `EK3_SRC1_YAW 6` takes yaw
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
  (`sigma_xy_m`, the `lidar-slam` planar scan matcher's while it rides
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
  fix whose reported horizontal accuracy, when sent, is within 5 m). No
  usable GPS for 2 s with a good pose switches to `extnav_set`, but only
  once the flight controller reports an EKF origin: EKF3 sets its origin
  from the GPS whichever set is active, and an origin first set after the
  takeover would leave the external position in a frame of its own (a
  receiver still starting, or a one-tick fix, sets none). Usable for 10 s
  switches back, and with the pose lost for 1 s any usable GPS does.
  Armed, a switch to GPS happens only when the origin existed at the
  takeover, and the first command after a start waits for the vehicle to
  be disarmed (or for the pose to be lost for 1 s with a usable GPS and an
  EKF origin): in Guided a position reset moves the vehicle by the offset.
  A new flight-controller boot (3 s of heartbeat silence, or its
  `time_boot_ms` going back by more than 0.5 s) starts the machine over and
  drops a command still being retried.

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
is a new boot), is disarmed, GPS is not primary, and the flight
controller's `EK3_SRC{extnav_set}_POSXY` reads 6 (ExternalNav); while
that parameter is unread no origin is sent.

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
(default `plugin.navigation.scan`, body FRD, clockwise from forward)
into `OBSTACLE_DISTANCE` (#330) at up to 10 Hz on component 197: 72
sectors of 5°, `MAV_FRAME_BODY_FRD`, the nearest return per sector in
cm (5-1200), `max_distance + 1` for a sector with no return. Before
binning it drops returns nearer than 0.35 m (the airframe), the bins the
scan lists as `blocked_bins`, and, with the flight controller's fresh
`ATTITUDE` and downward `DISTANCE_SENSOR`, returns a tilted scan plane
takes off the floor (within 0.25 m of the ground). Set `PRX1_TYPE 2` and
`AVOID_ENABLE` on the flight controller.

## `plugin.navigation.state`

The agent publishes a `plugin.navigation.state` event (5 Hz in `external_nav`, 1 Hz
otherwise):

```json
{"mode": "external_nav", "active_set": 1, "extnav_health": "good",
 "odom_rate_hz": 30.0, "last_reset_counter": 0, "obstacle_rate_hz": 10.0,
 "min_obstacle_m": 3.2, "gps_fix": 0, "source_hold": null, "health_reason": null}
```

Fields a mode does not produce are `null`. `source_hold` says why the
GPS-primary source set is held: `no_origin` (a lost GPS is not handed to
external navigation without an EKF origin), `no_position` (on external
navigation with the pose lost and no usable GPS; nothing to switch to, the
flight controller's EKF failsafe acts) or `unknown_set` (started while
armed, the flight controller's set unknown). `health_reason` says why the
pose is not good when there is a reason beyond its health:
`ekf_flow_disagree` (the flight controller's optical flow and its EKF
velocity parted), `corridor_unmeasured` (the LiDAR pose's free axis has
gone 0.3 s with no measured velocity), `flow_disagrees` (the LiDAR match
slid against the flow), else the FC IMU feed (`heading_unconfirmed`,
`imu_feed_<reason>`). The Navigation page shows the state as the
navigation feed card, and the cockpit panel shows it in flight.

## LiDAR SLAM and the live map

The `lidar-slam` service (`navigation-lidar-slam`, on drones and Visors)
runs the pose provider `navigation.pose_provider` names: `lidar` (2D scan
matching for a planar LiDAR, KISS-ICP for a 3D one), `vio` (Basalt on a
stereo pair and IMU) or `off`. `auto` (default) picks `lidar` when the rig
declares the configured LiDAR and IMU, else `vio` on a Visor with a front
stereo pair and IMU, else none. The flight controller's own EKF is not a
Navigation pose: it is the flight controller's fallback source set. A
provider that cannot start (no rig document, a missing binary) leaves the
service without a pose and says why in its log and on
`GET /navigation/status`; the rig's LiDAR still publishes its binned scans
for the obstacle feed.

It publishes:

| Topic | Rate | Payload |
|---|---|---|
| `plugin.navigation.pose` | pose interval | Body pose in `map_ned` (north-aligned: x north, y east, z down; the planar SLAM world rotated to north, origin at the first scan; the north is latched from `navigation.takeoff_heading_deg` when set, else taken from the flight controller's heading at take-off (followed while the vehicle sits within 0.5 m of where the first pose was taken, so a heading that turns onto true north once the EKF origin is set is the one kept); then moved onto the FC's `LOCAL_POSITION_NED` by a yaw and offset fitted against the EKF's GNSS track once the FC's `GPS_RAW_INT` has held a 3D fix for 15 s, frozen when GNSS goes; z is the FC's height above the first pose), with `ts_ms` (the unix time the pose describes: a lidar pose's scan's first return, not the publish time) and `t_ns` (the same instant on the host monotonic clock), `health`, `frame`, `source`, `north` (`declared`, `fc_heading` or `gnss`: where the map's north comes from) and, from the planar scan matcher, `sigma_xy_m` (horizontal one-sigma uncertainty in metres: growing by half the distance the motion prior carries the pose while scans ride it unmatched, and by half the height climbed while a climb without a measured velocity is matched band by band, up to 5; a matched scan otherwise carries the sigma of the submap it matched in, 0.05 for one started at a matched pose). Map lanes wait for the first heading |
| `plugin.navigation.scan` | per scan | The scan binned to 360 one-degree bins in the body FRD frame, nearest range per bin, `0` for no return. Every return is kept, airframe and floor included (the agent pilot reads an empty bin as free space); `blocked_bins` lists the bins the rig LiDAR's `blocked_azimuth_deg` sectors cover, absent when there are none |
| `plugin.navigation.map_status` | 1 Hz | Placement, this service's resident memory (`VmRSS`, null when unreadable) beside its map budget, voxel size, sweep points, mapped area, `pose_provider` (null when none runs), and `imu_feed` (the FC IMU feed's state, reason and `RAW_IMU` rate while `navigation.lidar.imu` is `fc` and the planar LiDAR pose is the one running; null otherwise, including KISS-ICP on a 3D LiDAR) |
| `plugin.navigation.lidar` | 1 Hz while the rig declares `navigation.lidar.lidar_id` | The LiDAR link: `v` (2), `device` (the opened port, the resolved `/dev/serial/by-id` path under `auto`; null when never opened), `connected`, `health` (`good`, `warning` or `error`) and `health_code` from the device's latest (re)connect (null when it never connected or the driver has no health report, as on the Livox), `rate_hz` (revolutions per second over the last second; null when not connected), `returns_last_scan` (the returns in the newest scan, `0` when it saw nothing; null when not connected or before the first scan), and `scans`, `empty_scans` (scans with no return at all), `dropped_scans` and `reconnects` since the run started. A scan arriving says the link is up, not that the sensor sees anything: an RPLIDAR behind a blocked window, with a failed receiver, facing open sky or in open air delivers scans with no returns. A Livox delivers a scan only around a valid point, so one that sees nothing stops sending scans and reads `connected: false`. A LiDAR absent when the service starts is retried every 2 s and reports `connected: false` with null device, health, rate and returns until it answers |
| `plugin.navigation.occupancy` | 1 Hz | Changed 64×64-cell tiles at 5 cm as grey PNG (0 occupied, 254 free, 205 unknown), `kind: "live"` |
| `plugin.navigation.scan_polar` | per placed scan, only with `navigation.map_placement = stream` | The placed planar scan with its sensor pose in `map_ned`, for a reconstruction node that builds the full-resolution map |
| `plugin.navigation.scan_raw` | per scan, only while a capture session records | One LiDAR scan as measured, in the sensor frame, in parts of at most 3,500 points (17 bytes each: `x, y, z` `f32`, `intensity` `u8`, `t_offset_ns` `u32`), for the capture recorder |
| `plugin.navigation.pose_track` | 10 Hz | Every estimate since the last batch in the provider's own world frame, for a capture session posed by Navigation |

The first four need `telemetry.read`; the last four need
`plugin.navigation.map.read`, which this extension declares. Occupancy
tiles farther than 200 m from the vehicle, or past the occupancy share of
the budget (farthest first), are written to the service's data directory
and leave RAM.

Routes on the plugin's HTTP socket
(`/api/plugins/com.altnautica.navigation/x/…`):

| Route | Method | Serves |
|---|---|---|
| `/navigation/status` | GET | provider, tracking health, pose age, LiDAR link, the map status |
| `/navigation/sweep?since=<seq>` | GET | the live sweep cloud: a 24-byte little-endian header (`seq: u64`, `count: u32`, `voxel_m: f32`, `reset: u32`, `reserved: u32`) then `count` `f32` `x, y, z` triples in `map_ned`; the deltas after `since`, or the whole cloud with `reset = 1` without `since`, with one older than the last 64 deltas, or after the cloud was coarsened |
| `/navigation/config` | PUT | patch the `navigation.*` keys; the patch is resolved with the stored keys and refused whole when any value is invalid |

`cargo run --release -p navigation-lidar-slam --example scan2d_replay --
<lidar0.polar.zst>` replays a recorded scan log through the scan matcher and
prints the wall time, a fixed workload for comparing CPUs.

Native helpers: basalt-ados (BSD-3) and the Livox SDK2 library (MIT) are
release payloads with their licence texts. The Livox SDK2 library builds on
every release leg (x86_64-linux and aarch64-linux) against glibc 2.28
(`lidar-slam/livox/build-sdk2.sh`); basalt-ados builds for x86_64-linux only
(`lidar-slam/basalt/README.md`). The LiDAR service links glibc dynamically so
it can load the Livox library. The
KISS-ICP odometry environment is built on the device by `uv sync --frozen` on
the first LiDAR start, which needs network once.

## Public docs

Public docs (full mode breakdown, per-mode pre-arm matrix, fallback
ladder, calibration walkthrough, troubleshooting decision trees,
architecture reference) live at:

- [Navigation Overview](https://docs.altnautica.com/drone-agent/navigation-overview)
- [Modes](https://docs.altnautica.com/drone-agent/navigation-modes)
- [Calibration](https://docs.altnautica.com/drone-agent/navigation-calibration)
- [Troubleshooting](https://docs.altnautica.com/drone-agent/navigation-troubleshooting)
- [Architecture](https://docs.altnautica.com/drone-agent/navigation-architecture)

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
[Calibration page](https://docs.altnautica.com/drone-agent/navigation-calibration)
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
pnpm --filter ./extensions/navigation/gcs build
pnpm --filter ./extensions/navigation/gcs test
```

Agent (Rust) build + tests, from the repo root:

```sh
cargo build -p navigation -p navigation-lidar-slam
cargo test -p navigation -p navigation-lidar-slam
```

Calibration helper (Python) tests:

```sh
cd extensions/navigation/agent/calibration-helper
python -m pip install -e .
python -m pytest -q
```

The shipped binaries are statically linked against musl so one build runs
on every supported board; the release workflow cross-compiles them with
zig (`cargo zigbuild`) and ships them as install-time payloads with the
native helpers (`scripts/navigation-payloads.tsv`).

## Surfaces contributed

| Slot | Purpose |
|------|---------|
| `cockpit.panel` | "Navigation" panel: the mode, the EKF source set, tracking health, odometry rate and pose age, the obstacle feed, the nearest obstacle and GPS; one line "No navigation state" when `plugin.navigation.state` is stale |
| `node.agent.page` | "Navigation" page in the drone's Agent tab, under Video and vision |
| `flight.skill` | "Engage" cockpit toggle that engages / disengages the estimator |

One iframe bundle renders both pages and routes on the contribution it is
mounted as (`ctx.mount.panelId`). The Agent page hosts:

- Mode list (the modes the agent advertises as runnable, active one
  marked; the mode is set in the per-drone settings)
- Sensors card (camera + IMU + rangefinder rows with the calibration
  and sync-offset pills)
- Estimator card (engine + state + flow quality + sync offset)
- Telemetry charts (inline-SVG sparklines for flow quality and sync
  offset)
- Flow health card (live OF metrics)
- Pre-arm status (mode-aware check rows)
- Navigation feed (the live `plugin.navigation.state`: active EKF source set,
  external-nav tracking, ODOMETRY and obstacle rates, nearest obstacle,
  GPS fix)
- Fallback banner (fires when the estimator is degraded or failed
  with reason plus suggested next action)

## Permissions

Agent:

- `vision.frame.read` (reads normalized frames from the shared vision
  frame bus; the plugin does not open a camera, so the camera / UVC /
  CSI / depth permissions are not requested)
- `hardware.uart`, optional (the companion-wired TF-Luna and a serial
  LiDAR such as an RPLIDAR)
- `telemetry.extend`, `event.publish`
- `event.subscribe`, `telemetry.read` (the shared pose and scan topics,
  and the capture state `lidar-slam` streams raw scans for)
- `node.info.read` (the node profile `lidar-slam` picks its provider by)
- `mavlink.read`, `mavlink.write`
- `estimator.pose.inject` (`ODOMETRY` and `SET_GPS_GLOBAL_ORIGIN` reach
  the flight controller's state estimator; the host requires it for both)
- `mavlink.component.vio` (the host reserves components 197 and 198
  for kind `vio`)
- `sensor.stream.read` (`lidar-slam`: the LiDAR and IMU from the sensor
  bus)
- `network.outbound`, optional (`lidar-slam`: a network LiDAR, an RPLIDAR
  over TCP or the Livox command channel)

GCS:

- `ui.slot.cockpit-panel`, `ui.slot.node-agent-page`, `ui.slot.flight-skill`
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
and restarts itself when any of them changes. The `lidar-slam` service
reads its own keys every 5 s and restarts its run on a change:

| Key | Purpose |
|---|---|
| `navigation.map_placement` | Where the heavy map is built: `onboard` (5 cm sweep cloud on the drone), `stream` (placed scans stream to a reconstruction node on `plugin.navigation.scan_polar`; the drone keeps a 20 cm cloud), `post_flight` (record, upload after landing), `hybrid` (10 cm on board plus the post-flight upload; default). The navigation map and the live occupancy always stay on board |
| `navigation.memory_budget_mb` | RAM the live map may hold (sweep cloud 60 %, occupancy 40 %); unset, 80 % of this extension's memory cap. A full sweep cloud doubles its voxel (`sweep_cloud_coarsened`) |
| `navigation.pose_provider` | Pose provider: `auto` (default), `vio` (on-device visual-inertial), `lidar` (on-device LiDAR odometry: 2D scan matching for a planar LiDAR, KISS-ICP for a 3D one) or `off`. `auto` picks `lidar` when the rig has the configured LiDAR and IMU (on a drone too, where the LiDAR pose is the navigation source the flight controller fuses), else on a Visor `vio` with a front stereo pair and IMU, else none |
| `navigation.lidar` | LiDAR input: `lidar_id` (rig sensor; its `lidar.model` picks the driver, `rplidar-c1` or `livox-mid360`), `imu` (the rig IMU a planar LiDAR is levelled with; `fc`, for a companion without an IMU, takes the flight controller's `RAW_IMU` gyro with gravity in the FC's reported attitude as the specific force (until `RAW_IMU` arrives, the attitude's roll and pitch only, never its yaw, which under external navigation is the matcher's own heading fed back; the matcher learns the gyro's z bias from the scans; the planar pose is degraded while the feed is that stand-in, or `RAW_IMU` stalls for over 250 ms or arrives under 40 Hz, logged as `nav_imu_feed_degraded`) and, when the FC has a flow sensor, its `OPTICAL_FLOW` (quality 128 and up) times its downward `DISTANCE_SENSOR` range as the ground velocity the matcher follows along a corridor the scans leave unconstrained; each sample reaches the matcher as its message arrives), and per-driver blocks `rplidar` (`device`: serial port, `tcp:<host>:<port>` or `auto`; `baud`) and `livox` (`sdk_library`, `host_ip`, `lidar_ips`). A planar LiDAR's matcher also takes the FC's EKF height and climb rate (`LOCAL_POSITION_NED`): it neither matches nor maps while the scan plane climbs or descends faster than 0.15 m/s, riding the IMU heading and the flow velocity, and keeps each submap to 0.3 m of height, since a planar scan cuts sloped walls at a different outline at every height |
| `navigation.depth` | Depth camera input: `source` (`off`, default; `realsense`, an Intel RealSense D435i or D455 through the `librealsense2` payload, depth at `realsense.width` × `realsense.height`, 848×480 or 640×480, and `realsense.fps`, 15 or 30, `realsense.serial` picking one camera; `we`, the World Engine depth stream, `ADOSDEPTH1` frames over UDP from `we_host` on `we_port`, 7810 plus the vehicle's SITL lease, subscribed every second; `vision`, the vision engine's metric camera depth, `vision.depth.v2` on the plugin bus, from the camera named in `vision_camera` (required), on which the service holds a depth lease while it runs, a frame mapped only while its scale fit is under `vision_max_fit_age_ms` old, 2000, and has inliers, the rest counted `unscaled`), and `extrinsic`, the camera's mount: `translation_m` in body forward-right-down from the body origin, and `rpy_deg` (roll, pitch, yaw, Z-Y-X; all zero looks forward, pitch -90 straight down) or `quat_wxyz`, not both; for `vision` without either, the pitch the engine reports. A RealSense needs the `hardware.usb` grant and `vision` the `vision.depth.read` grant; a missing library, camera or grant leaves the source unavailable and never stops the service |
| `navigation.pose_publish_interval_ms` | Interval between published navigation poses in the poses' own measurement time, in ms (100; 33 for 30 Hz, bounded by the provider's own rate). Kept on average; a pose up to half an interval early is published, so a provider at exactly the interval loses none. Poses are polled every 10 ms and published as soon as the gate allows |
| `navigation.takeoff_heading_deg` | The heading the vehicle faces where it starts, degrees clockwise from true north. Set, the navigation map's north is latched from it once at the first pose and never taken from the flight controller (pose `north: declared`), so the flight controller may take its yaw from the map (ArduPilot `EK3_SRCn_YAW` 6). Unset (default), the north follows the flight controller's heading until take-off (`north: fc_heading`), which must then not be the EKF's yaw source |

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
