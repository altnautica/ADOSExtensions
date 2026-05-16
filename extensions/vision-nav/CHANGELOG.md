# Changelog

All notable changes to the Vision Navigation extension.

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
- Auto-takeoff with vision navigation has been bench-validated at
  ground clearances of 1.5 m to 3 m. Higher takeoff altitudes work
  but are not bench-validated.
