# Vision Navigation

GPS-denied navigation for ADOS drones. The plugin ships a modular
estimator framework with three selectable modes: optical flow with a
rangefinder, optical flow without one, and off.

The agent half is a compiled **Rust** binary. It does not open a
camera: it subscribes to the shared vision frame bus, pairs each frame
with the closest IMU sample, runs the selected estimator, and emits the
`OPTICAL_FLOW_RAD` (and, when the rangefinder is companion-wired,
`DISTANCE_SENSOR`) messages that ArduPilot, PX4, and iNav fuse into
their estimators. The GCS half mounts a node-detail tab with the mode
picker, sensors card, estimator card, telemetry charts, pre-arm status,
EKF source-set switcher, and a fallback banner that fires when the
estimator goes degraded or fails.

The one-time, offline camera-IMU **calibration wizard stays Python**
(`agent/calibration-helper/`): it is an infrequent OpenCV-bound flow
(AprilTag detection, intrinsics solve, timeshift fit) that runs far off
the 30 Hz emit path. It produces a Kalibr-style `camchain.yaml`; the
Rust agent reads the camera-IMU timeshift out of that file at start-up
and uses it to align frames with IMU samples.

## Modes

| Mode | Estimator | Camera | IMU | Rangefinder | MAVLink |
|---|---|---|---|---|---|
| `off` | None | optional | optional | optional | none |
| `optical_flow` | Lucas-Kanade | downward | gyro | required | comp 198 |
| `optical_flow_degraded` | Lucas-Kanade | downward | gyro | optional | comp 198 |

`optical_flow_degraded` is the rangefinder-free path. Scale comes
from a four-rung ladder (baro from `GLOBAL_POSITION_INT.relative_alt`,
raw baro from `VFR_HUD.alt`, GPS altitude when outdoors with a
healthy 3D fix, or a 1.5 m static fallback) with per-rung quality
multipliers so the EKF auto-de-weights degraded scale sources.

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

The plugin auto-detects the active firmware and tunes its MAVLink
output accordingly.

## Supported cameras

- USB UVC global-shutter modules (the Arducam B0332 OV9281 mono module
  is a known-good pick)
- USB UVC rolling-shutter modules
- CSI modules exposed through V4L2

## Supported rangefinders

When a rangefinder is wired, the OF modes consume:

- TF-Luna over UART
- Garmin LIDAR-Lite v3 / v4 over I2C
- ST VL53L1X over I2C
- `fc_relay`: the FC owns the rangefinder and the plugin reads its
  `DISTANCE_SENSOR` stream

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

- Mode picker (the modes the agent advertises as runnable)
- Sensors card (camera + IMU + rangefinder rows with sync-offset
  pill and Calibrate CTA)
- Estimator card (engine + state + flow quality + sync offset)
- Telemetry charts (inline-SVG sparklines for flow quality and sync
  offset)
- Flow health card (live OF metrics)
- Pre-arm status (mode-aware check rows)
- EKF source-set switcher (GPS and optical-flow source sets; ArduPilot
  only, gated on companion health and flow quality)
- Fallback banner (fires when the estimator is degraded or failed
  with reason plus suggested next action)

## Permissions

Agent:

- `vision.frame.read` (reads normalized frames from the shared vision
  frame bus; the plugin does not open a camera, so the camera / UVC /
  CSI / depth permissions are not requested)
- `hardware.uart`, `hardware.i2c` (companion-wired rangefinders)
- `telemetry.extend`, `event.publish`
- `mavlink.read`, `mavlink.write`
- `mavlink.component.peripheral`

GCS:

- `ui.slot.node-detail-tab`, `ui.slot.flight-skill`
- `telemetry.subscribe.navigation`, `command.send`

Risk band: high. The plugin writes MAVLink and feeds a velocity source
the flight controller's estimator fuses for position hold, so a bad
flow feed moves the aircraft. Grant only on drones where vision
navigation is required.

## Configuration

See `config-schema.json` for the per-drone configuration form. Key
fields:

- `mode`: one of `off`, `optical_flow`, `optical_flow_degraded`.
- `camera`: device path, bus type (`uvc` or `csi`), resolution, and
  frame rate.
- `rangefinder`: topology (`companion`, `fc`, `none`), driver,
  optional device node, optional baud.
- `firmware`: `ardupilot`, `px4`, or `inav`, plus the EKF source-set
  index (ArduPilot only).
- `pre_arm`: optional auto-set of the EKF origin so the firmware
  arms before GPS lock.

## MAVLink components

- Component 198, `sub_id` 1: peripheral. Emits `OPTICAL_FLOW_RAD`,
  `DISTANCE_SENSOR` (when relaying a companion-wired rangefinder),
  `TIMESYNC`, and the 1 Hz companion `HEARTBEAT`.

No other component is claimed, and no pose is injected into the
firmware estimator.

## License

GPL-3.0-or-later. See the repository `LICENSE` file.
