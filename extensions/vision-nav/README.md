# Vision Navigation

GPS-denied navigation for ADOS drones. The plugin ships a modular
estimator framework with three selectable modes: optical flow with a
rangefinder, optical flow without one, and off.

The agent half is a compiled **Rust** binary. It does not open a
camera: it subscribes to the shared vision frame bus, pairs each frame
with the closest IMU sample, runs the selected estimator, and emits the
flow message for the configured firmware (`OPTICAL_FLOW` for ArduPilot
and iNav, `OPTICAL_FLOW_RAD` for PX4; plus `DISTANCE_SENSOR` when the
rangefinder is companion-wired) that the flight controller fuses into
its estimator. It asks the flight controller for `RAW_IMU` at 100 Hz
and `DISTANCE_SENSOR` at 20 Hz. The GCS half mounts a node-detail tab with the mode
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

`optical_flow_degraded` is the rangefinder-free path. Scale comes
from a ladder: baro from `GLOBAL_POSITION_INT.relative_alt`, raw baro
from `VFR_HUD.alt`, then (only with `outdoor: true`) GPS height above
the first good 3D fix, then a 1.5 m static fallback. Each rung carries
a quality multiplier so the EKF de-weights degraded scale sources. The
static rung blocks the pre-arm scale check; the pre-arm gate is
advisory and does not stop the flight controller from arming.

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
- Fallback banner (fires when the estimator is degraded or failed
  with reason plus suggested next action)

## Permissions

Agent:

- `vision.frame.read` (reads normalized frames from the shared vision
  frame bus; the plugin does not open a camera, so the camera / UVC /
  CSI / depth permissions are not requested)
- `hardware.uart` (the companion-wired TF-Luna)
- `telemetry.extend`, `event.publish`
- `mavlink.read`, `mavlink.write`
- `mavlink.component.vio` (the host reserves component 198 for kind
  `vio`)

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
  `optical_flow_degraded`.
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

The agent reads these keys live from the per-drone store once a second
and restarts itself when any of them changes.

## MAVLink components

- Component 198, `sub_id` 1: kind `vio`. Emits `OPTICAL_FLOW` or
  `OPTICAL_FLOW_RAD` (by firmware), the `MAV_CMD_SET_MESSAGE_INTERVAL`
  stream requests,
  `DISTANCE_SENSOR` (when relaying a companion-wired rangefinder),
  `TIMESYNC`, and the 1 Hz companion `HEARTBEAT`.

No other component is claimed, and no pose is injected into the
firmware estimator.

## License

GPL-3.0-or-later. See the repository `LICENSE` file.
