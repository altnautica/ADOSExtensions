# Vision Navigation

Hybrid extension that adds GPS-denied navigation to ADOS via a downward
optical-flow sensor pipeline. The agent half captures frames from a USB
UVC or CSI camera, runs a Lucas-Kanade flow estimator, scales the
result with a height source (companion rangefinder or relayed
`DISTANCE_SENSOR`), and emits the `OPTICAL_FLOW_RAD` MAVLink message
that ArduPilot and PX4 fuse into their EKFs. The GCS half mounts a
drone-detail tab that surfaces flow quality, flow rate, rangefinder
height, EKF source-set state, and a pre-arm origin helper.

Supported firmware:

- ArduPilot Copter 4.5 and newer (`FLOW_TYPE=5`, `EK3_SRC*_VELXY`
  source set wired to OpticalFlow).
- PX4 1.14 and newer (`EKF2_OF_CTRL=1` plus the EKF flow-fusion
  parameters).

Supported cameras:

- USB UVC global-shutter and rolling-shutter modules.
- CSI modules exposed through V4L2.

Supported rangefinders for height scaling:

- TF-Luna over UART.
- Garmin LIDAR-Lite v3 / v4 over I2C.
- ST VL53L1X over I2C.
- `fc_relay`: the flight controller is the rangefinder owner and the
  plugin only consumes its `DISTANCE_SENSOR` stream.

## Build

```sh
pnpm install
pnpm --filter ./extensions/vision-nav/gcs build
pnpm --filter ./extensions/vision-nav/gcs test
```

Agent tests:

```sh
cd extensions/vision-nav/agent
uv run pytest -q
# or, without uv:
python -m pytest -q
```

To produce a `.adosplug`:

```sh
scripts/pack.sh vision-nav
```

## Surfaces contributed

| Slot | Purpose |
|------|---------|
| `drone.detail.tab` | "Vision Nav" configuration tab on the drone detail panel. |
| `video.overlay` | Reserved for future flow-vector and quality overlays. |
| `notification.channel` | Channel for degraded-navigation alerts. |

## Permissions

Agent: `hardware.usb.uvc`, `hardware.camera.csi`, `hardware.uart`,
`hardware.i2c`, `sensor.camera.register`, `sensor.depth.register`,
`telemetry.extend`, `event.publish`, `event.subscribe`,
`mavlink.read`, `mavlink.write`, `mavlink.component.vio`,
`estimator.pose.inject`.

GCS: `ui.slot.drone-detail-tab`, `ui.slot.video-overlay`,
`ui.slot.notification-channel`, `telemetry.subscribe`,
`command.send`.

Risk band: high. The plugin writes MAVLink, registers as a vision
component, and can inject pose data into the firmware estimator. Grant
only on drones where vision navigation is required.

## Configuration

See `config-schema.json` for the per-drone configuration form. Key
fields:

- `mode`: `off` or `optical_flow`. Future releases add additional
  modes.
- `camera`: device path, bus type (`uvc` or `csi`), resolution, and
  frame rate.
- `rangefinder`: topology (`companion`, `fc`, `none`), driver, optional
  device node, optional baud.
- `firmware`: `ardupilot` or `px4`, plus the EKF source-set index
  (ArduPilot only).
- `pre_arm`: optional auto-set of the EKF origin so the firmware will
  arm before GPS lock.

## MAVLink component

The plugin registers a VIO peripheral component with `component_id`
198, `sub_id` 1. It emits `OPTICAL_FLOW_RAD` and consumes
`DISTANCE_SENSOR` when the rangefinder topology is `fc`.

## License

GPL-3.0-or-later. See the repository `LICENSE` file.
