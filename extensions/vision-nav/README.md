# Vision Navigation

GPS-denied navigation for ADOS drones. The plugin ships a modular
estimator framework with six selectable modes covering optical flow
(with or without a rangefinder), monocular visual-inertial odometry
through two engines, and a hybrid mode that runs both side-by-side.

The agent half captures frames from a USB UVC or CSI camera, pairs
each frame with the closest IMU sample, runs the selected estimator,
and emits the matching MAVLink messages that ArduPilot and PX4 fuse
into their EKFs. The GCS half mounts a drone-detail tab with the
mode picker, sensors card, estimator card, telemetry charts, pre-arm
status, EKF source-set switcher, and a fallback banner that fires
when the estimator goes degraded or fails.

## Modes

| Mode | Estimator | Camera | IMU | Rangefinder | MAVLink |
|---|---|---|---|---|---|
| `off` | None | optional | optional | optional | none |
| `optical_flow` | Lucas-Kanade | downward | gyro | required | comp 198 |
| `optical_flow_degraded` | Lucas-Kanade | downward | gyro | optional | comp 198 |
| `vio_openvins` | OpenVINS | forward | gyro+accel | optional | comp 197 |
| `vio_vins_fusion` | VINS-Fusion | forward | gyro+accel | optional | comp 197 |
| `hybrid_of_plus_vio` | Both | downward + forward | gyro+accel | optional | comp 198 + 197 |

`optical_flow_degraded` is the rangefinder-free path. Scale comes
from a four-rung ladder (baro from `GLOBAL_POSITION_INT.relative_alt`,
raw baro from `VFR_HUD.alt`, GPS altitude when outdoors with a
healthy 3D fix, or a 1.5 m static fallback) with per-rung quality
multipliers so the EKF auto-de-weights degraded scale sources.

The two VIO modes spawn a signed vendor binary (`ados_openvins_shim`
or `ados_vins_fusion_shim`) via the plugin host's subprocess sandbox.
Camera frames flow into the binary through a shared-memory ring;
IMU and pose messages flow through a Unix-domain socket encoded as
length-prefixed msgpack. The plugin emits `VISION_POSITION_ESTIMATE`
on MAVLink component 197 (`MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`).

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

- USB UVC global-shutter modules (preferred for VIO; the Arducam
  B0332 OV9281 mono module is a known-good pick)
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

The VIO modes need calibrated camera intrinsics + camera-IMU
extrinsics + the static time offset between the two clocks. The
plugin accepts Kalibr-compatible `camchain.yaml` files directly.
Both layouts work (`cam0` wrapper or bare block); the loader reads
only `cam0`. Multi-camera files (`cam1`, `cam2`, ...) are accepted
with the additional cameras ignored.

See the
[Calibration page](https://docs.altnautica.com/drone-agent/vision-nav-calibration)
for the full walkthrough.

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

The vendor binaries (`ados_openvins_shim`, `ados_vins_fusion_shim`)
are built by a separate CI job and signed under the same Ed25519 key
as the rest of the plugin archive. Without the binaries the plugin
falls back to OF-only modes; selecting a `vio_*` mode without the
binaries surfaces a clear "vendor binary missing" error in the
install dialog.

## Surfaces contributed

| Slot | Purpose |
|------|---------|
| `drone.detail.tab` | "Vision Nav" configuration tab on the drone detail panel |
| `video.overlay` | Reserved for flow-vector and quality overlays |
| `notification.channel` | Channel for degraded-navigation alerts |

The drone-detail tab hosts:

- Mode picker (six options filtered by the agent's available
  estimators)
- Sensors card (camera + IMU + rangefinder rows with sync-offset
  pill and Calibrate CTA)
- Estimator card (engine + state + feature count + drift + sync
  offset + reset counter)
- Telemetry charts (inline-SVG sparklines for flow quality, sync
  offset, feature count, drift)
- Flow health card (live OF metrics; OF modes only)
- Pre-arm status (mode-aware check rows)
- EKF source-set switcher (strict-gated on `vioSupported` so the
  VIO button is hidden when no VIO engine is wired)
- Fallback banner (fires when the estimator is degraded or failed
  with reason plus suggested next action)

## Permissions

Agent:

- `hardware.usb.uvc`, `hardware.camera.csi`, `hardware.uart`,
  `hardware.i2c`
- `sensor.camera.register`, `sensor.depth.register`
- `telemetry.extend`, `event.publish`, `event.subscribe`
- `mavlink.read`, `mavlink.write`
- `mavlink.component.peripheral`, `mavlink.component.vio`
- `estimator.pose.inject`
- `process.spawn` (for VIO vendor binaries)

GCS:

- `ui.slot.drone-detail-tab`, `ui.slot.video-overlay`,
  `ui.slot.notification-channel`
- `telemetry.subscribe`, `command.send`

Risk band: high. The plugin writes MAVLink, registers as a vision
component, and can inject pose data into the firmware estimator.
Grant only on drones where vision navigation is required.

## Configuration

See `config-schema.json` for the per-drone configuration form. Key
fields:

- `mode`: one of `off`, `optical_flow`, `optical_flow_degraded`,
  `vio_openvins`, `vio_vins_fusion`, `hybrid_of_plus_vio`.
- `camera`: device path, bus type (`uvc` or `csi`), resolution, and
  frame rate.
- `rangefinder`: topology (`companion`, `fc`, `none`), driver,
  optional device node, optional baud.
- `firmware`: `ardupilot` or `px4`, plus the EKF source-set index
  (ArduPilot only).
- `pre_arm`: optional auto-set of the EKF origin so the firmware
  arms before GPS lock.

## MAVLink components

- Component 198, `sub_id` 1: peripheral. Emits `OPTICAL_FLOW_RAD`
  and `DISTANCE_SENSOR` (when relaying the rangefinder).
- Component 197, `sub_id` 1: `MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY`.
  Emits `VISION_POSITION_ESTIMATE` when a VIO mode is active.

The component router reads each `EstimatorOutput.output_mode` and
picks the matching component per tick. Hybrid mode emits on both
components in parallel.

## Vendor binaries

The VIO modes use third-party open-source estimators packaged inside
the signed `.adosplug` archive:

- **OpenVINS** (https://github.com/rpng/open_vins), GPL-3.0-only.
  Used by `vio_openvins`. Filter-based MSCKF estimator. Lower CPU
  cost than VINS-Fusion.
- **VINS-Fusion** (https://github.com/HKUST-Aerial-Robotics/VINS-Fusion),
  GPL-3.0-only. Used by `vio_vins_fusion`. Sliding-window
  bundle-adjustment estimator. Higher CPU cost, tighter drift bound
  in feature-rich scenes.

Both binaries ship with their respective `LICENSE` text inside
`vendor/<engine>/LICENSE` and the manifest's `vendor_attribution`
block names them explicitly per GPL §6. The install dialog surfaces
the attribution at Stage 1 so operators see what is being installed.

## License

GPL-3.0-or-later. See the repository `LICENSE` file.
