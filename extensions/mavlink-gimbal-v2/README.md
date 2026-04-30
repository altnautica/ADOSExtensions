# MAVLink Gimbal v2 Controller

Cross-stack ADOS extension that controls a stabilised camera mount over
the open MAVLink Gimbal Manager Protocol v2. The agent half registers
component id 154 (`MAV_COMP_ID_GIMBAL`), translates GCS commands to
gimbal device messages, and publishes attitude on the event bus. The
GCS half mounts a control panel under Flight Control with manual
sliders, an ROI form, and a live state readout.

## Drivers

| Driver | Status | Path |
|--------|--------|------|
| MAVLink Gimbal v2 (ArduPilot SITL, Storm32 NT, Gremsy, any spec-compliant device) | Working | `agent/src/altnautica_gimbal_v2/mavlink_driver.py` |
| SimpleBGC native protocol over UART | Stub | `agent/src/altnautica_gimbal_v2/sbgc_driver.py` |
| Storm32 NT serial direct | Stub | `agent/src/altnautica_gimbal_v2/storm32_driver.py` |
| Gremsy serial direct | Stub | `agent/src/altnautica_gimbal_v2/gremsy_driver.py` |

The MAVLink driver is the first-class path. The serial drivers exist
as concrete `GimbalDriver` subclasses so vendors can fork the pattern;
they raise `NotImplementedError` from `open()` until their parsers land.

## Build

```sh
pnpm install
pnpm --filter ./extensions/mavlink-gimbal-v2/gcs build
pnpm --filter ./extensions/mavlink-gimbal-v2/gcs test

cd extensions/mavlink-gimbal-v2/agent
uv run pytest -q
```

`scripts/pack.sh mavlink-gimbal-v2` from the repo root produces a
signed-eligible `.adosplug` archive under `dist/`.

## Permissions

| Permission | Use |
|------------|-----|
| `agent.hardware.uart` | SimpleBGC bridging path. |
| `agent.mavlink.read` | Decode GCS and FC commands. |
| `agent.mavlink.write` | Emit gimbal manager and device messages. |
| `agent.mavlink.component.gimbal` | Register `MAV_COMP_ID_GIMBAL` (154). |
| `agent.event.publish` | Publish attitude and health events. |
| `agent.event.subscribe` | Read vehicle, attitude, position, command events. |
| `agent.telemetry.extend` | Add gimbal pitch/yaw/roll to telemetry. |
| `gcs.ui.slot.fc-tab` | Mount the control panel. |
| `gcs.ui.slot.video-overlay` | Mount the reticle. |
| `gcs.ui.slot.mission-template` | Add the orbit template. |
| `gcs.ui.slot.notification` | Surface health alerts. |
| `gcs.ui.slot.settings-section` | Settings UI under Settings -> Plugins. |
| `gcs.telemetry.subscribe.gimbal` | Read gimbal telemetry. |
| `gcs.telemetry.subscribe.mavlink` | Read related MAVLink messages. |
| `gcs.mission.read` and `gcs.mission.write` | Generate orbit missions. |
| `gcs.command.send` | Send gimbal commands to the agent. |

Risk band: medium. `command.send`, `mavlink.write`, and `hardware.uart`
trigger the host's "high" badge on `command.send`. No vehicle command,
no host file system, no network.

## MAVLink commands the driver issues

| Command id | Name | Use |
|------------|------|-----|
| 1000 | `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW` | Pitch and yaw setpoint plus rates. |
| 1001 | `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE` | Primary or secondary control assignment. |
| 195 | `MAV_CMD_DO_SET_ROI_LOCATION` | Lock on a lat/lon/alt target. |
| 197 | `MAV_CMD_DO_SET_ROI_NONE` | Release the ROI lock. |

Encoding follows the open MAVLink v2 wire format. The driver emits the
seven-argument `COMMAND_LONG` and `COMMAND_INT` byte payloads through
the agent's MAVLink router handle.

## Configuration

Edit transport, axis limits, behaviour, and pre-arm rules under
Settings -> Plugins -> Gimbal Controller. Schema lives at
`config-schema.json` and is rendered automatically by the host's JSON
Schema form.
