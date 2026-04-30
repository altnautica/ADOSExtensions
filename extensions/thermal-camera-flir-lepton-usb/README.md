# Thermal Camera FLIR Lepton USB UVC

Hybrid extension that adds FLIR Lepton 3.5 radiometric thermal imaging
to ADOS. The agent half opens a PureThermal 2 USB UVC dongle, decodes
Y16 frames into per-pixel kelvin via TLinear, registers a MAVLink
camera component, and publishes frames on the event bus. The GCS half
mounts a thermal video overlay with a draggable spot meter, three
palettes, and a configurable isotherm band.

The native libuvc binding is not wired in this version. The driver
talks to a `LibUvcBackend` Protocol; the in-tree implementation is a
`MockUvcBackend` that produces synthetic Y16 frames with a hot region
in the center. The real binding lands when hardware procurement
closes.

## Build

```sh
pnpm install
pnpm --filter ./extensions/thermal-camera-flir-lepton-usb/gcs build
pnpm --filter ./extensions/thermal-camera-flir-lepton-usb/gcs test
```

Agent tests:

```sh
cd extensions/thermal-camera-flir-lepton-usb/agent
uv run pytest -q
# or, without uv:
python -m pytest -q
```

To produce a `.adosplug`:

```sh
scripts/pack.sh thermal-camera-flir-lepton-usb
```

## Surfaces contributed

| Slot | Purpose |
|------|---------|
| `video.overlay` | Live thermal canvas above the visible-camera pane. |
| `fc.tab` | "Thermal Camera" configuration tab. |
| `mission.template` | "Thermal survey grid" lawnmower mission generator. |
| `notification.channel` | Alarm channel for max-temperature events. |
| `settings.section` | Plugin settings. |

## Permissions

Agent: `hardware.usb.uvc`, `sensor.camera.register`, `telemetry.extend`,
`event.publish`, `event.subscribe`, `recording.write`,
`mavlink.component.camera`.

GCS: `ui.slot.fc-tab`, `ui.slot.video-overlay`,
`ui.slot.mission-template`, `ui.slot.notification`,
`ui.slot.settings-section`, `telemetry.subscribe.thermal`,
`telemetry.subscribe.mavlink`, `mission.read`, `mission.write`,
`recording.write`.

Risk band: medium. No vehicle command, no MAVLink write, no network.

## TLinear conversion

The Lepton 3.5 in radiometric mode emits Y16 raw pixels at a fixed
resolution per count. The agent half computes:

```
temperature_K = raw_y16 / 100.0
temperature_C = temperature_K - 273.15
```

at 0.01 K per count. Implementation in `agent/.../tlinear.py`.

## Palettes

Three 256-entry RGB palettes ship in v1.0.0:

| Palette | Visual character |
|---------|-------------------|
| `ironbow` | Black to purple to red to yellow to white. Hot is bright. |
| `rainbow` | Blue to cyan to green to yellow to red. Quantitative. |
| `grayscale` | Black to white. Linear. |

The Python and TypeScript palette tables share the same anchor-stop
formulas so playback in the GCS matches the agent's encode-side
colorize step. A fourth `arctic` palette is reserved for v1.1.

## Hardware

PureThermal 2 (GroupGets) plus FLIR Lepton 3.5 plus a USB-C cable.
Detail in the spec. The native libuvc binding is deferred until the
hardware kit lands on the bench.
