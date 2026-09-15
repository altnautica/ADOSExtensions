> **Development preview. Not published and not installable.**
>
> The capture path depends on a native libuvc backend that does not exist in
> this repository, so there is no configuration in which this extension
> produces a thermal reading. It is excluded from the release tag patterns in
> `.github/workflows/release.yml`, so no archive of it is published. What is
> present and covered by tests is the driver, the TLinear conversion, the spot
> meter, the palettes, the config control loop and the overlay; they become
> functional when a concrete backend is supplied. Until then the plugin reports
> an explicit unavailable state and publishes no readings.

# Thermal Camera FLIR Lepton USB UVC

Hybrid extension that adds a FLIR Lepton 3.5 radiometric thermal driver
and cockpit surfaces to ADOS. The agent half drives a PureThermal 2 USB
UVC dongle: it discovers and opens the device, locks the radiometric
linear resolution, converts Y16 counts to degrees Celsius via TLinear,
meters a centre-reticle spot plus the frame extrema, and publishes that
read-back for the overlay. The GCS half mounts a thermal video overlay
with a spot meter, three palettes, and a configurable isotherm band.

**Reaching the dongle needs a native libuvc backend, and this repository
does not contain one.** The driver talks to the `LibUvcBackend` Protocol
and the agent injects a concrete backend through
`ThermalUsbPlugin(backend_factory=...)`. With no backend injected, no
device on the bus, a failed open, or a frame stream that will not start,
the plugin logs an error, publishes `{"connected": false, "reason": ...}`
on the `thermal` state channel, and starts neither its control loop nor
its frame stream. It never synthesises a temperature: the overlay shows
why the camera is unavailable rather than a plausible-looking reading.
The unit suite's synthetic backend lives in `agent/tests/mock_backend.py`
and is not importable from the installed package.

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
| `video.overlay` | Thermal canvas, spot readout, and palette rail above the video pane. |
| `settings.section` | Native palette and gain settings the agent reads each control tick. |
| Flight Skills | Cycle palette and flat field correction, fired from the Skill Bar. |

## Telemetry channels

| Channel | Payload |
|---------|---------|
| `thermal` | State read-back: `connected`, plus `reason` when there is no open session, plus `palette` and `gain`. |
| `camera.thermal.frame` | Frame read-back only, published only while a session is open: `width`, `height`, `spot`, `minC`, `maxC`, `resolutionKPerCount`. |
| `camera.thermal.palette`, `camera.thermal.ffc` | Per-Skill state; `disabled` with the reason while there is no session. |

## Permissions

Agent: `hardware.usb.uvc`, `video.source.set`, `telemetry.extend`,
`event.publish`, `mavlink.component.camera`.

GCS: `ui.slot.video-overlay`, `ui.slot.settings-section`,
`ui.slot.flight-skill`, `telemetry.subscribe.thermal`,
`telemetry.subscribe.camera.thermal.frame`.

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

Three 256-entry RGB palettes ship:

| Palette | Visual character |
|---------|-------------------|
| `ironbow` | Black to purple to red to yellow to white. Hot is bright. |
| `rainbow` | Blue to cyan to green to yellow to red. Quantitative. |
| `grayscale` | Black to white. Linear. |

The Python and TypeScript palette tables share the same anchor-stop
formulas so the cockpit render matches the agent's colorize step.

## Hardware

PureThermal 2 (GroupGets) plus FLIR Lepton 3.5 plus a USB-C cable. The
native libuvc backend is supplied by the agent, not by this repository;
see the note at the top.

## Why thermal is an overlay and not a cockpit video stream

The agent's video pipeline serves named streams and the cockpit stream switcher
flips between them, so a thermal leg would appear beside the primary EO feed.
This plugin does not advertise one, for a concrete missing dependency: unlike an
IP pod that already serves RTSP, the Lepton feed is per-pixel Y16 that this
plugin colorizes in-process, so a stream leg needs a colorized-stream source
(an RTSP or MJPEG endpoint the pipeline can pull) that nothing here provides.

`ctx.video.set_source([...])` is wired and is called when the `stream_source`
config key names an endpoint. With no endpoint configured no leg is advertised,
so the stream switcher never offers one that cannot be served.
