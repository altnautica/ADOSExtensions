# ADOS SIYI Optical Pod

A driver plugin for the SIYI optical-pod line. It speaks the SIYI Gimbal Camera
External SDK directly on the companion computer, so the agent controls the pod's
gimbal, zoom, thermal, laser rangefinder, and on-pod AI tracker — not just the
aim / zoom / photo / record a stock autopilot mount reaches.

## Supported models

One driver covers the whole line through per-model capability negotiation: the
plugin queries the pod's firmware and hardware id on connect, resolves the
model, and exposes only the controls it supports.

| Model | Gimbal | Optical zoom | Thermal | Laser | AI track |
|-------|--------|--------------|---------|-------|----------|
| A2 mini | fixed | — | — | — | — |
| A8 mini | yes | digital | — | — | yes |
| ZR10 | yes | yes | — | yes | — |
| ZR30 | yes | yes | — | yes | — |
| ZT6 | yes | — | yes | — | yes |
| ZT30 | yes | yes | yes | yes | yes |

## How it connects

- **Control + telemetry**: one SIYI SDK session over UDP (default
  `192.168.144.25:37260`), TCP, or a TTL serial port (115200 8N1).
- **Video**: the pod's RTSP streams (`rtsp://192.168.144.25:8554/…`) are ingested
  by the agent's video pipeline and served over WHEP; the plugin configures the
  source and drives the pod, it does not decode video.
- **Flight controller / other ground stations**: the plugin registers as a
  MAVLink gimbal component (154) and camera component (100) and mirrors gimbal
  attitude and the laser range, so a standard gimbal panel and the autopilot see
  the pod.

## Features

- Gimbal aim, rate, recenter, and lock / follow / FPV modes.
- Optical and absolute zoom and autofocus on zoom models.
- Thermal palette, gain, and spot temperature on thermal models.
- Laser rangefinder with subject geolocation: the measured slant range plus the
  gimbal angles and the aircraft pose resolve a subject latitude/longitude, which
  drops a marker on the map and can be mirrored to the flight controller.
- On-pod AI tracking republished onto the shared detection bus, so cockpit
  click-to-track and follow behaviours work with no on-board accelerator.

## Install

```
ados plugin install siyi-pod-<version>.signed.adosplug
```

Set the transport and address in the plugin's settings (defaults suit a pod on
the standard `192.168.144.x` network). Then open the **SIYI Pod** tab on the
drone to control it.

## Configuration

See `config-schema.json` for the connection settings (transport, host, port,
serial port, system id, camera id). Per-drone controls (zoom, sensor mode, gimbal
mode, palette, gain, laser arm, track) are exposed as parameters in the GCS and
written through the plugin's per-drone config.

## License

GPL-3.0-or-later.
