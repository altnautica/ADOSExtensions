# Changelog

## 0.3.1

- The console now has a per-leg source selector (main / sub) so the operator
  can reassign a leg to EO-wide or the on-pod split/PiP composite through the
  plugin; previously those sources were unreachable (the reassignment command
  had no UI). The live assignment is published in the pod state so each
  selector shows the current source.
- The pod's on-pod AI-track box is now read each telemetry tick and
  republished onto the shared `vision.detection` bus (stamped with the primary
  leg id), so cockpit click-to-track, the locked-target gate, and track
  geolocation work; a track drop publishes one "lost" batch. The exact
  AI-track output wire format is a placeholder resolved on the bench.

## 0.3.0

- Two-stream model corrected: the pod serves exactly two concurrent RTSP
  streams (`main` + `sub`), each assignable to a sensor, so a multi-sensor
  pod (ZT6/ZT30) now advertises exactly those two legs (dropping the phantom
  `/ir` leg) with distinct sources on start — `main` = EO-zoom, `sub` = IR.
  The console reaches EO-wide or the on-pod split/PiP composite by reassigning
  a leg's source (a new image-source command + `stream_assignment` config key;
  the exact SIYI opcodes are placeholders resolved on the bench).
- The republished AI-track box is stamped with the primary advertised leg id
  (`main`) so the cockpit overlay renders it; the stale camera-id config key
  is removed.
- The plugin no longer fails to start when the pod is unreachable at boot: it
  keeps the conservative fallback profile and re-negotiates until the pod
  answers, then brings the gimbal, telemetry, and video online.
- Capability profile reconciled: `sensors` are `eo_zoom` / `eo_wide` / `ir`,
  with a distinct assignable-`streams` set (+ `split`) and `supports_pip`.

## 0.2.0

- Multi-stream video: on start the plugin advertises one video leg per sensor
  the pod has (zoom/EO on `main`, wide EO on `sub`, thermal on `ir`), so a
  multi-sensor pod like the ZT30 serves all its sensors at once and the cockpit
  stream switcher flips between them. Auto-configured over the agent's video
  facade (requires agent 0.99.177+); older agents fall back to operator config.
- Removed the pod-side sensor-mode mux — each sensor is now its own stream, so
  the panel keeps only the per-sensor controls.

## 0.1.0

- Initial agent half: SIYI SDK frame codec (CRC16/XMODEM, verified against the
  SDK heartbeat), command set, and per-model capability negotiation across the
  optical-pod line (A2 mini, A8 mini, ZR10, ZR30, ZT6, ZT30).
- Transport backends: UDP, TCP, and TTL serial, plus an in-memory mock for
  hardware-free tests.
- Single-owner session with sequence-correlated replies, push fan-out, a
  serialized command queue, and a liveness counter.
- Capability-gated pod facade (gimbal, zoom, focus, photo/record, thermal, laser,
  AI track), the plugin lifecycle with a config-driven control loop, laser
  subject geolocation, the MAVLink interop bridge (gimbal attitude + distance),
  and the tracker republish onto the shared detection bus.
