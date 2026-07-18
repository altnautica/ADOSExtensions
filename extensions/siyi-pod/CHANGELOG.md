# Changelog

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
