# Changelog

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
