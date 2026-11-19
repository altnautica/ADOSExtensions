# Changelog

All notable changes to the Reconstruct extension.

## [1.0.0] (unreleased)

Reconstruct: the reconstruction node and perception offload, as their own
extension (`com.altnautica.reconstruct`).

### Added

- `reconstruct-offload`, the drone's process: the perception-offload
  reconciler, the live scan stream (Navigation's placed scans forwarded to
  the node as `reconstruct.live.scan` while the map is built there), the
  `reconstruct`, `job_status` and `tour_bundle` tools, and
  `/reconstruct/{status,config,node-credential}`.
- The Reconstruct surface: Jobs, Products, Editor (a finished cloud,
  read-only), device access and perception serving.

### Changed

- The node's capture ingest is `POST /api/capture/event` (probe
  `GET /api/capture/health`), its world stream `GET /ws/live/:device_id` with
  topics `reconstruct.{splat,pointcloud,mesh,occupancy}` and
  `reconstruct.live.{occupancy,pointcloud}`, and the credential lanes
  `capture.ingest` and `reconstruct.live`.
- A session reconstructs live while its drone streams scans to the node,
  instead of reading the drone's map status.
- Configuration keys are `reconstruct.offload.*`, `reconstruct.offload_camera`
  and `reconstruct.serving.*`; the status telemetry is `{"compute": …}`.
- The live reconstruction switches are `ADOS_RECONSTRUCT_LIVE*`.

### Removed

- Capture (sessions, keyframe selection, uploads, the relay) moved to the
  Capture extension; the navigation pose and live map to the Navigation
  extension.
