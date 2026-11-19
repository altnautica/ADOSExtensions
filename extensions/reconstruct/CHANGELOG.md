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
- The Reconstruct surface: Jobs, Products, Editor, device access and
  perception serving.
- The point-cloud editor: box and polygon selection with a height band,
  crop, delete, patch (a copy placed as its own layer), layer delete,
  decimation to a point target with its deviation, measurement (distance,
  area, height, volume, with an exact full-resolution volume) and export to
  LAZ or PLY in the product's frame or WGS-84 ECEF with a manifest. Every
  operation that rewrites points runs on the node as an `edit` job and
  writes the next version (`cloud.v1.laz`, `cloud.v2.laz`, …) beside an
  append-only provenance record (`edits.json`); undo opens the previous
  version. A second tab on the same product opens read-only.
- `POST /api/compute/edit/estimate` (predicted export size) and
  `POST /api/compute/edit/volume` on the node.
- The compute status `gpu` block carries `present`; a node with no hardware
  GPU says once that reconstruction runs on the CPU.

### Changed

- Needs ADOS Drone Agent 0.122.0 or later (`compatibility.ados_version`):
  it relies on that release's topic and route layout.

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
