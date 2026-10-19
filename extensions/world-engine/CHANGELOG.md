# Changelog

All notable changes to the World Engine extension.

## [Unreleased]

### Added

- **Capture contract `ados.capture/1`.** MIT schema files under `contract/`
  (JSON Schemas, protobuf messages, golden fixtures including a tiny MCAP
  session), the `ados-capture-contract` crate generated from them with an MCAP
  session writer and reader, and generated TypeScript types for the GCS half.
- The capture service and link run on the `visor` profile.
- **Session upload lane.** `/api/compute/sessions/<id>`: manifest POST,
  resumable `PUT files/<rel>` with `Content-Range`, `HEAD` offsets, and a
  commit that verifies every file against `checksums.b3`.
- **Staged reconstruction.** Job kind `reconstruct {session_id, preset}` with
  presets `preview`, `hq`, `tour`, `warehouse`, run as persisted stages
  (prepare, sfm, train, mesh, products) that resume after a node restart. Job
  status carries the stage, per-stage rows, GPU, ETA, tool versions and a
  structured error.
- Deliverables: SPZ and `.rad` splats, collision mesh, LAZ point cloud, ESDF,
  placement sidecars, a dataset export (COLMAP model + `transforms.json`), and
  for indoor presets a floorplan with rooms, aisles and a static tour bundle.
- Job kinds `change {a, b}` (cloud-to-cloud change detection) and
  `calibrate {session_id, target}` (IMU noise, intrinsics, camera-IMU,
  LiDAR-camera); `POST /api/compute/measure`; job files served with ranges.
- Pinned tool set (`agent/node/tools/tool-pins.json`) with fetch scripts.
- A standalone node (`--standalone`) and a Windows build of the node.

### Changed

- **Atlas envelope version 2.** Keyframes carry the contract camera model,
  geo anchor, a timebase-stamped exposure time and IMU samples with `t_ns` and
  `seq`; every artifact descriptor carries a placement. Version 1 frames are
  refused.
- `atlas.intrinsics.<camera>` takes the contract camera model: `model`
  (`pinhole`, `opencv`, `opencv_fisheye`, `kb4`), `fx`, `fy`, `cx`, `cy`,
  `dist`, `readout_ns`.
- The live-lane trainers resolve from the pinned tool set and fail with
  `E_TOOL_MISSING` instead of producing a placeholder.

### Removed

- The nerfstudio backend, the monocular-depth seed script and the random seed
  cloud; the placeholder reconstructor in release builds; the SLAM offload job
  kind.
- Compute-node datasets are written as the contract `transforms.json`, with
  the session anchor and per-frame exposure times.
- `atlas.intrinsics` and `atlas.pose_prior` refuse unknown keys, so a
  pre-contract override fails with an error naming the accepted keys.
- `atlas.pose_prior.clock_offset_sigma_ns` is renamed `timebase_sigma_ns`.
- The session anchor is the mean of the first 100 3D fixes, with its MSL
  altitude raised to the ellipsoid by the EGM2008 geoid (1° grid).
- A selected frame whose pose arrived more than 50 ms from the exposure is
  not sent as a keyframe; the count is logged.

## [1.0.1]

### Changed

- **Perception offload holds its own advertise grant.** The drone's offload
  link advertisement now needs `vision.offload.advertise`, so the manifest
  requests it; without it the drone refuses the advertisement and reports no
  offload tier. Requires agent 0.102.0 or newer.
- **Offloaded detections arrive bound to the extension.** The agent files the
  returned batches under `com.altnautica.world-engine/offload` and drops their
  remote track ids, because the extension does not hold track designation.
- **Install-time payloads come from GitHub releases only.** The local bench
  pack (`scripts/pack-world-engine-local.sh`) and `scripts/pack-rust.sh` now
  match the agent's download allowlist, which no longer admits S3 or
  `localhost` sources.
- The agent half reads its data directory from the launch values the host
  delivers as a credential, not from the process environment.

## [1.0.0]

First release. World-model capture, reconstruction and perception offload
run as an extension, installed on the nodes that use them.

### Added

- **Atlas capture on drones.** A capture service selects keyframes from the
  shared vision frame bus, tags each with its pose (flight-controller pose,
  offloaded SLAM, or both), and streams them to a paired compute node over
  the best available bearer: direct LAN, the WFB relay through a ground
  station, or the cloud. Start, pause, resume and stop run from the GCS.
- **Compute node on workstations and compute nodes.** Keyframe ingest, a job
  scheduler, and reconstruction through the host's COLMAP, Brush, msplat or
  nerfstudio, with an optional monocular-depth seed. Artifacts are stored on
  the node, logged to a Rerun recording, and announced on a per-drone
  world-model stream. Each output names the backend that produced it.
- **Perception offload.** A drone can send its detection work to a compute
  node (`offload.*`); the compute node serves it (`serving.*`).
- **Ground-station relay** of capture events heard over the air to the
  compute node (`relay.*`).
- **Node credentials.** The GCS issues each paired drone and ground station a
  credential scoped to the compute node's lanes, installs it on that node,
  re-provisions when the compute node no longer honours it, and lets the
  operator revoke it.
- **GCS pages.** `world-model`, `world-model-setup` and `live-world` on
  drones; `offload` on drones, workstations and compute nodes; `atlas-relay`
  on ground stations; and the Compute overview and Jobs & outputs surfaces on
  workstations and compute nodes. Viewers for Gaussian splats, point clouds
  and Rerun recordings, with a badge on any placeholder reconstruction.
- **Installer toggle**, on by default for the `workstation` and `compute`
  profiles.
