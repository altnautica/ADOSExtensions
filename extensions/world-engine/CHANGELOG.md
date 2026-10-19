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
- **Device capture surface.** The link serves `POST /atlas/capture/{start,
  stop,pause,resume,mark}`, `GET /atlas/capture/{state,coverage}`,
  `PUT /atlas/capture/exposure`, `GET /atlas/sessions`,
  `POST /atlas/sessions/<id>/{upload,reconstruct}`, `DELETE /atlas/sessions/<id>`
  and `GET /atlas/calib[/<version>]` over control protocol v2, with
  `{"error": CODE, "detail": ..}` refusals. `whep` and `PUT /atlas/rig` answer
  `503` (`E_WHEP_UNREACHABLE`, `E_RIG_UNREACHABLE`): the sandbox cannot reach
  the agent's own listeners.
- **Resumable session uploader.** Finished sessions go to the paired node over
  the LAN only, in ranged chunks of at most 8 MiB that resume from the node's
  `HEAD` offset after a dropped link; `.uploaded` is written after the commit
  ACK. `atlas.auto_upload` uploads when the device is idle or charging and not
  recording.
- **Device calibration store** (`<data dir>/calib/<rig_id>/<version>.json`,
  an `active` pointer per rig) and a Kalibr `camchain-imucam.yaml` + `imu.yaml`
  importer.
- MCP tools `capture_start`, `capture_stop`, `capture_mark`, `sessions_list`,
  `reconstruct`, `job_status`, `tour_bundle`.
- **Session recorder.** The capture service records `ados.capture/1` sessions
  under the agent's capture root (admission from the janitor sidecar,
  `E_STORAGE_FULL` otherwise): every camera frame stamped at its joined
  trigger, IMU, triggers, GNSS bytes and fixes, LiDAR scans, clock samples,
  `/pose/<source>` with covariance and a 1 Hz `/health` record; segments roll
  at 4 GiB or 10 min; `.recording` is kept and refreshed while a session is
  open; site, floor and MARK rooms land in the manifest.
- **Sensor-bus ingest** (`sensor.stream.read`): a 10 s IMU ring, trigger join,
  and the host-to-session clock map.
- **Pose providers** with SE(3) interpolation and a 50 ms extrapolation limit:
  the FC EKF (exact WGS-84 to ENU), Basalt stereo-inertial VIO
  (`capture/basalt`, BSD-3, live socket input), KISS-ICP LiDAR odometry
  (`capture/python/kiss_icp_live`) and a replay provider. A Livox SDK2 driver
  (`capture/livox`) feeds LiDAR scans.
- **WHEP preview and rig write through the agent.** `POST|DELETE
  /atlas/capture/whep` and `PUT /atlas/rig` relay through the host methods
  `video.whep.relay` and `sensors.rig.write`.
- **Crashed sessions.** A `.recording` session the capture service does not
  own and that is older than 120 s is listed as not finished; it can be
  deleted with `?force=true` and is never uploaded (`E_NOT_FINISHED`).
- **Session timebase.** A session on a non-host timebase starts only after a
  clock sample (`E_CLOCK_NOT_READY`); a `host_monotonic` session never maps
  its stamps. Exposure reports `applied: false` until the camera applies it and
  uses the nearest flicker-safe value under the mode cap. A bad item is skipped
  and counted; only a writer I/O error ends recording.
- Auto-queued uploads run only while the auto gate (idle or charging, not
  recording) stays open and pause, to resume from the node's offset, when it
  closes; operator uploads are not gated. IPv6 node addresses are bracketed.
- **LAN-only upload is pinned** to the checked address and never follows
  redirects. MCP tools answer within 4 s. Job errors decode as objects.
- **Release payloads**: `basalt-ados`, the Livox SDK2 library and `uv`
  (x86_64-linux; aarch64-linux is a manual release step). The KISS-ICP
  environment is synced on the device at the first LiDAR start.
- **Capture quality and HUD**: blur, texture, mains flicker, coverage voxels,
  corridor degeneracy, storage and clock chips; the `capture_hud` display page;
  B1 / B2 passthrough presses start/stop and MARK.

### Changed

- `atlas.pose_tier` and the offloaded-SLAM pose lane are removed; `atlas.pose_provider` (`auto|fc_ekf|vio|lidar`) replaces them. `PoseSource` is `fc_ekf|vio|lidar|replay`. Checksums skip dot-file markers.

- **Atlas envelope version 2.** Keyframes carry the contract camera model,
  geo anchor, a timebase-stamped exposure time and IMU samples with `t_ns` and
  `seq`; every artifact descriptor carries a placement. Version 1 frames are
  refused.
- `atlas.intrinsics.<camera>` takes the contract camera model: `model`
  (`pinhole`, `opencv`, `opencv_fisheye`, `kb4`), `fx`, `fy`, `cx`, `cy`,
  `dist`, `readout_ns`.
- The live-lane trainers resolve from the pinned tool set and fail with
  `E_TOOL_MISSING` instead of producing a placeholder.
- The exposure lock is applied to every enabled capture camera through the
  host camera control (`camera.control.write`), on `PUT /atlas/capture/exposure`
  and at run start for a configured lock; unlock returns the cameras to auto
  exposure. `applied` is true only when every camera took the values, which
  are then the read-back; otherwise `reason` names why (`not_v4l2`,
  `control_unsupported`, `timeout`) and the GCS shows "Requested, not applied".
- Front-panel presses are matched on the host's button label (`B1` REC, `B2`
  MARK) instead of a GPIO pin.

### Removed

- The nerfstudio backend, the monocular-depth seed script and the random seed
  cloud; the placeholder reconstructor in release builds; the SLAM offload job
  kind.
- Compute-node datasets are written as the contract `transforms.json`, with
  the session anchor and per-frame exposure times.
- `atlas.intrinsics` and `atlas.pose_prior` refuse unknown keys, so a
  pre-contract override fails with an error naming the accepted keys.
- `atlas.pose_prior.clock_offset_sigma_ns` is renamed `timebase_sigma_ns`.
- The `atlas.hud` pin settings (`rec_pin`, `mark_pin`).
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
