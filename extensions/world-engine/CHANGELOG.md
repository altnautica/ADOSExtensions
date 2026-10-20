# Changelog

All notable changes to the World Engine extension.

## [2.0.0]

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
- A standalone node (`--standalone`) and a Windows build of the node. The
  standalone node issues node credentials (`/api/compute/node-credentials`,
  owner bearer only) kept in its data directory, and admits a capture device
  presenting one on the lanes it was issued for (session upload, job submit
  and reads of its own jobs, artifacts). Its owner token and credential store
  are owner-only (mode 0600, or a protected single-user DACL on Windows), and
  each standalone node has its own persisted id.
- **Device capture surface.** The link serves `POST /atlas/capture/{start,
  stop,pause,resume,mark}`, `GET /atlas/capture/{state,coverage}`,
  `PUT /atlas/capture/exposure`, `GET /atlas/sessions`,
  `POST /atlas/sessions/<id>/{upload,reconstruct}`, `DELETE /atlas/sessions/<id>`
  and `GET /atlas/calib[/<version>]` over control protocol v2, with
  `{"error": CODE, "detail": ..}` refusals.
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
  `video.whep.relay` and `sensors.rig.write`. The preview resolves the capture
  camera to its media leg from the agent's video roster (the primary camera is
  served at `main`); a camera not in the roster answers `404`.
- **GCS capture surfaces.** Visor node pages `capture` (camera grid, coverage
  map, trajectory, health chips, REC/MARK, room and floor, exposure lock),
  `sessions` (list, upload progress, preset picker, job pipeline, viewer, tour
  bundle download) and `calibration`; drones gain `sessions`; the compute
  surface shows the job pipeline stages.
- **Spark viewer.** Splats render with Spark (SPZ and `.RAD` streaming) with
  walk and orbit modes, floorplan overlay, room hotspots and a measure tool; a
  standalone `tour-viewer` build feeds the tour bundle.
- **Crashed sessions.** A `.recording` session the capture service does not
  own and that is older than 120 s is listed as not finished; it can be
  deleted with `?force=true` and is never uploaded (`E_NOT_FINISHED`).
- **Session timebase.** A session on a non-host timebase starts only after a
  clock sample (`E_CLOCK_NOT_READY`); a `host_monotonic` session never maps
  its stamps. Exposure reports `applied: false` until the camera applies it and
  uses the nearest flicker-safe value under the mode cap. A bad item is skipped
  and counted; only a writer I/O error ends recording.
- Each capture session maps every stamp through its own clock map (host-fallback
  clock samples ignored, steps over 50 ms refused, never backwards), so a
  session never mixes timebases; the segment roll counts forward time from
  the segment's first message only, and the listed duration is the last
  segment end minus `t0_ns`.
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
  `control_unsupported`, `gain_unit_unknown`, `not_capture_camera`) and the
  GCS shows "Requested, not applied". Cameras are set in parallel; a reply
  still waiting after the host's engine wait plus a margin (inside the link's
  3 s control wait) answers `pending` and the apply finishes in the
  background, its read-back landing in the capture state. Gain is a
  multiplier (×) throughout.
- Compute-node datasets are written as the contract `transforms.json`, with
  the session anchor and per-frame exposure times.
- `atlas.intrinsics` and `atlas.pose_prior` refuse unknown keys, so a
  pre-contract override fails with an error naming the accepted keys.
- The session anchor is the mean of the first 100 3D fixes, with its MSL
  altitude raised to the ellipsoid by the EGM2008 geoid (1° grid).
- A selected frame whose pose arrived more than 50 ms from the exposure is
  not sent as a keyframe; the count is logged.
- Front-panel presses are matched on the host's button label (`B1` REC, `B2`
  MARK) instead of a GPIO pin.
- Reconstruction `prepare` selects a multi-camera session per time instant
  (one trigger sequence within half a frame period, falling back to time
  alone when per-camera trigger counters pair too few): keyframe picks, blur
  drops and the image cap keep or drop an instant's frames together, and an
  instant missing a camera's frame is dropped whole, so the COLMAP rig stays
  paired after a missed frame. An instant's images are named by its
  reference camera's time; `frames.json` keeps each frame's own `t_ns`.
  Cameras with too few common instants are exported outside a rig. The
  `prepare` and `sfm` stage details report `rig_frames_paired` (and `prepare`
  `rig_frames_dropped`) in place of the `sfm` camera count.
- The compute node answers `GET /api/compute/identity` (`{node_id}`, the id
  it advertises over mDNS) to any caller. A lane pinned to a node address
  (`offload.compute_node_addr`, or the relay's `relay.compute_base_url`) asks
  it first and presents only the credential that node issued. When the id
  cannot be read, it presents the one credential installed for that address
  (`POST /node-credential` takes an optional `node_addr`, kept with the
  credential) or the lane's sole credential, and nothing when several could
  fit. When auto-upload is on and that leaves no credential, the sessions it
  would take list `upload.state: "blocked"` with `error` `no_credential` or
  `ambiguous_credential` in `GET /atlas/sessions`, the GCS shows "Auto-upload
  blocked", and the link logs a warning once per reason.
- The world and tour viewers read SPZ version 4 (Spark 2.4), the version the
  products stage writes (`splat-transform --spz-version 4`, named rather
  than left to the converter's default); versions 1 to 3 still load. The
  Spark LoD builder is pinned to the same release (2.4.0). The viewer frames
  a splat on its solid splats (opacity at least 0.05, smallest scale at
  least 1% of the opaque splats' median middle scale, so at any model scale),
  the 1st to 99th percentile per axis, so far floaters no longer push the
  camera out until the scene is sub-pixel; a streamed `.rad` is re-framed
  as its pages arrive, until the paged count settles (at most 2 s) or the
  user moves the camera. A model with no solid splat shows "empty or fully
  transparent" instead of a blank view.

### Removed

- The nerfstudio backend, the monocular-depth seed script and the random seed
  cloud; the placeholder reconstructor in release builds; the SLAM offload job
  kind.
- The `atlas.hud` pin settings (`rec_pin`, `mark_pin`).
- The `@mkkellogg/gaussian-splats-3d` splat viewer.

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
