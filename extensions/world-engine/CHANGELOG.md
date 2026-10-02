# Changelog

All notable changes to the World Engine extension.

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
