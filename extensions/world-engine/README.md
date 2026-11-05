# World Engine

3D world models for ADOS drones. A drone captures keyframes with their
poses, streams them to a paired workstation or compute node, and the node
reconstructs a Gaussian splat (and a point cloud, mesh and occupancy grid)
that the ground station views live and after the flight. The same compute
node also serves perception offload: a drone can send its detection work to
the workstation's GPU instead of running it on board.

The extension has two halves:

- **Agent half** (Rust). On a drone, the Atlas capture service selects
  keyframes from the shared vision frame bus, tags each with its pose, and
  publishes them over the best available bearer (direct LAN, the WFB relay
  through a ground station, or the cloud). On a workstation or compute node,
  the compute node ingests those keyframes, runs the reconstruction jobs,
  stores the artifacts, logs a Rerun recording, and serves the per-drone
  world-model stream and the offload sessions. On a ground station, it relays
  capture events heard over the air to the compute node.
- **GCS half** (an inline module). Pages on the drone, ground-station,
  workstation and compute node views of Mission Control: capture controls and
  status, the Gaussian splat, point cloud and Rerun viewers, the compute job
  queue and its outputs, and the perception offload settings.

## Layout

| Path | Contents |
|---|---|
| `agent/capture/` | Drone-side capture service: keyframe selection, pose source, session state |
| `agent/node/` | Reconstruction node: session upload lane, staged pipeline (prepare, SfM, train, mesh, products), indoor products, calibration jobs, job API, artifacts, offload serving |
| `agent/node/tools/` | `tool-pins.json` (the exact third-party tool versions) and `fetch-tools.ps1` / `fetch-tools.sh` |
| `agent/node/python/gsplat_train/` | uv project: gsplat training, mesh, depth priors, masks and hero stills |
| `agent/transport/` | Keyframe bearers (LAN, WFB relay, cloud), mDNS discovery, offload client |
| `agent/protocol/` | Wire types and topic names shared by every agent crate |
| `agent/link/` | The extension's main process on every node: the bearer ladder and offload reconciler on a drone, the relay on a ground station, and the HTTP server the GCS reaches |
| `contract/` | The MIT capture contract `ados.capture/1` (schemas, protobuf messages, fixtures) |
| `gcs/` | The inline GCS module |
| `locales/` | GCS strings |

## Install

```sh
ados plugin install com.altnautica.world-engine
```

The node installer offers the extension as a toggle (`--world-engine` /
`--no-world-engine`). It is on by default for the `workstation` and
`compute` profiles and off for the others; install it on a drone or ground
station with the command above, or by turning the toggle on.

A drone needs a paired workstation or compute node to reconstruct anything.
Once both are paired to the same ground control station, the GCS issues the
drone a credential scoped to the compute node's lanes (keyframe ingest, the
world-model stream, offload, artifacts, job submission) and installs it on
the drone. A ground station is issued the ingest lane only. The compute
node's Jobs & outputs page lists and revokes these credentials.

Native helpers: basalt-ados (BSD-3) and the Livox SDK2 library (MIT) are
release payloads with their licence texts. x86_64-linux builds in the
release workflow; aarch64-linux is a manual release step
(`agent/capture/basalt/README.md`, `agent/capture/livox/build-sdk2.sh`). The
KISS-ICP odometry environment is built on the device by `uv sync --frozen` on
the first LiDAR start, which needs network once.

## GCS pages

Agent pages:

| Page | Nodes | What it shows |
|---|---|---|
| `world-model` | drone | The reconstructed world for a captured session, with a viewer switcher (splat, point cloud, Rerun), read from the paired compute node or the cloud job records; the capture setup view when no reconstruction exists yet |
| `world-model-setup` | drone | The capture service switch and the pose-source preference |
| `live-world` | drone | The live capture session: keyframe and ingest stats, the transport bearer, the building reconstruction, and the Start / Pause / Resume / Stop and Reconstruct controls |
| `offload` | drone, workstation, compute | On a drone, where perception runs and which compute node it offloads to; on a compute node, the serving switch, the detector model and the GPU |
| `atlas-relay` | ground-station | The relay's keyframes seen and forwarded, with a staleness badge |

Node surfaces:

| Surface | Nodes | Label | What it shows |
|---|---|---|---|
| `overview` | workstation, compute | Compute | GPU card with its utilisation history, compute-cluster status, a jobs summary |
| `compute` | workstation, compute | Jobs & outputs | The job queue, the staged reconstruction pipeline (stage, GPU, ETA, tools, a missing tool), the artifact viewer over finished jobs, and the paired drones and ground stations allowed to use this node |
| `capture` | visor | Capture | Live camera previews (WHEP), top-down coverage with the walked trajectory, the six health chips, record / mark with room and floor, exposure lock, return to start, storage left |
| `sessions` | visor, drone | Sessions | Recorded sessions with upload progress, reconstruction presets (Preview, HQ, Tour, Warehouse), job progress, the world viewer, tour bundle and dataset downloads, change detection between two sessions |
| `calibration` | visor | Calibration | The calibration wizard (IMU noise, camera intrinsics, camera–IMU, LiDAR–IMU, LiDAR–camera, GNSS lever arm) and the stored calibration versions |

The world viewer renders splats with Spark (SPZ, PLY, or a streamed `.rad`
level-of-detail file) with orbit, dollhouse and first-person walk modes, the
floorplan, room hotspots, warehouse aisles and a measure tool. Every
reconstruction names the backend that produced it. `gcs/tour-viewer/` is the
same viewer as a static page, copied into every tour bundle.

## Configuration

Per-node plugin configuration (`config-schema.json`):

| Key | Purpose |
|---|---|
| `atlas.enabled` | Run the capture service on this drone |
| `atlas.capture_profile` | Keyframe selection profile |
| `atlas.pose_provider` | Pose provider: `auto`, `fc_ekf` (flight controller), `vio` (on-device visual-inertial) or `lidar` (on-device LiDAR odometry: 2D scan matching for a planar LiDAR, KISS-ICP for a 3D one). `auto` picks `lidar` when the rig has the configured LiDAR and IMU (on a drone too, where the LiDAR pose is the navigation source the flight controller fuses), else on a visor `vio` with a front stereo pair and IMU, else `fc_ekf` |
| `atlas.lidar` | LiDAR input: `lidar_id` (rig sensor; its `lidar.model` picks the driver, `rplidar-c1` or `livox-mid360`), `imu` (the rig IMU a planar LiDAR is levelled with; `fc` synthesises it from the flight controller's attitude for a companion without an IMU), and per-driver blocks `rplidar` (`device`: serial port, `tcp:<host>:<port>` or `auto`; `baud`) and `livox` (`sdk_library`, `host_ip`, `lidar_ips`) |
| `atlas.auto_upload` | Upload finished sessions to the paired compute node over the LAN when the device is idle or charging and not recording (default `false`; a `post_flight` or `hybrid` placement uploads regardless) |
| `atlas.nav_pose` | `always`: publish the navigation pose, scan, live occupancy, sweep cloud and map status whenever the pose provider runs; `capture`: only with a session open. Default `always` when an RPLIDAR device is set |
| `atlas.compute_placement` | Where the heavy map products are built: `onboard` (5 cm sweep cloud on the drone), `stream` (keyframes, poses and raw polar scans stream to the compute node, which reconstructs in flight and serves a full-resolution live map on the device's world stream; the drone keeps a 20 cm cloud), `post_flight` (record, upload after landing), `hybrid` (10 cm on board plus the post-flight upload; default). Navigation SLAM and the live occupancy always stay on board |
| `atlas.memory_budget_mb` | RAM the live map stores may hold (keyframe queue 40 %, scan ring 10 %, sweep cloud 30 %, occupancy 20 %); unset, 80 % of the capture unit's memory cap. A full sweep cloud doubles its voxel (`sweep_cloud_coarsened`); full queues drop their oldest entry and count it |
| `atlas.rig_id` | Rig whose calibrations `GET /atlas/calib` lists; empty uses the only rig in the device calibration store |
| `atlas.reconstruct_steps` | Splat training steps (the detail level) for this drone's reconstructions, the GCS-submitted ones and those the compute node runs itself from the capture stream; carried on every capture status. Unset, the node uses `ADOS_COMPUTE_TRAIN_ITERS` (default 30000) |
| `atlas.hfov_deg` | Camera horizontal field of view used when no intrinsics are given |
| `atlas.cameras[]` | Cameras the capture service reads |
| `atlas.selection.*` | Keyframe selection thresholds |
| `atlas.intrinsics.*` | Camera intrinsics |
| `atlas.pose_prior.*` | Pose uncertainty priors: `position_sigma_m`, `gnss_uere_m` (2.5), `orientation_sigma_rad` (0.02) |
| `atlas.pose_publish_interval_ms` | Minimum interval between published navigation poses, in ms (100; 33 for 30 Hz, bounded by the provider's own rate) |
| `offload.enabled` | Perception offload: `auto`, `on` or `off` |
| `offload.compute_node_addr` | Compute node pinned as `host:port` for both the Atlas forwarder and the offload session; empty discovers one on the LAN over mDNS |
| `serving.enabled` | Serve perception offload on this compute node |
| `serving.detector_model` | Detector model the compute node serves |
| `relay.enabled` | Run the Atlas relay on this ground station (default `false`) |
| `relay.compute_base_url` | Compute node the ground-station relay forwards to |

## Navigation and live map lanes

While the pose provider runs (`atlas.nav_pose`), the capture service
publishes shared topics other extensions and the agent's navigation API read:

| Topic | Rate | Payload |
|---|---|---|
| `plugin.world-engine.pose` | pose interval | Body pose in `map_ned` (north-aligned: x north, y east, z down, origin at the first scan; the planar SLAM world rotated so its axes match `LOCAL_POSITION_NED`, first by the flight controller's compass heading latched at the first pose, then by the heading fitted between the SLAM track and the EKF's GNSS track once 10 m RMS of track has been flown on a steady 3D fix, frozen when GNSS goes; z is the FC's height above the first pose), with `health`, `frame` and `source`. Map lanes wait for the first heading |
| `plugin.world-engine.scan` | per scan | The scan binned to 360 one-degree bins in the body FRD frame, nearest range per bin, `0` for no return |
| `plugin.world-engine.occupancy` | 1 Hz | Live: changed 64×64-cell tiles at 5 cm as grey PNG (0 occupied, 254 free, 205 unknown), `kind: "live"`; node products say `kind: "product"` |
| `plugin.world-engine.pointcloud` | 1 Hz | Live: up to 20 000 new voxel centroids of the sweep cloud (every scan placed by SLAM x, y, yaw, the FC height and the IMU roll and pitch), `kind: "live"` |
| `plugin.world-engine.map_status` | 1 Hz | Placement, memory used against the budget, voxel size, keyframes dropped, sweep points, mapped area, upload and stream progress |

Occupancy tiles farther than 200 m from the vehicle, or past the occupancy
share of the budget (farthest first), are written to the session's
`map/occupancy/` and leave RAM. A closing session also gets the whole sweep
cloud as `map/sweep_cloud.f32` (little-endian `f32` xyz).

A planar LiDAR records into the session as `lidar/<sensor>.polar.zst`: a zstd
(level 3) stream of polar records, each with the sensor pose and 720 bins of
`u16` range in mm plus `u8` quality (3 bytes a bin). The `planar-sweep`
preset turns those alone into a LAZ cloud, occupancy per height band (0–3,
3–8 and 8–15 m above the first scan) in `map_server` PNG + YAML, and a
floorplan, after closing loops offline over the scans; a LiDAR-only session needs no pinned tool, and a session with camera frames also gets a preview splat and its mesh (COLMAP, splat-transform and Brush, `E_TOOL_MISSING` without them).

`cargo run --release -p world-engine-capture --example scan2d_replay --
<lidar0.polar.zst>` replays a recorded scan log through the scan matcher and
prints the wall time, a fixed workload for comparing CPUs.

## Reconstruction tools

The node drives pinned third-party tools listed in
`agent/node/tools/tool-pins.json` and installed under `agent/node/tools/cache/`
by `fetch-tools.ps1` (Windows) or `fetch-tools.sh` (Linux/macOS). It never
takes a tool from `PATH`; `ADOS_TOOL_<NAME>` overrides one explicitly.

- **COLMAP 4.2** for structure-from-motion: with a pose track, pose-chosen
  image pairs, verified pairs that contradict the track dropped, and
  prior-weighted mapping (or triangulation against a fixed 3D LiDAR
  trajectory); without one, incremental / hierarchical mapping.
- **GLIM** (docker) for the LiDAR-inertial trajectory of a 3D LiDAR rig. A
  planar LiDAR rig (RPLIDAR C1 class) needs no external tool: its scans are
  matched in-process and pose the keyframes as priors.
- **Brush** for the `preview` preset; **gsplat** (the `gsplat_train` uv project)
  for `hq`, `tour` and `warehouse`. Without working gsplat kernels the node
  trains with Brush at full length and records why.
- **splat-transform** (SPZ) and the **Spark LoD builder** (`.rad`).
- **Kalibr** and **direct_visual_lidar_calibration** containers (docker; WSL
  on Windows) for calibration jobs. A planar LiDAR's LiDAR–camera
  calibration runs natively from the AprilGrid board its scan line crosses.

A preset whose tools are not installed is refused with `E_TOOL_MISSING` and
the missing tool's name. On Windows the node runs standalone:
`world-engine-node --standalone --data-dir <dir> --listen 0.0.0.0:8092`, with
the owner bearer token in `<dir>/node/owner-token`. The owner issues a capture
device its credential with `POST /api/compute/node-credentials
{"peer_device_id": "<id>"}` (bearer); the issued set is kept in
`<dir>/node/node-credentials.json`, listed with `GET` and revoked with
`POST /api/compute/node-credentials/<id>/revoke`. Both files are readable by
the node's user only. The node's id (the issuer named in a credential) is
generated once into `<dir>/node/node-id`.

## Building the GCS module

From the repository root:

```sh
pnpm install
pnpm build:world-engine
pnpm test:world-engine
```

`pnpm build:world-engine` builds the plugin SDK and then the module, which
writes three files into `gcs/`:

| File | Contents |
|---|---|
| `world-engine.mjs` | The ESM module; React comes from the host |
| `plugin.css` | The module's styles, every utility under the `we:` prefix |
| `assets/re_viewer_bg.wasm` | The Rerun web viewer's wasm, served as a plugin asset |

`re_viewer_bg.wasm` is copied from the `@rerun-io/web-viewer` package at
build time. The release pipeline stages it into the package as a payload;
it is never committed.

## License

GPL-3.0-or-later. See the repository `LICENSE` file.
