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
| `../../crates/` | Shared with the other mapping extensions: the wire protocol and the MIT capture contract (`mapping-protocol`), transport (`mapping-transport`), pose inputs (`mapping-pose`), planar SLAM (`scan2d`) and the live map (`livemap`) |
| `agent/link/` | The extension's main process on every node: the bearer ladder and offload reconciler on a drone, the relay on a ground station, and the HTTP server the GCS reaches |
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
| `atlas.auto_upload` | Upload finished sessions to the paired compute node over the LAN when the device is idle or charging and not recording (default `false`; a `post_flight` or `hybrid` placement uploads regardless) |
| `atlas.compute_placement` | Where the heavy map products are built: `onboard` (5 cm sweep cloud on the drone), `stream` (keyframes, poses and raw polar scans stream to the compute node, which reconstructs in flight and serves a full-resolution live map on the device's world stream; the drone keeps a 20 cm cloud), `post_flight` (record, upload after landing), `hybrid` (10 cm on board plus the post-flight upload; default). Navigation SLAM and the live occupancy always stay on board |
| `atlas.memory_budget_mb` | RAM the capture queues may hold (keyframe queue 80 %, LiDAR scan queue 20 %); unset, 80 % of the capture unit's memory cap. Full queues drop their oldest entry and count it |
| `atlas.rig_id` | Rig whose calibrations `GET /atlas/calib` lists; empty uses the only rig in the device calibration store |
| `atlas.reconstruct_steps` | Splat training steps (the detail level) for this drone's reconstructions, the GCS-submitted ones and those the compute node runs itself from the capture stream; carried on every capture status. Unset, the node uses `ADOS_COMPUTE_TRAIN_ITERS` (default 30000) |
| `atlas.cameras[]` | Cameras the capture service reads. With no stored rig (or for a camera the stored rig lacks) each one is placed at the body origin, pointed by its agent roster entry's `mount_pitch_deg` (negative below the nose; else by role: `down` straight down with the image top toward the nose, `up` straight up, side and back cameras level) and given a pinhole from the roster's `fov_deg` (70 degrees when absent), which the vision engine stamps on every frame |
| `atlas.selection.*` | Keyframe selection thresholds |
| `atlas.intrinsics.*` | Camera intrinsics |
| `atlas.pose_prior.*` | Pose uncertainty priors: `position_sigma_m`, `gnss_uere_m` (2.5), `orientation_sigma_rad` (0.02) |
| `offload.enabled` | Perception offload: `auto`, `on` or `off` |
| `offload.compute_node_addr` | Compute node pinned as `host:port` for both the Atlas forwarder and the offload session; empty discovers one on the LAN over mDNS |
| `serving.enabled` | Serve perception offload on this compute node |
| `serving.detector_model` | Detector model the compute node serves |
| `relay.enabled` | Run the Atlas relay on this ground station (default `false`) |
| `relay.compute_base_url` | Compute node the ground-station relay forwards to |

## Navigation

The LiDAR and VIO pose, the live occupancy map and their shared topics are
the Navigation extension's (`../navigation`). The capture service poses its
keyframes with the flight controller's EKF.

A planar LiDAR records into the session as `lidar/<sensor>.polar.zst`: a zstd
(level 3) stream of polar records, each with the sensor pose and 720 bins of
`u16` range in mm plus `u8` quality (3 bytes a bin). The `planar-sweep`
preset turns those alone into a LAZ cloud, occupancy per height band (0–3,
3–8 and 8–15 m above the first scan) in `map_server` PNG + YAML, and a
floorplan, after closing loops offline over the scans; a LiDAR-only session needs no pinned tool, and a session with camera frames also gets a preview splat and its mesh (COLMAP, splat-transform and Brush, `E_TOOL_MISSING` without them).

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
the missing tool's name. On Windows, and on a Linux workstation, the node runs standalone:
`world-engine-node --standalone --data-dir <dir> --listen 0.0.0.0:8092`, with
the owner bearer token in `<dir>/node/owner-token`. The owner issues a capture
device its credential with `POST /api/compute/node-credentials
{"peer_device_id": "<id>"}` (bearer); the issued set is kept in
`<dir>/node/node-credentials.json`, listed with `GET` and revoked with
`POST /api/compute/node-credentials/<id>/revoke`. Both files are readable by
the node's user only. The node's id (the issuer named in a credential) is
generated once into `<dir>/node/node-id`.

On Linux, `fetch-tools.sh` installs Brush (the pinned x86_64-linux archive),
splat-transform (npm) and the Spark LoD builder (cargo). COLMAP publishes no
Linux binary: point `ADOS_TOOL_COLMAP` at a CUDA build from source, a
distribution package, or a script that runs a CUDA COLMAP container with the
node's data dir mounted at the same path, so job paths pass unchanged. The
gsplat kernels compile on first use and need the CUDA toolkit (`nvcc`);
without it, `ADOS_TOOL_GSPLAT_TRAIN` can name a script that runs the
`gsplat_train` project inside a CUDA devel container built from the project's
`uv.lock`. The node calls an override as `<override dir>/<script>.py <args>`,
so such a script runs the project's script of that name.

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
