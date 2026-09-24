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
| `agent/node/` | Compute node: ingest, job scheduler, reconstruction backends, artifacts, offload serving |
| `agent/transport/` | Keyframe bearers (LAN, WFB relay, cloud), mDNS discovery, offload client |
| `agent/protocol/` | Wire types and topic names shared by every agent crate |
| `agent/link/` | The extension's main process on every node: the bearer ladder and offload reconciler on a drone, the relay on a ground station, and the HTTP server the GCS reaches |
| `agent/python/depth_seed.py` | Optional monocular-depth seed for splat training |
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

Node surfaces on workstation and compute nodes:

| Surface | Label | What it shows |
|---|---|---|
| `overview` | Compute | GPU card with its utilisation history, compute-cluster status, a jobs summary |
| `compute` | Jobs & outputs | The job queue, the artifact viewer over finished jobs, and the paired drones and ground stations allowed to use this node |

Every reconstruction carries the backend that produced it. A placeholder
produced on a node with no reconstruction backend installed is badged as a
placeholder and never shown as a real world model.

## Configuration

Per-node plugin configuration (`config-schema.json`):

| Key | Purpose |
|---|---|
| `atlas.enabled` | Run the capture service on this drone |
| `atlas.capture_profile` | Keyframe selection profile |
| `atlas.pose_tier` | Pose source: `auto`, `local` (flight controller), `offload` (SLAM on the compute node) or `hybrid` |
| `atlas.reconstruct_steps` | Training steps for the reconstruction (the detail level) |
| `atlas.hfov_deg` | Camera horizontal field of view used when no intrinsics are given |
| `atlas.cameras[]` | Cameras the capture service reads |
| `atlas.selection.*` | Keyframe selection thresholds |
| `atlas.intrinsics.*` | Camera intrinsics |
| `atlas.live_reconstruct` | Rebuild the world incrementally on the compute node while keyframes arrive (default `false`) |
| `atlas.pose_prior.*` | Pose uncertainty priors: `position_sigma_m`, `gnss_uere_m` (2.5), `orientation_sigma_rad` (0.02), `slam_position_sigma_m` (0.25), `slam_orientation_sigma_rad` (0.01), `clock_offset_sigma_ns` (-1 = unknown) |
| `atlas.pose_max_age_ms` | Oldest pose a keyframe may be tagged with, in ms (500) |
| `atlas.pose_publish_interval_ms` | Interval between published world poses, in ms (100) |
| `offload.enabled` | Perception offload: `auto`, `on` or `off` |
| `offload.compute_node_addr` | Compute node pinned as `host:port` for both the Atlas forwarder and the offload session; empty discovers one on the LAN over mDNS |
| `serving.enabled` | Serve perception offload on this compute node |
| `serving.detector_model` | Detector model the compute node serves |
| `relay.enabled` | Run the Atlas relay on this ground station (default `false`) |
| `relay.listen_port` | Ground-station relay listen port |
| `relay.compute_base_url` | Compute node the ground-station relay forwards to |

## Optional host tools

The compute node shells out to reconstruction tools that are installed on
the host, not shipped in the package. It uses whichever is present:

- **COLMAP** for structure-from-motion poses and a sparse point cloud.
- **Brush** (Metal, Vulkan or wgpu) or **msplat** (Apple Silicon) for
  Gaussian-splat training; **nerfstudio** (`ns-train`, CUDA or MPS) is also
  supported.
- **Python 3** with `numpy`, `Pillow`, `torch` and `transformers` for the
  monocular-depth seed in `agent/python/depth_seed.py`. Without it, splat
  training starts from a random point cloud instead.

With no reconstruction backend installed, jobs complete with a placeholder
output that the GCS badges as such.

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
