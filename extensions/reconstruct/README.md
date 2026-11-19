# Reconstruct

Reconstruction and perception offload for ADOS. A workstation or compute node
turns captured sessions (from the Capture extension) into a Gaussian splat,
point cloud, mesh, occupancy and indoor products, and serves them to Mission
Control. The same node serves perception offload: a drone without an
on-board accelerator streams its camera to the node, the node runs the
detector, and detections return to the drone's own detection bus.

The extension has two halves:

- **Agent half** (Rust). On a workstation or compute node, the `node` service
  (`reconstruct-node`) receives sessions and keyframes, runs the staged
  reconstruction jobs, stores the artifacts, logs a Rerun recording, serves
  offload sessions, and builds the live map from a streaming drone's scans.
  On a drone, the main process (`reconstruct-offload`) runs the offload
  reconciler, forwards Navigation's placed scans to the node while the map is
  built there, and serves the reconstruction MCP tools.
- **GCS half** (an inline module): the Reconstruct surface on a workstation
  or compute node (Jobs, Products, Editor, device access, perception
  serving), and the drone's Perception offload page.

## Layout

| Path | Contents |
|---|---|
| `agent/node/` | The reconstruction node: session upload lane, staged pipeline (prepare, SfM, train, mesh, products), indoor products, calibration jobs, job API, artifacts, offload serving, the live map |
| `agent/node/tools/` | `tool-pins.json` (the exact third-party tool versions) and `fetch-tools.ps1` / `fetch-tools.sh` |
| `agent/node/python/gsplat_train/` | uv project: gsplat training, mesh, depth priors, masks and hero stills |
| `agent/offload/` | The drone's process: offload reconciler, live scan stream, MCP tools, drone routes |
| `gcs/` | The inline GCS module and the static tour viewer |
| `locales/` | GCS strings |
| `../../crates/` | Shared with the other mapping extensions: the wire protocol (`mapping-protocol`), transport and node pairing (`mapping-transport`), pose inputs (`mapping-pose`), planar SLAM (`scan2d`) and the live map (`livemap`) |
| `../../packages/mapping-ui/` | Shared GCS viewers, node client and UI kit |

## Install

```sh
ados plugin install com.altnautica.reconstruct
```

Install it on the workstation or compute node that reconstructs, and on a
drone that offloads perception or streams its map. Once a device and the node
are paired to the same ground control station, the GCS issues the device a
credential from the node scoped to each extension's lanes and installs it
(`POST /reconstruct/node-credential` for this extension's drone lanes:
offload, the live scan stream, job submission). The Reconstruct surface's
device access view lists and revokes these credentials.

## GCS

| Contribution | Profiles | What it shows |
|---|---|---|
| `reconstruct` (node surface) | workstation, compute | A header with the GPU, the cluster and the queue; **Jobs** (queue, history, pipeline stages, cancel, retry); **Products** (each finished job's artifacts in the model viewers, change detection between two sessions); **Editor** (a finished cloud, read-only); device access; perception serving |
| `offload` (Agent page) | drone | Where perception runs and which node it offloads to |

The model viewers render splats with Spark (SPZ, PLY, or a streamed `.rad`
level-of-detail file) with orbit, dollhouse and first-person walk modes, the
floorplan, room hotspots, warehouse aisles and a measure tool. Every
reconstruction names the backend that produced it. `gcs/tour-viewer/` is the
same viewer as a static page, copied into every tour bundle.

## HTTP API

On a workstation or compute node (`/api/plugins/com.altnautica.reconstruct/x/…`
and the same paths on `:8092`): `/api/compute/*` (status, identity, datasets,
jobs, outputs, files, tour bundles, measure, offload sessions, bulk session
upload, node credentials), `/artifacts/*path`, `/compute/status`,
`/ws/offload/:session_id`, `POST /api/capture/event` (keyframes, capture
state and live scans) with `GET /api/capture/health`, and the world stream
`GET /ws/live/:device_id` (reconstruction descriptors and the live map).

On a drone: `GET /reconstruct/status` (the offload setting and the live scan
stream), `PUT /reconstruct/config` (the `reconstruct.offload.*` keys) and
`POST /reconstruct/node-credential`.

## Configuration

| Key | Purpose |
|---|---|
| `reconstruct.offload.enabled` | Perception offload on a drone: `auto`, `on` or `off` |
| `reconstruct.offload.compute_node_addr` | Node pinned as `host:port` for offload, the live scan stream and the MCP tools; empty discovers one on the LAN over mDNS |
| `reconstruct.offload_camera` | The camera offload streams when the vision engine has no main tap |
| `reconstruct.serving.enabled` | Serve perception offload on this node |
| `reconstruct.serving.detector_model` | Detector model the node serves |

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
  planar LiDAR rig needs no external tool: its scans are matched in-process
  and pose the keyframes as priors.
- **Brush** for the `preview` preset; **gsplat** (the `gsplat_train` uv project)
  for `hq`, `tour` and `warehouse`. Without working gsplat kernels the node
  trains with Brush at full length and records why.
- **splat-transform** (SPZ) and the **Spark LoD builder** (`.rad`).
- **Kalibr** and **direct_visual_lidar_calibration** containers (docker; WSL
  on Windows) for calibration jobs. A planar LiDAR's LiDAR–camera
  calibration runs natively from the AprilGrid board its scan line crosses.

A planar LiDAR session (`lidar/<sensor>.polar.zst`) reconstructs with the
`planar-sweep` preset alone: a LAZ cloud, occupancy per height band and a
floorplan, after closing loops offline over the scans.

A preset whose tools are not installed is refused with `E_TOOL_MISSING` and
the missing tool's name. On Windows, and on a Linux workstation, the node runs
standalone: `reconstruct-node --standalone --data-dir <dir> --listen
0.0.0.0:8092`, with the owner bearer token in `<dir>/node/owner-token`. The
owner issues a capture device its credential with `POST
/api/compute/node-credentials {"peer_device_id": "<id>"}` (bearer); the issued
set is kept in `<dir>/node/node-credentials.json`, listed with `GET` and
revoked with `POST /api/compute/node-credentials/<id>/revoke`.

## Building the GCS module

From the repository root:

```sh
pnpm install
pnpm build:reconstruct
pnpm test:reconstruct
```

The build writes `gcs/reconstruct.mjs`, `gcs/plugin.css` (`rec:` utilities,
then the shared viewers' `map:` ones), `gcs/assets/re_viewer_bg.wasm` (staged
as a release payload, never committed) and `gcs/tour-viewer/`.

## License

GPL-3.0-or-later. See the repository `LICENSE` file.
