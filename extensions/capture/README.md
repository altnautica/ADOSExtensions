# Capture

Capture sessions for ADOS drones and Visors. A device records every enabled
camera, its IMU and GNSS, the LiDAR the Navigation extension measures, and
the pose each moment is tagged with, and selects pose-tagged keyframes from
one camera up to an all-sides rig. Sessions upload to a reconstruction node
(the Reconstruct extension on a workstation or compute node) and the
operator starts a reconstruction from the same page.

The extension has two halves:

- **Agent half** (Rust). On a drone or Visor, the capture service
  (`capture-service`) records sessions and selects keyframes; the link
  (`capture-link`) forwards keyframes over the best available bearer
  (direct LAN, the WFB relay through a ground station, or the cloud),
  uploads finished sessions, publishes `plugin.capture.state`, and serves
  the HTTP API. On a ground station, the link relays capture events heard
  over the air to the reconstruction node.
- **GCS half** (an inline module): the Capture surface (Live, Sessions and,
  on a drone, Model), the cockpit Capture panel, the map Coverage layer,
  and the Capture setup, Calibration and Relay pages.

## Layout

| Path | Contents |
|---|---|
| `agent/capture/` | The capture service: sessions and recording, keyframe selection, quality, calibration, the HUD page and buttons, the rig, camera control, and the pose source |
| `agent/link/` | The extension's main process: keyframe forwarder, shared state and status, session uploader, ground-station relay, HTTP API, MCP tools |
| `gcs/` | The inline GCS module |
| `locales/` | GCS strings |
| `../../crates/` | Shared with the other mapping extensions: the wire protocol and the capture contract (`mapping-protocol`), transport and node pairing (`mapping-transport`), pose inputs (`mapping-pose`) |
| `../../packages/mapping-ui/` | Shared GCS viewers, node client and UI kit |

## Install

```sh
ados plugin install com.altnautica.capture
```

A device needs a paired reconstruction node to upload to. Once both are
paired to the same ground control station, the GCS issues the device a
credential from the node scoped to Capture's lanes (keyframe ingest, session
upload, job submission) and installs it through `POST
/capture/node-credential`. A ground station is issued the ingest lane only.

## Pose

`capture.pose_source` picks where a session's pose comes from when it
starts: `navigation` (the Navigation extension's VIO or LiDAR pose, read from
`plugin.navigation.pose_track`), `fc_ekf` (the flight controller's EKF), or
`auto` (Navigation's pose when it streamed in the last 2 s, else the EKF). A
session keeps its pose source until it closes.

With Navigation running, a recording session also records each LiDAR scan
Navigation measures (`plugin.navigation.scan_raw`, reassembled from its parts;
a scan missing a part after 500 ms is dropped and counted in the health
record as `lidar_scans_incomplete`). Without Navigation, sessions are posed
by the EKF and record no LiDAR.

## GCS

| Contribution | Profiles | What it shows |
|---|---|---|
| `capture` (node surface) | drone, visor | **Live**: record / pause / stop / mark, camera previews (WHEP), the quality chips, exposure lock, return to start, storage left (a Visor also shows its coverage); a drone whose service is not running shows its setup and requirements. **Sessions**: recorded sessions with upload progress, reconstruction presets, job progress, the viewer and tour download. **Model** (drone): what the paired node built, read-only |
| `capture-chips` (cockpit panel) | drone | State, duration, frames, the quality chips and whether the session is enough to reconstruct |
| `capture-coverage` (map overlay) | drone, visor | The open session's coverage, as an inset over the map |
| `capture-setup` (Agent page) | drone, visor | The capture-service switch and the pose source |
| `calibration` (Agent page) | visor | The calibration wizard and the stored calibration versions |
| `relay` (Agent page) | ground-station | The relay's events seen and forwarded |

The Model view and credential provisioning read the reconstruction node's
job API through the Reconstruct extension (GCS permission
`plugin.http.com.altnautica.reconstruct`).

## HTTP API

On the plugin's socket, under `/api/plugins/com.altnautica.capture/x/`:
`GET /capture/readiness`, `PUT /capture/config` (a flat map of `capture.*`
keys; a key another extension owns is refused naming it),
`POST /capture/{start,stop,pause,resume,mark}`, `GET /capture/state`,
`GET /capture/coverage`, `PUT /capture/exposure`, `POST|DELETE
/capture/whep`, `PUT /capture/rig`, `GET /capture/calib[/:version]`,
`GET /capture/sessions`, `DELETE /capture/sessions/:id`,
`POST /capture/sessions/:id/upload`, `POST /capture/sessions/:id/reconstruct`,
`POST /capture/node-credential`; on a ground station
`GET /capture/relay/status`.

## Configuration

Per-device plugin configuration (`config-schema.json`):

| Key | Purpose |
|---|---|
| `capture.enabled` | Run the capture service on this device |
| `capture.capture_profile` | Keyframe selection profile |
| `capture.cameras[]` | Cameras the capture service reads. With no stored rig (or for a camera the stored rig lacks) each one is placed at the body origin, pointed by its agent roster entry's `mount_pitch_deg` (else by role) and given a pinhole from the roster's `fov_deg` (70 degrees when absent) |
| `capture.pose_source` | `auto`, `fc_ekf` or `navigation` (see Pose) |
| `capture.selection.*` | Keyframe selection thresholds |
| `capture.intrinsics.*` | Camera intrinsics |
| `capture.pose_prior.*` | Pose uncertainty priors: `position_sigma_m`, `gnss_uere_m` (2.5), `orientation_sigma_rad` (0.02) |
| `capture.record_on_start` | Open a session as soon as the service starts |
| `capture.image_encoding` | Recorded image encoding |
| `capture.mains_hz`, `capture.exposure` | Flicker-safe exposure |
| `capture.segment` | Segment roll size and duration |
| `capture.memory_budget_mb` | RAM the keyframe queue may hold; unset, 80 % of the extension's memory cap on this device |
| `capture.auto_upload` | Upload finished sessions to the paired node when idle or charging and not recording (default `false`; Navigation's `post_flight` or `hybrid` map placement uploads regardless) |
| `capture.rig_id` | Rig whose calibrations `GET /capture/calib` lists |
| `capture.reconstruct_steps` | Splat training steps for this device's reconstructions, carried on every capture status |
| `capture.compute_node_addr` | Reconstruction node pinned as `host:port`; empty discovers one on the LAN over mDNS |
| `capture.relay.enabled`, `capture.relay.compute_base_url` | The ground-station relay and the node it forwards to |

## Building the GCS module

From the repository root:

```sh
pnpm install
pnpm build:capture
pnpm test:capture
```

The build writes `gcs/capture.mjs` (React comes from the host),
`gcs/plugin.css` (`cap:` utilities, then the shared viewers' `map:` ones) and
`gcs/assets/re_viewer_bg.wasm` (the Rerun web viewer's wasm, staged as a
release payload, never committed).

## License

GPL-3.0-or-later. See the repository `LICENSE` file.
