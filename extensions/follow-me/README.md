# ADOS Follow-Me

Lock onto an operator-designated subject and fly a fixed-distance standoff
follow from the companion computer. Follow-Me is a flight Skill in the
cockpit Skill Bar, an interactive click-to-follow video overlay, and a
per-drone settings tab, all from one plugin.

`com.altnautica.follow-me` — hybrid (agent + GCS), risk: high.

## How it works

A generic person/object detector on the companion produces detections from
the configured camera. The operator clicks a subject in the live video; the
GCS overlay forwards that click to the agent, which designates the box with
the vision engine and owns the resulting track lock. From then on the agent:

1. Reads the locked subject's bounding box from the detection stream.
2. Projects the box center onto the ground with a pinhole camera model and
   the flight controller's attitude + above-ground-level height.
3. Computes a standoff follow point a fixed distance short of the subject
   along the line of sight, at a fixed follow height.
4. Sends the flight controller a guided position setpoint at a steady rate,
   and (optionally) points a gimbal at the subject.

A lock-state safety gate stops commanding the instant the tracker reports
the subject uncertain or lost, and never silently re-locks onto a different
subject. The operator re-designates to resume.

## Surfaces

- **Flight Skill** (`follow-me`): an armed-only, confirm-to-arm toggle in the
  cockpit Skill Bar (default hotkey `⇧F`). Arming writes the per-drone
  `active` config; the agent reads it live and reports `follow.state`.
- **Target actions** (`contributes.target_actions`): click a detected subject
  in the host's cockpit overlay and pick **Follow this target** (hotkey `f`)
  to designate and follow it, or **Stop following** (hotkey `x`) to stop. The
  host owns the overlay, the box drawing, the selection, and the designate +
  config write; this plugin only declares the actions.
- **Native settings** (`contributes.parameters`): the follow settings
  (distance, height, gimbal point, camera, field of view, mount pitch, and
  the detector model) render as native GCS controls from the manifest. The
  agent reads the same per-drone keys live; the iframe does not re-implement
  them.
- **Node-detail tab** (`node.detail.tab`, drone profile): Specs and a live
  read-back (lock state, the honest `commanding` flag, range, setpoints).

## Configuration

Per-drone configuration (see `config-schema.json`):

| Key                | Default  | Range     | Meaning                                   |
| ------------------ | -------- | --------- | ----------------------------------------- |
| `active`           | `false`  | —         | Armed/disarmed (toggled by the Skill).    |
| `follow_distance_m`| `8`      | `3`–`30`  | Standoff distance behind the subject.     |
| `follow_height_m`  | `4`      | `0`–`20`  | Follow height above the arming altitude.  |
| `gimbal_point`     | `true`   | —         | Point a gimbal at the subject if present. |
| `designate_camera` | `uvc-0`  | —         | Camera the follow loop consumes.          |
| `camera_hfov_deg`  | `70`     | `30`–`160`| Camera horizontal field of view.          |
| `mount_pitch_deg`  | `30`     | `0`–`90`  | Camera downward tilt below the horizon.   |
| `detector`         | `coco-person` | —    | Detection model the follow loop consumes. |

## Requirements

- A flight controller in a guided position-hold mode (ArduPilot Guided, PX4
  Offboard/position). The plugin sends guided setpoints; it does not change
  the flight mode itself.
- A camera bound to the vision pipeline producing the detection stream.
- A board with enough compute for on-companion detection (CM4/CM5,
  RK3582/RK3588S2/RK3576, Pi 5).

## Build, test, pack

```bash
pnpm -C gcs build      # esbuild -> gcs/plugin.bundle.js
pnpm -C gcs test       # vitest (GCS half)
( cd agent && python -m pytest )   # agent half
../../scripts/pack.sh follow-me    # -> dist/com.altnautica.follow-me-0.2.2.adosplug
```

## License

GPL-3.0-or-later.
