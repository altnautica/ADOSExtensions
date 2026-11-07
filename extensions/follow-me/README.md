# ADOS Follow-Me

Lock onto a subject the operator designates, then fly a standoff follow from
the companion computer. Follow-Me is three things in one plugin:

- a flight Skill in the cockpit Skill Bar;
- a click-to-follow target action;
- a per-drone settings tab.

`com.altnautica.follow-me` is a hybrid plugin (agent and GCS halves) with
risk `high`.

## How it works

A generic person/object detector on the companion produces detections from
the configured camera. The operator picks a subject in the live video. The
vision engine designates that box and owns the resulting track lock. From
then on the agent runs a 20 Hz loop:

1. It reads the locked subject's box from the detection stream.
2. It casts a ray through the box **bottom-centre** (the subject's feet)
   using a pinhole model, the flight controller's attitude, and the camera
   mount or gimbal angles.
3. It estimates the horizontal range from up to three cues, fused by
   inverse variance:
   - **ground**: where the feet ray meets the ground. It uses the height
     above ground from a downward rangefinder (`DISTANCE_SENSOR`,
     orientation 25) while that reading is fresh, inside its own range and
     the vehicle is tilted under 30 degrees. Otherwise it uses the barometric
     height;
   - **box height**: the slant range to a 1.70 m subject. It is used only
     when the box clears every frame edge;
   - **lidar**: the planar scan return at the subject's bearing, from the
     optional `plugin.world-engine.scan` topic. It is accepted only within
     25 % of the other cues.
4. A constant-velocity Kalman filter tracks the subject's north/east position
   and velocity in the flight controller's local frame (`LOCAL_POSITION_NED`).
5. It holds the agent pilot's motion lease and sends it a goal every tick
   (20 Hz). The pilot flies the goal through its obstacle planner, speed
   limits and geofence. Each goal carries:
   - **follow** mode: the point `follow_distance_m` from the subject at
     `follow_angle_deg` from its travel heading (180 behind, 90 on its right,
     270 on its left, 0 leading). The reference heading starts where the
     drone already is, so engaging causes no swing, and slews toward the
     travel heading at up to 45 °/s while the subject moves faster than
     0.5 m/s;
   - **orbit** mode: the point on a circle of `orbit_radius_m` around the
     subject, advancing at `orbit_rate_dps` in `orbit_direction` once the
     drone is within 2 m of it;
   - the subject's velocity (plus the orbit's tangential velocity) as
     feed-forward, clamped to `max_speed_mps`;
   - yaw toward the subject, and a terrain-following height of
     `follow_height_m` above the ground;
   - the subject's position and the ring the pilot's planner searches for a
     detour that keeps the subject in view (±90° of the follow angle, ±60°
     of the orbit phase), with `obstacle_margin_m`.
6. Optionally, it points a gimbal at the subject.

A lock-state safety gate stops commanding when the tracker reports the
subject uncertain, or when the pose or the heartbeat is missing; while the
lease is held a hold sends a position goal at the drone's current position.
Switching the skill off releases the lease and the pilot brakes to its own
hold.

### Pilot lease, sticks and stopping

- The lease is asked for on the first goal. If another skill holds it the
  follow holds with `lease-busy`; any other refusal (disarmed, no guided
  mode, a fixed-wing in forward flight) holds with `lease-refused`. Either is
  retried once a second.
- The pilot revokes the lease on a pause, a full stick takeover, land, RTH,
  an FC mode change or a control-mode change. The follow then writes
  `active=false` (every surface's skill toggle follows) and publishes
  `follow.stopped {reason}`.
- While the lease is held the operator's sticks nudge the parameters:

  | Mode   | Right stick sideways                 | Right stick forward/back  | Left stick up/down |
  |--------|--------------------------------------|---------------------------|--------------------|
  | follow | `follow_angle_deg` ±60 °/s           | `follow_distance_m` ±2 m/s (forward = closer) | `follow_height_m` ±1 m/s |
  | orbit  | `orbit_rate_dps` ±10 °/s² (below the minimum flips the direction) | `orbit_radius_m` ±2 m/s (forward = closer) | `follow_height_m` ±1 m/s |

  Values stay inside their config ranges and are written back to the
  per-drone config at most once a second, so the surfaces' sliders follow.

### Re-acquire after occlusion

`reacquire_mode` decides what a lost lock does:

- **Strict** gives the subject up, announces `follow.lost`, holds, and waits
  for the operator to designate again.
- **Smart** (the default) coasts toward the predicted standoff at half
  feed-forward for `reacquire_window_s`. In each new batch it looks for a
  `person` that passes two checks:
  - its re-identification embedding has cosine similarity of at least
    `reacquire_min_similarity` to the stored subject template;
  - it lies within a Mahalanobis distance of 3 from the prediction.

  The best match is designated with the vision engine and announced on
  `follow.reacquired {track_id, similarity, gap_s}`. When the window expires,
  the plugin announces `follow.lost` and holds.

Re-acquire compares appearance embeddings. The shipped default is the
engine's model-free colour histogram (`embedding_model` `"histogram"`). A
learned OSNet-x0.25 re-id model (registry id `reid_osnet_x0_25_v1`) is
available as an explicit operator install and for benchmarks only, because its
weights were trained on a research-use dataset; it is never installed by
default. When it is configured as `vision.reid`, `embedding_model` is that
model id and similarity is markedly more discriminative. Embeddings from
different models are never compared.

The vision engine never re-locks silently. A smart re-acquire is an explicit,
announced designate. Without embeddings on the detection stream, Smart coasts
and then gives up like Strict.

## Surfaces

- **Flight Skill** (`follow-me`): an armed-only, confirm-to-arm toggle in the
  cockpit Skill Bar (default hotkey `⇧F`). Arming writes the per-drone
  `active` config. The agent reads it live and reports `follow.state`.
- **Target actions** (`contributes.target_actions`): click a detected subject
  in the host's cockpit overlay and pick **Follow this target** (hotkey `f`)
  or **Stop following** (hotkey `x`).
- **Native settings** (`contributes.parameters`): distance, height, gimbal
  point, camera, field of view, mount pitch, the re-acquire mode (a two-value
  enum that hosts render as a Strict | Smart toggle), the re-acquire window and
  similarity, the speed cap and the detector model.
- **Node-detail tab** (`node.detail.tab`, drone profile): Specs and a live
  read-back covering:
  - phase and lock;
  - range ± σ and the cues it came from;
  - height and its source;
  - subject speed;
  - re-acquire mode and window;
  - detection-to-goal latency and goal rate.

## Events

| Topic               | Payload                                                         |
| ------------------- | --------------------------------------------------------------- |
| `follow.state`      | The read-back. `phase` is one of `idle`, `acquiring`, `following`, `coasting`, `searching`, `lost`, `holding`. The other fields are listed below. |
| `follow.reacquired` | `{track_id, similarity, gap_s}`                                 |
| `follow.lost`       | `{track_id, reason}`, where `reason` is `lock_lost` or `reacquire_timeout`. |

The other `follow.state` fields:

- **Range:** `range_m`, `range_sigma_m`, `range_cues`.
- **Subject:** `target_speed_mps`, `bearing_deg`, `depression_deg`,
  `target_ned`, `target_vel_ned`, `predicted_bbox`.
- **Height:** `agl_m`, `agl_source`.
- **Re-acquire:** `reacquire_mode`, `coast_remaining_s`.
- **Timing:** `pipeline_latency_ms`, `setpoint_rate_hz`.
- **Lock:** `track_id`, `confidence`, `lock_state`.
- **Commanding:** `commanding`, `hold_reason`.
- **Flight controller:** `fc_armed`, `fc_guided`.
- **Setpoints:** `distance_setpoint_m`, `height_setpoint_m`.
- **Mode:** `follow_mode`, `follow_angle_deg`, `orbit_radius_m`, `orbit_rate_dps`,
  `orbit_direction`, `orbit_phase_deg`.
- **Pilot:** `lease_id`, `nudged`.

## Configuration

Per-drone configuration (see `config-schema.json`):

| Key                        | Default       | Range       | Meaning                                         |
| -------------------------- | ------------- | ----------- | ----------------------------------------------- |
| `active`                   | `false`       | —           | Armed/disarmed (toggled by the Skill).          |
| `follow_distance_m`        | `8`           | `3`–`30`    | Standoff distance behind the subject.           |
| `follow_height_m`          | `4`           | `0`–`20`    | Follow height above the ground.                 |
| `gimbal_point`             | `true`        | —           | Point a gimbal at the subject if present.       |
| `designate_camera`         | `auto`        | —           | Camera the follow loop consumes; `auto` reads the designated subject's camera (any camera until one is designated). |
| `camera_hfov_deg`          | `70`          | `30`–`160`  | Camera horizontal field of view.                |
| `mount_pitch_deg`          | `30`          | `0`–`90`    | Camera downward tilt below the horizon.         |
| `reacquire_mode`           | `smart`       | `strict`, `smart` | What a lost lock does.                    |
| `reacquire_window_s`       | `4`           | `0.5`–`15`  | How long a smart re-acquire coasts.             |
| `reacquire_min_similarity` | `0.75`        | `0.5`–`0.99`| Appearance similarity needed to re-acquire.     |
| `max_speed_mps`            | `6`           | `0.5`–`15`  | Cap on the velocity feed-forward.               |
| `follow_mode`              | `follow`      | follow, orbit | Follow at an angle, or orbit the subject.     |
| `follow_angle_deg`         | `180`         | `0`–`355`   | Angle from the travel heading (follow mode).    |
| `orbit_radius_m`           | `8`           | `3`–`30`    | Orbit radius.                                   |
| `orbit_rate_dps`           | `10`          | `2`–`30`    | Orbit angular rate.                             |
| `orbit_direction`          | `cw`          | cw, ccw     | Orbit direction seen from above.                |
| `obstacle_margin_m`        | `1.5`         | `0.8`–`5`   | Clearance the pilot's planner keeps.            |
| `detector`                 | `coco-person` | —           | Detection model the follow loop consumes.       |

## Requirements

- ADOS agent 0.109.0 or later: the pilot flies the vehicle; the plugin only
  sends goals under the pilot's motion lease (capability `flight.motion`).
  A multirotor, or a VTOL in hover.
- A camera bound to the vision pipeline producing the detection stream. While
  the behaviour is active the plugin holds a detection lease on that camera
  (renewed every 2 s, 6 s hold, released when it goes inactive), so the engine
  runs detection there only while follow-me needs it.
- Recommended: a downward rangefinder on the flight controller for
  terrain-following height.
- Optional: the world-engine extension's planar scan, for the lidar range cue.

## Build, test, pack

```bash
pnpm build:follow-me && pnpm test:follow-me    # GCS half (repo root)
uv run pytest extensions/follow-me/agent/tests  # agent half (repo root)
./scripts/pack.sh follow-me
```

## License

GPL-3.0-or-later.
