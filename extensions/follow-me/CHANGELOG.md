# Changelog

All notable changes to ADOS Follow-Me are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/) and the project
uses independent semantic versioning per extension.

## [0.4.4] - 2026-11-12

### Fixed

- The tab no longer shows a silent drone's last report as live. A report
  2 s old or older blanks the live values and reads "Stale" with the time
  since the last report; the age is taken from the report's own timestamp
  when the host forwards it.
- Before the first report the tab says it is waiting for the drone's follow
  report and shows dashes, instead of readings such as Phase Idle and
  Commanding No that looked real.
- The Pilot lease row read "—" while the follow held a lease, because the
  pilot's lease id is a number. It now reads Held or None.
- In orbit the read-back no longer carries the follow distance as a
  distance setpoint, and the tab shows the orbit radius in its place.
- The Holding phase reads in the warning tone and Acquiring in the track
  tone, matching the video HUD.
- The tab uses the host's UI font, and every row label reads in the same
  muted tone.

### Changed

- The Mode summary, units and the designate camera are worded through the
  locale: "Orbit 10.5 m · 12°/s · Counter-clockwise", and an `auto` camera
  reads "Auto (first detect camera)".
- The plugin reports the same version as its manifest.

## [0.4.3] - 2026-11-08

### Fixed

- On a vehicle with a fixed camera and gimbal pointing left on (the
  default), every box after the first was projected as if the camera
  pointed where the plugin had asked a gimbal to point, at the previous
  estimate. Each measurement then agreed with the estimate rather than the
  subject: in a recorded flight the measured bearing read 21–38° for a
  subject at 89°, the estimate ran metres off, and the yaw turned the
  subject out of the frame. A commanded gimbal angle now counts only once a
  gimbal has reported `MOUNT_ORIENTATION`; until then the camera is fixed
  at its mount pitch.

### Changed

- The goal's yaw follows the latest sighting's measured bearing (through
  the attitude at its frame's exposure) while that sighting is at most 3 s
  old, instead of the bearing to the filtered position; the position goal
  still follows the filter. A hold while the lock is uncertain faces the
  same bearing instead of the current heading.
- The subject's velocity estimate is capped by its detector class: a
  `person` at 3 m/s. The estimate had read 4–6 m/s for a 1.2 m/s walker. A
  4 m/s runner is still followed, trailed by under a metre.
- While the lock is uncertain the estimate stops where it is (no velocity)
  instead of extrapolating its last velocity through the hold; the next
  sightings estimate the motion again.

## [0.4.2] - 2026-11-08

### Fixed

- The drone pitched back and forth by 10–20° while following, and its
  follow goal jumped by metres on every detection batch. A batch reaches
  the follow loop about 0.4 s after its frame was exposed, and the box was
  projected through the attitude at arrival: a drone that had pitched 10°
  since put the subject's feet metres nearer or farther (ground-range cue
  5.4 m, then 212 m, on a subject 8 m away), the goal followed, and the
  drone pitched again. The plugin now keeps three seconds of `ATTITUDE`
  and projects each box through the attitude at its frame's exposure
  (the batch's `ts_ms`); a frame the history does not cover takes the
  latest attitude, as before.

## [0.4.1] - 2026-11-07

### Fixed

- Stick nudges follow the pilot's signs: right stick moves the drone to its
  right around the subject. In `follow` the angle falls with right stick;
  in `orbit` right stick turns the orbit counter-clockwise.
- A nudge reaches the next goal at once instead of waiting up to a second
  for the config refresh.
- The follow angle is written back on the config's 5° grid in 0–355.
- The motion lease id is kept as the pilot sends it (an integer) and passed
  back unchanged to goals and the release.
- Start no longer fails on an agent without the stick subscription: follow
  flies without nudges and logs `follow_me_sticks_unavailable`.

## [0.4.0] - 2026-11-06

### Added

- Follow modes: `follow` holds `follow_angle_deg` from the subject's travel
  heading (180 behind, 90 right side, 0 lead) at `follow_distance_m`;
  `orbit` circles the subject at `orbit_radius_m` and `orbit_rate_dps`,
  `orbit_direction` cw or ccw. The follow reference heading starts where the
  drone is (no swing on engage) and slews at up to 45 °/s; the orbit phase
  advances only once the drone is within 2 m of the ring.
- The vehicle is flown by the agent's pilot: the plugin holds the pilot's
  motion lease (`flight.motion`) and sends goals carrying the subject and
  the detour ring, so obstacle avoidance comes from the pilot's planner with
  the skill's `obstacle_margin_m`. A refused lease holds with `lease-busy` or
  `lease-refused` and is retried once a second.
- A revoked lease (pause, stick takeover, land, RTH, FC mode change) writes
  `active=false` and publishes `follow.stopped {reason}`.
- Stick nudges while the lease is held: angle or orbit rate, distance or
  radius, and height, clamped to their ranges and written back to the
  per-drone config at most once a second.
- `follow.state` gains `follow_mode`, `follow_angle_deg`, `orbit_radius_m`,
  `orbit_rate_dps`, `orbit_direction`, `orbit_phase_deg`, `lease_id` and
  `nudged`; the settings tab shows the mode, the lease and whether the
  sticks nudged it.
- The skill declares `motion`, `nudge`, `params` and `vehicle_classes`
  (multirotor, vtol).

### Changed

- Requires agent 0.109.0. The `flight.guided_setpoint` permission and the
  local SET_POSITION_TARGET builder are gone; holds no longer send a burst
  of zero-velocity setpoints (the pilot holds).
- The `fc-not-guided` hold reason is retired: the pilot owns the flight mode.

### Fixed

- `follow_distance_m` is clamped to 3–30 m and `follow_height_m` to 0–20 m;
  they were unclamped.

## [0.3.1] - 2026-11-06

### Changed

- While the behaviour is active the plugin holds a detection lease on its
  follow camera, so the vision engine runs detection there only while
  follow-me needs it. The lease is renewed every 2 s with a 6 s hold, moves
  when the follow camera changes, and is released (hold 0) when the
  behaviour goes inactive or the plugin stops. The follow camera is the
  pinned `designate_camera`; on `auto` it is the camera of the designated
  subject while it is locked or being re-acquired (a designation itself
  keeps detection running on the camera it was made on).
- On an agent without the detection lease call the plugin logs it once and
  carries on; detection then runs wherever the agent runs it.

### Fixed

- On `auto` the plugin read every camera's batches even with a subject
  designated, so a second camera (a belly camera's whole-frame box) could
  feed the lock gate and the batch a re-acquire designates from. Once a
  subject is locked or being re-acquired, only its camera is read; with no
  subject any camera still carries a designation.

## [0.3.0] - 2026-11-03

### Changed

- Setpoints are `SET_POSITION_TARGET_LOCAL_NED` (frame `MAV_FRAME_LOCAL_NED`,
  type mask 2496) at 20 Hz. Each one carries position, a velocity
  feed-forward of the subject's velocity (clamped to `max_speed_mps`) and yaw
  toward the subject. The `SET_POSITION_TARGET_GLOBAL_INT` builder is removed.
- Control runs in the flight controller's local frame, from
  `LOCAL_POSITION_NED`. `GLOBAL_POSITION_INT` now supplies only the barometric
  height fallback and lat/lon. The agent (0.108.0 or later) streams
  `LOCAL_POSITION_NED` and `DISTANCE_SENSOR` at 20 Hz on a drone; the
  minimum agent version is raised to match.
- The subject is projected through the bottom-centre of its box (the feet)
  rather than the box centre, which projected long by about
  `0.85 m / tan(depression)`.
- Height above ground comes from a downward `DISTANCE_SENSOR` (orientation 25)
  while it is fresh (< 0.3 s), inside its reported range and the vehicle is
  tilted under 30 degrees. Otherwise the barometric height is used. The
  follow height is terrain-following.
- Range is the inverse-variance fusion of up to three cues: the ground
  intersection, the box height of a 1.70 m subject (only when the box clears
  every frame edge) and an optional planar-lidar return from
  `plugin.world-engine.scan`. The lidar return is accepted only within 25 % of
  the other cues.
- A constant-velocity Kalman filter tracks the subject's position and velocity.
  The standoff trails the subject along its direction of travel when it moves
  faster than 0.5 m/s.
- The hold reason `no-ground-fix` is renamed `no-range`. Every hold that
  follows commanding sends a short burst of zero-velocity setpoints at the
  current position, so the last feed-forward never stays active.

### Added

- `reacquire_mode` (`strict` | `smart`, default `smart`), plus
  `reacquire_window_s`, `reacquire_min_similarity` and `max_speed_mps`.
  - In `smart` mode, a lost lock coasts toward the predicted standoff at half
    feed-forward. A `person` that matches the subject's appearance embedding
    and lies within 3 sigma of the prediction is designated with the engine,
    and the plugin announces it on `follow.reacquired`.
  - When the window expires, or in `strict` mode, the plugin announces
    `follow.lost` and holds.
- New `follow.state` fields:
  - `phase`;
  - `range_sigma_m` and `range_cues`;
  - `target_speed_mps`, `agl_m` and `agl_source`;
  - `reacquire_mode` and `coast_remaining_s`;
  - `pipeline_latency_ms` (detection frame to setpoint send) and
    `setpoint_rate_hz`;
  - `track_id` and `confidence`;
  - `bearing_deg` and `depression_deg`;
  - `target_ned` and `target_vel_ned`;
  - `predicted_bbox` while coasting.
- The node-detail tab shows the phase, the re-acquire mode, the range with its
  uncertainty and cues, the height source, the subject speed, the
  re-acquire window, the latency and the setpoint rate.
- New permissions:
  - `vision.track.designate`, for the re-acquire designate;
  - `event.subscribe` and `telemetry.read`, for the optional scan topic.

### Fixed

- The GCS bundle reported version 0.2.9.
- The GCS default designate camera matches the agent (`auto`).

## [0.2.10]

### Fixed

- `supported_boards` now lists HAL board profile names or SoC ids (`cm4`,
  `cm5`, `rock-5c-lite`, `rk3588s2`, `rk3576`, `rpi5`, `cubie-a7s`,
  `cubie-a7z`), so the install gate matches probed boards.
- The documentation link points at the extensions catalog entry.
- The guided-mode check reads the vehicle class from the autopilot's
  heartbeat and uses that vehicle's ArduPilot mode numbers. On a plane it
  previously used Copter numbers, so ACRO and QLAND read as guided and
  GUIDED did not. Plane and QuadPlane GUIDED, Rover GUIDED and Copter
  GUIDED/GUIDED_NOGPS are recognised; an unrecognised vehicle never reads
  as guided.

### Changed

- The agent half is split into `fc_state.py` (flight-controller, pose and
  gimbal state and their MAVLink handlers) and `loop.py` (the follow loop and
  the `follow.state` read-back). Behaviour is unchanged.

## [0.2.9]

### Fixed

- The follow loop now reads the flight controller's MAVLink messages. The
  host delivers each subscribed message as its raw wire frame, and the
  handlers read decoded field names from that delivery, so on a real vehicle
  the arm state, the flight mode, the attitude, the position and the gimbal
  angles were never read: the follow gate always saw a disarmed vehicle and
  never commanded. Each delivery is now decoded, and a frame that does not
  decode is ignored.
- Only the autopilot's heartbeat sets the arm state and flight mode. Once the
  heartbeats decode, a ground station's or a gimbal's heartbeat would otherwise
  read as a disarmed vehicle.

### Removed

- The agent `event.subscribe` permission, which nothing in the agent half used.
- The in-code manifest mirror, which nothing read and which had drifted from
  `manifest.yaml`. Every version site now agrees.

## [0.2.8]

- Declares the MAVLink component the plugin transmits as (191) under
  `agent.mavlink_components`, and keeps the in-code manifest mirror in step. The
  one extension that commands the vehicle's flight path previously declared
  nothing about its MAVLink component surface.

## [0.2.7]

### Fixed

- The follow loop no longer flies on flight-controller telemetry that has
  stopped arriving. Vehicle attitude, position, arm state and gimbal
  attitude were each guarded by a flag set on the first message and never
  cleared, so once any of them had arrived once the loop treated it as
  current forever. If a telemetry subscription stalled while the write path
  still worked, the loop kept commanding at its full rate and projected
  fresh bounding boxes through a frozen attitude and position, steering
  toward a point computed from where the aircraft used to be, with the error
  growing for as long as the stall lasted. Each input now carries its own
  freshness window, sized from the rate that message is expected at, and
  attitude and position are aged separately because either can stop alone.
- The live read-back no longer reports the flight controller as armed and in
  a guided mode once the heartbeat carrying that state has stopped arriving.

### Added

- The read-back names which gate is holding the follow, and the tab renders
  it. Reporting only that the loop was not commanding made a stalled
  telemetry stream look exactly like a normal disarmed, on-the-ground hold.
  The two readings that mean an input stopped arriving render as faults.

### Changed

- Package versions across the extension are aligned to the manifest version,
  which the previous release left behind at 0.2.5.

## [0.2.6]

### Added

- Added Radxa Cubie A7S / Allwinner A733 (`cubie-a7s`, `a733`, `sun60iw2`) to
  `hardware_requirements.boards` and `compatibility.supported_boards`, so the
  board-compatibility gate no longer blocks install on that board.

## [0.2.5]

### Changed

- Replaced the hand-typed `designate_camera` string (defaulted to a device
  name) with a camera picker: the operator pins the camera whose detections
  the follow loop consumes, or leaves it on auto to follow the designated
  subject on whatever camera the engine feeds.

### Added

- Two conservative tools for an assistant: `follow_status` (read the follow
  read-back) and `stop_follow` (disarm). There is deliberately no start tool —
  a follow begins only when an operator designates a subject in the cockpit.

## [0.2.4]

### Changed

- The guided follow position setpoint is now emitted through the agent's
  scoped flight sender (`flight.guided_setpoint`) instead of a raw MAVLink
  write, so the flight-command surface is gated by a single high-risk
  capability and the agent's MAVLink router owns encoding and link stamping.
  The gimbal point-at-subject command still rides `mavlink.write`. Requires
  agent 0.99.179+.

## [0.2.3]

### Fixed

- The image-to-ground projection now uses the detection batch's real source
  frame size instead of guessing it from the bounding box, so the camera
  center and angular scale are correct and the ground setpoint lands where
  the subject is.
- When the gimbal points at the subject, the projection now uses the gimbal's
  actual reported attitude (MOUNT_ORIENTATION), falling back to the last angle
  the plugin commanded, then the fixed mount tilt. It previously assumed a
  fixed forward boresight while commanding the gimbal elsewhere, which threw
  the ground setpoint off.
- While coasting on the last sighting through the brief lock-hold window, the
  loop now holds the last commanded setpoint instead of re-projecting the
  frozen bounding box through fresh vehicle attitude, so the setpoint no
  longer drifts on stale image data.

## [0.2.2]

### Added

- A `mount_pitch_deg` setting (the camera's fixed downward tilt below the
  horizon, default 30, range 0 to 90) exposed as a native parameter, in
  `config-schema.json`, and in the GCS config. A forward camera left at 0
  never resolves a ground point, so the follow loop would never command; the
  new default and control fix that.
- A "Stop following" cockpit target action (hotkey `x`) so a followed subject
  can be released without opening the Skill Bar.
- The live read-back now carries the flight controller's armed and
  guided-mode state (`fc_armed`, `fc_guided`), shown on the node-detail tab.

### Changed

- `commanding` is now reported true only when the flight controller is armed
  AND in a guided or offboard mode that accepts the setpoints, so the
  read-back no longer claims to command an FC that would ignore it.
- The Skill Bar toggle now confirms before it arms (arming streams guided
  setpoints) and moved to `⇧F` so it no longer shares the `f` key with the
  "Follow this target" action.
- The follow loop reads the full config on a slow interval and only the live
  arm/disarm toggle each tick, cutting the per-loop config IPC.
- Changing the designate camera at runtime now takes effect without a
  restart.

## [0.2.1]

### Changed

- The click-to-follow interaction now rides the host's shared target overlay
  and a `target.action` ("Follow this target") instead of a private
  `video.overlay`, so designation is consistent with every other target
  behaviour and the box drawing is host-owned.
- Adopted the shared locked-target safety gate: the follow behaviour stops
  and holds on an uncertain or lost lock and never auto-re-acquires, matching
  the platform's one canonical safety contract.

### Added

- First release. Hybrid plugin contributing from one manifest: a
  `follow-me` flight Skill, an interactive `video.overlay` click-to-follow
  surface, native settings (`contributes.parameters`, including a model
  picker bound to the shared vision detector), and a `node.detail.tab`
  live-metrics tab on the drone profile.
- Agent half (Python): subscribes to the vision detection stream and the
  flight controller pose, designates the operator-clicked subject, projects
  its image position onto the ground with a pinhole model, and emits guided
  position setpoints (plus optional gimbal point-at-subject) at a steady
  rate.
- Lock-state safety gate: commanding stops on uncertain or lost, with no
  silent re-lock onto a different subject.
- Live `follow.state` read-back carrying the honest `commanding` flag, lock
  state, target id, range, and the distance/height setpoints.
