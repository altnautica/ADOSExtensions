# Changelog

All notable changes to ADOS Follow-Me are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/) and the project
uses independent semantic versioning per extension.

## [Unreleased]

### Added

- The follow read-back carries `frame` (`fc_local_ned`, the frame of every
  position in it), `target_bbox` (the followed track's box from its newest
  batch while the lock is held, with its frame size and the frame's capture
  times `ts_ms` and `ts_sensor_ms`) and `predicted_ned` (the predicted
  subject position every 0.5 s over the next 5 s while there is an
  estimate), so a replay draws the lock box and the prediction from the run.

## [0.5.2] - 2026-11-20

### Changed

- The navigation state is read from the shared topic
  `plugin.navigation.state` (Navigation 1.0.0 publishes it there), which
  the plugin host lets Follow-Me subscribe to with `telemetry.read`.

- The optional planar scan is read from `plugin.navigation.scan`: the
  Navigation extension publishes it now.

## [0.5.1] - 2026-11-17

### Fixed

- A standing subject is no longer left on range outliers. With a 6 Hz
  detector missing about a third of its detections, a 1.5 s exit span can
  hold only six fixes; three range outliers among them pulled the span's
  median 1.6 m off the centre on two fixes in a row, standing ended, and
  the vehicle stepped 0.8 m and back. Standing now ends on that median only
  while the newest fix also lies out along its offset, at least half as far
  (`STILL_EXIT_CONFIRM`): a walk carries the newest fix ahead of the
  median, outliers leave it near the centre. Walking off is told as soon as
  before.
- The flight-sim tests start their clock at a fixed time instead of the
  host's uptime, so a result no longer depends on the machine.

## [0.5.0] - 2026-11-17

Needs ADOS agent 0.118.0 or later: the pilot's vantage goal and its goal
status, the goal `vel_mode` the pilot shapes and flies, and the cockpit,
kiosk, dashboard and terminal UI that show the recovery phases and the
waiting prompt. Needs a person detector at 6 Hz or more; slower detection
is not supported.

### Changed

- The follow never flies lower than 3.4 m above the ground: a 2 m subject,
  the rotors' 0.4 m reach and 1 m between them, whatever the detector calls
  the subject. A lower `follow_height_m` (it could be 0) is flown at 3.4 m,
  read back as `height_raised`, and the panel says the height is kept
  above head height. A subject who turns straight back into the vehicle
  passes under it with the rotors at least 1 m clear: in the closed-loop
  sim, at a walk, a jog and a run, turning back from 60° off to straight
  on, at 10 Hz and 6 Hz detection (it came within 0.7 m of the rotors, or
  struck them, at a 1.5 m height before).
- The follow keeps clear of the subject: toward it the velocity is held to
  what still stops short of the clear distance (2.5 m plus two sigmas of
  the range error, held at least 2 m inside the standoff or orbit radius
  so range noise does not set it off: 2 m at a 4 m standoff, 1 m at 3 m,
  where the 3.4 m height floor keeps the rotors clear), a path that passes
  beside it is kept, and within it the vehicle backs away. The cut in the
  closing speed is not slewed: under the Sport profile a jogger who stops
  dead 6 m ahead is no longer overflown. The newest fixes' range is kept
  clear of when it is nearer than the subject estimate's, which trails a
  subject turning back. Goals carry `subject_sigma_m`, the error of the
  subject's range, for the pilot's own subject brake (agent 0.118.0),
  bounded so the distance that brake keeps stays 2 m inside the standoff.
- The shortest follow distance and orbit radius flown depend on how well
  the subject's range is measured, read from how far the fixes really
  scatter: 3 m on a measured range (a lidar or a depth camera, σ 0.2 m or
  better), 5 m on a camera's alone, where range noise at 3-4 m set the
  guard and the pilot's brake off every few seconds. A shorter setting is
  flown farther, read back as `standoff_min_m`, `standoff_raised` and
  `standoff_reason` (`camera-range`, `range-unknown`, `subject-standing`
  or `closing-in`), and the panel says which. Until the range is known
  the follow keeps the distance it engaged at; a larger minimum applies
  at once, a smaller one is flown in to at 0.2 m/s and only while the
  subject moves, never toward a person standing still. A stick nudge on
  the distance or radius starts from the one flown and stops at the
  minimum.
- On a measured range the command is still smoothed a little (0.1 s): a
  walker followed at 3-4 m is commanded within 3.5 m/s³ (95th percentile),
  where the raw command jerked to 6.
- Once the guard's cut in the closing speed ends, the command goes on
  from the vehicle's own velocity away from the subject, not from the
  cut's: a fresh track's first noisy fixes, which have the guard back
  away, no longer carry the vehicle metres past the standoff.
- Follow and orbit goals carry the follow's complete desired velocity
  (`vel_mode` `command`); the pilot shapes it within the vehicle's jerk and
  acceleration limits and adds no position loop of its own. The velocity
  comes from a speed law: the subject's own motion plus a closing speed
  that is full when far, ramps down at no more than 0.6 m/s² into the
  standoff and is nothing at it, never changing by more than 1.0 m/s² (the
  pilot's Normal profile) per second. Follow speed no longer depends on the
  flight controller's waypoint speed and acceleration.
- Range noise no longer swings the follow speed. The subject's velocity is
  fed forward smoothed, the range error is corrected only past a deadband
  and smoothed, and so is the command, each by the error the subject
  estimate really carries: its spread scaled by how far the fixes actually
  scatter. On clean ranges next to none of it applies. Following a walking
  person under monocular range noise in the closed-loop sim, the vehicle
  tilts past 5° about once in six minutes (was 19-30 times a minute) and
  the commanded jerk stays near 2 m/s³ (95th percentile; was 25-39), at
  10 Hz and 6 Hz detection.
- A standing subject is held still: once the median of its fixes stops
  moving the follow is built round a fixed centre with no velocity, the
  goal moves only past a 0.5 m deadband and the vehicle holds within 0.2 m
  of it. It follows again once the subject walks off, or at once when four
  fixes in a row land well off the centre on one side (a subject breaking
  into a run), counting fixes the outlier gate refused.
- The follow speed limit (`max_speed_mps`) can be raised to 10 m/s (was
  15); the default stays 6 m/s.
- The yaw turns toward the subject at no more than 30°/s, from a rate law
  with a deadband (6° round a standing subject, 2° otherwise), instead of
  stepping to each new sighting.
- Range outliers are gated out of the subject estimate (χ² 3 sigma; fixes
  past 2 sigma are weighed down), and three gated fixes in a row that agree
  with each other are taken as a real move. Each fix is placed at its
  frame's exposure time and the vehicle's position then, and fused with a
  range sigma of at least 0.75 m (6 % of the range). The person speed cap
  is 5 m/s.
- A missed detection no longer stops or slows the follow: up to 2 s
  without a fused fix are ridden through on the prediction at full speed.
- A hold names the point it began at and a zero velocity, so the pilot
  stops on its profile instead of chasing the current pose.

- A lost subject is recovered instead of coasted on. In Smart mode the
  follow predicts where the subject went (`predicting`), asks the pilot for a
  vantage point that sees past the obstacle within `search_radius_m` of the
  loss (`repositioning`), and sweeps the camera from there, widening step by
  step (`searching`), all within `search_budget_s`. When the search runs out,
  in Strict mode at once, and with predictive search off after the
  prediction, the vehicle holds at a latched point and waits for the
  operator (`holding-await-operator`). The `coasting` and `lost` phases are
  gone.
- The default standoff is 6 m (was 8), where a walking person is still tall
  enough in the frame to detect.
- An unsure lock keeps following the prediction at reduced speed in Smart
  mode (`uncertain_mode` `follow`, the default) or holds (`hold`, and always
  in Strict); an unsure spell longer than 3 s is handled as a loss.
- The subject's appearance template only learns from a solid lock, and two
  people inside the search gate who look alike are never re-locked.
- `follow.reacquired` is announced when the engine locks a re-acquire
  designate, naming the locked track, and carries `mahalanobis` and the
  `phase` it happened in. `follow.lost` carries the reason
  (`lock_lost`, `search_exhausted` or `predictive_search_off`),
  `awaiting_operator` and the hover point `hold_ne`.
- The `follow.state` read-back replaces `coast_remaining_s` with
  `reacquire_remaining_s`. The hold reason `lock-lost` is replaced by
  `awaiting-operator`, `search-exhausted` and `predictive-search-off`.
- The subject's predicted spread, used for the re-lock position check, the
  search area and the vantage request, is bounded by how far the subject can
  have gone at its speed cap, so a long gap no longer admits anyone in a wide
  circle. The whole configured search time (up to 60 s) is searched: the
  prediction lasts 70 s, the widest step repeats until the budget is spent,
  and Resume after a long wait searches around the last estimate instead of
  holding with `no-range`.
- An operator designate is followed at once, also in the middle of a search.
  A re-acquire designate whose request failed is still the search's own, so
  its lock is announced and never taken for an operator designate.
- The last search step sweeps all the way round. A vantage the vehicle cannot
  settle at still gets its sweep, after hovering within 1.5 m of it for 2 s
  or once the flight there is 3 s overdue.
- The `camera-lost` hold no longer ends a recovery. The tracks it gave up on
  stay excluded, and a follow waiting for the operator still waits when the
  frames return, instead of locking onto the lost track the engine reports
  again. While it waits the hold stays at the latched point, the phase stays
  `holding-await-operator` with the reason `camera-lost`, and the prompt
  offers no resume until the frames return.
- The follow tab offers no resume in Strict mode, where a resumed search
  gives up again at once.
- Orbit mode keeps circling when obstacles or a detour make the drone lag.
  The orbit point advances while it leads the drone's bearing from the
  subject by at most 45°, instead of only while the drone is within 2 m of
  it, so a detour round a tree, a drone held inside or outside the circle,
  or an error in the subject's range no longer stops the orbit, and a drone
  ahead of the orbit point is no longer pulled back. After a re-acquire the
  orbit restarts from where the drone is, instead of heading for an orbit
  point the search may have left across the circle. Switching the follow
  back on after a stop or after the pilot took the drone back does the same,
  and the follow angle is again measured from where the drone is, instead
  of swinging the drone back to where it was.

### Added

- A 320×320 CPU variant of the person model, declared for the detector's
  crop model (the crop pass round the designated subject; the agent sets it
  through the detector's crop model setting). The 640×640 variant stays the
  whole-frame model.
- Settings `uncertain_mode`, `predictive_search` (turned back on at every
  arm), `search_budget_s`, `search_radius_m` and `search_speed_mps`, and the
  `operator_resume_seq` key a surface writes to resume a search.
- `follow.state` fields `search_budget_s`, `search_step`, `search_area`,
  `predicted_path_ne`, `vantage_ne`, `vantage_state`, `occluder`, `hold_ne`,
  `uncertain_mode`, `predictive_search`, `operator_resume_seq`, and
  `outpaced` with `speed_limit_mps`: the subject has been measured faster
  than the limit in force (the lower of `max_speed_mps` and the pilot's
  2 m/s cap without GNSS) for 2 s; the follow tab shows it. The cap is
  rebuilt with the pilot's rule for unknown EKF source parameters: uncapped
  while the source set last named by the vision-nav `nav.state`
  (`active_set`, set 1 before any) is 1-3 and `GPS_RAW_INT` has a fresh 3-D
  fix. vision-nav's external-navigation mode on the GPS set is not capped.
- The vantage goal carries `hypotheses_ne`: where the subject was last seen
  and 3 m back along its track, for the pilot to cover besides the
  prediction.
- An EKF position reset is detected in `LOCAL_POSITION_NED` (samples at most
  0.3 s apart), and the subject estimate, the search anchor, the vantage
  point and a latched hover point move with the frame.
- The follow tab shows the search (time left, step, vantage point, what hides
  the subject) and, while the follow waits, a prompt to resume the search,
  return home, land or stop following, or to click the subject to follow it
  again.

### Removed

- The `reacquire_window_s` setting.

### Known limitations

- The pilot does not forward its speed cap to plugins, and a plugin cannot
  read the EKF source parameters. A source set whose `EK3_SRCn_POSXY` names
  a non-GPS source therefore reads as uncapped in `speed_limit_mps`, while
  the pilot, which reads that parameter, flies it capped at 2 m/s.
- With a 6 Hz detector, a standing subject who walks or runs off is told
  from range noise about as late as the still-subject check allows: in the
  sim a run is seen 1.2 s after it starts (1.2 s or less at 10 Hz) and a
  walk straight away from the vehicle opens the range to 10.2 m (10 m or
  less at 10 Hz). A range cue better than monocular (a lidar fused with the
  subject's range) closes this.
- A subject who turns straight back at the vehicle from the standoff can
  still pass under it at a walk or a run: monocular range takes half a
  second to a second of fixes to show the turn, and the vehicle about as
  long again to stop. The follow height keeps the rotors clear then (see
  above); a range cue better than monocular shortens it.

## [0.4.9] - 2026-11-14

### Added

- A `camera-lost` hold reason. The vision engine publishes a batch for every
  inferred frame, an empty one too, so with no batch from the follow camera
  for more than 2 s (after the first 5 s of an activation) the follow holds,
  ends a smart re-acquire, stops the subject estimate and logs the episode
  once. It takes precedence over the lock reasons. The tab labels it
  "Camera lost".
- `follow.state` reports the camera geometry in use: `camera_hfov_deg`,
  `camera_mount_pitch_deg` (negative below the nose) and
  `camera_geometry_source` (`roster`, `settings` or `default`). The tab
  shows it as "Camera geometry" with its source.

### Changed

- The camera's field of view and mount pitch come from the camera roster,
  carried on each detection batch (`hfov_deg`, `mount_pitch_deg`), when the
  agent sends it. Otherwise the `camera_hfov_deg` and `mount_pitch_deg`
  settings apply, and with those unset the reference front camera: 87°
  wide, 15° below the nose. The two settings no longer have a default of
  their own (70° and 30° before). A reported or commanded gimbal angle still
  replaces the mount tilt.

### Fixed

- The attitude history is kept on the flight controller's clock
  (`time_boot_ms`), mapped onto the companion's clock by the least arrival
  offset over the last 64 messages; the offset falls at once and rises no
  faster than 5 % of the elapsed time, and restarts when the flight
  controller's time steps back by more than 1 s. A MAVLink delivery stall
  that hands over a burst of attitude samples at once no longer stamps them
  all at one instant, so a frame exposed during the stall is projected
  through the attitude at its exposure instead of the newest one.
- A frame is placed at its exposure on the flight controller's clock: its
  sensor stamp (`ts_sensor_ms`) when that lies within 1 s of its host stamp
  mapped onto that clock, otherwise the mapped host stamp. A simulator
  frame whose host stamp is its late arrival is no longer projected through
  the attitude 0.1-0.3 s after its exposure.
## [0.4.8] - 2026-11-12

### Fixed

- In the follow tab, the last report delivered again after an older one was
  dropped (a failed poll falling back to a slower source) is still ignored,
  so a feed that has stopped reads stale 2000 ms after its last report
  arrived instead of staying live.

## [0.4.7] - 2026-11-12

### Fixed

- An older follow report that arrives after a newer one (the host falling
  back to a slower source after a failed poll) no longer replaces the newer
  phase or keeps the tab reading live.

## [0.4.6] - 2026-11-12

### Fixed

- A live follow report no longer reads Stale when the drone clock runs
  behind or a poll is slow. Only the first report is judged by its own
  timestamp, so an old report shown on opening the tab still reads Stale,
  and the tab still reads Stale 2 s after the reports stop.

## [0.4.5] - 2026-11-12

### Fixed

- The About row names the configured camera the way the host does,
  "Name (id)", from the camera list the host sends. It shows the id when
  the host has no name for that camera.

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
