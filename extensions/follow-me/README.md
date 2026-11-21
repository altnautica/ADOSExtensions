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
   using a pinhole model, the flight controller's attitude at the frame's
   exposure, and the camera mount or gimbal angles. The field of view and
   the mount pitch come from the camera roster, carried on each detection
   batch (`hfov_deg`, `mount_pitch_deg`). An agent that does not carry them
   leaves them to the `camera_hfov_deg` and `mount_pitch_deg` settings, and
   with those unset the reference front camera applies, 87° wide and 15°
   below the nose. The tab shows which source is in use. A commanded
   gimbal angle counts only once a gimbal has reported
   (`MOUNT_ORIENTATION`); without one the camera is fixed and the mount
   pitch applies.

   The attitude history is kept on the flight controller's clock
   (`time_boot_ms`), mapped onto the companion's clock by the least
   arrival offset over the last 64 messages, so a delivery stall that
   hands over a burst of samples at once still places each at its instant.
   A frame's exposure is its sensor stamp (`ts_sensor_ms`) when that lies
   within 1 s of its host stamp mapped onto the flight controller's clock,
   otherwise the mapped host stamp.
3. It estimates the horizontal range from up to three cues, fused by
   inverse variance:
   - **ground**: where the feet ray meets the ground. It uses the height
     above ground from a downward rangefinder (`DISTANCE_SENSOR`,
     orientation 25) while that reading is fresh, inside its own range and
     the vehicle is tilted under 30 degrees. Otherwise it uses the barometric
     height;
   - **box height**: the slant range to a 1.70 m subject, its error taken as
     8 % of the range. It is used only when the box clears every frame edge;
   - **lidar**: the planar scan return at the subject's bearing, from the
     optional `plugin.navigation.scan` topic. It is accepted only within
     25 % of the other cues.
4. A constant-velocity Kalman filter tracks the subject's north/east position
   and velocity in the flight controller's local frame (`LOCAL_POSITION_NED`).
   The velocity estimate is capped by the subject's detector class: a
   `person` at 5 m/s, a run on a trail; faster readings are range noise.
   Fixes beyond 3 sigma of the prediction are gated out, unless three in a
   row agree with each other (a real move): the estimate then starts over
   on them, its velocity theirs. Other classes are not capped there. A fix
   0.5 s or more after the last one fused (a re-acquired box) is held out
   when it lies beyond three sigmas of the fix's noise and the estimate's
   position spread past what the subject can reach over the gap (1.5 m/s²
   from the prediction): a false re-lock, short of the subject or beyond
   it, is not fused into a velocity. The estimate predicts on; the next fix
   that agrees with it is fused, and two such held-out fixes, under a
   second apart, that agree with each other start it over there. Held-out
   fixes do not outlast a hold. With no fix fused for three of the track's
   usual fix intervals (at least 0.5 s: a coast), the velocity the follow
   flies on decays: on 2 s for a velocity known to 0.5 m/s, the sooner the
   less the fixes have pinned it.
5. It holds the agent pilot's motion lease and sends it a goal every tick
   (20 Hz). The goal carries the follow's complete desired velocity
   (`vel_mode` `command`); the pilot shapes it within the vehicle's
   acceleration and jerk limits and flies it through its obstacle planner,
   speed limits and geofence, adding no position loop of its own. Each goal
   carries:
   - **follow** mode: the point `follow_distance_m` from the subject at
     `follow_angle_deg` from its travel heading (180 behind, 90 on its right,
     270 on its left, 0 leading). The reference heading starts where the
     drone already is, so engaging causes no swing, and slews toward the
     travel heading at up to 45 °/s while the subject moves faster than
     0.5 m/s;
   - **orbit** mode: the point on a circle of `orbit_radius_m` around the
     subject, advancing at `orbit_rate_dps` in `orbit_direction` while it
     leads the drone's bearing from the subject by at most 45°, so a detour
     round an obstacle or a radial offset slows the orbit but never stops it;
   - the velocity from a speed law: the subject's velocity (plus the orbit's
     tangential velocity) and a closing speed toward the point, full when
     far, ramping down at no more than 0.6 m/s² into the standoff and
     nothing at it, changing by at most 2.5 m/s² per second (the flight
     controller's guided acceleration; the pilot shapes it to its own
     profile) and never above `max_speed_mps`. Under range noise the
     subject's velocity, the range error and the command are smoothed and
     the range error is corrected only past a deadband, each scaled by the
     error the subject estimate really carries (next to none on clean
     ranges, where the command is still smoothed over 0.1 s), so noise does
     not swing the speed. A standing subject is followed round a fixed
     centre with no velocity, the point moving only past a 0.5 m deadband;
   - a yaw that turns toward the latest sighting's measured bearing (the
     estimate's bearing once no sighting is newer than 3 s) at no more than
     30°/s, with a deadband (6° round a standing subject, 2° otherwise), and
     a terrain-following height of `follow_height_m` above the ground, never
     below 3.4 m (a 2 m subject, the rotors' 0.4 m reach and 1 m between
     them; a lower setting is flown at 3.4 m and read back as
     `height_raised`). Under a canopy the height is lowered: the pilot
     reports the leaves its leaning scans reach over the way to the subject
     (the `flight.overhead` event to the lease holder, `fly_under_agl_m`:
     the highest height whose swept band, the airframe's top plus 0.3 m,
     stays under them), and the follow flies the highest height between
     3.4 m and the one set that clears them, for 5 s after the last such
     report or until the pilot reports clear air; when 3.4 m does not
     clear them it holds 3.4 m and the pilot's brake keeps it out of the
     leaves. Read back as `height_lowered` and `height_reason`
     (`canopy`, `canopy_floor`);
   - the subject's position, with the error its range carries
     (`subject_sigma_m`, bounded so the distance the pilot's subject brake
     keeps stays 2 m inside the standoff), and the ring the pilot's planner searches for a
     detour that keeps the subject in view (±90° of the follow angle, ±60°
     of the orbit phase), with `obstacle_margin_m`.

   The velocity never closes on the subject inside the clear distance:
   2.5 m plus two sigmas of the range error, held at least 2 m inside the
   standoff or orbit radius so range noise at a short standoff does not
   set it off. At a 4 m standoff that is 2 m, at 3 m only 1 m; there the
   3.4 m height floor is what keeps the rotors a metre above the subject.
   On a coast it widens by two sigmas of the spread the estimate gained,
   up to 1 m inside the standoff.
   Toward the subject the velocity is held to what still stops short of
   the clear distance, braking at 0.6 m/s² (the slowest profile's goal
   brake) a second after the cut, at the faster of the command's closing
   speed and the vehicle's own; a path that passes it on one side is kept,
   and within it the vehicle backs away; the cut in the closing speed is
   not slewed, the rest of the change is, and once the cut ends the command
   goes on from the vehicle's own velocity away from the subject. The
   newest fixes' range is kept clear of when it is nearer than the
   estimate's, which trails a subject that turns back. On top, a hard guard
   keeps the 2.5 m safe distance (widened on a coast) from the subject's
   worst case: the nearer of the estimate flown on and its undecayed
   prediction (a coasting subject coming toward the vehicle keeps coming),
   or a standing subject's centre, with the newest fixes' range, and on a
   coast a sigma of its velocity's error faster toward the vehicle. The
   goal point never lies nearer that worst case than the standoff less
   1 m, and a recovery that loses a subject coming toward the vehicle backs
   away from its prediction rather than hold for a vantage. A subject who
   turns straight back at the vehicle from the standoff can still pass
   under it: range noise takes half a second to a second of fixes to show
   the turn, and the vehicle about as long to stop at the pilot's profile.
   The follow height keeps the rotors at least 1 m above the subject then.
   That floor is height above the ground under the vehicle: on barometer
   height alone (about 1.5 m of error) or with ground rising toward the
   subject, the metre can be used up, and the turn-back case is proven only
   over flat ground with a rangefinder.

   Every margin above is built from the detector's frame-to-frame box noise
   (8 % of range). The subject's own height is assumed to be 1.70 m: a
   1.5 m adult ranges about 13 % long and a 1.2 m child about 42 % long, so
   the 2.5 m hard guard is then about 2.2 m and 1.8 m in truth.

   The shortest standoff (`follow_distance_m` or `orbit_radius_m`) flown
   depends on the range the follow has, read from how far the fixes really
   scatter about the estimate: 3 m on a measured range (a lidar or a depth
   camera, σ 0.2 m or better), 5 m on a camera's alone, where the range
   noise at 3-4 m sets the guard and the pilot's brake off every few
   seconds. A shorter setting is flown farther and read back as
   `standoff_min_m`, `standoff_raised` and `standoff_reason`:
   `camera-range`, `range-unknown` (until the range is known the follow
   keeps the distance it engaged at, between the two), `subject-standing`
   (a smaller minimum is flown in to only while the subject moves, never
   toward a person standing still) or `closing-in` (at 0.2 m/s); a larger
   minimum applies at once. A stick nudge on the distance or radius starts
   from the one flown and stops at the minimum.
6. Optionally, it points a gimbal at the subject.

A lock-state safety gate stops commanding when the tracker reports the
subject uncertain, or when the pose or the heartbeat is missing; while the
lease is held a hold sends a position goal at the drone's current position.
While the lock is uncertain that hold faces the latest sighting, and the
subject estimate stops where it is instead of carrying its velocity on.
Switching the skill off releases the lease and the pilot brakes to its own
hold.

Hold reasons (`hold_reason`): `inactive`, `no-lock`, `lock-uncertain`,
`awaiting-operator`, `search-exhausted`, `predictive-search-off`,
`pose-stale`, `fc-stale`, `fc-disarmed`, `no-range`, `lease-busy`,
`lease-refused` and `camera-lost`. The vision engine publishes a batch for
every inferred frame, an empty one too, so `camera-lost` means no batch from
the follow camera for more than 2 s (after the first 5 s of an activation).
It stops the subject estimate and takes precedence over the lock reasons. A
recovery is kept through it, so a follow waiting for the operator still
waits when the frames return; meanwhile it keeps the phase
`holding-await-operator` and holds at the latched point.

### The agent's subject estimate

An agent that publishes the designated subject's world state
(`vision.subject.v1`, read with `event.subscribe` and the optional
`vision.subject.read`) gives the follow a second estimate: the subject's
position, velocity and spread from every cue the agent fuses, and its
predictions 0.5-3 s ahead, kept out of mapped obstacles. While a state is
at most 0.5 s old and names the locked track on the follow camera, the
goals, the gimbal, the read-back's `target_ned` and predictions, and a
recovery's predicted position and vantage request are built on it, and a
lock commands on it without a range fix of the follow's own. Otherwise the
follow flies its own estimate as above. A release of the locked track (the
designation ended, or the agent lost the subject) is a lost lock.

When the state is fed by the operator's phone alone (vision has lost the
subject), Smart with `uncertain_mode` `follow` keeps following an unsure or
lost lock on it, within the same separation guard, the phone's spread
widening the clear distance; the read-back says `subject_source: beacon`.
Once the phone's fixes stop for 0.5 s the loss is handled as usual. Strict,
and `hold`, never fly on the phone.

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

An uncertain lock first. In Smart, with `uncertain_mode` `follow` (the
default), the follow keeps flying the prediction, at full speed while a fix
was fused in the last 2 s and at half the subject's velocity after that;
with `hold`, and always in Strict, it holds. Either way an unsure spell
longer than 3 s is handled as a loss.

`reacquire_mode` decides what a lost lock does:

- **Strict** stops at a latched hover point, announces `follow.lost`, and
  holds for the operator (`holding-await-operator`).
- **Smart** (the default) recovers within `search_budget_s`:
  1. `predicting`: flies toward the predicted standoff at half feed-forward and
     at no more than `search_speed_mps`, for up to 2 s, while the prediction
     has not spread by more than 1.5 m;
  2. `repositioning`: sends the pilot a `vantage` goal (the predicted
     position, its covariance, an ellipse scale, the standoff,
     `search_radius_m` around where the loss happened, and two more places to
     cover: where the subject was last seen, in case it stopped, and 3 m back
     along its track, in case it turned back) at no more than
     `search_speed_mps`; the pilot chooses a point that sees them past the
     obstacles and reports it on `flight.goal.status`. A report within 0.3 s
     of a new request may be about the previous one and is ignored. The sweep
     starts where the vehicle is when the pilot stops reporting for 2 s, when
     the vehicle has been within 1.5 m of the vantage for 2 s, or when the
     flight there has taken 3 s longer than the distance at the search speed
     (gusts, a fence, a pilot that keeps replanning);
  3. `searching`: at that point (or where it is, when the pilot finds none),
     sweeps yaw and gimbal pitch across the predicted position, then asks for
     a wider vantage, step by step. The last step sweeps all the way round,
     and repeats until the budget is spent.

  The search is centred on where the subject was still, with no velocity,
  when it stood still at the last sighting: over the last 6 s it was seen
  (at least 6 sightings over 5 s) its estimated speed was at most 0.3 m/s
  at the median, the estimate got no further than 0.3 m/s would take it,
  and its speed at the last sighting was at most 0.6 m/s. Range noise on a
  standing subject leaves the estimate a slow walk's speed, and a search
  that ran it on would carry its vantage points onto the subject. A subject
  that was moving is searched on the prediction whose velocity fades after
  the last sighting (the coast), never on the last velocity run on. While
  searching, the follow backs away from the nearest place the subject may
  be (the prediction, the last estimate, where it stood, the places the
  search looks), and its goals name the search's spread as
  `subject_sigma_m`, so the pilot's subject brake keeps a wider distance.

  The prediction's spread is bounded by how far the subject can have gone
  (its class speed cap, 5 m/s for a person, times the time since it was last
  seen, plus that sighting's own spread), so the position check and the
  search area stay meaningful in a long gap. The prediction stops 70 s
  after the last sighting, past the longest budget; after that a search (a
  Resume after a long wait, for one) looks around the last estimate, or
  where the subject stood, with the bounded spread.

  In each new batch it looks for a `person` that passes two checks:
  - its re-identification embedding has cosine similarity of at least
    `reacquire_min_similarity` to the stored subject template, at least 0.08
    ahead of any other person inside the gate;
  - it lies within a Mahalanobis distance of 3 from the prediction, and no
    further from the last estimate than the subject can have gone.

  The best match is designated with the vision engine. `follow.reacquired`
  is announced when the engine locks that designate, naming the track it
  locked. A designate whose request failed is still treated as the search's
  own (the engine may have applied it), so its lock is announced too and is
  never taken for an operator designate. A designate the engine reports
  lost, or has not locked within 3 s, is abandoned and matching continues
  (on newer batches only). When the budget runs out, or after the prediction
  phase with `predictive_search` off, the plugin announces `follow.lost` and
  holds for the operator.

Holding for the operator keeps the motion lease and a latched hover point
(`hold_ne`, and its height) that a drifting pose estimate cannot move. It
never re-locks on its own: the lost track and any abandoned re-acquire
designate are never adopted again, even if the engine reports them locked,
and a resumed search keeps them excluded; only a new operator designate (the
engine gives each one a fresh track id) is followed, and it is followed at
once, in the middle of a search too. The operator resumes the search (a
surface writes `operator_resume_seq` + 1; a fresh budget, around the current
prediction or the last estimate, and only newer detections are matched),
designates a subject, or stops. `predictive_search` is turned back on at
every arm (an arm seen after a disarm, not the first armed heartbeat after
the plugin starts).

An EKF position reset (a `LOCAL_POSITION_NED` step more than 1 m from where
the reported velocity carried the vehicle since the previous sample, with
samples at most 0.3 s apart, as on a switch between GNSS and external
navigation) moves the subject estimate, the search anchor, the vantage point
and a latched hover point with the frame, so they stay put in the world.

`outpaced` turns true once the subject has been measured faster than the
limit in force for 2 s: its displacement over the last 2 s of fixes (not the
subject filter's estimate, which caps a person at 5 m/s) above the limit, or
the range opening while the vehicle already flies at the limit. The limit,
published as `speed_limit_mps`, is `max_speed_mps`, or the pilot's 2 m/s
GNSS-denied cap when lower: the pilot's cap is not forwarded to plugins, so
it is rebuilt with the pilot's rule for unknown EKF source parameters:
uncapped while the active EKF source set is 1-3 and `GPS_RAW_INT` has a
fresh 3-D fix. The active set is the last one the navigation extension's
`plugin.navigation.state` named (`active_set`), and the boot set 1 before any; its
external-navigation mode alone does not cap. A source set whose
`EK3_SRCn_POSXY` is not GPS reads as uncapped here while the pilot caps it.

The subject template learns only from a solid lock: locked, confidence at
least 0.6, and no other person's box overlapping the subject's.

Re-acquire compares appearance embeddings. The shipped default is the
engine's model-free colour histogram (`embedding_model` `"histogram"`). A
learned OSNet-x0.25 re-id model (registry id `reid_osnet_x0_25_v1`) is
available as an explicit operator install and for benchmarks only, because its
weights were trained on a research-use dataset; it is never installed by
default. When it is configured as `vision.reid`, `embedding_model` is that
model id and similarity is markedly more discriminative. Embeddings from
different models are never compared.

The vision engine never re-locks silently. A smart re-acquire is an explicit,
announced designate. Without embeddings on the detection stream, Smart searches
and then holds for the operator like Strict.

## Surfaces

- **Flight Skill** (`follow-me`): an armed-only, confirm-to-arm toggle in the
  cockpit Skill Bar (default hotkey `⇧F`). Arming writes the per-drone
  `active` config. The agent reads it live and reports `follow.state`.
- **Target actions** (`contributes.target_actions`): click a detected subject
  in the host's cockpit overlay and pick **Follow this target** (hotkey `f`)
  or **Stop following** (hotkey `x`).
- **Native settings** (`contributes.parameters`): distance, height, gimbal
  point, camera, field of view, mount pitch, the re-acquire mode (a two-value
  enum that hosts render as a Strict | Smart toggle), the uncertain-lock
  mode, predictive search, the search time, radius and speed, the re-acquire
  similarity, the speed cap and the detector model. The field of view and
  mount pitch apply only where the camera roster does not give them.
- **Node-detail tab** (`node.detail.tab`, drone profile): Specs and a live
  read-back covering:
  - phase and lock;
  - range ± σ and the cues it came from;
  - height and its source;
  - subject speed;
  - re-acquire mode and, while recovering, the search time left, the search
    step, the vantage point and what hides the subject;
  - detection-to-goal latency and goal rate.

  While the follow holds for the operator the tab shows why and offers
  **Resume search** (writes `operator_resume_seq` + 1), **Return home**,
  **Land** and **Stop following** through `command.send`; the host asks the
  operator to confirm the vehicle commands.

## Events

| Topic               | Payload                                                         |
| ------------------- | --------------------------------------------------------------- |
| `follow.state`      | The read-back. `phase` is one of `idle`, `acquiring`, `following`, `holding`, `predicting`, `repositioning`, `searching`, `holding-await-operator`. The other fields are listed below. |
| `follow.reacquired` | `{track_id, similarity, gap_s, mahalanobis, phase}`; `phase` is the phase the re-lock happened in. |
| `follow.lost`       | `{track_id, reason, awaiting_operator, hold_ne}`, where `reason` is `lock_lost` (strict), `search_exhausted` or `predictive_search_off`. |

The other `follow.state` fields:

- **Range:** `range_m`, `range_sigma_m`, `range_cues`.
- **Subject:** `target_speed_mps`, `bearing_deg`, `depression_deg`,
  `target_ned`, `target_vel_ned`, `predicted_ned` (every 0.5 s over the
  next 5 s, over the agent's 3 s on its subject state), `predicted_bbox`,
  `target_bbox` (the locked box with its frame's capture times `ts_ms` and
  `ts_sensor_ms`), `subject_source` (the estimate in use: `vision`,
  `beacon` or `predicted` from the agent's subject state, null on the
  follow's own).
- **Frame:** `frame`, `fc_local_ned`: every position above is in the flight
  controller's local NED.
- **Height:** `agl_m`, `agl_source`, `height_raised`, `height_lowered`,
  `height_reason` (`canopy`: flown under leaves the pilot reports ahead;
  `canopy_floor`: held at 3.4 m, the leaves lower still).
- **Re-acquire:** `reacquire_mode`, `uncertain_mode`, `predictive_search`,
  `reacquire_remaining_s`, `search_budget_s`, `search_step`, `search_area`,
  `predicted_path_ne`, `vantage_ne`, `vantage_state`, `occluder`, `hold_ne`,
  `operator_resume_seq`.
- **Timing:** `pipeline_latency_ms`, `setpoint_rate_hz`.
- **Lock:** `track_id`, `confidence`, `lock_state`.
- **Commanding:** `commanding`, `hold_reason`.
- **Flight controller:** `fc_armed`, `fc_guided`.
- **Setpoints:** `distance_setpoint_m`, `height_setpoint_m`.
- **Mode:** `follow_mode`, `follow_angle_deg`, `orbit_radius_m`, `orbit_rate_dps`,
  `orbit_direction`, `orbit_phase_deg`.
- **Pilot:** `lease_id`, `nudged`.
- **Camera:** `camera_hfov_deg`, `camera_mount_pitch_deg` (negative below the
  nose), `camera_geometry_source` (`roster`, `settings` or `default`).
- **Speed:** `outpaced`, `speed_limit_mps`.

## Configuration

Per-drone configuration (see `config-schema.json`):

| Key                        | Default       | Range       | Meaning                                         |
| -------------------------- | ------------- | ----------- | ----------------------------------------------- |
| `active`                   | `false`       | —           | Armed/disarmed (toggled by the Skill).          |
| `follow_distance_m`        | `6`           | `3`–`30`    | Standoff distance behind the subject; under 5 m it is flown at 5 m without a measured range. |
| `follow_height_m`          | `4`           | `3.4`–`20`  | Follow height above the ground; a lower stored value is flown at 3.4 m. |
| `gimbal_point`             | `true`        | —           | Point a gimbal at the subject if present.       |
| `designate_camera`         | `auto`        | —           | Camera the follow loop consumes; `auto` reads the designated subject's camera (any camera until one is designated). |
| `camera_hfov_deg`          | unset         | `30`–`160`  | Camera horizontal field of view, where the camera roster does not give it; unset is 87°. |
| `mount_pitch_deg`          | unset         | `0`–`90`    | Camera downward tilt below the horizon, where the camera roster does not give it; unset is 15°. |
| `reacquire_mode`           | `smart`       | `strict`, `smart` | What a lost lock does.                    |
| `uncertain_mode`           | `follow`      | `follow`, `hold` | What an uncertain lock does.               |
| `predictive_search`        | `true`        | —           | Search from a vantage point; on again at every arm. |
| `search_budget_s`          | `25`          | `5`–`60`    | How long a smart recovery lasts.                |
| `search_radius_m`          | `12`          | `3`–`12`    | Farthest vantage point from the loss.           |
| `search_speed_mps`         | `2`           | `0.5`–`4`   | Speed limit while repositioning.                |
| `operator_resume_seq`      | `0`           | `≥ 0`       | Written as current + 1 to resume a search.      |
| `reacquire_min_similarity` | `0.75`        | `0.5`–`0.99`| Appearance similarity needed to re-acquire.     |
| `max_speed_mps`            | `6`           | `0.5`–`10`  | The follow's speed limit.                       |
| `follow_mode`              | `follow`      | follow, orbit | Follow at an angle, or orbit the subject.     |
| `follow_angle_deg`         | `180`         | `0`–`355`   | Angle from the travel heading (follow mode).    |
| `orbit_radius_m`           | `8`           | `3`–`30`    | Orbit radius; under 5 m it is flown at 5 m without a measured range. |
| `orbit_rate_dps`           | `10`          | `2`–`30`    | Orbit angular rate.                             |
| `orbit_direction`          | `cw`          | cw, ccw     | Orbit direction seen from above.                |
| `obstacle_margin_m`        | `1.5`         | `0.8`–`5`   | Clearance the pilot's planner keeps.            |
| `detector`                 | `coco-person` | —           | Detection model the follow loop consumes.       |

## Requirements

- ADOS agent 0.118.0 or later: the pilot flies the vehicle; the plugin only
  sends goals under the pilot's motion lease (capability `flight.motion`).
  A multirotor, or a VTOL in hover.
- A camera bound to the vision pipeline producing the detection stream, with
  a person detector at 6 Hz or more. While the behaviour is active the plugin
  holds a detection lease on that camera (renewed every 2 s, 6 s hold,
  released when it goes inactive), so the engine runs detection there only
  while follow-me needs it.
- Recommended: a downward rangefinder on the flight controller for
  terrain-following height.
- Optional: the Navigation extension's planar scan, for the lidar range cue.

## Build, test, pack

```bash
pnpm build:follow-me && pnpm test:follow-me    # GCS half (repo root)
uv run pytest extensions/follow-me/agent/tests  # agent half (repo root)
./scripts/pack.sh follow-me
```

## License

GPL-3.0-or-later.
