# Changelog

All notable changes to ADOS Follow-Me are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/) and the project
uses independent semantic versioning per extension.

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
