# Changelog

All notable changes to ADOS Follow-Me are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/) and the project
uses independent semantic versioning per extension.

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
