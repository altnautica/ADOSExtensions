# Changelog

All notable changes to ADOS Follow-Me are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/) and the project
uses independent semantic versioning per extension.

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
