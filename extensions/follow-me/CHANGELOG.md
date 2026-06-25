# Changelog

All notable changes to ADOS Follow-Me are documented here. The format
follows [Keep a Changelog](https://keepachangelog.com/) and the project
uses independent semantic versioning per extension.

## [0.1.0]

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
