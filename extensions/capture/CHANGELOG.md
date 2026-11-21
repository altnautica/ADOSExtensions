# Changelog

All notable changes to the Capture extension.

## [Unreleased]

### Changed

- Every session segment's MCAP header names the recorder as well as the
  contract writer: `com.altnautica.capture/<version> ados-capture-contract/<version>`,
  both taken from the build. A test keeps the Capture crate version equal to
  the extension's manifest version.

### Fixed

- The flight controller's poses and samples are placed on the host clock by a drift bound that follows the plugin's ADOS time: up to 0.95 of host time on a node following a simulator's clock, 0.001 on wall time (a hardware FC drifts by parts per million), so a growing MAVLink delivery backlog is no longer read as clock drift.

## [1.0.0] - 2026-11-20

Capture: session recording, keyframe selection and session upload, as its
own extension (`com.altnautica.capture`).

### Added

- A session is posed by the Navigation extension's pose
  (`plugin.navigation.pose_track`) or the flight controller's EKF, chosen at
  session start by `capture.pose_source` (`auto`, `fc_ekf`, `navigation`).
- A recording session records the LiDAR scans Navigation measures
  (`plugin.navigation.scan_raw`), reassembled from their parts in any order;
  a scan missing a part after 500 ms is counted in the health record as
  `lidar_scans_incomplete`.
- `plugin.capture.state`: the capture status plus the session's upload
  progress, published on change and every second for every plugin holding
  `telemetry.read`.
- The cockpit Capture panel (state, duration, frames, quality chips and
  whether the session is enough to reconstruct) and the map Coverage layer.
- `PUT /capture/config` refuses a key another extension owns, naming it.

### Changed

- Needs ADOS Drone Agent 0.122.0 or later (`compatibility.ados_version`):
  it relies on that release's topic and route layout.

- Routes are `/capture/*`, the configuration keys `capture.*` (the relay's
  `capture.relay.*`), the node pin `capture.compute_node_addr`, the status
  telemetry `{"capture": …}`, and the private bus `capture.sock` /
  `capture-control.sock` with topics `capture.keyframe` and `capture.state`.
- The drone's capture surface is one Capture surface with Live, Sessions
  and Model views; the setup page is `capture-setup`, the ground-station page
  `relay`.
- The uploader reads where map products are built from Navigation's map
  status instead of a configuration key.

### Removed

- Reconstruction, perception offload, the live map and the reconstruction
  tools moved to the Reconstruct extension; the navigation pose, LiDAR
  drivers and live occupancy to the Navigation extension.
