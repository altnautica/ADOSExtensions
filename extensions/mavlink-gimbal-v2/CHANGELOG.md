# Changelog

All notable changes to the MAVLink Gimbal v2 Controller extension.

## 1.3.0

- Added four cockpit Skills: Aim (toggle the visual servo), Recenter, Nadir
  (point straight down to the driver's lower pitch limit), and Rate mode
  (toggle between angle and rate control). Recenter and Nadir are one-shot
  commands the agent fires on the rising edge and then clears; Aim and Rate
  mode publish a state read-back so the Skill Bar reflects the true state.
- Added a rate-control mode: when on, the aim servo commands the gimbal by
  angular rate instead of an absolute angle, sharing one error model with the
  position servo.
- Replaced the hand-typed `designate_camera` string with a camera picker: the
  operator pins the camera the aim servo follows or leaves it on auto (the
  first detect camera).
- Exposed three tools (status, point-at, recenter) for an assistant, each
  going through the same clamped driver paths the Skills use.
- Enriched the manifest with a long description, feature list, hardware
  requirements, resource estimate, telemetry field, and a documentation link.

## 1.2.0

- Manifest on the current contribution platform (schema_version 2): the control
  tab moves from the fleet-wide FC-config slot to a per-node detail tab
  (`node.detail.tab`), and the agent half is marked per-drone so each drone
  keeps its own gimbal settings. Resolves the earlier schema_version 1 manifest
  that already declared a schema-2 `target_actions` block. The reticle overlay
  and the aim-at-target action are unchanged.

## 1.1.1

Adds an "Aim at this target" `target.action`: a visual-servo controller
points the gimbal at the operator-designated subject, riding the shared
locked-target safety gate (stop + hold on an uncertain/lost lock). The aim
sign/gain (`invert_pitch`/`invert_yaw`/`gain`) are conservative defaults to be
confirmed against real gimbal hardware before wider use.

## 1.0.2

Re-sign release with the unified first-party private key. No functional changes.

## 1.0.1

Re-sign release with the first-party publisher id (label only — key bytes still mismatched). No functional changes.

## 1.0.0

Initial release.

- `MavlinkGimbalDriver` subclass of the agent SDK `GimbalDriver` ABC. Sends `MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW`, `MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE`, `MAV_CMD_DO_SET_ROI_LOCATION`, and `MAV_CMD_DO_SET_ROI_NONE` through the agent MAVLink router.
- Stub `GimbalDriver` subclasses for SimpleBGC, Storm32 NT, and Gremsy serial paths. Concrete classes that raise `NotImplementedError` from `open()` so vendors can fork the pattern.
- Pure-Python MAVLink command encoder helpers for the four gimbal commands. No external MAVLink dialect import is required.
- Plugin entry point that wires the driver to the agent supervisor.
- GCS panel with pitch, yaw, and roll sliders plus a "point at lat/lon/alt" form and a live state readout.
- ROI release button that emits `MAV_CMD_DO_SET_ROI_NONE`.
- English locale.
- JSON-Schema-driven configuration form under Settings -> Plugins.
- Iframe-sandbox isolation per the GCS plugin host contract.
- Subprocess isolation per the agent plugin host contract.
