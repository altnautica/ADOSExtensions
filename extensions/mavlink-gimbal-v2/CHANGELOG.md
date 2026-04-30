# Changelog

All notable changes to the MAVLink Gimbal v2 Controller extension.

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
