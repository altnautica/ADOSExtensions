# Changelog

All notable changes to the Thermal Camera FLIR Lepton USB UVC extension.

## 1.4.2

- A live thermal feed no longer reads "frames stopped" when the drone clock
  runs behind or a poll is slow. The drone's timestamp now judges only the
  first reading after the overlay opens or the feed resumes, so an old
  reading still shows as stopped at once, and a feed that stops still reads
  stopped 2 s after its last frame.

## 1.4.1

- The camera comes back on its own after its frames stop. When the stream
  fails, the agent tries to reopen it after 2 s and then every 5 s until
  the camera answers. The camera then reads connected again, the spot
  temperature returns, and palette, gain and FFC work again. Before, the
  camera stayed unavailable until the plugin was restarted.
- The overlay now ages the spot reading from when the drone recorded it,
  not from when it reached the ground station. An old reading shown when
  the overlay opens reads "Thermal frames stopped" right away instead of
  briefly looking live.
- The palette shown on the overlay changes only once the camera accepts
  it. A palette the camera rejected is no longer shown as applied, and the
  Cycle palette Skill no longer writes a rejected palette back to settings.
  The same now holds for the gain setting.
- The plugin reports its own version correctly.

## 1.4.0

- Frame read-backs now carry `timestampMs`. When the spot reading is 2 s
  old or more, the overlay stops showing the temperature and reads
  "Thermal frames stopped" with how long ago the last reading arrived.
- When the camera stops delivering frames for about a second, the agent
  closes the session and reports the camera as unavailable ("stopped
  delivering frames") instead of leaving it connected.
- The overlay's styles are now built into the plugin and applied when it
  loads, so the readout and spot marker sit correctly over the video.
- Removed the palette dropdown from the overlay: it changed nothing on the
  drone. Change the palette with the Palette setting or the Cycle palette
  Skill; the overlay now shows the palette the agent applied.
- All overlay text comes from the locale file, and a missing sensor has one
  message everywhere.

## 1.3.3

- `supported_boards` now lists HAL board profile names or SoC ids (`cm4`,
  `cm5`, `rock-5c-lite`, `rk3576`, `rk3566`, `generic-x86_64`), so the
  install gate matches probed boards.
- The documentation link points at the extensions catalog entry.

## 1.3.2

- Default Skill keys no longer collide with cockpit bindings: Cycle palette
  `shift+o` (was `p`, the picture-in-picture toggle) and FFC `shift+i` (was
  `shift+f`, the Follow-Me Skill key).

## 1.3.1

- Dropped the `mavlink.component.camera` permission and the component-100
  reservation block: the plugin never registers a MAVLink component or sends
  MAVLink, so the install dialog listed a role it does not take.

## 1.3.0

- The extension now requires a real UVC backend. `ThermalUsbPlugin` no longer
  defaults `backend_factory` to a synthetic backend, so it can no longer open
  a device that is not there. The synthetic backend has moved out of the
  package into the unit suite (`agent/tests/mock_backend.py`) and is not
  importable from `altnautica_thermal_camera`; `uvc_backend` now ships only
  the `LibUvcBackend` Protocol and its data classes.
- `on_start` fails closed on every path that leaves it without an open
  radiometric session — no injected backend, no device on the bus, a failed
  open, or a frame stream that will not start. In each case it logs an error,
  publishes `{"connected": false, "reason": ...}` on the `thermal` state
  channel, and starts neither the control loop nor the frame stream, so no
  spot temperature or frame extrema is published without a device behind it.
  Previously the default synthetic backend meant a bench install published
  invented temperatures on `camera.thermal.frame` as live telemetry and
  reported `connected: true`.
- The video overlay renders the unavailable state and names its reason
  instead of showing "Awaiting thermal frames..." forever, and it shows an
  explicit unknown state before the agent has reported anything. It now
  subscribes to the `thermal` state channel as well as
  `camera.thermal.frame`, so `telemetry.subscribe.thermal` is declared (the
  host checks one capability per subscribed topic).
- Both cockpit Skills now publish the state topics they declare
  (`camera.thermal.palette`, `camera.thermal.ffc`), reporting `disabled` with
  the reason while there is no session. Nothing published them before, so the
  Skill Bar had no state to read.
- Manifest reconciled against the code. Removed the `thermal-config` tab
  contribution and `ui.slot.node-detail-tab`: the bundle builds only the
  video overlay, so a "Thermal Camera" tab opened a floating HUD. Removed the
  `thermal-alarm` notification contribution and `ui.slot.notification-channel`
  (nothing publishes a notification), the GCS `mission.read`, `mission.write`
  and `recording.write` grants (no such call site in the bundle), and the
  agent `event.subscribe` and `recording.write` grants (no such call site in
  the agent half).
- `description`, `description_long` and `features` no longer claim a working
  live capture path. They describe the driver, palettes, temperature
  conversion and spot metering that are implemented, and state that capture
  needs a UVC backend the agent supplies.

## 1.2.0

- The plugin now opens and drives the Lepton directly. It replaces the old
  camera-driver registration (which was never awaited against the async host
  and so never registered) with its own device open plus a control loop that
  applies the palette, high/low gain, and flat field correction from config,
  and it advertises the thermal stream leg to the video pipeline through
  `ctx.video.set_source` when a colorized stream endpoint is configured. The
  `sensor.camera.register` capability is replaced by `video.source.set`.
- Added two cockpit Skills: Cycle palette and Flat field correction.
- Added palette and gain (high/low) settings, read live each control tick.
- Publishes a `thermal` read-back (connected, palette, gain) on change.
- Enriched the manifest with a long description, feature list, hardware
  requirements, resource estimate, telemetry field, and a documentation link.
- Note: exposing the thermal feed as a live video stream leg still needs a
  colorized RTSP/MJPEG endpoint and real hardware; until an endpoint is
  configured no stream leg is advertised (no phantom stream).

## 1.1.0

- Manifest on the current contribution platform (schema_version 2): the config
  tab moves from the fleet-wide FC-config slot to a per-node detail tab
  (`node.detail.tab`), and the agent half is marked per-drone so each drone
  keeps its own thermal settings. The thermal video overlay is unchanged.
- Fixed the telemetry subscription permission: the overlay subscribes to the
  `camera.thermal.frame` topic, but the manifest granted `telemetry.subscribe.thermal`,
  which the host would deny (the per-topic capability must match). The grant now
  matches the subscribed topic, and the unused mavlink telemetry grant is dropped.
- Documented the future path to publishing the colorized thermal feed as its
  own cockpit video stream (hardware-gated; see the README roadmap).

## 1.0.2

Re-sign release with the unified first-party private key. No functional changes.

## 1.0.1

Re-sign release with the first-party publisher id (label only — key bytes still mismatched). No functional changes.

## 1.0.0

Initial release.

- `LeptonUvcDriver` subclass of `CameraDriver` with discover, open,
  close, capabilities, frame iterator, and parameter setters.
- TLinear Y16-to-kelvin and kelvin-to-celsius conversion plus the
  reverse.
- Three RGB palette LUTs: ironbow, rainbow, grayscale.
- `LibUvcBackend` Protocol and `MockUvcBackend` synthetic-frame
  fixture for tests; native binding deferred until hardware lands.
- Plugin entry point that registers the driver with the peripheral
  manager.
- GCS half: canvas-based thermal overlay, spot-meter helper,
  palette LUTs ported to TypeScript, plugin entry that subscribes
  to `camera.thermal.frame`.
- English locale.
- JSON Schema for configuration form.
- Iframe-sandbox isolation per the GCS plugin host contract.
