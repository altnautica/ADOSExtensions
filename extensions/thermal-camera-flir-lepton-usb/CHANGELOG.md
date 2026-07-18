# Changelog

All notable changes to the Thermal Camera FLIR Lepton USB UVC extension.

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
