# Changelog

All notable changes to the Thermal Camera FLIR Lepton USB UVC extension.

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
