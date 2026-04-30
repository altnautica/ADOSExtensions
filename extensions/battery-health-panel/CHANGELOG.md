# Changelog

All notable changes to the Battery Health Panel.

## 1.0.0

Initial release.

- Live cell-tile grid keyed off the host's normalized battery telemetry stream.
- Predictive time-to-reserve readout with configurable window and target.
- Six-rule anomaly engine: low cell, critical cell, cell divergence, voltage drop, temperature spike, predictive low.
- 5-second hysteresis on live anomalies.
- Notification emission to the host's anomaly channel.
- Recording markers on anomaly when a recording is active.
- JSON-Schema-driven configuration form under Settings -> Plugins.
- English locale.
- Iframe-sandbox isolation per the GCS plugin host contract.
