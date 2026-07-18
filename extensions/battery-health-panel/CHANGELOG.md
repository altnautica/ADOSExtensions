# Changelog

All notable changes to the Battery Health Panel.

## 1.2.0

- Enriched manifest: a long description, a feature list, and the `battery`
  telemetry field, so the extension detail page and the registry surface the
  panel's real capabilities instead of a one-line summary.
- Dropped the unused `ui.slot.settings-section` capability. The panel
  contributes a node-detail tab and notifications only; it never rendered a
  settings section.

## 1.1.0

- Manifest on the current contribution platform (schema_version 2): the panel
  moves from the fleet-wide FC-config slot to a per-node detail tab
  (`node.detail.tab`) so it appears on each drone's detail panel, and the
  compatibility floors are raised to the current platform. No functional change
  to the diagnostics.

## 1.0.2

Re-sign release with the unified first-party private key. No functional changes.

## 1.0.1

Re-sign release with the first-party publisher id (label only — key bytes still mismatched). No functional changes.

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
