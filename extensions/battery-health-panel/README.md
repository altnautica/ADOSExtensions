# Battery Health Panel

Cell-level battery diagnostics for Mission Control. Subscribes to the
host's normalized battery telemetry, runs a small rule engine against
each new sample, and renders cell tiles, predictive time-to-reserve,
and a live anomaly list.

## Build

```sh
pnpm install
pnpm --filter ./extensions/battery-health-panel/gcs build
pnpm --filter ./extensions/battery-health-panel/gcs test
```

The build script produces `gcs/plugin.bundle.js`. Run `scripts/pack.sh
battery-health-panel` from the repo root to produce a signed-eligible
`.adosplug` archive.

## Permissions

| Permission | Use |
|------------|-----|
| `ui.slot.fc-tab` | Mount the panel under the Flight Control tab. |
| `ui.slot.notification` | Emit anomaly notifications. |
| `ui.slot.settings-section` | Render the configuration form under Settings -> Plugins. |
| `telemetry.subscribe.battery` | Read normalized battery samples. |
| `telemetry.subscribe.mavlink` | Pick up FC-emitted battery alarm strings from STATUSTEXT. |
| `recording.write` | Add markers to active recordings on anomaly. |

Risk band: low. No vehicle command, no host file system, no network.

## Anomaly rules

| Rule | Condition |
|------|-----------|
| `cell_low` | Min cell voltage < `lowCellVoltageV` (default 3.5 V) and >= `criticalCellVoltageV`. |
| `cell_critical` | Min cell voltage < `criticalCellVoltageV` (default 3.3 V). |
| `cell_divergence` | Spread between max and min cell exceeds `cellDivergenceMv` (default 50 mV). |
| `voltage_drop` | Total voltage falls faster than `voltageDropRateVPerSec` (default 0.5 V/s). |
| `temp_spike` | Pack temperature rises faster than `tempSpikeRateCPerSec` (default 5 °C/s). |
| `predictive_low` | Projected time to reserve falls under 60 s while discharging. |

Hysteresis is 5 s: an anomaly stays in the live list until the underlying
condition has been clear for at least 5 seconds.

## Predictive math

Discharge rate is averaged over a sliding window (default 30 s). Time
to reserve is `(remaining_percent - target_percent) / rate`. Within
~10% of ground truth on the linear region of a LiPo discharge curve.
A Coulomb-counted nonlinear model is reserved for v1.1.

## Configuration

Edit thresholds and the predictive window under
Settings -> Plugins -> Battery Health -> Configuration. Schema lives at
`config-schema.json` and is rendered automatically by the host's JSON
Schema form.
