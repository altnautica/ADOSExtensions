# ADOSExtensions

First-party plugins for the ADOS Drone Agent and ADOS Mission Control,
shipped as signed `.adosplug` archives.

## Repo layout

```
extensions/
  battery-health-panel/            GCS-only panel: cell-level diagnostics, predictive time-to-min, anomaly alerts
  thermal-camera-flir-lepton-usb/  Hybrid: FLIR Lepton 3.5 USB UVC capture + GCS overlay + FC tab
  mavlink-gimbal-v2/               Hybrid: SimpleBGC / Storm32 NT / Gremsy driver + GCS controls + ROI lock
  vision-nav/                      Hybrid: optical flow + monocular VIO (OpenVINS / VINS-Fusion) for GPS-denied flight
scripts/
  pack.sh             Build + manifest hash + zip to .adosplug
  sign.sh             Ed25519-sign the archive against the publisher key
.github/workflows/
  release.yml         Build, sign, and release on tag push
```

## Workspaces

- `pnpm-workspace.yaml` declares the GCS halves (TypeScript / React) and root
  tooling. Run `pnpm install` at the repo root.
- `pyproject.toml` declares the agent halves and shared Python utilities for
  the hybrid extensions.

Each extension carries its own `CHANGELOG.md` and is versioned independently.
The vision-nav extension publishes through its own `extensions/vision-nav-v*`
tags on GitHub Releases.

## Building one extension

```sh
cd extensions/battery-health-panel
pnpm install
pnpm build
../../scripts/pack.sh battery-health-panel
```

This produces `dist/com.altnautica.battery-health-panel-<version>.adosplug`.
The archive layout matches the public extension spec at
[docs.altnautica.com/developers/manifest](https://docs.altnautica.com/developers/manifest).

## Signing

Tagged releases run `scripts/sign.sh` in CI against the `altnautica-2026-A`
publisher key. The signed archive is published as a GitHub Release asset and,
for the four first-party extensions, also surfaces inside the Mission Control
Plugins tab via the hosted registry.

## Installing on a drone

From Mission Control: open a drone, switch to the **Plugins** tab, browse the
registry, pick the extension, approve the two-stage install dialog. The agent
downloads the signed archive, verifies the Ed25519 signature, and stages the
plugin under its supervisor.

From the agent CLI:

```bash
ados plugin install https://github.com/altnautica/ADOSExtensions/releases/download/<release>/<extension>.adosplug
```

## Contributing

First-party only at launch. Community contributions land via the hosted
registry submission flow when the registry hits v1.0. See the
[hosted registry developer doc](https://docs.altnautica.com/developers/distribution-registry)
for the policy.
