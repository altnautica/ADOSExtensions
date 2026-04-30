# ADOSExtensions

First-party plugins for the ADOS Drone Agent and ADOS Mission Control,
shipped as signed `.adosplug` archives.

## Repo layout

```
extensions/
  battery-health-panel/   GCS-only panel: cell-level diagnostics, predictive time-to-min, anomaly alerts
  thermal-camera-flir-lepton/   (planned) Hybrid: USB UVC capture + GCS overlay
  mavlink-gimbal-v2/            (planned) Hybrid: gimbal driver + GCS controls
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
  the future hybrid extensions.

Each extension carries its own `CHANGELOG.md` and is versioned independently.

## Building one extension

```sh
cd extensions/battery-health-panel
pnpm install
pnpm build
../../scripts/pack.sh battery-health-panel
```

This produces `dist/com.altnautica.battery-health-panel-<version>.adosplug`.
The archive layout matches `product/specs/ados-extensions/02-extension-format.md`
in the upstream Altnautica monorepo.

## Signing

Tagged releases run `scripts/sign.sh` in CI against the `altnautica-2026-A`
publisher key. The signed archive is published as a GitHub Release asset.

## Contributing

First-party only at launch. Community contributions land via the registry
submission flow when the registry hits v1.0. See
`product/specs/ados-extensions/04-governance.md` upstream for review policy.
