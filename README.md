# ADOS Extensions

**First-party plugins for the ADOS Drone Agent and ADOS Mission Control.**

![License: GPL-3.0](https://img.shields.io/badge/License-GPL--3.0--or--later-green.svg) [![Discord](https://img.shields.io/badge/Discord-Join-5865F2.svg)](https://discord.gg/uxbvuD4d5q)

An extension adds a capability to ADOS without forking it: a new panel in Mission Control, a driver for a sensor or gimbal on the drone, or a behavior that spans both. Each one ships as a single signed `.adosplug` archive that an operator installs from the Plugins tab or the agent CLI, with the permissions it needs shown up front.

> **Part of the ADOS ecosystem.** Extensions build on the plugin system in [ADOS Drone Agent](https://github.com/altnautica/ADOSDroneAgent) (Python and Rust, supervised subprocess per plugin) and [ADOS Mission Control](https://github.com/altnautica/ADOSMissionControl) (React panels in a sandboxed iframe). One manifest describes both halves.

<p align="center">
  <strong><a href="https://github.com/altnautica/ADOSDroneAgent">ADOS Drone Agent</a></strong> |
  <strong><a href="https://github.com/altnautica/ADOSMissionControl">ADOS Mission Control</a></strong> |
  <strong><a href="https://docs.altnautica.com/developers">Developer Docs</a></strong> |
  <strong><a href="https://discord.gg/uxbvuD4d5q">Discord</a></strong>
</p>

---

## Two ways in

**Run an extension (operator).** From Mission Control, open a drone, switch to the **Plugins** tab, pick an extension, and approve the install. The agent downloads the signed archive, verifies the signature, and stages the plugin under its supervisor. No SSH, no rebuild.

**Build an extension (developer).** Scaffold a plugin with `create-ados-plugin`, write the GCS half in TypeScript and the agent half in Python or Rust against the SDKs, then pack and sign it into a `.adosplug`. The same plugin system powers both first-party and community extensions.

---

## Extensions

| Extension | Kind | What it adds |
|-----------|------|--------------|
| **battery-health-panel** | GCS panel | Cell-level battery diagnostics, predicted time to minimum voltage, and anomaly alerts |
| **thermal-camera-flir-lepton-usb** | Agent + GCS | FLIR Lepton 3.5 thermal capture over USB UVC, with a live GCS overlay and a flight-controller tab |
| **mavlink-gimbal-v2** | Agent + GCS | MAVLink Gimbal v2 mount control with SimpleBGC, Storm32 NT, and Gremsy drivers, plus region-of-interest lock |
| **vision-nav** | Agent + GCS | GPS-denied navigation from optical flow and monocular visual-inertial odometry (OpenVINS, VINS-Fusion) |

---

## Toolkit

Packages an extension author builds against:

| Package | Use |
|---------|-----|
| `create-ados-plugin` | Scaffolder. `npx create-ados-plugin` with GCS-only, agent-only, or hybrid templates |
| `plugin-sdk` | TypeScript SDK for the GCS half: telemetry hooks, postMessage RPC to the agent, and a test harness |
| `extension-ui` | Reusable React UI primitives that match the Mission Control theme |

---

## Repo layout

```
extensions/                        first-party extensions, versioned independently
  battery-health-panel/
  thermal-camera-flir-lepton-usb/
  mavlink-gimbal-v2/
  vision-nav/
packages/
  create-ados-plugin/              scaffolder + templates
  plugin-sdk/                      TypeScript GCS SDK
  extension-ui/                    shared React UI primitives
scripts/
  pack.sh                          build + manifest hash + zip to .adosplug
  sign.sh                          Ed25519-sign the archive against the publisher key
.github/workflows/
  release.yml                      build, sign, and release on tag push
```

`pnpm-workspace.yaml` declares the GCS halves and root tooling (run `pnpm install` at the repo root). `pyproject.toml` declares the agent halves and shared Python utilities. Each extension carries its own `CHANGELOG.md` and is versioned independently through its own release tags.

## Building one extension

```sh
cd extensions/battery-health-panel
pnpm install
pnpm build
../../scripts/pack.sh battery-health-panel
```

This produces `dist/com.altnautica.battery-health-panel-<version>.adosplug`. The archive layout matches the public extension spec at [docs.altnautica.com/developers/manifest](https://docs.altnautica.com/developers/manifest).

## Signing

Tagged releases run `scripts/sign.sh` in CI against the publisher key. The signed archive is published as a GitHub Release asset and, for first-party extensions, surfaces inside the Mission Control Plugins tab through the hosted registry.

## Installing on a drone

From Mission Control: open a drone, switch to the **Plugins** tab, browse the registry, pick the extension, and approve the two-stage install dialog. The agent downloads the signed archive, verifies the Ed25519 signature, and stages the plugin under its supervisor.

From the agent CLI:

```bash
ados plugin install https://github.com/altnautica/ADOSExtensions/releases/download/<release>/<extension>.adosplug
```

## Contributing

First-party only at launch. Community contributions land through the hosted registry submission flow when the registry reaches v1.0. See the [hosted registry developer doc](https://docs.altnautica.com/developers/distribution-registry) for the policy, and the [developer docs](https://docs.altnautica.com/developers) for the manifest, SDKs, and permission model.

## License

[GPL-3.0-or-later](LICENSE). Each extension is free to use, modify, and distribute under the same terms.
