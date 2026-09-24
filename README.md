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
| **thermal-camera-flir-lepton-usb** | Agent + GCS | Development preview, not published. FLIR Lepton 3.5 radiometric driver, palettes and spot metering with a GCS overlay. The capture path needs a libuvc backend that does not exist in this repository, so it reports an explicit unavailable state instead of producing readings |
| **mavlink-gimbal-v2** | Agent + GCS | MAVLink Gimbal v2 manager control, region-of-interest lock, and an aim-at-target visual servo with a lock-state safety gate |
| **vision-nav** | Agent + GCS | GPS-denied navigation from downward optical flow, with a calibration wizard and a pre-arm gate |
| **follow-me** | Agent + GCS | Operator-designated subject follow at a fixed standoff via guided setpoints, with a lock-state safety gate |
| **siyi-pod** | Agent + GCS | Native SIYI optical-pod driver with per-model capability negotiation: gimbal, zoom, thermal, and a laser rangefinder with subject geolocation |

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
  follow-me/
  mavlink-gimbal-v2/
  siyi-pod/
  thermal-camera-flir-lepton-usb/
  vision-nav/
packages/
  create-ados-plugin/              scaffolder + templates
  plugin-sdk/                      TypeScript GCS SDK
  extension-ui/                    shared React UI primitives
scripts/
  pack.sh                          build + manifest hash + zip to .adosplug
  pack-rust.sh                     same, for an extension whose agent half is a crate
  build-rust.sh                    cross-compile a rust agent half
  lint-manifest.mjs                check a manifest against the code it describes
  sign.sh                          Ed25519-sign the archive against the publisher key
.github/workflows/
  release.yml                      build, sign, and release on tag push
```

`pnpm-workspace.yaml` declares the GCS halves and root tooling (run `pnpm install` at the repo root). `pyproject.toml` declares the agent halves and shared Python utilities. Each extension carries its own `CHANGELOG.md` and is versioned independently through its own release tags.

## Building one extension

```sh
cd extensions/mavlink-gimbal-v2
pnpm install
pnpm build
../../scripts/pack.sh mavlink-gimbal-v2
```

This produces `dist/com.altnautica.mavlink-gimbal-v2-<version>.adosplug`. The archive layout matches the public extension spec at [docs.altnautica.com/developers/manifest](https://docs.altnautica.com/developers/manifest).

`pack.sh` refuses to write a half-archive: if the manifest declares a GCS entrypoint the built bundle has to be in the archive, and if it declares a Python `agent.entrypoint` the module that entrypoint names has to be there too. An archive with no agent source installs cleanly and verifies, then the supervisor dies importing a module that was never packed.

Check a manifest against the code it describes before you pack:

```sh
node scripts/lint-manifest.mjs extensions/mavlink-gimbal-v2/manifest.yaml
```

It fails on a declared permission with no call site, a declared UI slot with no implementation, and a version that disagrees across `manifest.yaml`, the two `package.json` files and the `definePlugin({ version })` literal the host registers.

## Building a Rust extension

An extension whose agent half is a crate (`vision-nav`) builds against the agent SDK in a sibling checkout, so the two repositories have to sit next to each other:

```
<root>/ADOSDroneAgent
<root>/ADOSExtensions
```

`Cargo.toml` uses a path dependency on that sibling for local development. CI clones the agent repository and checks out the revision in the repo-root `ADOS_AGENT_REV` file, so a release tag resolves the SDK at a fixed commit rather than whatever `main` happened to be. Update `ADOS_AGENT_REV` when a release needs a newer SDK.

Cross-compiling for a drone needs the musl target and an aarch64 musl linker. Apple's `ld` is not one, so a plain macOS build compiles everything and then fails at the link step; `scripts/build-rust.sh` documents the linker env vars, including a `zig cc` fallback where no musl toolchain is installed.

```sh
rustup target add aarch64-unknown-linux-musl
./scripts/pack-rust.sh vision-nav
```

`pack-rust.sh` asserts both halves are in the archive before it zips: the compiled binary at the manifest's `agent.entrypoint` and, when one is declared, the built GCS bundle.

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
