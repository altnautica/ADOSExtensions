# Vision-Nav Vendor Binaries

The two VIO modes (`vio_openvins`, `vio_vins_fusion`) spawn a C++
binary that runs the upstream estimator inside the plugin host's
subprocess sandbox. The Rust agent's VIO bridge (`agent/src/vio.rs`)
feeds camera frames over a shared-memory ring and exchanges IMU
samples + pose messages over a length-prefixed msgpack channel on a
Unix domain socket.

Two binaries live here, one per estimator:

| Binary | Source repo | Estimator |
|---|---|---|
| `ados_openvins_shim` | https://github.com/rpng/open_vins (v2.7.1) | OpenVINS MSCKF |
| `ados_vins_fusion_shim` | https://github.com/HKUST-Aerial-Robotics/VINS-Fusion | VINS-Fusion bundle adjustment |

Both binaries:

* Open `/run/ados/vision-nav/<mode>.sock` as a connect-side UDS.
* Open the SHM region at `/dev/shm/ados-vision-nav-frames` for camera
  frame I/O.
* Speak the length-prefixed msgpack wire format the Rust agent's VIO
  bridge implements (`agent/src/vio.rs`).

The binaries are not committed to git. CI builds them on tag push and
attaches the signed tarballs to the GitHub release; the install path
unpacks them into `<install_dir>/vendor/` at plugin install time.
The plugin manifest's `subprocess_spawn` allowlist references the
binary names; the plugin host's sandbox rejects any spawn whose
basename is not in the allowlist.

## Build matrix

| Triple | Target | Status |
|---|---|---|
| `aarch64-unknown-linux-gnu` | Debian-based aarch64 (Pi 4B, Pi 5) | scaffolded |
| `aarch64-rockchip-linux-gnu` | Radxa BSP (RK3582, RK3588S2) | scaffolded |
| `x86_64-unknown-linux-gnu` | Desktop tests | scaffolded |

Each triple has its own sysroot under `vendor/<estimator>/sysroots/`
(not committed). The CI workflow at
`.github/workflows/vision-nav-vendor-binaries.yml` uses `cross` plus
the matching sysroot to produce the binary, runs the conformance test
suite, and signs the artefact with the existing `ADOS_SIGNING_KEY`
secret.

## Signing

The vendor binaries are wrapped into the `.adosplug` archive at
release time and signed alongside the rest of the plugin payload.
The signing key is the same Ed25519 key the plugin host already
verifies; no separate binary-specific key is needed.

## Conformance

Each binary runs a self-test on `--conformance` that feeds a recorded
EuRoC bag and asserts the pose output matches the upstream reference
within tolerance. The conformance test runs in CI before the artefact
is signed.

## Licensing

Both upstream projects ship under GPL-3.0. The vendor directory
preserves the upstream LICENSE files verbatim. The plugin manifest's
`vendor_attribution` block surfaces the attribution at install time
so operators see the licence terms before the binary executes.

## On-rig build

The cross-compile pipeline below is the CI path. To build the
binaries on a real Pi 4B or Rock 5C Lite for local testing:

```bash
cd vendor/openvins
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DOPENVINS_VERSION=v2.7.1
cmake --build build -j$(nproc)
sudo cp build/ados_openvins_shim /opt/ados/plugins/com.altnautica.vision-nav/vendor/
```

Replace `openvins` with `vins-fusion` for the second binary. Plugin
restart picks up the new binary on the next mode switch.
