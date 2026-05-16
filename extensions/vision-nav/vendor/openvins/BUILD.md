# Building `ados_openvins_shim`

The binary wraps upstream OpenVINS (`rpng/open_vins`) and exposes the
shim IPC contract documented in `../../agent/src/altnautica_vision_nav/shim/ipc.py`.

## Sysroot dependencies

Before building, the sysroot needs:

* Eigen 3.3+
* OpenCV 4.5+ with `core`, `imgproc`, `features2d`, `calib3d`
* Boost 1.74+ (`system`, `thread`, `filesystem`)
* msgpack-c (cpp variant)
* CMake 3.16+

On a Debian-flavoured aarch64 sysroot:

```bash
apt-get install --no-install-recommends \
  build-essential cmake \
  libeigen3-dev libopencv-dev libboost-all-dev \
  libmsgpack-cxx-dev
```

## Native build (on a Pi 4B or Rock 5C Lite)

```bash
cd vendor/openvins
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
sudo install -Dm0755 build/ados_openvins_shim \
  /opt/ados/plugins/com.altnautica.vision-nav/vendor/ados_openvins_shim
```

## Cross-compile via `cross`

The CI workflow at `.github/workflows/vision-nav-vendor-binaries.yml`
runs `cross` with a Debian-based aarch64 image:

```bash
cargo install cross --version 0.2.5
cross build --release --target aarch64-unknown-linux-gnu \
  --manifest-path build/Cargo.toml  # generated wrapper, see workflow
```

The image's `Dockerfile.aarch64` provisions the sysroot deps above.

## Conformance

The binary's `--conformance` flag replays an EuRoC bag (`V1_01_easy`)
through the upstream estimator and asserts pose deltas stay within
upstream-reference tolerance. CI runs this before signing the artefact:

```bash
./ados_openvins_shim --conformance --fixture euroc_v1_01_easy.bag
```

CI downloads the fixture from the upstream EuRoC dataset host; it is
not committed.

## Signing

The build product is signed by the workflow using the existing
`ADOS_SIGNING_KEY` Ed25519 secret, then bundled into the
`.adosplug` archive at release time.
