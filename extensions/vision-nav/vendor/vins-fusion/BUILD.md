# Building `ados_vins_fusion_shim`

The binary wraps upstream VINS-Fusion (`HKUST-Aerial-Robotics/VINS-Fusion`)
and exposes the same shim IPC contract as the OpenVINS shim
(`../../agent/src/altnautica_vision_nav/shim/ipc.py`).

## Sysroot dependencies

In addition to the OpenVINS sysroot deps, VINS-Fusion needs:

* Ceres Solver 2.0+
* glog 0.4+
* gflags 2.2+

On a Debian-flavoured aarch64 sysroot:

```bash
apt-get install --no-install-recommends \
  build-essential cmake \
  libeigen3-dev libopencv-dev libboost-all-dev \
  libceres-dev libgoogle-glog-dev libgflags-dev \
  libmsgpack-cxx-dev
```

## Source pinning

Unlike OpenVINS, VINS-Fusion has no formal release tags; the
canonical pin is a commit sha on `master`. The pin lives in
`CMakeLists.txt` under the `VINS_FUSION_REF` cache variable. Update
only via a release-bump PR that documents the upstream changes in
the commit message.

## Native build (Pi 4B or Rock 5C Lite)

```bash
cd vendor/vins-fusion
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
sudo install -Dm0755 build/ados_vins_fusion_shim \
  /opt/ados/plugins/com.altnautica.vision-nav/vendor/ados_vins_fusion_shim
```

## CI cross-compile

Handled by `.github/workflows/vision-nav-vendor-binaries.yml` in the
same matrix as the OpenVINS shim. The workflow's `cross` image
provisions the Ceres + glog + gflags packages on top of the
OpenVINS sysroot.

## Conformance

`./ados_vins_fusion_shim --conformance --fixture euroc_v1_01_easy.bag`
replays the same EuRoC bag fixture as the OpenVINS shim and asserts
the upstream-reference tolerance.

## Signing

Same `ADOS_SIGNING_KEY` Ed25519 path as the OpenVINS shim. The two
binaries ride together in the `.adosplug` archive.
