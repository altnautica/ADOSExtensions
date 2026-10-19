# basalt-ados

Stereo-inertial visual odometry for the capture service's `vio` pose
provider (`agent/capture/src/pose/vio/`). `basalt-ados` wraps
[Basalt](https://gitlab.com/VladyslavUsenko/basalt) (frame-to-frame optical
flow + square-root keypoint VIO) behind a Unix-socket input and a JSON-lines
output, and runs as a supervised child of `world-engine-capture`.

| Path | What | Licence |
|---|---|---|
| `upstream/` | Basalt, git submodule pinned at `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` | BSD-3-Clause (`upstream/LICENSE`) |
| `ados_input.cpp`, `ados_input.h` | the `basalt-ados` executable | BSD-3-Clause (`LICENSE`) |
| `CMakeLists.txt`, `vcpkg.json`, `vcpkg-configuration.json`, `build.sh` | build | BSD-3-Clause |
| `vio_config.json` | estimator config, also embedded by the Rust provider | BSD-3-Clause |
| `synth_check.py` | synthetic end-to-end check | BSD-3-Clause |

Only the VIO subset of Basalt is compiled (optical flow, linearisation,
square-root keypoint VIO/VO, float instantiations). No GUI, Pangolin, rosbag,
calibration or mapping code is built. The dependencies come from upstream's
pinned vcpkg (`upstream/thirdparty/vcpkg`, baseline `05442024`) and the
overlay ports in `upstream/vcpkg/ports` (which pin `basalt-headers` and
`opengv`): Eigen 5, oneTBB, OpenCV 4 core/imgproc/features2d, fmt,
magic_enum, nlohmann-json, cereal, Sophus, basalt-headers, opengv. Each keeps
its own licence; vcpkg installs the copyright files under
`build/<arch>/vcpkg_installed/<triplet>/share/<port>/copyright`. The Linux
distribution packages cannot be used instead: basalt-headers requires
Eigen >= 5.0, and Sophus, magic_enum and opengv are not packaged for
Ubuntu 24.04.

## Build

```sh
git submodule update --init extensions/world-engine/agent/capture/basalt/upstream
cd extensions/world-engine/agent/capture/basalt
./build.sh --arch x86_64 --jobs 2          # or --arch aarch64 on an aarch64 host
# -> build/out/bin/x86_64-linux/basalt-ados
```

Host tools: a C++17 compiler (GCC 13 tested), cmake >= 3.24, git, curl, zip,
unzip, tar, pkg-config; ninja is used when present. On Ubuntu 24.04:
`apt-get install build-essential cmake git curl zip unzip tar pkg-config ninja-build`.
Nothing else is installed system-wide: `build.sh` initialises the vcpkg
submodule, bootstraps it, and builds every dependency into `build/` with
`--clean-after-build` (static libraries, release only, triplets
`x64-linux-release` / `arm64-linux-release`). Options: `--jobs N` (default 2;
also caps vcpkg's parallelism), `--march M` (default `x86-64-v2` /
`armv8-a`), `--out DIR`.

The binary links every library statically except glibc, so it runs on any
Linux with glibc at least as new as the build host's (2.39 on Ubuntu 24.04).

Builds are native only. For aarch64 without an aarch64 machine, run the same
script inside an aarch64 container under QEMU user emulation (much slower),
from the repository root:

```sh
docker run --rm --platform linux/arm64 -v "$PWD":/src -w /src/extensions/world-engine/agent/capture/basalt \
  ubuntu:24.04 sh -c 'apt-get update && apt-get install -y build-essential cmake git curl zip unzip tar pkg-config ninja-build && ./build.sh --arch aarch64 --jobs 2'
```

## Runtime path

The extension payload installs the binary at `bin/<arch>-linux/basalt-ados`
(`x86_64-linux`, `aarch64-linux`), beside `world-engine-capture`.

## Command line

```text
basalt-ados <socket> <calib.json> <config.json>
```

`calib.json` is a Basalt calibration (cereal JSON, two cameras: left then
right, `T_imu_cam`, intrinsics `pinhole` / `pinhole-radtan8` / `kb4`, IMU
noise in continuous-time units); the capture service writes it from the rig.
`config.json` is a Basalt `VioConfig` (`vio_config.json`). `basalt-ados`
connects to `<socket>`, which the capture service has bound.

## Input protocol (socket)

Little-endian. Every message is a 5-byte header and a payload:

| Field | Type |
|---|---|
| type | `u8` |
| len | `u32`, payload bytes |
| payload | `len` bytes |

| Type | Name | Payload |
|---|---|---|
| 0 | HELLO | `u32 version` = 1. Must be the first message. |
| 1 | IMU | `u64 t_ns`, `f64 gyro[3]` rad/s, `f64 accel[3]` m/s^2 (specific force), IMU sensor frame; len = 56 |
| 2 | STEREO | `u64 t_ns`, `u32 width`, `u32 height`, `u8 left[width*height]`, `u8 right[width*height]`; Gray8, row-major, unpadded; len = 16 + 2·width·height |

Times are host `CLOCK_MONOTONIC` nanoseconds (IMU and frames on the same
clock, frame time = exposure midpoint when known). Unknown types are skipped;
a payload above 32 MiB, a frame whose size does not match its length, or a
missing HELLO ends the process (exit 2). A frame whose size does not match the
calibration resolution is dropped with a log line. End of stream drains the
pipeline and exits 0.

## Output (stdout, one JSON object per line)

```json
{"status":"initializing"}
{"status":"tracking"}
{"t_ns":1000000000,"T_w_i":[1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1],"cov":null,"features":118}
{"status":"lost"}
```

- `T_w_i`: row-major 4x4 IMU-to-world transform. The world frame is Basalt's:
  gravity-aligned with z up, origin and yaw where the estimator initialised
  (yaw is unobservable and arbitrary). Each process start is a new world.
- `cov`: 6x6 row-major over (x, y, z, rot_x, rot_y, rot_z), or `null`. Basalt
  does not expose the marginal covariance of the current state, so this build
  always prints `null`; the capture service substitutes a nominal covariance
  by health.
- `features`: optical-flow keypoints tracked in the left image of that frame.
- Status: `initializing` from start until the first state whose frame tracks
  at least 10 features; then `tracking`; `lost` after 15 consecutive frames
  under 10 features, or no state for 1 s while frames keep arriving; back to
  `tracking` on the next state with at least 10 features.

Logs go to stderr. Exit codes: 0 input closed, 2 bad arguments, calibration,
config or input, 3 lost for 3 s, no first state within 15 s of the first
frame, or a non-finite pose. The capture service restarts the process after
any exit (backoff 1 s doubling to 30 s), which re-initialises the estimator.

## Synthetic check

`synth_check.py` renders a stereo rig translating past a textured wall with a
consistent IMU stream, feeds it through the socket in real time and checks
that `basalt-ados` reaches `tracking`, produces a state for most frames and
ends within 10 % of the distance travelled after yaw alignment:

```sh
uv run synth_check.py build/out/bin/x86_64-linux/basalt-ados
```
