# Vision Navigation calibration helper (Python)

This is the one-time, offline camera-IMU calibration wizard for the
Vision Navigation plugin. It is **not** part of the runtime hot path.
The plugin's agent half is a Rust binary (`../src/`); the only thing it
needs from calibration is a Kalibr-style `camchain.yaml` on disk, which
this helper produces.

Why it stays Python: the wizard is a heavyweight, infrequent,
OpenCV-bound flow (AprilGrid t36h11 detection with `cv2.aruco`,
monocular pinhole + radial-tangential intrinsics via
`cv2.calibrateCamera`, and a golden-section search for the joint
camera-IMU timeshift over the recorded IMU motion trace). It runs once
per camera mount, far off the 30 Hz pose-emit path, so there is no
reliability or latency reason to port the OpenCV pipeline to Rust.

## What it produces

A `camchain.yaml` with the `cam0` block the Rust agent reads:

- `camera_model: pinhole`
- `intrinsics: [fx, fy, cx, cy]`
- `distortion_model: radtan | equidistant | none`
- `distortion_coeffs: [...]`
- `resolution: [width, height]`
- `T_cam_imu` (4x4 SE(3))
- `timeshift_cam_imu` (seconds, Kalibr convention `t_imu = t_cam + ts`)

The Rust agent loads this file at start-up; VIO modes feed it to the
vendor estimator and the time aligner.

## Modules

- `altnautica_vision_nav_calib.intrinsics` — Kalibr `cam0` intrinsics
  loader/validator.
- `altnautica_vision_nav_calib.extrinsics` — `T_cam_imu` + timeshift
  loader/validator.
- `altnautica_vision_nav_calib.runner` — the wizard coroutine: decodes
  the captured frame bundle, runs detection + the intrinsics solve +
  the timeshift fit, and emits substep progress through injected hooks.

## Run

```sh
cd calibration-helper
python -m pip install -e .
python -m pytest -q   # if dev extras installed
```
