# Vision Navigation calibration loaders (Python)

The plugin's agent half is a Rust binary (`../src/`). The only thing it needs
from a camera calibration is a Kalibr-style `camchain.yaml` in the plugin's data
directory, which it reads at start-up. Produce that file with Kalibr
(`kalibr_calibrate_imu_camera`) for the camera and IMU on the vehicle, then copy
it to `camchain.yaml` in the plugin's data directory. The Vision Nav tab shows
whether the agent loaded it.

This package validates such a file before it goes onto a vehicle. It is not part
of the runtime and is not shipped in the plugin archive.

## What the agent reads

The `cam0` block (a bare block without the `cam0:` wrapper is accepted too):

- `camera_model: pinhole`
- `intrinsics: [fx, fy, cx, cy]`
- `distortion_model: radtan | equidistant | none`
- `distortion_coeffs: [...]`
- `resolution: [width, height]`
- `T_cam_imu` (4x4 SE(3))
- `timeshift_cam_imu` (seconds, Kalibr convention `t_imu = t_cam + ts`)

The agent uses `timeshift_cam_imu` to pair each frame with the gyro sample taken
at the same instant.

## Modules

- `altnautica_vision_nav_calib.intrinsics`: `cam0` intrinsics loader and
  validator.
- `altnautica_vision_nav_calib.extrinsics`: `T_cam_imu` and timeshift loader
  and validator.

## Use

```sh
cd calibration-helper
python -m pip install -e .
python -c "from altnautica_vision_nav_calib import load_intrinsics, load_extrinsics; \
print(load_intrinsics('camchain.yaml')); print(load_extrinsics('camchain.yaml'))"
```
