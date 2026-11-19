# ADOS capture contract (`ados.capture/1`)

Schema-only contract for capture sessions and reconstruction artifacts shared by the ADOS agent, the world-engine extension (capture and reconstruction node) and any simulator or tool that produces or consumes the same data.

- Licence: MIT (see `LICENSE`). This directory holds no code.
- Version: `1.0.0`.
- Every JSON document carries `"contract": "ados.capture/1"`.
- Consumers may vendor these files byte-identical and generate their own types from them.

## Files

|Path|Content|
|---|---|
|`LICENSE`|MIT|
|`schema/*.schema.json`|JSON Schema 2020-12: manifest, rig, camera model, geo anchor, placement, artifact descriptor, `transforms.json` extension|
|`proto/ados/capture/v1/*.proto`|Protobuf messages for MCAP channels: IMU, trigger, clock, GNSS, pose covariance|
|`fixtures/`|Golden JSON documents (`manifest.json`, `rig.json`, `placement.json`, `artifact.json`, `transforms.json`) and `tiny-session/`, a golden session directory (two 64×48 `mono8` frames, two triggers, 20 IMU samples, one clock sample, `timebase: "sim"`)|

Foxglove message schemas (`foxglove.RawImage`, `foxglove.CompressedImage`, `foxglove.LocationFix`, `foxglove.PointCloud`, `foxglove.PoseInFrame`) are referenced by name and not copied here.

## Conventions

### Time
- Every sensor sample carries `t_ns: u64` on the session timebase.
- `manifest.timebase` is one of `gnss`, `hub_freerun`, `host_monotonic`, `sim`.
- The manifest carries `t0_ns`, `utc_at_t0_ns` (`i64` or `null`) and `sigma_ns` (`i64`, `-1` = unmeasured).
- Simulated sessions use `timebase: "sim"` with `t_ns` equal to simulation time in nanoseconds.

### Frames
- World: local ENU metres at the `GeoAnchor`.
- Body: FLU.
- Camera: OpenCV (x right, y down, z forward).
- LiDAR: sensor native FLU.
- Transforms are named `T_<to>_<from>` and stored as a row-major 4×4 `[f64; 16]`.

### GeoAnchor
`{lat_deg, lon_deg, h_ellipsoid_m, geoid_undulation_m, datum: "WGS84", source: "gnss_mean" | "rtk_fixed" | "operator" | "sim"}`. Heights are always ellipsoidal. An anchor may be `null` for a session in a local frame (for example indoors without GNSS).

### CameraModel
`{model, width, height, fx, fy, cx, cy, dist: [f64], calibrated: bool, readout_ns: u64}`

|`model`|`dist`|
|---|---|
|`pinhole`|empty|
|`opencv`|Brown–Conrady `k1, k2, p1, p2, k3` (OpenCV order)|
|`opencv_fisheye`|`k1..k4`|
|`kb4`|`k1..k4` (Kannala–Brandt)|

`readout_ns = 0` means global shutter.

### LidarModel
`{model, pattern, h_fov_deg, v_min_deg, v_max_deg, rate_hz, points_per_s, min_range_m, max_range_m, range_sigma_m, blocked_azimuth_deg?: [[start, end], ...]}`

- `model`: device model id the drivers dispatch on (`rplidar-c1`, `livox-mid360`, ...).
- `pattern`: `planar` (one level ring), `spinning` (multi-beam rotating head) or `solid` (solid-state pattern).
- Angles are degrees in the sensor FLU frame; azimuth is counter-clockwise from +x.
- `blocked_azimuth_deg`: sectors the rig itself occludes (for example the operator behind a handheld); a sector may wrap through 0. Default empty.

### Rig (`rig.json`)
`{rig_id, calib_version, sensors: [{id, kind, role, T_body_sensor, camera?, lidar?, imu_noise?: {gyro_nd, accel_nd, gyro_rw, accel_rw}, time_offset_ns: i64, lever_arm_m?}]}`

- `kind`: `camera`, `imu`, `lidar`, `gnss`, `thermal`, `baro`, `mag`.
- `role`: `front_left`, `front_right`, `left`, `right`, `down`, `up`, `back`, `nav`, `texture`, `primary` (the vehicle's main camera).
- A `lidar` sensor must carry its `lidar` LidarModel; readers reject the rig otherwise (`rig sensor <id>: a lidar sensor needs a "lidar" model block`).

### Manifest (`manifest.json`)
`{contract, session_id, device_id, profile: "drone" | "visor" | "sim", timebase, t0_ns, utc_at_t0_ns, sigma_ns, anchor: GeoAnchor | null, rig: {rig_id, calib_version}, operator: {site?, floor?, rooms: [{name, floor, t_start_ns, t_end_ns}]}, segments: [{file, t_start_ns, t_end_ns, bytes}], health: {blur, texture, flicker, corridor, frames_dropped_pct}}`

- `operator.rooms` are the segments the operator started with MARK, in order.
- `health` flags are true when the condition was raised at any point in the session.

### `checksums.b3`
One line per file in the session directory except itself: `<blake3 hex>  <relative path>`, `/`-separated, sorted by path. Any file or directory whose name starts with `.` (device state markers such as `.recording` and `.uploaded`) is never listed, and a consumer requires the listed set to equal the session's files.

### Session directory
```
<session_id>/
  manifest.json
  rig.json
  segments/NNNNNN.mcap     rolled at 4 GiB or 10 minutes
  keyframes/               optional JPEG/PNG previews
  checksums.b3             BLAKE3 hash per file
```
`session_id = <device_id>-<UTC yyyymmddThhmmssZ>-<4 hex>`.

### MCAP profile `ados.capture/1`
Protobuf message encoding, lz4-compressed chunks. Every message's MCAP log and publish time equal its `t_ns`. Each segment carries `rig.json` and a snapshot of the manifest taken when the segment opened as attachments; `manifest.json` in the session directory is authoritative. A protobuf channel's schema data is a `FileDescriptorSet` containing the message's file and its imports.

|Channel|Message|
|---|---|
|`/cam/<id>/image`|`foxglove.RawImage` or `foxglove.CompressedImage`|
|`/cam/<id>/trigger`|`ados.capture.v1.TriggerRecord {seq, t_mid_ns, exposure_ns, gain}`|
|`/imu/<id>`|`ados.capture.v1.ImuBatch {samples[{t_ns, seq, gyro[3] rad/s, accel[3] m/s², temp_c}]}`, body FLU|
|`/gnss/<id>/raw`|`ados.capture.v1.GnssRaw {t_ns, bytes}` (receiver binary passthrough)|
|`/gnss/<id>/fix`|`foxglove.LocationFix`|
|`/lidar/<id>/points`|`foxglove.PointCloud`, frame `<id>`, with a per-point `t_offset_ns` field (`UINT32`, nanoseconds after the message time); a `planar` LiDAR's cloud has z = 0 in the sensor frame|
|`/pose/<source>`|`foxglove.PoseInFrame`, frame `world`; `<source>` is `truth` (simulation), `fc_ekf`, `vio` or `lidar`|
|`/pose/<source>/cov`|`ados.capture.v1.PoseCov {t_ns, cov[36], health}` for the pose with the same `t_ns`|
|`/clock`|`ados.capture.v1.ClockSample {t_ns, host_mono_ns, utc_ns, locked, sigma_ns}`; producers always set `sigma_ns` (`-1` = unmeasured), since an absent field decodes as `0` (exact)|
|`/health`|JSON|

### Artifact descriptor and placement sidecar
Each artifact `<file>` has a sidecar `<file>.place.json`:

`{contract, kind: "splat" | "mesh" | "pointcloud" | "occupancy" | "trajectory", file, format: "ply" | "spz" | "glb" | "copc" | "laz" | "esdf", sha256, anchor: GeoAnchor | null, R_enu_artifact: [f64; 9], t_enu_artifact: [f64; 3], scale, generation, source_session}`

Point clouds use `laz` (LAZ 1.4); `copc` is reserved for COPC-ordered files. `R_enu_artifact` is row-major and may be any rotation, including an axis permutation.

### Dataset export
- COLMAP sparse model: `cameras.bin`, `images.bin`, `points3D.bin`, plus `rigs.bin` and `frames.bin` for COLMAP 4 and later.
- Nerfstudio-style `transforms.json` extended with `ados_anchor` (a `GeoAnchor`) and a per-frame `t_ns`.

## Versioning

Semantic versioning. Additive, backward-compatible changes (new optional fields, new channels) bump the minor version. A breaking change bumps the major version and the contract id (`ados.capture/2`). Fixtures are regenerated with every change and consumers vendoring the files must update them byte-identical.
