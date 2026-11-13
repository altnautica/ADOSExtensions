# Changelog

All notable changes to the World Engine extension.

## [2.2.1] - Unreleased

Ships after 2.2.0, which takes the [Unreleased] section below.

### Fixed

- **Every LiDAR pose reaches the flight controller, one poll after it is
  estimated.** The navigation pose was gated on the wall-clock time of the
  100 ms poll that handed it over, so a 10 Hz matcher's pose that arrived
  99 ms after the last was dropped (4-7.6 Hz published, gaps of 200-400 ms)
  and each pose waited up to a poll for publication. The pose lane now polls
  every 10 ms and gates on the estimate's own measurement time:
  `atlas.pose_publish_interval_ms` keeps the published rate on average, and a
  pose up to half an interval early is published, so a provider at exactly
  the interval loses none. A measurement time that goes backwards (provider
  reset) publishes at once.

## [Unreleased]

### Added

- **Navigation lanes independent of capture.** With `atlas.nav_pose =
  always` (the default when an RPLIDAR device is set) the capture service
  publishes the navigation pose in the north-aligned `map_ned` (the SLAM
  world rotated so it matches `LOCAL_POSITION_NED`: to the flight
  controller's compass heading at take-off, then onto its GNSS track
  by a fitted yaw and offset) with its `health`, `frame` and
  `source`, the binned body-frame scan (`plugin.world-engine.scan`), live
  occupancy tiles and sweep-cloud deltas, and `plugin.world-engine.map_status`
  whether or not a session is open. The pose is no longer published from the
  camera frame path.
- **Live occupancy and sweep cloud.** 5 cm occupancy tiles built by ray
  tracing every return, evicted to the session past 200 m or past their
  budget share; a voxel-centroid cloud of every scan placed by the 6-DoF pose
  that doubles its voxel when its budget share fills.
- **`atlas.memory_budget_mb`** bounds the keyframe queue, the scan ring, the
  sweep cloud and the occupancy tiles (40/10/30/20 %); unset it is 80 % of the
  capture unit's memory cap (`ADOS_PLUGIN_MEMORY_MAX_BYTES` from the host,
  else the cgroup). Queues drop their oldest entry and count it.
- **`atlas.compute_placement`** (`onboard`, `stream`, `post_flight`,
  `hybrid`) sets the live cloud resolution, has the compute node reconstruct
  in flight for `stream` and build a full-resolution live map from the raw
  polar scans the drone sends (`atlas.scan`), served on the device's world
  stream (the link reports the bearer and the rate it carries in the map
  status), and uploads the session after landing for
  `post_flight` and `hybrid`. The live map lanes stay on the drone and are
  not forwarded to the node.
- **Polar scan sessions.** A planar LiDAR records `lidar/<sensor>.polar.zst`
  (3 bytes a bin, zstd) instead of the 17-byte-a-point Cartesian cloud.
- **`planar-sweep` preset**: LAZ cloud, occupancy per height band and
  floorplan from the loop-closed polar scans, with no pinned tool for a
  LiDAR-only session; with camera frames it also trains a preview splat and
  meshes it.
- The GCS world-model card draws the live map (occupancy tiles and the sweep
  cloud) as it arrives.
- Per-profile resource caps (`agent.resources_by_profile`): 768 MB on a
  drone, 1536 MB on a visor, 512 MB on a ground station.
- `atlas.lidar.imu = "fc"` levels a planar LiDAR with IMU samples
  synthesised from the flight controller's attitude, for a companion with no
  IMU of its own (the rig declares an IMU sensor `fc` at the body origin).
- `scan2d_replay` example: replays a recorded scan log through the scan
  matcher and prints the wall time.

### Removed

- `atlas.live_reconstruct`. A compute node reconstructs a session in flight
  when its drone streams (`atlas.compute_placement = stream`, read from the
  drone's map status); `ADOS_ATLAS_LIVE` still forces it on a bench node.

### Changed

- `atlas.pose_provider = auto` picks LiDAR odometry on a drone too when the
  rig has the configured LiDAR and IMU: there the LiDAR pose is the
  navigation source the flight controller fuses, so its own EKF would be
  circular.
- **Floor plans.** The wall slice adds a band 0.4–0.1 m under a detected
  ceiling to the body-height band; points sampled from the extracted mesh
  join the cloud, so plain camera-only walls are found; rays from the
  keyframe cameras and LiDAR scan origins carve free space, so sparse clutter
  crossed by many rays no longer becomes a wall. A floor nobody marked rooms
  on gets its rooms found automatically (`Room 1..n`, largest first) by
  morphological segmentation of the free map.
- **Aisles.** Corridors come from the medial axis of the free floor between
  0.2 m and 2.0 m, split into straight runs; `aisles.geojson` features carry
  `kind` (`aisle` or `cross_aisle`), and the aisles output reports both counts.
- **Splat delivery.** Before SPZ and LoD, the trained splat drops non-finite
  splats, everything outside the keyframe bounds (±3 m across, 1 m below,
  3 m above) and floaters (splat-transform's GPU floater filter at 5 cm).
- **LiDAR input is picked by the rig's LiDAR model.** The capture service
  starts the live driver the rig sensor's `lidar.model` names (`rplidar-c1`
  over its serial protocol, `livox-mid360` through the Livox SDK); any other
  model is refused by name. `atlas.lidar` takes `lidar_id`, `imu`, and
  per-driver blocks `rplidar` (`device`, default `auto`; `baud`) and `livox`
  (`sdk_library`, `host_ip`, `lidar_ips`); the flat Livox keys are gone.
- `atlas.pose_provider = auto` on a visor picks LiDAR odometry when the rig
  has the configured LiDAR and IMU, ahead of VIO. A planar LiDAR runs 2D scan
  matching on the device; a 3D one keeps KISS-ICP, with its ranges taken
  from the LiDAR model.

### Added

- `tour.json` names the occupancy ESDF (`esdf`, its header beside it) and
  navigation `nodes` thinned from the keyframe poses to one per 1.5 m. The
  viewer's walk mode slides along walls with a 0.25 m clearance, and nodes
  show as floor discs that jump the walker there. The plugin viewer walks
  against the job's ESDF too.
- The prepare stage records the LiDAR scan origins (`lidar_origins.json`).
- **Planar LiDAR pose.** `PlanarLidarPose` levels each scan with the IMU and
  matches it into probability-grid submaps on a worker thread; it is `lost`
  until the first submap holds 10 scans and `degraded` in featureless
  corridors. The capture state's `pose.single_floor` is set, and the device
  HUD and the capture page show "Planar LiDAR pose: single floor only".
- The manifest requests `hardware.uart` (optional) so the visor's capture
  service can open a serial LiDAR.
- **Planar LiDAR sessions on the node.** A rig whose LiDAR is `planar`
  (RPLIDAR C1 class) is scan-matched in-process ahead of SfM (`ados-scan2d`:
  local SLAM, loop search, pose graph). Its body trajectory goes to
  `lidar2d/trajectory.tum` with `lidar2d/summary.json`, is aligned onto a
  recorded pose track when there is one, re-poses the keyframes as `lidar2d`
  priors (0.03 m across, 0.25 m in height) for the pose-prior mapper (never
  fixed poses), and re-registers the sweeps and scan origins. The sfm stage
  detail carries `lidar2d` and `prior_source`. GLIM and its submit-time check
  apply to 3D (solid-state or spinning) LiDAR rigs only, and a planar rig
  keeps the monocular depth prior in training.
- **Planar LiDAR-camera calibration.** `lidar_camera` on a planar-LiDAR rig
  runs natively: the AprilGrid board is found in each frame paired with a
  scan (within 50 ms), the scan points on the board are line-fitted, and the
  extrinsic is solved linearly and refined with a Huber loss. It needs at
  least 10 views spanning 30° of tilt about both camera axes and fails above
  2 cm RMS point-to-board error. 3D LiDARs keep the container method.
- The mirror/glass mask pass ignores cloud points beyond the rig LiDAR's
  maximum range (`masks.py --lidar-max-range`).

### Fixed

- **The exported splat's floater filter finishes on a large outdoor site.**
  The filter voxelised the scene at a fixed 5 cm, and its run time grows with
  the cube of the scene extent over the voxel: a preview of a 270 x 240 m
  forest walk sat in the products stage for over an hour. The voxel is now
  5 cm, coarsened over a large site to a thousandth of the walked box's
  diagonal (36 cm for that walk, under a minute); a room keeps 5 cm. 2.1.12.
- **Linux reconstruction nodes are documented.** `fetch-tools.sh` installs
  Brush, splat-transform and the Spark LoD builder on Linux with the pinned
  entries as they are; COLMAP (no upstream Linux binary) and, without a CUDA
  toolkit, the gsplat project run through `ADOS_TOOL_COLMAP` and
  `ADOS_TOOL_GSPLAT_TRAIN`, for instance as scripts that run CUDA containers
  with the node's data dir mounted at the same path. 2.1.12.
- **The drone's detail level reaches the compute node's own reconstructions.**
  `atlas.reconstruct_steps` was read only by the Atlas API, so the reconstruct
  jobs the node submits itself (the final bag, a session finalized after going
  silent, and live cycles) always trained for the node's
  `ADOS_COMPUTE_TRAIN_ITERS` (default 30000). The capture service now carries
  the setting on every capture status (`reconstruct_steps`, omitted when
  unset), the node stamps it on that session's jobs, and the node default
  applies only when the drone sends none. A new value reaches a running
  capture on the next config poll without restarting it.
- **A splat's gaussian count is read from the exported PLY.** The live-lane
  trainers (Brush, msplat) reported the count parsed from their log, and
  Brush 0.3.0 prints none, so a 10597-splat output showed
  `gaussian_count: 0`. The count is now the PLY header's `element vertex N`,
  with the log parse only as a fallback when the file cannot be read.
- **The navigation pose reaches the flight controller on an ArduPilot drone
  outside a capture session.** The map's north and the planar matcher's
  levelling came only from `ATTITUDE_QUATERNION`, which the drone stream
  table does not request (it asks for `ATTITUDE`; the quaternion only while a
  capture session runs). With no attitude the lidar SLAM pose was never
  north-aligned and never published, so external navigation sent no
  `ODOMETRY` and a GNSS-denied drone could not arm. The capture service now
  also reads `ATTITUDE` (Euler angles) for the same attitude. 2.1.1.
- **The planar lidar SLAM no longer slides down an aisle of rack uprights.**
  The matcher rated a scan's constraints from points whose neighbours fit a
  line, so isolated posts and uprights (one or two returns each) counted for
  nothing. A still drone that saw only uprights along a racked aisle was
  rated degenerate along it, kept its constant-velocity prior there, and the
  prior's velocity never decayed: the pose drifted 2-3 m in 20 s on the
  ground and tens of metres in flight. Points on compact objects seen in
  silhouette (free space or a farther background on both sides) now
  constrain both directions, as point-to-point matches; a wall seen at a
  grazing angle still constrains only across itself. Along a direction a
  match leaves unconstrained, the motion prior now coasts on the velocity
  last observed there and fades to rest (1 s time constant) instead of
  carrying an unobserved velocity forever. A recorded still sequence among
  uprights now holds within one grid cell. 2.1.2.
- **The navigation map sits on the flight controller's GNSS frame, so a GNSS
  loss no longer makes the drone jump.** `map_ned` took its heading from the
  flight controller's compass at the first pose, which is typically a few
  degrees off on the ground, and its origin from the first scan. ArduPilot's
  MAVLink visual odometry (`VISO_TYPE` 1) applies no yaw or position
  correction, and when the EKF switches to external navigation it resets
  its position to the external one, so at a GNSS loss the drone jumped by the
  map's error at that point: 8 m 137 m from the take-off in the forest
  bench run. While the flight controller's own `GPS_RAW_INT` has held a 3D
  fix for 15 s, the capture service now fits the yaw and offset that put the
  SLAM track on the EKF's `LOCAL_POSITION_NED` track (the yaw over at least
  a 10 m RMS baseline, weighted to the last 200 m flown; the offset over the
  last 5 s) and freezes both when GNSS goes; the compass heading and the
  first scan stand only until then. The fit and the freeze are logged
  (`nav_map_gnss_fitted`, `nav_map_gnss_frozen`). 2.1.3 fitted the yaw
  only, gated on the agent's telemetry; 2.1.4.
- **A GNSS-denied map takes the flight controller's heading at take-off, not
  its magnetic heading before the EKF has an origin.** `map_ned`'s north was
  latched from the FC's compass heading at the first pose. Indoors the first
  pose comes before the companion sets the EKF origin, and until then
  ArduPilot's heading is magnetic: once the origin brings the declination
  in, the EKF's yaw turns onto true north over a few seconds while the map
  kept the magnetic heading. At the bench's location that left the map
  12.8° off the EKF's own yaw; the EKF fused the inconsistent external
  navigation into a variance failsafe and landed (warehouse and mine bench
  runs). The north now follows the FC heading while the vehicle sits where
  the first pose was taken and is fixed once it has moved 0.5 m from there,
  horizontally or in climb (`nav_map_north_fixed`). 2.1.5.
- **The planar lidar SLAM's heading follows bends and fast yaw turns.** The
  matcher's rotation prior integrated the IMU's rate about the world
  vertical, but a scan levelled by roll and pitch turns with the yaw of the
  attitude, and the two differ by `∫roll̇·sin(pitch)`. On an airframe that
  pitches and rolls as it manoeuvres that does not average out: 17° over a
  125° forest trail, so the prior fell 13 % short of every turn and pulled
  the match with it. Replayed on the recorded revolutions with the true
  attitude at the flight controller's 20 Hz, the forest heading error at the
  GNSS loss went from 0.8° to 0.01° (157 m flown), a 190° take-off yaw slew
  in a 12 m room from 5.4° to 0.5°, and 450° of waypoint turns in a
  warehouse aisle from 23° to 1.4°. The prior is now the heading of the
  attitude itself. 2.1.6.
- **The navigation pose is stamped with the time it describes.**
  `plugin.world-engine.pose` carried its publish time in `ts_ms`, while a
  lidar pose describes its scan's first return, at least a revolution
  earlier: about 0.2 s behind in the bench runs, which a consumer fusing it
  as current took for a heading and position that lag through every turn.
  `ts_ms` is now the unix time of the pose's own measurement. 2.1.6.
- **The lidar SLAM's rotation prior is the flight controller's gyro, not its
  fused attitude.** With `atlas.lidar.imu = "fc"` the IMU stream was
  synthesised from consecutive EKF attitudes, so every yaw correction the
  EKF made reached the matcher as a rotation, and the matcher follows its
  prior. With external navigation the EKF fuses the matcher's own pose,
  which closed a loop: after a 7° yaw wobble in a warehouse aisle the EKF
  and the SLAM agreed on a heading 8° off the airframe's and the drone flew
  into the racks. The stream is now the FC's `RAW_IMU` (its calibrated gyro
  and accelerometer at the 100 Hz the vision-nav plugin asks for), and the
  attitude stands in only until `RAW_IMU` arrives (`nav_imu_source_raw_imu`
  is logged when it does). Replayed on that flight's recorded revolutions,
  the heading error over the wobble went from 15° (end 10.4°) to 0.27°
  (end 0.1°); the 60°/s take-off slew in the 12 m room stays within 0.4° and
  450° of warehouse waypoint turns within 1.2°. 2.1.7.
- **Along a corridor the lidar SLAM follows the flight controller's ground
  velocity.** In a straight drift or aisle the planar scans pin the pose
  across the walls only; along them the matcher kept its motion prior, whose
  velocity came from its own recent poses. Accelerating down a 5 m wide mine
  drift with rough rock walls, that prior lagged, the matches followed it
  and the pose covered 39 % of the 27 m flown while its health mostly read
  good; the flight controller, fusing it, sped up to 8 m/s until its EKF
  failed. With `atlas.lidar.imu = "fc"` the capture service now also reads
  the FC's own `OPTICAL_FLOW` (its flow sensor's rate with the body rate
  taken out) and its downward `DISTANCE_SENSOR`. Their product, levelled
  with the matcher's attitude, is the motion prior's velocity while it
  arrives (`nav_velocity_source_optical_flow` is logged when it first
  does), so the pose follows the measured motion where the scans cannot see
  it and reads `degraded` there. Replayed on that flight's recorded
  revolutions, gyro and flow, the distance matched down the drift went from
  39 % to 97 % of the distance flown. The vision-nav plugin (0.6.4) asks the
  FC for `OPTICAL_FLOW` at 20 Hz. 2.1.8.
- **The lidar SLAM learns the flight controller's gyro bias from its scans
  and levels with the FC's attitude, not its accelerometer.** Since the
  rotation prior became the FC's `RAW_IMU` gyro (2.1.7) the matcher, which
  keeps most of its prior on every scan, adopted the gyro's z bias: a
  simulated FC whose bias drifted between 0.03 and 0.23°/s turned the
  heading of a GNSS-denied forest flight 1.8° off within a minute, and the
  map with it. The matcher now learns that bias from the heading the scans
  turned against the gyro's, on scans whose points pin the rotation to 0.1°
  while the heading turns slower than 20°/s, over a 12 s time constant and
  bounded to 1°/s, and takes it out of every sample before the prior and the
  deskew; only the scans teach it, so the EKF's yaw still never reaches the
  prior. The specific force beside the gyro was `RAW_IMU`'s accelerometer,
  which on a multirotor reads the thrust: braking into and out of two fast
  turns at an aisle end it leant 15 to 20° off gravity, the matcher's
  attitude filter levelled with it, and the heading came out of the turns 2
  to 4° off and stayed there up the return aisle. It is now gravity in the
  attitude the FC last reported (its EKF has the manoeuvre taken out, and
  gravity carries none of its yaw). Replayed on those flights' revolutions
  with the FC's own 25 Hz IMU, the forest heading 15, 45, 60 and 150 s on
  from a reference went from -0.07, -0.73, -1.77 and -1.79° off to +0.02,
  +0.06, +0.24 and +0.28° (on the true attitude: +0.01, +0.04, +0.16,
  +0.25°), and the warehouse return leg from 4.5° off on average to 0.02°
  (worst 0.11°).
  New recorded-sequence tests: a forest slice whose bias rises from 0.08 to
  0.13°/s (learnt to within 0.01°/s, heading 0.09° off on average against
  0.35° before) and the warehouse return leg (0.05° off on average with the
  FC's gravity, 3.3° with the accelerometer). 2.1.9.
- **Low-quality optical flow is not used as the ground velocity.** An
  `OPTICAL_FLOW` sample below quality 128 (of 255), a sensor tracking too few
  features over dark or bare ground, no longer reaches the matcher's motion
  prior. 2.1.9.
- **Every LiDAR scan reaches the live map, the session record and the node
  stream.** A scan's pose describes its first return, and the planar matcher
  estimates it on its worker thread after the scan is fed. The capture
  service asked for that pose at once, found the pose track ending a
  revolution earlier (past the 50 ms it extrapolates) and placed the scan
  without one, so it went into neither the live occupancy and sweep cloud,
  the session's polar record nor the raw scans the `stream` placement sends.
  With a 10 Hz planar LiDAR about 97 % of the scans were lost: a nine-minute
  forest flight kept 1 788 sweep voxels at 5 cm where its scans hold about
  87 000, and a mine flight's map stayed empty. Scans now wait, in arrival
  order, until the pose track reaches their time (at most 0.5 s, after which
  they are recorded without a pose as before); the body-frame scan lane is
  still published as each scan arrives. 2.1.10.
- **`map_status.memory_used_mb` is the capture process's resident memory**
  (`VmRSS`), null when `/proc` cannot be read, in place of the bytes the map
  stores held, which read 0 while the map was empty. 2.1.10.
- **The lidar SLAM matches a scan only once the flight controller's IMU
  covers it.** The FC's `RAW_IMU` reached the matcher only on the 100 ms
  pose interval while each scan went to it at once, so a scan was matched
  before the IMU across it arrived and the matcher extrapolated the heading
  (about 7° lost in the bends of a forest flight). The FC's IMU and ground
  velocity are now fed before each scan, and the matcher holds a scan (at
  most one revolution) until the newest IMU sample is within 10 ms of its
  last point. `nav_imu_feed` logs the IMU rate and the widest scan-to-IMU
  gap every 10 s. 2.1.11.
- **A planar LiDAR's take-off no longer slides the pose.** A planar scan cuts
  the scene at its own height; in a chamber whose rock walls lean, each
  revolution of a climb drew a different outline, the matcher followed it
  against the floor-level submap, and the pose kept the slide: 0.7 to 0.9 m
  across the drift of a mine survey, enough to leave the first waypoint
  1.1 to 1.3 m short. The matcher now takes the flight controller's EKF
  height and climb rate (`LOCAL_POSITION_NED`): while the scan plane climbs
  or descends faster than `max_climb_rate_mps` (0.15 m/s) the pose rides its
  motion prior (IMU heading, flow velocity) and nothing is mapped, and a
  submap takes scans only within `submap_height_band_m` (0.3 m) of the height
  it started at. Replayed on the recorded mine survey, the error after the
  climb falls from 0.98 m to 0.03 m. 2.1.13.
- **The FC's `RAW_IMU` reaches the planar matcher as it arrives.** It was
  queued and handed over on the pose tick (100 ms) or the next scan, so a
  scan waiting for the IMU to cover its last point could wait up to a tick.
  Each `RAW_IMU` sample, flow velocity and EKF height now goes from the
  MAVLink delivery straight to the matcher, which matches the scan as soon
  as the covering sample lands. `nav_imu_feed` logs the median scan-to-IMU
  gap (`p50_scan_imu_gap_ms`) beside the widest. 2.1.13.
- **A shaft descent and a terrain-following leg are matched again.** 2.1.13
  rode the motion prior through every climb faster than 0.15 m/s: down a
  mine shaft the pose drifted 2.4 m into the walls over 12 m of descent,
  and a forest leg that climbs as it travels went unmatched. Only a climb
  that is mostly vertical (faster than the horizontal speed) rides the
  prior now, and only for its first `max_unmatched_climb_m` (3 m), which
  covers a take-off; past that the climb is a transit and is matched against
  the shaft's walls, without the height band, until it has paused for 2 s.
  Replayed on the recorded shaft descent the error stays under 0.55 m. 2.1.14.
- **A camera the stored rig lacks is added to the session's rig.** The
  session records `/cam/<id>/image` for every enabled camera, but a stored
  rig (`PUT /api/v1/sensors/rig`) was taken as it was, and the node indexes
  only the rig's cameras: a drone whose rig listed its LiDAR and IMU recorded
  5.2 GB of frames that reconstructed nothing ("the session has no camera
  frames"). An enabled camera with frames and no rig sensor of its id now
  joins the rig as a derived camera would (configured or `atlas.hfov_deg`
  intrinsics, `calibrated: false`), logged as `rig_camera_derived`. 2.1.15.
- **The exported splat keeps the whole scene, and its floater filter works on
  the site rather than its mirror image.** splat-transform reads a PLY's x and
  y negated, so the box handed to it kept the mirror image of the walked site;
  and cutting the splat to any box dropped the distant scenery and sky the
  views see (held-out PSNR on a 270 m outdoor walk: 18.0 dB uncut, 13.6 dB
  cut to the box). The floater filter now runs only inside a box, the walked
  bounds grown to the reconstructed cloud's 1st to 99th percentile in
  splat-transform's frame (it still bounds the filter's voxel grid), and the
  Gaussians outside it are kept unfiltered: 18.0 dB, the filter finishing in
  2.5 min on that walk. 2.1.15.
- **A late subscriber to the live occupancy gets the map around the vehicle.**
  `plugin.world-engine.occupancy` carried only the tiles that changed since
  the last payload, so a consumer that started after take-off (or missed a
  payload) never received a wall the LiDAR had already mapped. Every fifth
  payload tick (5 s) the payload now also carries every tile within 20 m of
  the vehicle, changed or not; consumers overwrite tiles, so a repeat is
  harmless. 2.1.16.
- **Structure-from-motion no longer stalls on a session with repeated image
  messages.** The input stage exported a keyframe once per copy of its image
  message, so `frames.json` and the COLMAP image list could name one image up
  to seven times. On such a list `feature_extractor` stopped after the first
  image and never finished. The input stage now exports each frame once.
  Reading `frames.json` keeps the first record of each image name, so a
  dataset staged by an earlier node also resumes. Test: a frames list with
  every frame three times gives an image list with each name once. 2.1.17.
- **The Live World page says when a capture builds no world model.** A link
  that carries no keyframes shows "No keyframes on this link" with the
  reason; a capture still waiting for its position anchor, one that reached
  its keyframe limit, and keyframes the drone dropped are shown in the
  Capture card, with the pose source. The header no longer shows a degraded
  capture in the healthy colour. 2.1.18.
- **A drone that is not answering no longer reads as idle.** The header
  shows "unknown" before the drone first answers and "not answering" once it
  stops, and Start, Pause and Stop are disabled while it is not answering.
  2.1.18.
- **Why a capture control is disabled is written under the controls**, not
  only in a tooltip. Buttons, the viewer switcher, the splat viewer's toolbar
  and walk-to points and the capture switch have a touch-sized target on a
  touch screen and a visible keyboard focus ring. 2.1.18.
- **Capture states, tracking health, links and viewer names are worded from
  the extension's strings** instead of the agent's raw ids, with "Other" for
  a value it does not know; the viewers' loading and error messages are worded
  the same way. The extension ships English only: the host's
  string service does not tell an extension which language is selected.
  2.1.18.
- **The compute node and the offload target are shown by name**, with the
  device id or address on hover. 2.1.18.
- **Removed the "Relay decimation" row**, which the drone never reports and
  always read "—". 2.1.18.
- **Start waits for the drone to answer on the Live World page.** Until the
  drone reports its capture readiness, Start is disabled with "Waiting for
  the drone to report its capture readiness" written under it, as on the
  setup page. A drone that has never answered now reads "not answering" after
  three readiness checks in a row get no reply (about 20 s), instead of
  "unknown" for as long as the page is open. 2.1.19.
- **"No keyframes on this link" goes away when forwarding stops.** The link,
  its keyframe notice and its reason are cleared once the drone reports a
  capture state with no link, instead of keeping the last link's notice.
  2.1.19.
- **The point cloud's "showing … / … points" label is worded from the
  extension's strings.** 2.1.19.
- **A declared take-off heading holds the navigation map's north.** With
  `atlas.takeoff_heading_deg` set (the heading the drone faces where it
  starts, degrees from true north), the map's north is latched from it once
  at the first pose and never taken from the flight controller. Before, the
  north followed the flight controller's heading until take-off; with the
  flight controller taking its yaw from the map (ArduPilot `EK3_SRCn_YAW` 6)
  that heading was the map's own yaw back plus gyro drift, so nothing held
  north and it walked about 19° on the ground before take-off. Unset, the
  north follows the flight controller's heading as before. The navigation
  pose now says where its north comes from (`north`: `declared`,
  `fc_heading` or `gnss`), so a consumer can refuse to feed the flight
  controller a yaw that only echoes its own. 2.1.20.
- **A tilted planar scan drops its floor returns.** Pitched 25° at 3 m, a
  12 m planar LiDAR's forward beams reach the floor about 7 m ahead; levelled,
  those returns drew a false wall across the scan and pulled the match. With
  the height fed, a levelled return lower than the height less 0.25 m
  (`floor_margin_m`, which also absorbs an uneven floor) is dropped, and one
  within 0.25 m of a configured ceiling (`ceiling_m`, unset by default) too.
  On the ground, where the margin exceeds the height, nothing is dropped.
  2.2.0.
- **A climb or descent without a measured ground velocity rides the motion
  prior for at most 2 s** (`max_unmatched_climb_s`), then its scans are
  matched again, band by band in submaps of their own height, until it ends.
  Before, a mostly vertical climb rode the prior for its first 3 m whatever
  its speed: a slow descent drifting in the wind took 10 s to cover them,
  and the prior, which cannot see the drift, held the pose still while the
  vehicle moved. With an optical-flow velocity the climb rides the measured
  velocity as before. 2.2.0.
- **The navigation pose carries its horizontal uncertainty**
  (`sigma_xy_m`, metres). While the planar scan matcher's scans ride the
  motion prior unmatched (a climb, too few points) the σ grows by half the
  distance the prior carries the pose, and while a climb without a measured
  velocity is matched band by band it grows by half the height climbed (a
  leaning wall moves the outline with height); up to 5 m. A matched scan is
  otherwise as good as the submap it matched in: 0.05 m in one started at a
  matched pose, the start pose's σ in one the prior placed (the submap of a
  new height after a climb). Before, a pose held on the prior alone through
  a climb read as good to 5 cm to the flight controller. Estimators that
  report no σ publish none. 2.2.0.
- **A full disk ends the recording and leaves navigation running.** When a
  segment write failed (`ENOSPC`) while the mcap writer was starting a
  chunk, the writer lost its output stream and its next call or its drop
  panicked (`self.writer should never be None`); release builds abort on
  panic, so the whole capture process exited and the scan matcher restarted
  in a new frame. The segment writer now latches the first I/O error,
  never shows it to the mcap writer, and fails every later write and the
  close with it: the recording ends with the error and the navigation lanes
  keep publishing. 2.2.0.

## [2.0.0]

### Added

- **Capture contract `ados.capture/1`.** MIT schema files under `contract/`
  (JSON Schemas, protobuf messages, golden fixtures including a tiny MCAP
  session), the `ados-capture-contract` crate generated from them with an MCAP
  session writer and reader, and generated TypeScript types for the GCS half.
- The capture service and link run on the `visor` profile.
- **Session upload lane.** `/api/compute/sessions/<id>`: manifest POST,
  resumable `PUT files/<rel>` with `Content-Range`, `HEAD` offsets, and a
  commit that verifies every file against `checksums.b3`.
- **Staged reconstruction.** Job kind `reconstruct {session_id, preset}` with
  presets `preview`, `hq`, `tour`, `warehouse`, run as persisted stages
  (prepare, sfm, train, mesh, products) that resume after a node restart. Job
  status carries the stage, per-stage rows, GPU, ETA, tool versions and a
  structured error.
- Deliverables: SPZ and `.rad` splats, collision mesh, LAZ point cloud, ESDF,
  placement sidecars, a dataset export (COLMAP model + `transforms.json`), and
  for indoor presets a floorplan with rooms, aisles and a static tour bundle.
- Job kinds `change {a, b}` (cloud-to-cloud change detection) and
  `calibrate {session_id, target}` (IMU noise, intrinsics, camera-IMU,
  LiDAR-camera); `POST /api/compute/measure`; job files served with ranges.
- Pinned tool set (`agent/node/tools/tool-pins.json`) with fetch scripts.
- A standalone node (`--standalone`) and a Windows build of the node. The
  standalone node issues node credentials (`/api/compute/node-credentials`,
  owner bearer only) kept in its data directory, and admits a capture device
  presenting one on the lanes it was issued for (session upload, job submit
  and reads of its own jobs, artifacts). Its owner token and credential store
  are owner-only (mode 0600, or a protected single-user DACL on Windows), and
  each standalone node has its own persisted id.
- **Device capture surface.** The link serves `POST /atlas/capture/{start,
  stop,pause,resume,mark}`, `GET /atlas/capture/{state,coverage}`,
  `PUT /atlas/capture/exposure`, `GET /atlas/sessions`,
  `POST /atlas/sessions/<id>/{upload,reconstruct}`, `DELETE /atlas/sessions/<id>`
  and `GET /atlas/calib[/<version>]` over control protocol v2, with
  `{"error": CODE, "detail": ..}` refusals.
- **Resumable session uploader.** Finished sessions go to the paired node over
  the LAN only, in ranged chunks of at most 8 MiB that resume from the node's
  `HEAD` offset after a dropped link; `.uploaded` is written after the commit
  ACK. `atlas.auto_upload` uploads when the device is idle or charging and not
  recording.
- **Device calibration store** (`<data dir>/calib/<rig_id>/<version>.json`,
  an `active` pointer per rig) and a Kalibr `camchain-imucam.yaml` + `imu.yaml`
  importer.
- MCP tools `capture_start`, `capture_stop`, `capture_mark`, `sessions_list`,
  `reconstruct`, `job_status`, `tour_bundle`.
- **Session recorder.** The capture service records `ados.capture/1` sessions
  under the agent's capture root (admission from the janitor sidecar,
  `E_STORAGE_FULL` otherwise): every camera frame stamped at its joined
  trigger, IMU, triggers, GNSS bytes and fixes, LiDAR scans, clock samples,
  `/pose/<source>` with covariance and a 1 Hz `/health` record; segments roll
  at 4 GiB or 10 min; `.recording` is kept and refreshed while a session is
  open; site, floor and MARK rooms land in the manifest.
- **Sensor-bus ingest** (`sensor.stream.read`): a 10 s IMU ring, trigger join,
  and the host-to-session clock map.
- **Pose providers** with SE(3) interpolation and a 50 ms extrapolation limit:
  the FC EKF (exact WGS-84 to ENU), Basalt stereo-inertial VIO
  (`capture/basalt`, BSD-3, live socket input), KISS-ICP LiDAR odometry
  (`capture/python/kiss_icp_live`) and a replay provider. A Livox SDK2 driver
  (`capture/livox`) feeds LiDAR scans.
- **WHEP preview and rig write through the agent.** `POST|DELETE
  /atlas/capture/whep` and `PUT /atlas/rig` relay through the host methods
  `video.whep.relay` and `sensors.rig.write`. The preview resolves the capture
  camera to its media leg from the agent's video roster (the primary camera is
  served at `main`); a camera not in the roster answers `404`.
- **GCS capture surfaces.** Visor node pages `capture` (camera grid, coverage
  map, trajectory, health chips, REC/MARK, room and floor, exposure lock),
  `sessions` (list, upload progress, preset picker, job pipeline, viewer, tour
  bundle download) and `calibration`; drones gain `sessions`; the compute
  surface shows the job pipeline stages.
- **Spark viewer.** Splats render with Spark (SPZ and `.RAD` streaming) with
  walk and orbit modes, floorplan overlay, room hotspots and a measure tool; a
  standalone `tour-viewer` build feeds the tour bundle.
- **Crashed sessions.** A `.recording` session the capture service does not
  own and that is older than 120 s is listed as not finished; it can be
  deleted with `?force=true` and is never uploaded (`E_NOT_FINISHED`).
- **Session timebase.** A session on a non-host timebase starts only after a
  clock sample (`E_CLOCK_NOT_READY`); a `host_monotonic` session never maps
  its stamps. Exposure reports `applied: false` until the camera applies it and
  uses the nearest flicker-safe value under the mode cap. A bad item is skipped
  and counted; only a writer I/O error ends recording.
- Each capture session maps every stamp through its own clock map (host-fallback
  clock samples ignored, steps over 50 ms refused, never backwards), so a
  session never mixes timebases; the segment roll counts forward time from
  the segment's first message only, and the listed duration is the last
  segment end minus `t0_ns`.
- Auto-queued uploads run only while the auto gate (idle or charging, not
  recording) stays open and pause, to resume from the node's offset, when it
  closes; operator uploads are not gated. IPv6 node addresses are bracketed.
- **LAN-only upload is pinned** to the checked address and never follows
  redirects. MCP tools answer within 4 s. Job errors decode as objects.
- **Release payloads**: `basalt-ados`, the Livox SDK2 library and `uv`
  (x86_64-linux; aarch64-linux is a manual release step). The KISS-ICP
  environment is synced on the device at the first LiDAR start.
- **Capture quality and HUD**: blur, texture, mains flicker, coverage voxels,
  corridor degeneracy, storage and clock chips; the `capture_hud` display page;
  B1 / B2 passthrough presses start/stop and MARK.

### Changed

- `atlas.pose_tier` and the offloaded-SLAM pose lane are removed; `atlas.pose_provider` (`auto|fc_ekf|vio|lidar`) replaces them. `PoseSource` is `fc_ekf|vio|lidar|replay`. Checksums skip dot-file markers.

- **Keyframes and capture QA.** The node's `prepare` stage keeps the sharpest
  frame within 0.3 s of each threshold crossing (relative to its camera's
  median) and drops a pick only when even that frame is under 0.35 × the
  median; with a LiDAR the baseline is 0.08 × the median range of the nearest
  scan (0.05–0.50 m), and the live selector uses the same rule clamped to
  0.15–0.50 m. The blur chip combines the gyro-predicted motion blur
  (warn > 1 px, bad > 2.5 px) with the Laplacian score relative to the
  camera's running average (its reading); coverage rays stop at the LiDAR
  range around their azimuth; the single 20 m loop check is replaced by a
  revisit rule (back within 2 m of a pose at least 30 s old) and a
  `close_loop_prompt` raised every 40 m walked without one.

- **Atlas envelope version 2.** Keyframes carry the contract camera model,
  geo anchor, a timebase-stamped exposure time and IMU samples with `t_ns` and
  `seq`; every artifact descriptor carries a placement. Version 1 frames are
  refused.
- `atlas.intrinsics.<camera>` takes the contract camera model: `model`
  (`pinhole`, `opencv`, `opencv_fisheye`, `kb4`), `fx`, `fy`, `cx`, `cy`,
  `dist`, `readout_ns`.
- The live-lane trainers resolve from the pinned tool set and fail with
  `E_TOOL_MISSING` instead of producing a placeholder.
- The exposure lock is applied to every enabled capture camera through the
  host camera control (`camera.control.write`), on `PUT /atlas/capture/exposure`
  and at run start for a configured lock; unlock returns the cameras to auto
  exposure. `applied` is true only when every camera took the values, which
  are then the read-back; otherwise `reason` names why (`not_v4l2`,
  `control_unsupported`, `gain_unit_unknown`, `not_capture_camera`) and the
  GCS shows "Requested, not applied". Cameras are set in parallel; a reply
  still waiting after the host's engine wait plus a margin (inside the link's
  3 s control wait) answers `pending` and the apply finishes in the
  background, its read-back landing in the capture state. Gain is a
  multiplier (×) throughout.
- Compute-node datasets are written as the contract `transforms.json`, with
  the session anchor and per-frame exposure times.
- `atlas.intrinsics` and `atlas.pose_prior` refuse unknown keys, so a
  pre-contract override fails with an error naming the accepted keys.
- The session anchor is the mean of the first 100 3D fixes, with its MSL
  altitude raised to the ellipsoid by the EGM2008 geoid (1° grid).
- A selected frame whose pose arrived more than 50 ms from the exposure is
  not sent as a keyframe; the count is logged.
- Front-panel presses are matched on the host's button label (`B1` REC, `B2`
  MARK) instead of a GPIO pin.
- Reconstruction `prepare` selects a multi-camera session per time instant
  (one trigger sequence within half a frame period, falling back to time
  alone when per-camera trigger counters pair too few): keyframe picks, blur
  drops and the image cap keep or drop an instant's frames together, and an
  instant missing a camera's frame is dropped whole, so the COLMAP rig stays
  paired after a missed frame. An instant's images are named by its
  reference camera's time; `frames.json` keeps each frame's own `t_ns`.
  Cameras with too few common instants are exported outside a rig. The
  `prepare` and `sfm` stage details report `rig_frames_paired` (and `prepare`
  `rig_frames_dropped`) in place of the `sfm` camera count.
- The compute node answers `GET /api/compute/identity` (`{node_id}`, the id
  it advertises over mDNS) to any caller. A lane pinned to a node address
  (`offload.compute_node_addr`, or the relay's `relay.compute_base_url`) asks
  it first and presents only the credential that node issued. When the id
  cannot be read, it presents the one credential installed for that address
  (`POST /node-credential` takes an optional `node_addr`, kept with the
  credential) or the lane's sole credential, and nothing when several could
  fit. When auto-upload is on and that leaves no credential, the sessions it
  would take list `upload.state: "blocked"` with `error` `no_credential` or
  `ambiguous_credential` in `GET /atlas/sessions`, the GCS shows "Auto-upload
  blocked", and the link logs a warning once per reason.
- The world and tour viewers read SPZ version 4 (Spark 2.4), the version the
  products stage writes (`splat-transform --spz-version 4`, named rather
  than left to the converter's default); versions 1 to 3 still load. The
  Spark LoD builder is pinned to the same release (2.4.0). The viewer frames
  a splat on its solid splats (opacity at least 0.05, smallest scale at
  least 1% of the opaque splats' median middle scale, so at any model scale),
  the 1st to 99th percentile per axis, so far floaters no longer push the
  camera out until the scene is sub-pixel; a streamed `.rad` is re-framed
  as its pages arrive, until the paged count settles (at most 2 s) or the
  user moves the camera. A model with no solid splat shows "empty or fully
  transparent" instead of a blank view.
- Reconstruction `sfm` on a posed session matches only the image pairs the
  poses make plausible (within 2 s of keyframes in time, 20 to 60; centres
  within 4 m with view axes under 30° apart; the cameras of one rig
  instant) through `matches_importer`, verifies each pair's relative pose
  and drops those that contradict the priors (rotation over 10°, or
  direction over 20° across a baseline of at least 0.3 m), then maps with
  `pose_prior_mapper`, each image's prior position weighted by the track's
  recorded covariance or the source's default (`lidar2d` 0.03/0.03/0.25 m,
  `vio` 0.05 m, `fc_ekf` 0.5 m) under a robust loss. A fixed LiDAR
  trajectory and the `preview` preset still triangulate against the poses.
  An unposed session's sequential matcher uses a linear overlap of 2 s of
  keyframes with no quadratic overlap or loop detection. Every pair needs 30
  verified inliers and an image 50 to register; a calibrated rig is
  configured for prior-weighted and unposed mapping alike, with rig
  verification. Points seen by fewer than three images, or whose depth
  disagrees with the LiDAR cloud or the scaled depth prior by more than
  0.30 m, are dropped before training. The stage detail reports
  `pairs_rejected_by_prior`, `points_dropped_short_track` and
  `points_dropped_depth`.

### Removed

- The nerfstudio backend, the monocular-depth seed script and the random seed
  cloud; the placeholder reconstructor in release builds; the SLAM offload job
  kind.
- The `atlas.hud` pin settings (`rec_pin`, `mark_pin`).
- The `@mkkellogg/gaussian-splats-3d` splat viewer.

## [1.0.1]

### Changed

- **Perception offload holds its own advertise grant.** The drone's offload
  link advertisement now needs `vision.offload.advertise`, so the manifest
  requests it; without it the drone refuses the advertisement and reports no
  offload tier. Requires agent 0.102.0 or newer.
- **Offloaded detections arrive bound to the extension.** The agent files the
  returned batches under `com.altnautica.world-engine/offload` and drops their
  remote track ids, because the extension does not hold track designation.
- **Install-time payloads come from GitHub releases only.** The local bench
  pack (`scripts/pack-world-engine-local.sh`) and `scripts/pack-rust.sh` now
  match the agent's download allowlist, which no longer admits S3 or
  `localhost` sources.
- The agent half reads its data directory from the launch values the host
  delivers as a credential, not from the process environment.

## [1.0.0]

First release. World-model capture, reconstruction and perception offload
run as an extension, installed on the nodes that use them.

### Added

- **Atlas capture on drones.** A capture service selects keyframes from the
  shared vision frame bus, tags each with its pose (flight-controller pose,
  offloaded SLAM, or both), and streams them to a paired compute node over
  the best available bearer: direct LAN, the WFB relay through a ground
  station, or the cloud. Start, pause, resume and stop run from the GCS.
- **Compute node on workstations and compute nodes.** Keyframe ingest, a job
  scheduler, and reconstruction through the host's COLMAP, Brush, msplat or
  nerfstudio, with an optional monocular-depth seed. Artifacts are stored on
  the node, logged to a Rerun recording, and announced on a per-drone
  world-model stream. Each output names the backend that produced it.
- **Perception offload.** A drone can send its detection work to a compute
  node (`offload.*`); the compute node serves it (`serving.*`).
- **Ground-station relay** of capture events heard over the air to the
  compute node (`relay.*`).
- **Node credentials.** The GCS issues each paired drone and ground station a
  credential scoped to the compute node's lanes, installs it on that node,
  re-provisions when the compute node no longer honours it, and lets the
  operator revoke it.
- **GCS pages.** `world-model`, `world-model-setup` and `live-world` on
  drones; `offload` on drones, workstations and compute nodes; `atlas-relay`
  on ground stations; and the Compute overview and Jobs & outputs surfaces on
  workstations and compute nodes. Viewers for Gaussian splats, point clouds
  and Rerun recordings, with a badge on any placeholder reconstruction.
- **Installer toggle**, on by default for the `workstation` and `compute`
  profiles.
