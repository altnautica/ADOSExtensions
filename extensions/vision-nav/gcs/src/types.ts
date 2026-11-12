/**
 * Shared types for the vision-nav GCS plugin. The shape of
 * VisionNavTelemetry mirrors the agent heartbeat's navigation
 * capability block: optical flow stats, the companion process state,
 * the selected estimator mode, and the camera + IMU + calibration
 * health the estimator reports. The agent emits this as a single
 * object on the "navigation" telemetry topic.
 */

export type EstimatorMode =
  | "off"
  | "optical_flow"
  | "optical_flow_degraded"
  | "external_nav";

export type EstimatorState =
  | "off"
  | "init"
  | "converging"
  | "converged"
  | "degraded"
  | "failed";

export type ScaleSource = "rangefinder" | "baro" | "gps" | "vision" | "static";

export type ImuSourceId =
  | "mavlink-raw-imu"
  | "mavlink-scaled-imu2"
  | "direct-i2c"
  | "direct-dronecan";

export interface VisionNavTelemetry {
  flowQuality?: number | null;
  flowRateHz?: number | null;
  flowDistanceM?: number | null;
  companionState?: "active" | "critical" | "terminating" | "inactive";
  opticalFlowSupported: boolean;
  /**
   * Where the rangefinder is wired. Mirrors the agent's
   * ``RangefinderConfig.topology`` field. The cloud relay validator
   * accepts ``"companion" | "fc" | "both" | null`` so the GCS keeps
   * the same union.
   */
  rangefinderTopology?: "companion" | "fc" | "both" | null;
  /**
   * The camera the flow reads: the configured ``camera.camera_id`` when
   * set, else the configured device path (the flow then takes every
   * camera). Surfaced verbatim in the GCS so the operator can confirm it.
   */
  recommendedCameraId?: string | null;
  /**
   * Currently-selected estimator key. Mirrors the agent ``mode``
   * config field. Free-form string so a new estimator can land on the
   * agent without a GCS release.
   */
  mode?: EstimatorMode | string;
  /** Estimator keys the running plugin instance can actually run. */
  availableEstimators?: string[];
  /** Estimator state machine, mirrored from the BaseEstimator contract. */
  estimatorState?: EstimatorState | string;
  /** Where the optical-flow scale comes from this tick. */
  flowScaleSource?: ScaleSource | null;
  /** Which IMU source the estimator is currently consuming. */
  imuSource?: ImuSourceId | string;
  /** IMU sample rate in Hz, smoothed by the source's EMA. */
  imuRateHz?: number | null;
  /** Whether camera intrinsics have been loaded from calibration. */
  cameraIntrinsicsLoaded?: boolean;
  /** Rolling residual between camera frames and IMU samples in ms. */
  cameraImuSyncOffsetMs?: number | null;
  /** Aggregate pre-arm report from the gate. The GCS pre-arm card
   * renders each check with its severity and detail string. */
  preArmReport?: PreArmReport;
  /** MAVLink sends that failed since the plugin started. */
  emitErrors?: number;
  /** Why the companion rangefinder cannot read, or ``null``. */
  rangefinderError?: string | null;
  /** Inputs the plugin could not wire, e.g.
   * ``mavlink_subscribe_denied:RAW_IMU``. */
  degradedReasons?: string[];
  /** Watched flight-controller parameters the FC has reported
   * (``EKF2_OF_CTRL``, ``EKF2_AID_MASK``). */
  fcParams?: string[];
  /** External nav: median age (ms) of a pose when its ODOMETRY is sent,
   * or ``null`` before the first one. */
  poseAgeMs?: number | null;
}

export type PreArmCheckSeverity = "ok" | "pending" | "blocking";

export interface PreArmCheck {
  id: string;
  severity: PreArmCheckSeverity;
  detail: string;
}

export interface PreArmReport {
  mode: string;
  armable: boolean;
  checks: PreArmCheck[];
}

export type FirmwareType = "ardupilot" | "px4" | "betaflight" | "inav";

/** The event topic the agent publishes the navigation feed state on. */
export const NAV_STATE_TOPIC = "nav.state";

export type ExtnavHealth = "good" | "degraded" | "lost";

/** The `nav.state` payload as the agent sends it (snake_case). */
export interface RawNavState {
  mode?: string;
  active_set?: number | null;
  extnav_health?: ExtnavHealth | null;
  odom_rate_hz?: number | null;
  last_reset_counter?: number | null;
  obstacle_rate_hz?: number | null;
  min_obstacle_m?: number | null;
  gps_fix?: number | null;
}

/** What the navigation feed is doing now. Absent fields are null. */
export interface NavState {
  mode: string | null;
  activeSet: number | null;
  extnavHealth: ExtnavHealth | null;
  odomRateHz: number | null;
  lastResetCounter: number | null;
  obstacleRateHz: number | null;
  minObstacleM: number | null;
  gpsFix: number | null;
}

export const EMPTY_NAV_STATE: NavState = {
  mode: null,
  activeSet: null,
  extnavHealth: null,
  odomRateHz: null,
  lastResetCounter: null,
  obstacleRateHz: null,
  minObstacleM: null,
  gpsFix: null,
};

function num(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

export function normalizeNavState(raw: RawNavState | null | undefined): NavState {
  if (!raw) return EMPTY_NAV_STATE;
  const health = raw.extnav_health;
  return {
    mode: typeof raw.mode === "string" ? raw.mode : null,
    activeSet: num(raw.active_set),
    extnavHealth:
      health === "good" || health === "degraded" || health === "lost" ? health : null,
    odomRateHz: num(raw.odom_rate_hz),
    lastResetCounter: num(raw.last_reset_counter),
    obstacleRateHz: num(raw.obstacle_rate_hz),
    minObstacleM: num(raw.min_obstacle_m),
    gpsFix: num(raw.gps_fix),
  };
}

/** The event topic the agent publishes the engage state on, each second. */
export const ENGAGE_TOPIC = "navigation.engage";

/** The `navigation.engage` payload as the agent sends it. */
export interface RawEngage {
  state?: string;
}

/** Engaged (`true`), idle (`false`), or a payload the tab does not know (`null`). */
export function normalizeEngage(raw: RawEngage | null | undefined): boolean | null {
  if (raw?.state === "active") return true;
  if (raw?.state === "idle") return false;
  return null;
}
