/**
 * Shared types for the vision-nav GCS plugin. The shape of
 * VisionNavTelemetry mirrors the agent heartbeat's navigation
 * capability block: optical flow stats, optional VIO stats, the
 * companion process state, the selected estimator mode, and the
 * camera + IMU + calibration health every estimator now reports.
 * The agent emits this as a single object on the "navigation"
 * telemetry topic.
 */

export type EstimatorMode =
  | "off"
  | "optical_flow"
  | "optical_flow_degraded"
  | "vio_openvins"
  | "vio_vins_fusion"
  | "hybrid_of_plus_vio";

export type EstimatorState =
  | "off"
  | "init"
  | "converging"
  | "converged"
  | "degraded"
  | "failed";

export type ScaleSource = "rangefinder" | "baro" | "gps" | "vision";

export type ImuSourceId =
  | "mavlink-raw-imu"
  | "mavlink-scaled-imu2"
  | "direct-i2c"
  | "direct-dronecan";

export interface VisionNavTelemetry {
  flowQuality?: number;
  flowRateHz?: number;
  flowDistanceM?: number | null;
  vioState?: "active" | "degraded" | "lost" | "absent";
  vioResetCounter?: number;
  vioQuality?: number;
  companionState?: "active" | "critical" | "terminating" | "inactive";
  opticalFlowSupported: boolean;
  vioSupported: boolean;
  /**
   * Where the rangefinder is wired. Mirrors the agent's
   * ``RangefinderConfig.topology`` field. The cloud relay validator
   * accepts ``"companion" | "fc" | "both" | null`` so the GCS keeps
   * the same union.
   */
  rangefinderTopology?: "companion" | "fc" | "both" | null;
  /**
   * Camera device id the agent suggests as the primary capture source.
   * Surfaced verbatim in the GCS so the operator can confirm the path
   * the agent picked.
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
  /** Features tracked by the VIO estimator. ``null`` for non-VIO. */
  estimatorFeatureCount?: number;
  /** Rolling drift estimate from the VIO estimator, in metres. */
  estimatorDriftEstimateM?: number;
  /** Which IMU source the estimator is currently consuming. */
  imuSource?: ImuSourceId | string;
  /** IMU sample rate in Hz, smoothed by the source's EMA. */
  imuRateHz?: number;
  /** Whether camera intrinsics have been loaded from calibration. */
  cameraIntrinsicsLoaded?: boolean;
  /** Rolling residual between camera frames and IMU samples in ms. */
  cameraImuSyncOffsetMs?: number;
  /** Aggregate pre-arm report from the gate. The GCS pre-arm card
   * renders each check with its severity and detail string. */
  preArmReport?: PreArmReport;
  /** Mode the agent's auto-detect pass recommends for the host's
   * current hardware. The GCS ModeCard renders this as the default
   * selection when the operator has not picked one explicitly. */
  suggestedMode?: string;
  /** One-line explanation of why the suggestion was picked. Surfaced
   * on the ModeCard tooltip. */
  suggestedModeReason?: string;
  /** Camera direction the agent recommends for the suggested mode.
   * Only meaningful for VIO modes; optical-flow modes are always
   * downward. ``"auto"`` means the wizard should let the operator
   * pick (or defer to HAL board metadata). */
  recommendedCameraOrientation?: "forward" | "downward" | "side" | "auto";
  /** Number of cameras the auto-detect pass found on the host. */
  detectedCameraCount?: number;
  /** Driver name of the rangefinder the auto-detect pass selected,
   * or ``null`` when none was detected and the plugin fell back to
   * FC relay or the baro/GPS scale ladder. */
  detectedRangefinderDriver?: string | null;
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

export type EkfSourceSet = 1 | 2 | 3;

export interface EkfSourceOption {
  set: EkfSourceSet;
  label: string;
  description: string;
}
