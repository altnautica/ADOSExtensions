/**
 * Shared types for the Follow-Me GCS half.
 *
 * The iframe runs sandboxed with no access to the host's source, so the
 * event shapes the host streams in are mirrored here as plain serializable
 * interfaces.
 *
 * @license GPL-3.0-or-later
 */

/** Lock state words shared with the agent + vision contract. */
export type LockState = "locked" | "uncertain" | "lost";

/**
 * Why the loop is not commanding, as published by the agent.
 *
 * `commanding: false` alone cannot be read: a disarmed flight controller is
 * a normal pre-flight state, while telemetry that stopped arriving mid-follow
 * is a fault, and both look identical without this. `pose-stale` and
 * `fc-stale` are the fault readings.
 */
export type HoldReason =
  | "inactive"
  | "no-lock"
  | "lock-uncertain"
  | "lock-lost"
  | "pose-stale"
  | "fc-stale"
  | "fc-disarmed"
  | "fc-not-guided"
  | "no-ground-fix";

const HOLD_REASONS: readonly HoldReason[] = [
  "inactive",
  "no-lock",
  "lock-uncertain",
  "lock-lost",
  "pose-stale",
  "fc-stale",
  "fc-disarmed",
  "fc-not-guided",
  "no-ground-fix",
];

/** Hold reasons that mean an input the follow depends on stopped arriving,
 * rather than a normal not-flying-yet state. */
const STALE_HOLD_REASONS: readonly HoldReason[] = ["pose-stale", "fc-stale"];

/** Whether a hold reason represents a telemetry fault worth flagging. */
export function isStaleHold(reason: HoldReason | null): boolean {
  return reason != null && STALE_HOLD_REASONS.includes(reason);
}

/** The locale key for a hold reason label. */
export function holdReasonLabelKey(reason: HoldReason): string {
  const camel = reason.replace(/-([a-z])/g, (_m, c: string) =>
    c.toUpperCase(),
  );
  return `hold.${camel}`;
}

/** The topic the agent publishes its follow read-back on (must equal the
 * manifest skill state.topic). */
export const FOLLOW_STATE_TOPIC = "follow.state";

/** The follow read-back the agent publishes on FOLLOW_STATE_TOPIC. */
export interface FollowState {
  active: boolean;
  lockState: LockState | null;
  targetId: number | null;
  rangeM: number | null;
  distanceSetpointM: number | null;
  heightSetpointM: number | null;
  commanding: boolean;
  /** Flight-controller armed state, as of the last HEARTBEAT. False once
   * that heartbeat goes stale: a remembered arm state is not an observed one. */
  fcArmed: boolean;
  /** Flight controller in a guided/offboard mode that accepts setpoints. */
  fcGuided: boolean;
  /** Which gate is holding, when not commanding. Null while commanding. */
  holdReason: HoldReason | null;
}

/** The agent emits snake_case keys; normalize to the camelCase shape. */
export interface RawFollowState {
  active?: boolean;
  lock_state?: LockState | null;
  target_id?: number | null;
  range_m?: number | null;
  distance_setpoint_m?: number | null;
  height_setpoint_m?: number | null;
  commanding?: boolean;
  fc_armed?: boolean;
  fc_guided?: boolean;
  hold_reason?: HoldReason | string | null;
}

/** The per-drone config the settings tab writes through ctx.config. */
export interface FollowConfig {
  active: boolean;
  follow_distance_m: number;
  follow_height_m: number;
  gimbal_point: boolean;
  designate_camera: string;
  camera_hfov_deg: number;
  mount_pitch_deg: number;
}

export const DEFAULT_CONFIG: FollowConfig = {
  active: false,
  follow_distance_m: 8,
  follow_height_m: 4,
  gimbal_point: true,
  designate_camera: "uvc-0",
  camera_hfov_deg: 70,
  mount_pitch_deg: 30,
};

/** Empty follow state baseline before the first read-back arrives. */
export const EMPTY_FOLLOW_STATE: FollowState = {
  active: false,
  lockState: null,
  targetId: null,
  rangeM: null,
  distanceSetpointM: null,
  heightSetpointM: null,
  commanding: false,
  fcArmed: false,
  fcGuided: false,
  holdReason: null,
};

/** Map the agent's snake_case read-back onto the camelCase FollowState. */
export function normalizeFollowState(raw: RawFollowState | null): FollowState {
  if (!raw || typeof raw !== "object") return { ...EMPTY_FOLLOW_STATE };
  return {
    active: raw.active === true,
    lockState: normalizeLockState(raw.lock_state),
    targetId: typeof raw.target_id === "number" ? raw.target_id : null,
    rangeM: typeof raw.range_m === "number" ? raw.range_m : null,
    distanceSetpointM:
      typeof raw.distance_setpoint_m === "number"
        ? raw.distance_setpoint_m
        : null,
    heightSetpointM:
      typeof raw.height_setpoint_m === "number" ? raw.height_setpoint_m : null,
    commanding: raw.commanding === true,
    fcArmed: raw.fc_armed === true,
    fcGuided: raw.fc_guided === true,
    holdReason: normalizeHoldReason(raw.hold_reason),
  };
}

function normalizeLockState(v: unknown): LockState | null {
  if (v === "locked" || v === "uncertain" || v === "lost") return v;
  return null;
}

function normalizeHoldReason(v: unknown): HoldReason | null {
  // An unrecognized reason is dropped rather than rendered raw: a key the
  // locale has no label for would surface as an untranslated token, and a
  // wrong-looking label is worse than none on a status surface.
  return HOLD_REASONS.includes(v as HoldReason) ? (v as HoldReason) : null;
}
