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

/** The follow phase the agent publishes. Once the lock is lost the follow
 * predicts, repositions to a vantage point and searches, then holds for the
 * operator. */
export type FollowPhase =
  | "idle"
  | "acquiring"
  | "following"
  | "holding"
  | "predicting"
  | "repositioning"
  | "searching"
  | "holding-await-operator";

const PHASES: readonly FollowPhase[] = [
  "idle",
  "acquiring",
  "following",
  "holding",
  "predicting",
  "repositioning",
  "searching",
  "holding-await-operator",
];

/** The phases in which the follow is looking for a lost subject. */
export const RECOVERY_PHASES: readonly FollowPhase[] = [
  "predicting",
  "repositioning",
  "searching",
];

/** What the follow does when the lock on the subject is lost. */
export type ReacquireMode = "strict" | "smart";

/** What an uncertain lock does: keep following the prediction, or hold. */
export type UncertainMode = "follow" | "hold";

/** The pilot's report on the vantage point it was asked for. */
export type VantageState = "enroute" | "arrived" | "no-vantage";

/** What hides the subject from the vantage point: a trunk or column
 * (`narrow`), or a wall or corner (`wide`). */
export interface Occluder {
  ne: [number, number];
  kind: "narrow" | "wide";
}

/** The predicted subject's spread, as an ellipse on the ground. */
export interface SearchArea {
  centerNe: [number, number];
  semiMajorM: number;
  semiMinorM: number;
  /** Bearing of the major axis from north. */
  headingDeg: number;
}

/** Where the height above ground came from: the downward rangefinder or the
 * barometric height. */
export type AglSource = "rng" | "baro";

/** Where the follow camera geometry came from: the camera roster carried on
 * the detection batches, the operator's settings (an agent that does not
 * report the roster), or the built-in reference camera. */
export type CameraGeometrySource = "roster" | "settings" | "default";

export type RangeCueKind = "ground" | "box_height" | "lidar";

/** One range cue the agent fused into the subject range. */
export interface RangeCue {
  kind: RangeCueKind;
  rangeM: number;
  sigmaM: number;
}

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
  | "awaiting-operator"
  | "search-exhausted"
  | "predictive-search-off"
  | "pose-stale"
  | "fc-stale"
  | "fc-disarmed"
  | "no-range"
  | "lease-busy"
  | "lease-refused"
  | "camera-lost";

const HOLD_REASONS: readonly HoldReason[] = [
  "inactive",
  "no-lock",
  "lock-uncertain",
  "awaiting-operator",
  "search-exhausted",
  "predictive-search-off",
  "pose-stale",
  "fc-stale",
  "fc-disarmed",
  "no-range",
  "lease-busy",
  "lease-refused",
  "camera-lost",
];

/** Follow at a distance and an angle, or orbit the subject. */
export type FollowMode = "follow" | "orbit";
export type OrbitDirection = "cw" | "ccw";

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
  phase: FollowPhase;
  lockState: LockState | null;
  targetId: number | null;
  /** Detector confidence of the locked subject's latest box. */
  confidence: number | null;
  /** Horizontal range to the subject and its 1-sigma uncertainty. */
  rangeM: number | null;
  rangeSigmaM: number | null;
  rangeCues: RangeCue[];
  targetSpeedMps: number | null;
  aglM: number | null;
  aglSource: AglSource | null;
  distanceSetpointM: number | null;
  heightSetpointM: number | null;
  reacquireMode: ReacquireMode;
  uncertainMode: UncertainMode;
  /** Whether a smart loss searches from a vantage point. */
  predictiveSearch: boolean;
  /** Seconds left of the recovery's search budget; 0 outside a recovery. */
  reacquireRemainingS: number;
  searchBudgetS: number;
  /** 0-based widening step while searching, else null. */
  searchStep: number | null;
  searchArea: SearchArea | null;
  /** At most 10 points 0.5 s apart while recovering, else null. */
  predictedPathNe: [number, number][] | null;
  vantageNe: [number, number] | null;
  vantageState: VantageState | null;
  occluder: Occluder | null;
  /** The latched hover point while holding for the operator. */
  holdNe: [number, number] | null;
  /** The resume number the follow has seen; a surface writes this + 1. */
  operatorResumeSeq: number;
  /** Detection frame to goal send, milliseconds. */
  pipelineLatencyMs: number | null;
  /** Pilot goals sent per second over the last 2 s. */
  setpointRateHz: number;
  commanding: boolean;
  /** Flight-controller armed state, as of the last HEARTBEAT. False once
   * that heartbeat goes stale: a remembered arm state is not an observed one. */
  fcArmed: boolean;
  /** Flight controller in a guided/offboard mode that accepts setpoints. */
  fcGuided: boolean;
  /** Which gate is holding, when not commanding. Null while commanding. */
  holdReason: HoldReason | null;
  followMode: FollowMode;
  followAngleDeg: number;
  orbitRadiusM: number;
  orbitRateDps: number;
  orbitDirection: OrbitDirection;
  /** Bearing of the vehicle's ring point from the subject; null outside an orbit. */
  orbitPhaseDeg: number | null;
  /** The pilot motion lease the follow holds (the pilot issues an integer),
   * or null. Shown only as held / none, never as the raw id. */
  leaseId: number | string | null;
  /** The operator nudged the parameters with the sticks this session. */
  nudged: boolean;
  /** The follow camera's horizontal field of view in use, degrees. */
  cameraHfovDeg?: number | null;
  /** The follow camera's mount pitch in use, degrees, negative below the
   * nose (the camera roster's sign). */
  cameraMountPitchDeg?: number | null;
  /** Where that geometry came from: the camera roster, the settings or the
   * built-in default. */
  cameraGeometrySource?: CameraGeometrySource | null;
}

/** The agent emits snake_case keys; normalize to the camelCase shape. */
export interface RawFollowState {
  active?: boolean;
  phase?: FollowPhase | string;
  lock_state?: LockState | null;
  target_id?: number | null;
  confidence?: number | null;
  range_m?: number | null;
  range_sigma_m?: number | null;
  range_cues?: { kind?: string; range_m?: number; sigma_m?: number }[];
  target_speed_mps?: number | null;
  agl_m?: number | null;
  agl_source?: AglSource | string | null;
  distance_setpoint_m?: number | null;
  height_setpoint_m?: number | null;
  reacquire_mode?: ReacquireMode | string;
  uncertain_mode?: string;
  predictive_search?: boolean;
  reacquire_remaining_s?: number;
  search_budget_s?: number;
  search_step?: number | null;
  search_area?: {
    center_ne?: unknown;
    semi_major_m?: number;
    semi_minor_m?: number;
    heading_deg?: number;
  } | null;
  predicted_path_ne?: unknown[] | null;
  vantage_ne?: unknown;
  vantage_state?: string | null;
  occluder?: { ne?: unknown; kind?: string } | null;
  hold_ne?: unknown;
  operator_resume_seq?: number;
  pipeline_latency_ms?: number | null;
  setpoint_rate_hz?: number;
  commanding?: boolean;
  fc_armed?: boolean;
  fc_guided?: boolean;
  hold_reason?: HoldReason | string | null;
  follow_mode?: string;
  follow_angle_deg?: number;
  orbit_radius_m?: number;
  orbit_rate_dps?: number;
  orbit_direction?: string;
  orbit_phase_deg?: number | null;
  lease_id?: number | string | null;
  /** Agent wall-clock time of the report, when the host forwards it. */
  ts_ms?: number;
  nudged?: boolean;
  camera_hfov_deg?: number | null;
  camera_mount_pitch_deg?: number | null;
  camera_geometry_source?: CameraGeometrySource | string | null;
}

/** The per-drone config the tab reads through ctx.config. */
export interface FollowConfig {
  active: boolean;
  follow_distance_m: number;
  follow_height_m: number;
  gimbal_point: boolean;
  designate_camera: string;
  reacquire_mode: ReacquireMode;
  reacquire_min_similarity: number;
  uncertain_mode: UncertainMode;
  predictive_search: boolean;
  search_budget_s: number;
  search_radius_m: number;
  search_speed_mps: number;
  max_speed_mps: number;
  follow_mode: FollowMode;
  follow_angle_deg: number;
  orbit_radius_m: number;
  orbit_rate_dps: number;
  orbit_direction: OrbitDirection;
  obstacle_margin_m: number;
}

export const DEFAULT_CONFIG: FollowConfig = {
  active: false,
  follow_distance_m: 6,
  follow_height_m: 4,
  gimbal_point: true,
  designate_camera: "auto",
  reacquire_mode: "smart",
  reacquire_min_similarity: 0.75,
  uncertain_mode: "follow",
  predictive_search: true,
  search_budget_s: 25,
  search_radius_m: 12,
  search_speed_mps: 2,
  max_speed_mps: 6,
  follow_mode: "follow",
  follow_angle_deg: 180,
  orbit_radius_m: 8,
  orbit_rate_dps: 10,
  orbit_direction: "cw",
  obstacle_margin_m: 1.5,
};

/** Empty follow state baseline before the first read-back arrives. */
export const EMPTY_FOLLOW_STATE: FollowState = {
  active: false,
  phase: "idle",
  lockState: null,
  targetId: null,
  confidence: null,
  rangeM: null,
  rangeSigmaM: null,
  rangeCues: [],
  targetSpeedMps: null,
  aglM: null,
  aglSource: null,
  distanceSetpointM: null,
  heightSetpointM: null,
  reacquireMode: "smart",
  uncertainMode: "follow",
  predictiveSearch: true,
  reacquireRemainingS: 0,
  searchBudgetS: 25,
  searchStep: null,
  searchArea: null,
  predictedPathNe: null,
  vantageNe: null,
  vantageState: null,
  occluder: null,
  holdNe: null,
  operatorResumeSeq: 0,
  pipelineLatencyMs: null,
  setpointRateHz: 0,
  commanding: false,
  fcArmed: false,
  fcGuided: false,
  holdReason: null,
  followMode: "follow",
  followAngleDeg: 180,
  orbitRadiusM: 8,
  orbitRateDps: 10,
  orbitDirection: "cw",
  orbitPhaseDeg: null,
  leaseId: null,
  nudged: false,
  cameraHfovDeg: null,
  cameraMountPitchDeg: null,
  cameraGeometrySource: null,
};

/** Map the agent's snake_case read-back onto the camelCase FollowState. */
export function normalizeFollowState(raw: RawFollowState | null): FollowState {
  if (!raw || typeof raw !== "object") return { ...EMPTY_FOLLOW_STATE };
  return {
    active: raw.active === true,
    phase: PHASES.includes(raw.phase as FollowPhase)
      ? (raw.phase as FollowPhase)
      : "idle",
    lockState: normalizeLockState(raw.lock_state),
    targetId: num(raw.target_id),
    confidence: num(raw.confidence),
    rangeM: num(raw.range_m),
    rangeSigmaM: num(raw.range_sigma_m),
    rangeCues: normalizeCues(raw.range_cues),
    targetSpeedMps: num(raw.target_speed_mps),
    aglM: num(raw.agl_m),
    aglSource:
      raw.agl_source === "rng" || raw.agl_source === "baro"
        ? raw.agl_source
        : null,
    distanceSetpointM: num(raw.distance_setpoint_m),
    heightSetpointM: num(raw.height_setpoint_m),
    reacquireMode: normalizeReacquireMode(raw.reacquire_mode),
    uncertainMode: normalizeUncertainMode(raw.uncertain_mode),
    predictiveSearch: raw.predictive_search === true,
    reacquireRemainingS: num(raw.reacquire_remaining_s) ?? 0,
    searchBudgetS: num(raw.search_budget_s) ?? EMPTY_FOLLOW_STATE.searchBudgetS,
    searchStep: num(raw.search_step),
    searchArea: normalizeSearchArea(raw.search_area),
    predictedPathNe: normalizePath(raw.predicted_path_ne),
    vantageNe: pair(raw.vantage_ne),
    vantageState:
      raw.vantage_state === "enroute" ||
      raw.vantage_state === "arrived" ||
      raw.vantage_state === "no-vantage"
        ? raw.vantage_state
        : null,
    occluder: normalizeOccluder(raw.occluder),
    holdNe: pair(raw.hold_ne),
    operatorResumeSeq: num(raw.operator_resume_seq) ?? 0,
    pipelineLatencyMs: num(raw.pipeline_latency_ms),
    setpointRateHz: num(raw.setpoint_rate_hz) ?? 0,
    commanding: raw.commanding === true,
    fcArmed: raw.fc_armed === true,
    fcGuided: raw.fc_guided === true,
    holdReason: normalizeHoldReason(raw.hold_reason),
    followMode: normalizeFollowMode(raw.follow_mode),
    followAngleDeg: num(raw.follow_angle_deg) ?? 180,
    orbitRadiusM: num(raw.orbit_radius_m) ?? 8,
    orbitRateDps: num(raw.orbit_rate_dps) ?? 10,
    orbitDirection: normalizeOrbitDirection(raw.orbit_direction),
    orbitPhaseDeg: num(raw.orbit_phase_deg),
    leaseId:
      (typeof raw.lease_id === "number" && Number.isFinite(raw.lease_id)) ||
      (typeof raw.lease_id === "string" && raw.lease_id)
        ? (raw.lease_id ?? null)
        : null,
    nudged: raw.nudged === true,
    cameraHfovDeg: num(raw.camera_hfov_deg),
    cameraMountPitchDeg: num(raw.camera_mount_pitch_deg),
    cameraGeometrySource:
      raw.camera_geometry_source === "roster" ||
      raw.camera_geometry_source === "settings" ||
      raw.camera_geometry_source === "default"
        ? raw.camera_geometry_source
        : null,
  };
}

/** Anything but exactly "orbit" is the follow mode (the agent's rule). */
export function normalizeFollowMode(v: unknown): FollowMode {
  return v === "orbit" ? "orbit" : "follow";
}

/** Anything but exactly "ccw" is clockwise (the agent's rule). */
export function normalizeOrbitDirection(v: unknown): OrbitDirection {
  return v === "ccw" ? "ccw" : "cw";
}

/** "Follow 180° · 8 m" or "Orbit 8 m · 10°/s · Clockwise", built from the
 * locale so the mode and direction words are translated. */
export function modeSummary(
  f: FollowState,
  distanceM: number | null,
  t: (key: string, params?: Record<string, string | number>) => string,
): string {
  const mode = t(`follow_mode.${f.followMode}`);
  if (f.followMode === "orbit") {
    return t("mode.orbitSummary", {
      mode,
      radius: t("unit.m", { value: f.orbitRadiusM }),
      rate: t("unit.dps", { value: f.orbitRateDps }),
      direction: t(`orbit_direction.${f.orbitDirection}`),
    });
  }
  const angle = t("unit.deg", { value: Math.round(f.followAngleDeg) });
  if (distanceM == null) return t("mode.followSummaryNoDistance", { mode, angle });
  return t("mode.followSummary", {
    mode,
    angle,
    distance: t("unit.m", { value: distanceM }),
  });
}

/** Strict unless the value is exactly "smart": an unknown mode must never
 * read as the mode that re-designates on its own. */
export function normalizeReacquireMode(v: unknown): ReacquireMode {
  return v === "smart" ? "smart" : "strict";
}

/** Follow only when the value is exactly "follow": an unknown mode reads as
 * the conservative hold (the agent's rule). */
export function normalizeUncertainMode(v: unknown): UncertainMode {
  return v === "follow" ? "follow" : "hold";
}

function num(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}

function pair(v: unknown): [number, number] | null {
  if (!Array.isArray(v) || v.length !== 2) return null;
  const [a, b] = [num(v[0]), num(v[1])];
  return a === null || b === null ? null : [a, b];
}

function normalizePath(v: unknown): [number, number][] | null {
  if (!Array.isArray(v)) return null;
  const out = v.map(pair).filter((p): p is [number, number] => p !== null);
  return out.length > 0 ? out : null;
}

function normalizeSearchArea(v: RawFollowState["search_area"]): SearchArea | null {
  const center = pair(v?.center_ne);
  const major = num(v?.semi_major_m);
  const minor = num(v?.semi_minor_m);
  const heading = num(v?.heading_deg);
  if (center === null || major === null || minor === null || heading === null) return null;
  return { centerNe: center, semiMajorM: major, semiMinorM: minor, headingDeg: heading };
}

function normalizeOccluder(v: RawFollowState["occluder"]): Occluder | null {
  const ne = pair(v?.ne);
  if (ne === null || (v?.kind !== "narrow" && v?.kind !== "wide")) return null;
  return { ne, kind: v.kind };
}

function normalizeCues(v: RawFollowState["range_cues"]): RangeCue[] {
  if (!Array.isArray(v)) return [];
  const out: RangeCue[] = [];
  for (const c of v) {
    if (
      (c?.kind === "ground" || c?.kind === "box_height" || c?.kind === "lidar") &&
      typeof c.range_m === "number" &&
      typeof c.sigma_m === "number"
    ) {
      out.push({ kind: c.kind, rangeM: c.range_m, sigmaM: c.sigma_m });
    }
  }
  return out;
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
