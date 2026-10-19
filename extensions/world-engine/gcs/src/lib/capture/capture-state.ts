/**
 * @module capture/capture-state
 * @description The capture service's live state (`GET atlas/capture/state`)
 * and its top-down coverage (`GET atlas/capture/coverage`), coerced to
 * camelCase. Health ids are exactly the six the agent reports; an unknown id
 * is dropped rather than shown as a chip with no meaning.
 */

import { arr, bool, num, obj, objOrNull, str, vec } from "./coerce";

/** The six capture health chips, in display order. */
export const HEALTH_IDS = ["blur", "texture", "flicker", "corridor", "storage", "clock"] as const;
export type HealthId = (typeof HEALTH_IDS)[number];
export type HealthLevel = "ok" | "warn" | "bad";

export interface HealthSignal {
  id: HealthId;
  level: HealthLevel;
  value: number | null;
}

export type CaptureLifecycle = "idle" | "recording" | "paused" | "closing" | (string & {});
export type ExposureMode = "walking" | "slow" | "short";

export interface CameraFrames {
  id: string;
  fps: number | null;
  dropped: number | null;
}

export interface ExposureState {
  locked: boolean;
  exposureUs: number | null;
  gain: number | null;
  mode: ExposureMode | null;
}

export interface CaptureState {
  state: CaptureLifecycle;
  sessionId: string | null;
  durationS: number | null;
  site: string | null;
  floor: number;
  room: string | null;
  segment: number | null;
  frames: { total: number | null; droppedPct: number | null; perCamera: CameraFrames[] };
  pose: { source: string | null; health: string | null; trackedFeatures: number | null; driftHintM: number | null };
  coverage: { voxels: number | null; roomPct: number | null; loopClosed: boolean | null; distanceFromStartM: number | null };
  health: HealthSignal[];
  exposure: ExposureState;
  storage: { freeBytes: number | null; estRemainingS: number | null };
}

function healthLevel(v: unknown): HealthLevel | null {
  return v === "ok" || v === "warn" || v === "bad" ? v : null;
}

function exposureMode(v: unknown): ExposureMode | null {
  return v === "walking" || v === "slow" || v === "short" ? v : null;
}

/** Health signals in {@link HEALTH_IDS} order; unknown ids and levels dropped. */
export function coerceHealth(raw: unknown): HealthSignal[] {
  const byId: Partial<Record<HealthId, HealthSignal>> = {};
  for (const item of arr(raw)) {
    const e = obj(item);
    const id = HEALTH_IDS.find((h) => h === e.id);
    const level = healthLevel(e.level);
    if (id && level) byId[id] = { id, level, value: num(e.value) };
  }
  return HEALTH_IDS.flatMap((id) => (byId[id] ? [byId[id]] : []));
}

export function coerceExposure(raw: unknown): ExposureState {
  const e = obj(raw);
  return {
    locked: bool(e.locked) === true,
    exposureUs: num(e.exposure_us),
    gain: num(e.gain),
    mode: exposureMode(e.mode),
  };
}

/** Coerce a `GET atlas/capture/state` body, or null when it is not an object. */
export function coerceCaptureState(raw: unknown): CaptureState | null {
  const e = objOrNull(raw);
  if (!e || typeof e.state !== "string") return null;
  const frames = obj(e.frames);
  const pose = obj(e.pose);
  const coverage = obj(e.coverage);
  const storage = obj(e.storage);
  return {
    state: e.state,
    sessionId: str(e.session_id),
    durationS: num(e.duration_s),
    site: str(e.site),
    floor: num(e.floor) ?? 0,
    room: str(e.room),
    segment: num(e.segment),
    frames: {
      total: num(frames.total),
      droppedPct: num(frames.dropped_pct),
      perCamera: arr(frames.per_camera).flatMap((c) => {
        const cam = obj(c);
        const id = str(cam.id);
        return id ? [{ id, fps: num(cam.fps), dropped: num(cam.dropped) }] : [];
      }),
    },
    pose: {
      source: str(pose.source),
      health: str(pose.health),
      trackedFeatures: num(pose.tracked_features),
      driftHintM: num(pose.drift_hint_m),
    },
    coverage: {
      voxels: num(coverage.voxels),
      roomPct: num(coverage.room_pct),
      loopClosed: bool(coverage.loop_closed),
      distanceFromStartM: num(coverage.distance_from_start_m),
    },
    health: coerceHealth(e.health),
    exposure: coerceExposure(e.exposure),
    storage: { freeBytes: num(storage.free_bytes), estRemainingS: num(storage.est_remaining_s) },
  };
}

/** One top-down coverage cell: grid index and frustum hit count. */
export interface CoverageCell {
  ix: number;
  iy: number;
  hits: number;
}

/** The capture's top-down coverage and walked trajectory, local ENU metres. */
export interface CaptureCoverage {
  cellM: number;
  cells: CoverageCell[];
  trajectory: [number, number, number][];
  start: [number, number, number] | null;
  current: [number, number, number] | null;
  headingDeg: number | null;
}

/** Coerce a `GET atlas/capture/coverage` body, or null when it carries no grid. */
export function coerceCoverage(raw: unknown): CaptureCoverage | null {
  const e = objOrNull(raw);
  const cellM = num(e?.cell_m);
  if (!e || cellM === null || cellM <= 0) return null;
  return {
    cellM,
    cells: arr(e.cells).flatMap((c) => {
      const t = Array.isArray(c) ? c : [];
      const [ix, iy, hits] = [num(t[0]), num(t[1]), num(t[2])];
      return ix !== null && iy !== null && hits !== null ? [{ ix, iy, hits }] : [];
    }),
    trajectory: arr(e.trajectory).flatMap((p) => {
      const v = vec(p, 3);
      return v ? [v] : [];
    }),
    start: vec(e.start, 3),
    current: vec(e.current, 3),
    headingDeg: num(e.heading_deg),
  };
}

/** After this long recording the HUD offers the way back to the start. */
export const RETURN_PROMPT_AFTER_S = 180;
/** Within this distance of the start a loop closure is offered. */
export const LOOP_CLOSE_RADIUS_M = 2;

/** Compass bearing (degrees clockwise from north, 0..360) from `from` to `to`
 * in ENU metres. */
export function bearingDeg(from: readonly number[], to: readonly number[]): number {
  const deg = (Math.atan2((to[0] ?? 0) - (from[0] ?? 0), (to[1] ?? 0) - (from[1] ?? 0)) * 180) / Math.PI;
  return (deg + 360) % 360;
}

export type ReturnPrompt =
  | { kind: "none" }
  | {
      kind: "return";
      distanceM: number;
      /** Compass bearing to the start, or null without a position fix. */
      bearingDeg: number | null;
      /** Bearing relative to the operator's heading (−180..180, + = right). */
      relativeDeg: number | null;
    }
  | { kind: "close_loop" };

/** What the "return to start" prompt says now: nothing, the distance and
 * bearing back (from the coverage's current position and heading), or that
 * the walk is close enough to close the loop. */
export function returnToStartPrompt(state: CaptureState, coverage: CaptureCoverage | null = null): ReturnPrompt {
  const d = state.coverage.distanceFromStartM;
  if (state.state !== "recording" || d === null || (state.durationS ?? 0) < RETURN_PROMPT_AFTER_S) {
    return { kind: "none" };
  }
  if (state.coverage.loopClosed === true) return { kind: "none" };
  if (d <= LOOP_CLOSE_RADIUS_M) return { kind: "close_loop" };
  const bearing = coverage?.current && coverage.start ? bearingDeg(coverage.current, coverage.start) : null;
  const heading = coverage?.headingDeg ?? null;
  const relativeDeg = bearing !== null && heading !== null ? ((bearing - heading + 540) % 360) - 180 : null;
  return { kind: "return", distanceM: d, bearingDeg: bearing, relativeDeg };
}
