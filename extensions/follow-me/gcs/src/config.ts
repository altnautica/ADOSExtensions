/**
 * Per-drone config read for the detail tab.
 *
 * The follow settings are edited through the host's native parameter
 * controls (declared as `contributes.parameters` in the manifest), not from
 * this iframe. The tab only READS the current config to show the live camera
 * in its Specs section; the SDK's `ctx.config.onChange` delivers it.
 *
 * @license GPL-3.0-or-later
 */

import {
  DEFAULT_CONFIG,
  normalizeFollowMode,
  normalizeOrbitDirection,
  normalizeReacquireMode,
  type FollowConfig,
} from "./types";

/** Coerce a raw config payload into the typed FollowConfig with defaults. */
export function normalizeConfig(raw: unknown): FollowConfig {
  const r = (raw ?? {}) as Partial<Record<keyof FollowConfig, unknown>>;
  return {
    active: r.active === true,
    follow_distance_m: numberOr(r.follow_distance_m, DEFAULT_CONFIG.follow_distance_m),
    follow_height_m: numberOr(r.follow_height_m, DEFAULT_CONFIG.follow_height_m),
    gimbal_point: r.gimbal_point === undefined
      ? DEFAULT_CONFIG.gimbal_point
      : r.gimbal_point === true,
    designate_camera:
      typeof r.designate_camera === "string" && r.designate_camera
        ? r.designate_camera
        : DEFAULT_CONFIG.designate_camera,
    reacquire_mode:
      r.reacquire_mode === undefined
        ? DEFAULT_CONFIG.reacquire_mode
        : normalizeReacquireMode(r.reacquire_mode),
    reacquire_window_s: numberOr(r.reacquire_window_s, DEFAULT_CONFIG.reacquire_window_s),
    reacquire_min_similarity: numberOr(
      r.reacquire_min_similarity,
      DEFAULT_CONFIG.reacquire_min_similarity,
    ),
    max_speed_mps: numberOr(r.max_speed_mps, DEFAULT_CONFIG.max_speed_mps),
    follow_mode: normalizeFollowMode(r.follow_mode),
    follow_angle_deg: numberOr(r.follow_angle_deg, DEFAULT_CONFIG.follow_angle_deg),
    orbit_radius_m: numberOr(r.orbit_radius_m, DEFAULT_CONFIG.orbit_radius_m),
    orbit_rate_dps: numberOr(r.orbit_rate_dps, DEFAULT_CONFIG.orbit_rate_dps),
    orbit_direction: normalizeOrbitDirection(r.orbit_direction),
    obstacle_margin_m: numberOr(r.obstacle_margin_m, DEFAULT_CONFIG.obstacle_margin_m),
  };
}

function numberOr(v: unknown, fallback: number): number {
  const n = typeof v === "string" ? Number(v) : v;
  return typeof n === "number" && Number.isFinite(n) ? n : fallback;
}

/** Host event carrying the node's camera roster: `{cameras: [{id, name|null}]}`. */
export const CAMERAS_CHANGED_TOPIC = "cameras.changed";

/** Map camera id to display name; entries without a usable name are left out. */
export function cameraNameMap(raw: unknown): ReadonlyMap<string, string> {
  const names = new Map<string, string>();
  const list = (raw as { cameras?: unknown } | null)?.cameras;
  if (!Array.isArray(list)) return names;
  for (const entry of list) {
    const e = entry as { id?: unknown; name?: unknown } | null;
    if (typeof e?.id === "string" && typeof e.name === "string" && e.name.trim()) {
      names.set(e.id, e.name.trim());
    }
  }
  return names;
}
