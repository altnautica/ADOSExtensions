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

import { DEFAULT_CONFIG, type FollowConfig } from "./types";

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
    camera_hfov_deg: numberOr(r.camera_hfov_deg, DEFAULT_CONFIG.camera_hfov_deg),
  };
}

function numberOr(v: unknown, fallback: number): number {
  const n = typeof v === "string" ? Number(v) : v;
  return typeof n === "number" && Number.isFinite(n) ? n : fallback;
}
