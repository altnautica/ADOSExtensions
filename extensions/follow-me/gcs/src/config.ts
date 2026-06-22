/**
 * Per-drone config read/write for the settings tab.
 *
 * The settings tab writes follow parameters through the host's plugin
 * config surface; the agent reads the same per-drone keys live each loop.
 * The SDK's `ctx.config.onChange` delivers the current config to the
 * iframe; writes go through `ctx.command.send` on a config-write command
 * the host bridges to the per-drone plugin config store.
 *
 * @license GPL-3.0-or-later
 */

import type { PluginContext } from "@altnautica/plugin-sdk";

import { DEFAULT_CONFIG, type FollowConfig } from "./types";

/** The command the host maps to a per-drone plugin config write. */
export const CONFIG_WRITE_COMMAND = "plugin.config.write";

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

/** Write a single config key for this drone's plugin instance. */
export async function writeConfigKey<K extends keyof FollowConfig>(
  ctx: PluginContext,
  key: K,
  value: FollowConfig[K],
): Promise<boolean> {
  try {
    await ctx.command.send(CONFIG_WRITE_COMMAND, { key, value });
    return true;
  } catch {
    return false;
  }
}

function numberOr(v: unknown, fallback: number): number {
  const n = typeof v === "string" ? Number(v) : v;
  return typeof n === "number" && Number.isFinite(n) ? n : fallback;
}
