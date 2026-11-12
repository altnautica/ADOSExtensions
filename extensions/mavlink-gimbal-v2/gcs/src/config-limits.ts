/**
 * Axis limits from the per-drone config, remount-on-change only.
 *
 * The host posts `config.changed` after every accepted config write,
 * including the panel's own `point` / `roi` writes and the agent's reset of
 * those one-shot keys. Rebuilding the panel on each of those lost drag and
 * focus state and made the tab flicker on every slider release, so the
 * panel is rebuilt only when the merged limits actually differ.
 */

import type { PluginContext } from "@altnautica/plugin-sdk";

import { DEFAULT_LIMITS, type AxisLimits } from "./types";

const AXIS_KEYS = Object.keys(DEFAULT_LIMITS) as Array<keyof AxisLimits>;

/** Config-schema (and agent) key for each panel limit. */
const SCHEMA_KEYS: Record<keyof AxisLimits, string> = {
  pitchMinDeg: "pitch_min_deg",
  pitchMaxDeg: "pitch_max_deg",
  yawMinDeg: "yaw_min_deg",
  yawMaxDeg: "yaw_max_deg",
  rollMinDeg: "roll_min_deg",
  rollMaxDeg: "roll_max_deg",
};

/** The limits a config snapshot asks for, defaults filled in. */
export function mergeLimits(next: unknown): AxisLimits {
  const merged: AxisLimits = { ...DEFAULT_LIMITS };
  if (next === null || typeof next !== "object" || !("limits" in next)) return merged;
  const raw = next.limits;
  if (raw === null || typeof raw !== "object") return merged;
  for (const key of AXIS_KEYS) {
    const schemaKey = SCHEMA_KEYS[key];
    const v: unknown = schemaKey in raw ? Reflect.get(raw, schemaKey) : undefined;
    if (typeof v === "number" && Number.isFinite(v)) merged[key] = v;
  }
  return merged;
}

/**
 * Call `onChange` with the merged limits whenever a config change moves
 * them away from `initial`. A null or non-object snapshot is ignored.
 */
export function watchLimits(
  ctx: PluginContext,
  initial: AxisLimits,
  onChange: (limits: AxisLimits) => void,
): void {
  let current = initial;
  ctx.config.onChange<unknown>((next) => {
    if (next === null || typeof next !== "object") return;
    const merged = mergeLimits(next);
    if (AXIS_KEYS.every((key) => merged[key] === current[key])) return;
    current = merged;
    onChange(merged);
  });
}
