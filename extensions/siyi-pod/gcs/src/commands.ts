/**
 * Command emitters.
 *
 * The GCS half controls the pod by writing the agent plugin's per-drone config,
 * NOT by sending vehicle commands: every emitter routes through the reserved
 * `plugin.config.write` command with a `{ key, value }` argument, which the host
 * forwards to the LAN agent's per-plugin config store (scoped to this plugin id).
 * The agent's control loop reads those keys live each tick. Declarative state
 * keys carry their value directly; one-shot actions carry a monotonically
 * increasing nonce so the agent fires them exactly once.
 *
 * The emitters do not touch the DOM; the panel and overlay wire them up, which
 * keeps the tests focused on the RPC argument shape.
 */

import type { PluginContext } from "@altnautica/plugin-sdk";

const CONFIG_WRITE = "plugin.config.write";

/** A monotonic nonce source that survives remounts (epoch milliseconds). */
export function nextNonce(): number {
  return Date.now();
}

export async function writeConfig(
  ctx: PluginContext,
  key: string,
  value: unknown,
): Promise<unknown> {
  return ctx.command.send(CONFIG_WRITE, { key, value });
}

// -- declarative state ------------------------------------------------------
export const setZoom = (ctx: PluginContext, zoom: number) =>
  writeConfig(ctx, "zoom", zoom);

export const setSensorMode = (ctx: PluginContext, mode: string) =>
  writeConfig(ctx, "sensor_mode", mode);

export const setGimbalMode = (ctx: PluginContext, mode: string) =>
  writeConfig(ctx, "gimbal_mode", mode);

export const setPalette = (ctx: PluginContext, palette: number) =>
  writeConfig(ctx, "palette", palette);

export const setGain = (ctx: PluginContext, high: boolean) =>
  writeConfig(ctx, "thermal_gain", high);

export const setTrackActive = (ctx: PluginContext, active: boolean) =>
  writeConfig(ctx, "track_active", active);

// -- one-shot actions (nonce) ----------------------------------------------
export const takePhoto = (ctx: PluginContext) =>
  writeConfig(ctx, "photo_nonce", nextNonce());

export const toggleRecord = (ctx: PluginContext) =>
  writeConfig(ctx, "record_nonce", nextNonce());

export const recenter = (ctx: PluginContext) =>
  writeConfig(ctx, "recenter_nonce", nextNonce());

export const fireLaser = (ctx: PluginContext) =>
  writeConfig(ctx, "laser_fire_nonce", nextNonce());

/** Hand the pod a box to lock its on-pod tracker onto (pod-frame pixels). */
export async function designate(
  ctx: PluginContext,
  box: { x: number; y: number; width: number; height: number },
): Promise<void> {
  await writeConfig(ctx, "track_designate", box);
  await writeConfig(ctx, "track_designate_nonce", nextNonce());
}
