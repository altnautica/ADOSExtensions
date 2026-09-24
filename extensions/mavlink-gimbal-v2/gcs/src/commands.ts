/**
 * Command emitters.
 *
 * The panel steers the gimbal by writing the agent plugin's per-drone config,
 * not by sending vehicle commands: the host's `command.send` refuses raw
 * MAVLink from a plugin, and the agent half owns the gimbal manager session.
 * Each emitter routes through the reserved `plugin.config.write` command with a
 * `{ key, value }` argument. The agent's control loop runs `point`, `roi` and
 * `roi_clear` once each and then resets the key, so writing the same value
 * again fires it again.
 *
 * The emitters do not touch the DOM; the panel wires them up, which keeps the
 * tests focused on the RPC argument shape.
 */

import type { PluginContext } from "@altnautica/plugin-sdk";

import type { RoiTarget } from "./types";

const CONFIG_WRITE = "plugin.config.write";

export interface PitchYawArgs {
  pitchDeg: number;
  yawDeg: number;
}

/** Point the gimbal at an absolute pitch and yaw, in degrees. */
export async function sendPitchYaw(
  ctx: PluginContext,
  args: PitchYawArgs,
): Promise<unknown> {
  return ctx.command.send(CONFIG_WRITE, {
    key: "point",
    value: { pitch_deg: args.pitchDeg, yaw_deg: args.yawDeg },
  });
}

/** Lock the gimbal onto a location. */
export async function sendRoiLocation(
  ctx: PluginContext,
  target: RoiTarget,
): Promise<unknown> {
  return ctx.command.send(CONFIG_WRITE, {
    key: "roi",
    value: { lat_deg: target.latDeg, lon_deg: target.lonDeg, alt_m: target.altM },
  });
}

/** Release a location lock. */
export async function sendRoiNone(ctx: PluginContext): Promise<unknown> {
  return ctx.command.send(CONFIG_WRITE, { key: "roi_clear", value: true });
}
