/**
 * Command emitters.
 *
 * Each function composes the argument map for the host's `command.send`
 * RPC and delegates to `ctx.command.send`. The emitters do NOT touch
 * the DOM and do NOT subscribe to telemetry; the panel module wires
 * them up. This split keeps the unit tests focused on argument shape.
 */

import type { PluginContext } from "@altnautica/plugin-sdk";

import {
  DEFAULT_TARGET_COMPONENT_ID,
  DEFAULT_TARGET_SYSTEM_ID,
  MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
  MAV_CMD_DO_SET_ROI_LOCATION,
  MAV_CMD_DO_SET_ROI_NONE,
  type RoiTarget,
} from "./types";

export interface CommandTargets {
  targetSystem: number;
  targetComponent: number;
}

const DEFAULT_TARGETS: CommandTargets = {
  targetSystem: DEFAULT_TARGET_SYSTEM_ID,
  targetComponent: DEFAULT_TARGET_COMPONENT_ID,
};

export interface PitchYawArgs extends Partial<CommandTargets> {
  pitchDeg: number;
  yawDeg: number;
  pitchRateDps?: number;
  yawRateDps?: number;
  flags?: number;
  gimbalDeviceId?: number;
}

export async function sendPitchYaw(
  ctx: PluginContext,
  args: PitchYawArgs,
): Promise<unknown> {
  const targets = withDefaults(args);
  return ctx.command.send("mavlink.command_int", {
    command: MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
    target_system: targets.targetSystem,
    target_component: targets.targetComponent,
    param1: args.pitchDeg,
    param2: args.yawDeg,
    param3: args.pitchRateDps ?? 0,
    param4: args.yawRateDps ?? 0,
    param5: args.flags ?? 0,
    param6: args.gimbalDeviceId ?? 0,
    param7: 0,
  });
}

export async function sendRoiLocation(
  ctx: PluginContext,
  target: RoiTarget,
  targets: Partial<CommandTargets> = {},
): Promise<unknown> {
  const t = withDefaults(targets);
  return ctx.command.send("mavlink.command_int", {
    command: MAV_CMD_DO_SET_ROI_LOCATION,
    target_system: t.targetSystem,
    target_component: t.targetComponent,
    param1: 0,
    param2: 0,
    param3: 0,
    param4: 0,
    param5: Math.round(target.latDeg * 1e7),
    param6: Math.round(target.lonDeg * 1e7),
    param7: target.altM,
  });
}

export async function sendRoiNone(
  ctx: PluginContext,
  targets: Partial<CommandTargets> = {},
): Promise<unknown> {
  const t = withDefaults(targets);
  return ctx.command.send("mavlink.command_int", {
    command: MAV_CMD_DO_SET_ROI_NONE,
    target_system: t.targetSystem,
    target_component: t.targetComponent,
    param1: 0,
    param2: 0,
    param3: 0,
    param4: 0,
    param5: 0,
    param6: 0,
    param7: 0,
  });
}

function withDefaults(input: Partial<CommandTargets>): CommandTargets {
  return {
    targetSystem: input.targetSystem ?? DEFAULT_TARGETS.targetSystem,
    targetComponent: input.targetComponent ?? DEFAULT_TARGETS.targetComponent,
  };
}
