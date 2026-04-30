/**
 * Command emitter tests.
 *
 * The plugin SDK harness records every RPC call. Each test invokes a
 * command emitter and asserts the resulting envelope shape: method,
 * capability, and the seven-arg parameter map.
 */

import { describe, expect, it } from "vitest";

import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

import {
  sendPitchYaw,
  sendRoiLocation,
  sendRoiNone,
} from "../src/commands";
import {
  MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
  MAV_CMD_DO_SET_ROI_LOCATION,
  MAV_CMD_DO_SET_ROI_NONE,
} from "../src/types";

interface CapturedArgs {
  command: number;
  target_system: number;
  target_component: number;
  param1: number;
  param2: number;
  param3: number;
  param4: number;
  param5: number;
  param6: number;
  param7: number;
}

async function withHarness(): Promise<{
  ctx: ReturnType<typeof createPluginHarness>["ctx"];
  calls: ReturnType<typeof createPluginHarness>["calls"];
  teardown: () => Promise<void>;
}> {
  const harness = createPluginHarness({
    grantedCapabilities: ["command.send"],
    mount: () => undefined,
  });
  await harness.start();
  return {
    ctx: harness.ctx,
    calls: harness.calls,
    teardown: harness.teardown,
  };
}

function lastInner(calls: ReadonlyArray<{ args: unknown }>): CapturedArgs {
  const last = calls[calls.length - 1];
  expect(last).toBeDefined();
  const outer = last!.args as { args: CapturedArgs };
  return outer.args;
}

describe("sendPitchYaw", () => {
  it("emits MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW with the operator's pitch and yaw", async () => {
    const { ctx, calls, teardown } = await withHarness();
    await sendPitchYaw(ctx, { pitchDeg: -30, yawDeg: 45 });
    const args = lastInner(calls);
    expect(args.command).toBe(MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW);
    expect(args.param1).toBe(-30);
    expect(args.param2).toBe(45);
    expect(args.target_component).toBe(154);
    await teardown();
  });

  it("zeroes the rate fields when only position is provided", async () => {
    const { ctx, calls, teardown } = await withHarness();
    await sendPitchYaw(ctx, { pitchDeg: 0, yawDeg: 0 });
    const args = lastInner(calls);
    expect(args.param3).toBe(0);
    expect(args.param4).toBe(0);
    await teardown();
  });

  it("forwards rate fields when supplied", async () => {
    const { ctx, calls, teardown } = await withHarness();
    await sendPitchYaw(ctx, {
      pitchDeg: 0,
      yawDeg: 0,
      pitchRateDps: 12.5,
      yawRateDps: -7.5,
    });
    const args = lastInner(calls);
    expect(args.param3).toBe(12.5);
    expect(args.param4).toBe(-7.5);
    await teardown();
  });

  it("respects custom target_system and target_component", async () => {
    const { ctx, calls, teardown } = await withHarness();
    await sendPitchYaw(ctx, {
      pitchDeg: 0,
      yawDeg: 0,
      targetSystem: 7,
      targetComponent: 154,
    });
    const args = lastInner(calls);
    expect(args.target_system).toBe(7);
    expect(args.target_component).toBe(154);
    await teardown();
  });
});

describe("sendRoiLocation", () => {
  it("emits MAV_CMD_DO_SET_ROI_LOCATION with int32-scaled lat and lon", async () => {
    const { ctx, calls, teardown } = await withHarness();
    await sendRoiLocation(ctx, {
      latDeg: 12.971,
      lonDeg: 77.594,
      altM: 50,
    });
    const args = lastInner(calls);
    expect(args.command).toBe(MAV_CMD_DO_SET_ROI_LOCATION);
    expect(args.param5).toBe(129710000);
    expect(args.param6).toBe(775940000);
    expect(args.param7).toBe(50);
    await teardown();
  });

  it("scales negative coordinates correctly", async () => {
    const { ctx, calls, teardown } = await withHarness();
    await sendRoiLocation(ctx, {
      latDeg: -33.8688,
      lonDeg: 151.2093,
      altM: 80,
    });
    const args = lastInner(calls);
    expect(args.param5).toBe(-338688000);
    expect(args.param6).toBe(1512093000);
    await teardown();
  });
});

describe("sendRoiNone", () => {
  it("emits MAV_CMD_DO_SET_ROI_NONE with zeroed parameters", async () => {
    const { ctx, calls, teardown } = await withHarness();
    await sendRoiNone(ctx);
    const args = lastInner(calls);
    expect(args.command).toBe(MAV_CMD_DO_SET_ROI_NONE);
    expect(args.param1).toBe(0);
    expect(args.param2).toBe(0);
    expect(args.param5).toBe(0);
    expect(args.target_component).toBe(154);
    await teardown();
  });
});

describe("RPC routing", () => {
  it("routes every emitter through the command.send capability", async () => {
    const { ctx, calls, teardown } = await withHarness();
    await sendPitchYaw(ctx, { pitchDeg: 0, yawDeg: 0 });
    await sendRoiLocation(ctx, { latDeg: 1, lonDeg: 2, altM: 3 });
    await sendRoiNone(ctx);
    expect(calls.every((c) => c.capability === "command.send")).toBe(true);
    expect(calls).toHaveLength(3);
    await teardown();
  });
});
