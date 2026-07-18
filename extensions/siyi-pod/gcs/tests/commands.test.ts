import { describe, expect, it } from "vitest";

import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

import {
  designate,
  fireLaser,
  setGimbalMode,
  setPalette,
  setTrackActive,
  setZoom,
  takePhoto,
} from "../src/commands";

interface Outer {
  command: string;
  args: { key: string; value: unknown };
}

async function withHarness() {
  const harness = createPluginHarness({
    grantedCapabilities: ["command.send"],
    mount: () => undefined,
  });
  await harness.start();
  return { ctx: harness.ctx, calls: harness.calls, teardown: harness.teardown };
}

function nth(calls: ReadonlyArray<{ args: unknown }>, fromEnd: number): Outer {
  const call = calls[calls.length - fromEnd];
  expect(call).toBeDefined();
  return call!.args as Outer;
}

describe("config-write commands", () => {
  it("setZoom writes zoom via plugin.config.write over command.send", async () => {
    const { ctx, calls, teardown } = await withHarness();
    await setZoom(ctx, 5);
    const o = nth(calls, 1);
    expect(o.command).toBe("plugin.config.write");
    expect(o.args.key).toBe("zoom");
    expect(o.args.value).toBe(5);
    expect(calls[calls.length - 1]!.capability).toBe("command.send");
    await teardown();
  });

  it("setGimbalMode and setPalette carry their values", async () => {
    const { ctx, calls, teardown } = await withHarness();
    await setGimbalMode(ctx, "lock");
    expect(nth(calls, 1).args).toEqual({ key: "gimbal_mode", value: "lock" });
    await setPalette(ctx, 3);
    expect(nth(calls, 1).args).toEqual({ key: "palette", value: 3 });
    await setTrackActive(ctx, true);
    expect(nth(calls, 1).args).toEqual({ key: "track_active", value: true });
    await teardown();
  });

  it("one-shot actions write a numeric nonce", async () => {
    const { ctx, calls, teardown } = await withHarness();
    await takePhoto(ctx);
    let o = nth(calls, 1);
    expect(o.args.key).toBe("photo_nonce");
    expect(typeof o.args.value).toBe("number");
    await fireLaser(ctx);
    o = nth(calls, 1);
    expect(o.args.key).toBe("laser_fire_nonce");
    expect(typeof o.args.value).toBe("number");
    await teardown();
  });

  it("designate writes the box then a nonce", async () => {
    const { ctx, calls, teardown } = await withHarness();
    await designate(ctx, { x: 1, y: 2, width: 3, height: 4 });
    const box = nth(calls, 2);
    expect(box.args.key).toBe("track_designate");
    expect(box.args.value).toEqual({ x: 1, y: 2, width: 3, height: 4 });
    const nonce = nth(calls, 1);
    expect(nonce.args.key).toBe("track_designate_nonce");
    expect(typeof nonce.args.value).toBe("number");
    await teardown();
  });

  it("every emitter routes through command.send", async () => {
    const { ctx, calls, teardown } = await withHarness();
    await setZoom(ctx, 2);
    await takePhoto(ctx);
    expect(calls.every((c) => c.capability === "command.send")).toBe(true);
    await teardown();
  });
});
