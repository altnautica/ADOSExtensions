/**
 * Command emitter tests.
 *
 * The panel steers the gimbal through the plugin's own per-drone config
 * (`plugin.config.write`), the one command path the host lets a plugin use
 * for this. Each test invokes an emitter and asserts the recorded envelope:
 * the command name and the `{ key, value }` the agent control loop reads.
 */

import { describe, expect, it } from "vitest";

import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

import { sendPitchYaw, sendRoiLocation, sendRoiNone } from "../src/commands";

interface Envelope {
  command: string;
  args: { key: string; value: unknown };
}

async function withHarness() {
  const harness = createPluginHarness({
    grantedCapabilities: ["command.send"],
    mount: () => undefined,
  });
  await harness.start();
  return harness;
}

function last(calls: ReadonlyArray<{ args: unknown }>): Envelope {
  expect(calls.length).toBeGreaterThan(0);
  return calls[calls.length - 1]!.args as Envelope;
}

describe("gimbal command emitters", () => {
  it("never sends a raw MAVLink command, which the host refuses", async () => {
    const h = await withHarness();
    await sendPitchYaw(h.ctx, { pitchDeg: -30, yawDeg: 45 });
    await sendRoiLocation(h.ctx, { latDeg: 1, lonDeg: 2, altM: 3 });
    await sendRoiNone(h.ctx);
    for (const call of h.calls) {
      expect((call.args as Envelope).command).toBe("plugin.config.write");
    }
    await h.teardown();
  });

  it("points with the operator's pitch and yaw in degrees", async () => {
    const h = await withHarness();
    await sendPitchYaw(h.ctx, { pitchDeg: -30, yawDeg: 45 });
    expect(last(h.calls).args).toEqual({
      key: "point",
      value: { pitch_deg: -30, yaw_deg: 45 },
    });
    await h.teardown();
  });

  it("locks the ROI on an unscaled latitude, longitude and altitude", async () => {
    const h = await withHarness();
    await sendRoiLocation(h.ctx, { latDeg: 12.9716, lonDeg: 77.5946, altM: 50 });
    expect(last(h.calls).args).toEqual({
      key: "roi",
      value: { lat_deg: 12.9716, lon_deg: 77.5946, alt_m: 50 },
    });
    await h.teardown();
  });

  it("clears the ROI with the roi_clear key", async () => {
    const h = await withHarness();
    await sendRoiNone(h.ctx);
    expect(last(h.calls).args).toEqual({ key: "roi_clear", value: true });
    await h.teardown();
  });
});
