/**
 * Remount gating: only a change in the merged axis limits rebuilds the panel.
 */

import { describe, expect, it } from "vitest";

import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

import { mergeLimits, watchLimits } from "../src/config-limits";
import { DEFAULT_LIMITS, type AxisLimits } from "../src/types";

describe("watchLimits", () => {
  it("remounts only when the merged limits change", async () => {
    const remounts: AxisLimits[] = [];
    const h = createPluginHarness({
      mount: (ctx) => watchLimits(ctx, DEFAULT_LIMITS, (l) => remounts.push(l)),
    });
    await h.start();

    // The panel's own one-shot writes and the agent's key reset.
    h.pushConfig({ point: { pitch_deg: -10, yaw_deg: 5 } });
    h.pushConfig({ point: false, roi: false });
    // Limits restated at their current values.
    h.pushConfig({ limits: { ...DEFAULT_LIMITS } });
    // A null snapshot is ignored, not treated as "reset to defaults".
    h.pushConfig(null);
    expect(remounts).toHaveLength(0);

    h.pushConfig({ limits: { pitchMinDeg: -45 } });
    h.pushConfig({ limits: { pitchMinDeg: -45 }, point: false });
    expect(remounts).toEqual([{ ...DEFAULT_LIMITS, pitchMinDeg: -45 }]);
    await h.teardown();
  });

  it("ignores non-numeric limit values", () => {
    expect(mergeLimits({ limits: { yawMaxDeg: "90", pitchMaxDeg: Number.NaN } })).toEqual(
      DEFAULT_LIMITS,
    );
  });
});
