import { describe, it, expect } from "vitest";

import type {
  PerceptionDetectionBatch,
  PerceptionSessionHealth,
  PerceptionTierInfo,
} from "../src/api";
import { createPluginHarness } from "../src/harness";

function sampleBatch(): PerceptionDetectionBatch {
  return {
    modelId: "yolov8n",
    cameraId: "cam0",
    frameId: 42,
    tsMs: 1_700_000_000_000,
    frameWidth: 1280,
    frameHeight: 720,
    detections: [
      {
        bbox: { x: 100, y: 120, width: 40, height: 90 },
        classLabel: "person",
        confidence: 0.87,
        trackId: 7,
        lockState: "locked",
      },
    ],
  };
}

describe("ctx.perception", () => {
  it("delivers a pushed detection batch and stops after unsubscribe", async () => {
    const seen: PerceptionDetectionBatch[] = [];
    let off: (() => void) | undefined;
    const harness = createPluginHarness({
      grantedCapabilities: ["perception.subscribe"],
      mount: async (ctx) => {
        off = await ctx.perception.subscribeDetections((b) => seen.push(b));
      },
    });
    await harness.start();

    // The subscribe RPC was issued during mount.
    expect(harness.calls.map((c) => c.method)).toContain("perception.subscribe");

    harness.pushPerceptionDetection(sampleBatch());
    expect(seen).toHaveLength(1);
    expect(seen[0]?.detections[0]?.classLabel).toBe("person");
    expect(seen[0]?.detections[0]?.trackId).toBe(7);

    // Unsubscribe stops delivery and fires the host-side unsubscribe.
    off?.();
    expect(harness.calls.map((c) => c.method)).toContain("perception.unsubscribe");
    harness.pushPerceptionDetection(sampleBatch());
    expect(seen).toHaveLength(1);

    await harness.teardown();
  });

  it("resolves readTier from the stubbed tier response", async () => {
    let tier: PerceptionTierInfo | undefined;
    const harness = createPluginHarness({
      grantedCapabilities: ["perception.read"],
      mount: async (ctx) => {
        tier = await ctx.perception.readTier();
      },
    });
    harness.stubPerceptionTier({
      tier: "offload",
      offloadTarget: "workstation-1",
      npuTops: 0,
      hasAccelerator: false,
    });
    await harness.start();
    expect(tier?.tier).toBe("offload");
    expect(tier?.offloadTarget).toBe("workstation-1");
    expect(tier?.hasAccelerator).toBe(false);
    await harness.teardown();
  });

  it("resolves readSessionHealth from the stubbed health response", async () => {
    let health: PerceptionSessionHealth | undefined;
    const harness = createPluginHarness({
      grantedCapabilities: ["perception.read"],
      mount: async (ctx) => {
        health = await ctx.perception.readSessionHealth();
      },
    });
    const stub: PerceptionSessionHealth = {
      session: "live",
      feed: "fresh",
      ageMs: 33,
      batchesPerSecond: 28.5,
      boundNode: "workstation-1",
    };
    harness.stubPerceptionHealth(stub);
    await harness.start();
    expect(health).toEqual(stub);
    await harness.teardown();
  });

  it("denies perception reads when the capability is not granted", async () => {
    let denied: string | null = null;
    const harness = createPluginHarness({
      grantedCapabilities: [],
      mount: async (ctx) => {
        try {
          await ctx.perception.readTier();
        } catch (err) {
          denied = (err as Error).message;
        }
      },
    });
    await harness.start();
    expect(denied).toMatch(/permission_denied|lacks capability/);
    await harness.teardown();
  });
});
