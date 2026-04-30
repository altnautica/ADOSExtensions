import { describe, it, expect } from "vitest";

import { createPluginHarness } from "../src/harness";

describe("createPluginHarness", () => {
  it("captures every RPC the plugin issues during mount", async () => {
    const harness = createPluginHarness({
      grantedCapabilities: ["telemetry.subscribe.battery"],
      mount: async (ctx) => {
        await ctx.telemetry.subscribe<{ pct: number }>("battery", () => {});
      },
    });
    await harness.start();
    expect(harness.calls.map((c) => c.method)).toContain("telemetry.subscribe");
    await harness.teardown();
  });

  it("delivers pushed telemetry into the plugin's subscription handler", async () => {
    const seen: number[] = [];
    const harness = createPluginHarness({
      grantedCapabilities: ["telemetry.subscribe.battery"],
      mount: async (ctx) => {
        await ctx.telemetry.subscribe<{ pct: number }>("battery", (s) => {
          seen.push(s.pct);
        });
      },
    });
    await harness.start();
    harness.pushTelemetry("battery", { pct: 80 });
    harness.pushTelemetry("battery", { pct: 60 });
    expect(seen).toEqual([80, 60]);
    await harness.teardown();
  });

  it("rejects RPCs whose required capability is not granted", async () => {
    let denied: string | null = null;
    const harness = createPluginHarness({
      grantedCapabilities: [],
      mount: async (ctx) => {
        try {
          await ctx.command.send("ARM");
        } catch (err) {
          denied = (err as Error).message;
        }
      },
    });
    await harness.start();
    expect(denied).toMatch(/permission_denied|lacks capability/);
    await harness.teardown();
  });

  it("captures notifications and recording marks separately for assertions", async () => {
    const harness = createPluginHarness({
      grantedCapabilities: [
        "ui.slot.notification",
        "recording.write",
      ],
      mount: async (ctx) => {
        await ctx.notifications.publish({
          channelId: "test",
          severity: "warning",
          title: "Boom",
        });
        await ctx.recording.mark({ label: "Boom" });
      },
    });
    await harness.start();
    expect(harness.notifications).toHaveLength(1);
    expect(harness.recordingMarks).toHaveLength(1);
    await harness.teardown();
  });

  it("simulates host failure on a future RPC via failNext", async () => {
    let caught: { code?: string; message: string } | null = null;
    const harness = createPluginHarness({
      grantedCapabilities: ["mission.read"],
      mount: async (ctx) => {
        try {
          await ctx.mission.read("m1");
        } catch (err) {
          caught = err as { code: string; message: string };
        }
      },
    });
    harness.failNext("mission.read", "host_io_error", "disk full");
    await harness.start();
    expect(caught?.code).toBe("host_io_error");
    expect(caught?.message).toBe("disk full");
    await harness.teardown();
  });

  it("delivers config and theme events to plugin subscribers", async () => {
    const seenConfig: unknown[] = [];
    const seenTheme: Record<string, string>[] = [];
    const harness = createPluginHarness({
      mount: (ctx) => {
        ctx.config.onChange((next) => seenConfig.push(next));
        ctx.theme.onChange((vars) => seenTheme.push(vars));
      },
    });
    await harness.start();
    harness.pushConfig({ thresholds: { lowCellVoltageV: 3.6 } });
    harness.pushTheme({ "--bhp-bg": "#000" });
    expect(seenConfig).toHaveLength(1);
    expect(seenTheme[0]?.["--bhp-bg"]).toBe("#000");
    await harness.teardown();
  });

  it("formats locale strings with parameter interpolation", async () => {
    const harness = createPluginHarness({
      locale: {
        "anomaly.cellLow": "Cell low: {voltage} V",
      },
      mount: () => {},
    });
    await harness.start();
    expect(
      harness.ctx.i18n.t("anomaly.cellLow", { voltage: "3.4" }),
    ).toBe("Cell low: 3.4 V");
    expect(harness.ctx.i18n.t("missing.key")).toBe("missing.key");
    await harness.teardown();
  });
});
