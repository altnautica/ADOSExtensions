/**
 * Panel tests.
 *
 * The panel renders into a real DOM (via happy-dom) and emits commands
 * through the harness. Tests assert that the slider, the ROI form,
 * and the release button drive the right RPC calls and that the live
 * state readout updates when telemetry arrives.
 */

import { afterEach, beforeEach, describe, expect, it } from "vitest";

import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

import { mountPanel, type PanelHandle } from "../src/panel";
import type { GimbalState } from "../src/types";

let rootEl: HTMLElement;

beforeEach(() => {
  document.body.innerHTML = "";
  rootEl = document.createElement("div");
  rootEl.id = "gimbal-root";
  document.body.appendChild(rootEl);
});

afterEach(() => {
  document.body.innerHTML = "";
});

interface OuterArgs {
  command: string;
  args: { key: string; value: unknown };
}

async function makeHarness(): Promise<{
  ctx: ReturnType<typeof createPluginHarness>["ctx"];
  calls: ReturnType<typeof createPluginHarness>["calls"];
  teardown: () => Promise<void>;
}> {
  const h = createPluginHarness({
    grantedCapabilities: ["command.send"],
    mount: () => undefined,
  });
  await h.start();
  return { ctx: h.ctx, calls: h.calls, teardown: h.teardown };
}

describe("mountPanel", () => {
  it("renders the empty-state message before any telemetry arrives", async () => {
    const { ctx, teardown } = await makeHarness();
    const panel = mountPanel(ctx, rootEl);
    expect(rootEl.textContent ?? "").toContain("Awaiting gimbal state");
    panel.destroy();
    await teardown();
  });

  it("populates the live readout when setState receives a sample", async () => {
    const { ctx, teardown } = await makeHarness();
    const panel = mountPanel(ctx, rootEl);
    const sample: GimbalState = {
      timestampMs: 1000,
      pitchDeg: -30.0,
      yawDeg: 45.0,
      rollDeg: 0.0,
      mode: "manual",
    };
    panel.setState(sample);
    expect(rootEl.textContent ?? "").toContain("-30.0 deg");
    expect(rootEl.textContent ?? "").toContain("45.0 deg");
    expect(rootEl.textContent ?? "").toContain("manual");
    panel.destroy();
    await teardown();
  });

  it("writes the point key once when a slider is released", async () => {
    const { ctx, calls, teardown } = await makeHarness();
    const panel = mountPanel(ctx, rootEl);
    panel.setSlider("pitch", -30);
    await new Promise((r) => setTimeout(r, 0));
    const outer = calls[calls.length - 1]!.args as OuterArgs;
    expect(outer.command).toBe("plugin.config.write");
    expect(outer.args).toEqual({ key: "point", value: { pitch_deg: -30, yaw_deg: 0 } });
    panel.destroy();
    await teardown();
  });

  it("sends nothing while a slider is dragged, and one write on release", async () => {
    const { ctx, calls, teardown } = await makeHarness();
    const panel = mountPanel(ctx, rootEl);
    const yaw = rootEl.querySelector<HTMLInputElement>(".agm-yaw")!;
    const before = calls.length;
    for (const v of ["10", "20", "45"]) {
      yaw.value = v;
      yaw.dispatchEvent(new Event("input"));
    }
    await new Promise((r) => setTimeout(r, 0));
    expect(calls.length).toBe(before);
    expect(rootEl.textContent ?? "").toContain("Yaw45.0 deg");
    yaw.dispatchEvent(new Event("change"));
    await new Promise((r) => setTimeout(r, 0));
    expect(calls.length).toBe(before + 1);
    const outer = calls[calls.length - 1]!.args as OuterArgs;
    expect(outer.args).toEqual({ key: "point", value: { pitch_deg: 0, yaw_deg: 45 } });
    panel.destroy();
    await teardown();
  });

  it("submitRoi writes the roi key with the target location", async () => {
    const { ctx, calls, teardown } = await makeHarness();
    const panel: PanelHandle = mountPanel(ctx, rootEl);
    await panel.submitRoi({ latDeg: 12.971, lonDeg: 77.594, altM: 50 });
    const outer = calls[calls.length - 1]!.args as OuterArgs;
    expect(outer.command).toBe("plugin.config.write");
    expect(outer.args).toEqual({
      key: "roi",
      value: { lat_deg: 12.971, lon_deg: 77.594, alt_m: 50 },
    });
    panel.destroy();
    await teardown();
  });

  it("releaseRoi writes the roi_clear key", async () => {
    const { ctx, calls, teardown } = await makeHarness();
    const panel: PanelHandle = mountPanel(ctx, rootEl);
    await panel.releaseRoi();
    const outer = calls[calls.length - 1]!.args as OuterArgs;
    expect(outer.args).toEqual({ key: "roi_clear", value: true });
    panel.destroy();
    await teardown();
  });

  it("ROI status text reflects a successful lock", async () => {
    const { ctx, teardown } = await makeHarness();
    const panel: PanelHandle = mountPanel(ctx, rootEl);
    await panel.submitRoi({ latDeg: 12.971, lonDeg: 77.594, altM: 50 });
    expect(rootEl.textContent ?? "").toContain("Locked on 12.971");
    panel.destroy();
    await teardown();
  });

  it("destroy clears the root element", async () => {
    const { ctx, teardown } = await makeHarness();
    const panel: PanelHandle = mountPanel(ctx, rootEl);
    panel.destroy();
    expect(rootEl.children).toHaveLength(0);
    await teardown();
  });

  it("getState returns null before any sample arrives, then reflects the latest", async () => {
    const { ctx, teardown } = await makeHarness();
    const panel: PanelHandle = mountPanel(ctx, rootEl);
    expect(panel.getState()).toBeNull();
    const sample: GimbalState = {
      timestampMs: 2000,
      pitchDeg: 1,
      yawDeg: 2,
      rollDeg: 3,
      mode: "manual",
    };
    panel.setState(sample);
    expect(panel.getState()).toBe(sample);
    panel.destroy();
    await teardown();
  });

  it("ROI status text reflects a release", async () => {
    const { ctx, teardown } = await makeHarness();
    const panel: PanelHandle = mountPanel(ctx, rootEl);
    await panel.releaseRoi();
    expect(rootEl.textContent ?? "").toContain("ROI released");
    panel.destroy();
    await teardown();
  });
});
