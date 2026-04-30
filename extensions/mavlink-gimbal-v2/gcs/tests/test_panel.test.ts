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
import {
  MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
  MAV_CMD_DO_SET_ROI_LOCATION,
  MAV_CMD_DO_SET_ROI_NONE,
  type GimbalState,
} from "../src/types";

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
  args: { command: number; param1: number; param2: number; param5?: number };
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

  it("emits MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW when the pitch slider changes", async () => {
    const { ctx, calls, teardown } = await makeHarness();
    const panel = mountPanel(ctx, rootEl);
    panel.setSlider("pitch", -30);
    // Allow the async send to flush.
    await new Promise((r) => setTimeout(r, 0));
    const last = calls[calls.length - 1];
    expect(last).toBeDefined();
    const outer = last!.args as OuterArgs;
    expect(outer.args.command).toBe(MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW);
    expect(outer.args.param1).toBe(-30);
    panel.destroy();
    await teardown();
  });

  it("emits MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW when the yaw slider changes", async () => {
    const { ctx, calls, teardown } = await makeHarness();
    const panel = mountPanel(ctx, rootEl);
    panel.setSlider("yaw", 45);
    await new Promise((r) => setTimeout(r, 0));
    const last = calls[calls.length - 1];
    expect(last).toBeDefined();
    const outer = last!.args as OuterArgs;
    expect(outer.args.command).toBe(MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW);
    expect(outer.args.param2).toBe(45);
    panel.destroy();
    await teardown();
  });

  it("submitRoi emits MAV_CMD_DO_SET_ROI_LOCATION with scaled lat and lon", async () => {
    const { ctx, calls, teardown } = await makeHarness();
    const panel: PanelHandle = mountPanel(ctx, rootEl);
    await panel.submitRoi({ latDeg: 12.971, lonDeg: 77.594, altM: 50 });
    const last = calls[calls.length - 1];
    expect(last).toBeDefined();
    const outer = last!.args as OuterArgs;
    expect(outer.args.command).toBe(MAV_CMD_DO_SET_ROI_LOCATION);
    expect(outer.args.param5).toBe(129710000);
    panel.destroy();
    await teardown();
  });

  it("releaseRoi emits MAV_CMD_DO_SET_ROI_NONE", async () => {
    const { ctx, calls, teardown } = await makeHarness();
    const panel: PanelHandle = mountPanel(ctx, rootEl);
    await panel.releaseRoi();
    const last = calls[calls.length - 1];
    expect(last).toBeDefined();
    const outer = last!.args as OuterArgs;
    expect(outer.args.command).toBe(MAV_CMD_DO_SET_ROI_NONE);
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
