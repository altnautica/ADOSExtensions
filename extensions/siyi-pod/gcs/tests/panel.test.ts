import { describe, expect, it } from "vitest";

import {
  createPluginHarness,
  type PluginHarness,
} from "@altnautica/plugin-sdk/harness";

import { mountPanel } from "../src/panel";
import { PodStateStore } from "../src/pod-state";

async function withHarness(): Promise<PluginHarness> {
  const harness = createPluginHarness({
    grantedCapabilities: ["command.send"],
    mount: () => undefined,
  });
  await harness.start();
  return harness;
}

function mount(harness: PluginHarness, raw: unknown) {
  const store = new PodStateStore();
  store.ingest(raw);
  const root = document.createElement("div");
  document.body.appendChild(root);
  const handle = mountPanel(harness.ctx, root, store);
  return { root, handle };
}

function buttonLabels(root: HTMLElement): string[] {
  return Array.from(root.querySelectorAll("button")).map(
    (b) => b.textContent ?? "",
  );
}

/** A pod that has identified itself as a ZT30: every control is available. */
const ZT30 = {
  model: "ZT30",
  known: true,
  connected: true,
  capabilities: {
    gimbal: true,
    zoom: true,
    thermal: true,
    laser: true,
    max_zoom: 180,
    sensors: ["eo_zoom", "eo_wide", "ir"],
  },
  assignment: { main: "eo_zoom", sub: "ir" },
};

/**
 * A pod that has NOT identified itself. The agent leaves the capability block
 * empty in this state, so the console must offer no model-specific control:
 * a control the pod may not have is worse than no control, because pressing it
 * looks like it did something.
 */
const UNIDENTIFIED = {
  model: "Unknown SIYI pod",
  known: false,
  connected: false,
  capabilities: {},
  assignment: {},
};

describe("console capability gating", () => {
  it("renders the model-specific controls an identified pod supports", async () => {
    const harness = await withHarness();
    const { root, handle } = mount(harness, ZT30);

    const labels = buttonLabels(root);
    expect(labels).toContain("High gain");
    expect(labels).toContain("Range");
    expect(root.querySelectorAll("input[type=range]").length).toBeGreaterThan(0);

    handle.destroy();
    root.remove();
    await harness.teardown();
  });

  it("offers no model-specific control while the pod is unidentified", async () => {
    const harness = await withHarness();
    const { root, handle } = mount(harness, UNIDENTIFIED);

    const labels = buttonLabels(root);
    expect(labels).not.toContain("High gain");
    expect(labels).not.toContain("Low gain");
    expect(labels).not.toContain("Range");
    expect(labels).not.toContain("Center");
    expect(root.querySelectorAll("input[type=range]").length).toBe(0);
    expect(root.querySelectorAll("input[type=number]").length).toBe(0);

    handle.destroy();
    root.remove();
    await harness.teardown();
  });

  it("hides thermal and laser on a model that carries neither", async () => {
    const harness = await withHarness();
    const { root, handle } = mount(harness, {
      model: "A8 mini",
      known: true,
      connected: true,
      capabilities: {
        gimbal: true,
        zoom: true,
        thermal: false,
        laser: false,
        max_zoom: 6,
        sensors: ["eo_zoom"],
      },
      assignment: { main: "eo_zoom" },
    });

    const labels = buttonLabels(root);
    expect(labels).not.toContain("High gain");
    expect(labels).not.toContain("Range");
    // Zoom is supported, so its slider is still rendered.
    expect(root.querySelectorAll("input[type=range]").length).toBeGreaterThan(0);

    handle.destroy();
    root.remove();
    await harness.teardown();
  });
});
