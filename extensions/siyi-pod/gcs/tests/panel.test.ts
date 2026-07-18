import { describe, expect, it } from "vitest";

import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

import { mountPanel } from "../src/panel";
import { PodStateStore } from "../src/pod-state";

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
  return harness;
}

function mount(harness: Awaited<ReturnType<typeof withHarness>>, raw: unknown) {
  const store = new PodStateStore();
  store.ingest(raw);
  const root = document.createElement("div");
  document.body.appendChild(root);
  const handle = mountPanel(harness.ctx, root, store);
  return { root, handle };
}

function streamSelects(root: HTMLElement): HTMLSelectElement[] {
  // The stream-source selects are the ones offering EO-wide (the gimbal-mode
  // select offers lock/follow/fpv).
  return Array.from(root.querySelectorAll("select")).filter((s) =>
    Array.from(s.options).some((o) => o.value === "eo_wide"),
  );
}

const ZT30 = {
  model: "ZT30",
  known: true,
  connected: true,
  capabilities: {
    gimbal: true,
    sensors: ["eo_zoom", "eo_wide", "ir"],
    streams: ["eo_zoom", "eo_wide", "ir", "split"],
    supports_pip: true,
  },
  assignment: { main: "eo_zoom", sub: "ir" },
};

describe("stream source selector", () => {
  it("renders a per-leg selector on a two-leg pod and reassigns a leg", async () => {
    const harness = await withHarness();
    const { root, handle } = mount(harness, ZT30);

    const sels = streamSelects(root);
    expect(sels.length).toBe(2); // main + sub
    const [main, sub] = sels as [HTMLSelectElement, HTMLSelectElement];
    expect(main.value).toBe("eo_zoom"); // main shows EO-zoom
    expect(sub.value).toBe("ir"); // sub shows IR (the live assignment)

    // Reassign the sub leg to EO-wide -> stream_assignment { sub: eo_wide }.
    sub.value = "eo_wide";
    sub.dispatchEvent(new Event("change"));
    await new Promise((r) => setTimeout(r, 0));

    const last = harness.calls[harness.calls.length - 1]!;
    const o = last.args as Outer;
    expect(last.capability).toBe("command.send");
    expect(o.command).toBe("plugin.config.write");
    expect(o.args).toEqual({ key: "stream_assignment", value: { sub: "eo_wide" } });

    handle.destroy();
    root.remove();
    await harness.teardown();
  });

  it("offers the split composite as a selectable source when supported", async () => {
    const harness = await withHarness();
    const { root, handle } = mount(harness, ZT30);
    const first = streamSelects(root)[0]!;
    const values = Array.from(first.options).map((o) => o.value);
    expect(values).toContain("split");
    handle.destroy();
    root.remove();
    await harness.teardown();
  });

  it("hides the selector on a single-sensor pod", async () => {
    const harness = await withHarness();
    const { root, handle } = mount(harness, {
      model: "A8 mini",
      known: true,
      connected: true,
      capabilities: { gimbal: true, sensors: ["eo_zoom"], streams: ["eo_zoom"] },
      assignment: { main: "eo_zoom" },
    });
    expect(streamSelects(root).length).toBe(0);
    handle.destroy();
    root.remove();
    await harness.teardown();
  });
});
