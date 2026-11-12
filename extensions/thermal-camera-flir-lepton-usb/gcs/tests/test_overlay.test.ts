// @vitest-environment happy-dom
import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

import enLocale from "../../locales/en.json";
import { mountOverlay } from "../src/overlay";

const STATE = { connected: true, palette: "ironbow", gain: true };
const FRAME = {
  timestampMs: 1_000,
  width: 160,
  height: 120,
  spot: { x: 80, y: 60, temperatureC: 42.46 },
};

async function mounted() {
  const harness = createPluginHarness({
    locale: enLocale as Record<string, string>,
    grantedCapabilities: [
      "telemetry.subscribe.thermal",
      "telemetry.subscribe.camera.thermal.frame",
    ],
    mount: () => undefined,
  });
  await harness.start();
  const handle = await mountOverlay(harness.ctx, document);
  return { harness, handle };
}

const readoutText = () =>
  document.querySelector(".thm-overlay__readout")?.textContent ?? "";

describe("mountOverlay", () => {
  beforeEach(() => {
    document.head.innerHTML = "";
    document.body.innerHTML = "";
    vi.useFakeTimers();
  });
  afterEach(() => {
    vi.useRealTimers();
  });

  it("injects the plugin stylesheet at mount", async () => {
    const { handle } = await mounted();
    const style = document.head.querySelector("style#thm-plugin-style");
    expect(style).not.toBeNull();
    expect(style?.textContent).toMatch(/\.thm-overlay__readout\s*\{/);
    expect(style?.textContent).toMatch(/\.thm-overlay__canvas\s*\{/);
    // Mounting twice does not stack copies.
    handle.unmount();
    await mounted();
    expect(document.head.querySelectorAll("style#thm-plugin-style")).toHaveLength(1);
  });

  it("ages the spot and says the frames stopped instead of the value", async () => {
    const { harness, handle } = await mounted();
    harness.pushTelemetry("thermal", STATE);
    harness.pushTelemetry("camera.thermal.frame", FRAME);
    await Promise.resolve();
    expect(readoutText()).toBe("Spot 42.5 °C");

    // A repeat of the same read-back is not a fresh frame.
    vi.advanceTimersByTime(1_000);
    harness.pushTelemetry("camera.thermal.frame", FRAME);
    vi.advanceTimersByTime(1_000);
    expect(readoutText()).toBe("Thermal frames stopped — last reading 2 s ago");
    const el = document.querySelector(".thm-overlay__readout");
    expect(el?.classList.contains("thm-overlay__readout--stale")).toBe(true);

    // A new read-back brings the value back.
    harness.pushTelemetry("camera.thermal.frame", { ...FRAME, timestampMs: 3_000 });
    expect(readoutText()).toBe("Spot 42.5 °C");
    handle.unmount();
  });

  it("shows the agent-applied palette read-only and no palette control", async () => {
    const { harness, handle } = await mounted();
    harness.pushTelemetry("thermal", STATE);
    expect(
      document.querySelector(".thm-overlay__palette")?.textContent,
    ).toBe("Palette: Ironbow");
    expect(document.querySelector(".thm-action-rail")).toBeNull();
    handle.unmount();
  });

  it("renders no interactive control below the 48 px touch target", async () => {
    const { handle } = await mounted();
    const controls = document.querySelectorAll("button, select, input");
    for (const c of Array.from(controls)) {
      expect(getComputedStyle(c).minHeight).toBe("48px");
    }
    expect(controls).toHaveLength(0);
    handle.unmount();
  });
});
