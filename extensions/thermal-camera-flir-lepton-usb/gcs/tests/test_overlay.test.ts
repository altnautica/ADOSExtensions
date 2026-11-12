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
    // The overlay over the video pane is display-only: the palette, gain and
    // FFC are driven from settings and the Skill Bar, so nothing here takes
    // a touch.
    expect(
      document.querySelectorAll("button, select, input, [role='button']"),
    ).toHaveLength(0);
    handle.unmount();
  });

  it("shows a read-back the agent stamped 5 s ago as stopped on first paint", async () => {
    vi.setSystemTime(100_000);
    const { harness, handle } = await mounted();
    harness.pushTelemetry("thermal", { ...STATE, ts_ms: 95_000 });
    harness.pushTelemetry("camera.thermal.frame", {
      ...FRAME,
      timestampMs: 95_000,
      ts_ms: 95_000,
    });
    expect(readoutText()).toBe("Thermal frames stopped — last reading 5 s ago");
    const el = document.querySelector(".thm-overlay__readout");
    expect(el?.classList.contains("thm-overlay__readout--stale")).toBe(true);
    handle.unmount();
  });

  it("does not refresh the age on a repeated report", async () => {
    vi.setSystemTime(100_000);
    const { harness, handle } = await mounted();
    harness.pushTelemetry("thermal", STATE);
    const entry = { ...FRAME, timestampMs: 2_000, ts_ms: 100_000 };
    harness.pushTelemetry("camera.thermal.frame", entry);
    expect(readoutText()).toBe("Spot 42.5 °C");
    // The same sidecar entry delivered again is not a new sample.
    vi.advanceTimersByTime(1_500);
    harness.pushTelemetry("camera.thermal.frame", { ...entry });
    vi.advanceTimersByTime(500);
    expect(readoutText()).toBe("Thermal frames stopped — last reading 2 s ago");
    handle.unmount();
  });

  it("ages from the browser clock when the drone clock runs ahead", async () => {
    vi.setSystemTime(100_000);
    const { harness, handle } = await mounted();
    harness.pushTelemetry("thermal", STATE);
    harness.pushTelemetry("camera.thermal.frame", { ...FRAME, ts_ms: 160_000 });
    expect(readoutText()).toBe("Spot 42.5 °C");
    vi.advanceTimersByTime(2_000);
    expect(readoutText()).toBe("Thermal frames stopped — last reading 2 s ago");
    handle.unmount();
  });

  // The host cadence: a poll every 750 ms ± 100 ms, every fifth tick
  // skipped (a 1500 ms gap), each entry already ~1000 ms old on arrival by
  // the browser clock, stamped by a drone clock running `skewMs` behind.
  function runHostCadence(
    harness: Awaited<ReturnType<typeof mounted>>["harness"],
    handle: Awaited<ReturnType<typeof mounted>>["handle"],
    skewMs: number,
  ): void {
    let seed = 7;
    const rand = () => {
      seed = (seed * 1_103_515_245 + 12_345) % 2_147_483_648;
      return seed / 2_147_483_648;
    };
    let stamp = 1_000;
    let tick = 0;
    for (let poll = 0; poll < 60; poll += 1, tick += 1) {
      if (tick % 5 === 4) {
        vi.advanceTimersByTime(750);
        tick += 1;
      }
      stamp += 1;
      harness.pushTelemetry("camera.thermal.frame", {
        ...FRAME,
        timestampMs: stamp,
        ts_ms: Date.now() - 1_000 - skewMs,
      });
      const gap = 750 + Math.round((rand() - 0.5) * 200);
      for (let t = 0; t < gap; t += 50) {
        vi.advanceTimersByTime(Math.min(50, gap - t));
        if (poll === 0) continue;
        expect(handle.getReadout().kind).toBe("spot");
        expect(readoutText()).toBe("Spot 42.5 °C");
      }
    }
  }

  it("stays live at the host cadence with the drone clock 2 s behind", async () => {
    vi.setSystemTime(100_000);
    const { harness, handle } = await mounted();
    harness.pushTelemetry("thermal", STATE);
    runHostCadence(harness, handle, 2_000);
    handle.unmount();
  });

  it("stays live at the host cadence with the drone clock 5 s behind", async () => {
    vi.setSystemTime(100_000);
    const { harness, handle } = await mounted();
    harness.pushTelemetry("thermal", STATE);
    runHostCadence(harness, handle, 5_000);
    handle.unmount();
  });

  it("reads a frame stamped 5 s ago as stopped on mount", async () => {
    vi.setSystemTime(100_000);
    const { harness, handle } = await mounted();
    harness.pushTelemetry("thermal", STATE);
    harness.pushTelemetry("camera.thermal.frame", { ...FRAME, ts_ms: 95_000 });
    expect(handle.getReadout().kind).not.toBe("spot");
    expect(readoutText()).toBe("Thermal frames stopped — last reading 5 s ago");
    handle.unmount();
  });

  it("reads stopped 2000 ms after the last frame of a live feed", async () => {
    vi.setSystemTime(100_000);
    const { harness, handle } = await mounted();
    harness.pushTelemetry("thermal", STATE);
    for (let i = 0; i < 4; i += 1) {
      harness.pushTelemetry("camera.thermal.frame", {
        ...FRAME,
        timestampMs: 10 + i,
        ts_ms: Date.now() - 1_000,
      });
      vi.advanceTimersByTime(750);
    }
    harness.pushTelemetry("camera.thermal.frame", {
      ...FRAME,
      timestampMs: 99,
      ts_ms: Date.now() - 1_000,
    });
    vi.advanceTimersByTime(1_999);
    expect(readoutText()).toBe("Spot 42.5 °C");
    vi.advanceTimersByTime(1);
    expect(readoutText()).toBe("Thermal frames stopped — last reading 2 s ago");
    handle.unmount();
  });
});
