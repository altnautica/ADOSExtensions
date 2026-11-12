import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

import {
  createPluginHarness,
  type PluginHarness,
} from "@altnautica/plugin-sdk/harness";

import { hudLines } from "../src/overlay-draw";
import { mountPanel } from "../src/panel";
import { dataAgeS, isLive, PodStateStore, STALE_AFTER_MS } from "../src/pod-state";
import enLocale from "../../locales/en.json";

const t = (key: string) => (enLocale as Record<string, string>)[key] ?? key;

const T0 = 1_800_000_000_000;

const ZT30 = {
  model: "ZT30",
  known: true,
  connected: true,
  link_ok: true,
  laser_enabled: true,
  capabilities: { gimbal: true, zoom: true, thermal: true, laser: true, max_zoom: 180 },
};

async function withHarness(): Promise<PluginHarness> {
  const harness = createPluginHarness({
    grantedCapabilities: ["command.send"],
    mount: () => undefined,
  });
  await harness.start();
  harness.ctx.i18n.t = t;
  return harness;
}

/**
 * Host delivery: the sidecar is polled every 750 ms (with jitter) and a slow
 * request skips the next tick, leaving 1500 ms gaps. Each delivery carries the
 * agent's write time; the agent writes at most every 500 ms.
 */
function hostGaps(): number[] {
  const gaps: number[] = [];
  for (let i = 0; i < 24; i++) gaps.push(i % 4 === 3 ? 1500 : 750 + ((i * 37) % 60) - 30);
  return gaps;
}

describe("staleness at the real host cadence", () => {
  beforeEach(() => {
    vi.useFakeTimers();
    vi.setSystemTime(T0);
  });
  afterEach(() => vi.useRealTimers());

  it("uses the shared 2000 ms cutoff", () => {
    expect(STALE_AFTER_MS).toBe(2_000);
  });

  it("never shows 'No pod data' or cancels a pending Range confirm across poll gaps", async () => {
    const harness = await withHarness();
    const store = new PodStateStore();
    store.ingest({ ...ZT30, ts_ms: Date.now() });
    const root = document.createElement("div");
    document.body.appendChild(root);
    const handle = mountPanel(harness.ctx, root, store);
    const btn = root.querySelector<HTMLButtonElement>('[data-testid="siyi-range-btn"]')!;
    const link = root.querySelector('[data-testid="siyi-link"]')!;
    btn.click();
    expect(btn.textContent).toBe("Confirm: fire laser");

    let elapsed = 0;
    for (const gap of hostGaps()) {
      // Sample just before the next delivery: the worst case.
      vi.advanceTimersByTime(gap - 1);
      expect(link.textContent).not.toBe("No pod data");
      expect(hudLines(store, Date.now(), t)).not.toEqual(["No pod data"]);
      if (elapsed + gap < 5_000) expect(btn.textContent).toBe("Confirm: fire laser");
      vi.advanceTimersByTime(1);
      elapsed += gap;
      // The agent stamped this write up to 500 ms before MC fetched it.
      store.ingest({ ...ZT30, ts_ms: Date.now() - 400 });
    }
    handle.destroy();
  });

  it("renders a 5 s old ts_ms payload stale on first paint (panel and HUD)", async () => {
    const harness = await withHarness();
    const store = new PodStateStore();
    store.ingest({ ...ZT30, ts_ms: Date.now() - 5_000 });
    expect(isLive(store)).toBe(false);
    expect(dataAgeS(store)).toBe(5);
    expect(hudLines(store, Date.now(), t)).toEqual(["No pod data"]);
    const root = document.createElement("div");
    document.body.appendChild(root);
    const handle = mountPanel(harness.ctx, root, store);
    expect(root.querySelector('[data-testid="siyi-link"]')!.textContent).toBe("No pod data");
    expect(root.querySelector<HTMLButtonElement>('[data-testid="siyi-range-btn"]')!.disabled).toBe(true);
    handle.destroy();
  });

  it("a repeated ts_ms does not refresh the age", () => {
    const store = new PodStateStore();
    const ts = Date.now();
    store.ingest({ ...ZT30, ts_ms: ts });
    vi.advanceTimersByTime(1_500);
    store.ingest({ ...ZT30, ts_ms: ts });
    vi.advanceTimersByTime(600);
    store.ingest({ ...ZT30, ts_ms: ts });
    expect(isLive(store)).toBe(false);
  });

  it("caps a drone clock ahead of the browser at arrival time", () => {
    const store = new PodStateStore();
    store.ingest({ ...ZT30, ts_ms: Date.now() + 60_000 });
    vi.advanceTimersByTime(2_000);
    expect(isLive(store)).toBe(false);
  });

  it("falls back to arrival time without ts_ms", () => {
    const store = new PodStateStore();
    store.ingest(ZT30);
    vi.advanceTimersByTime(1_999);
    expect(isLive(store)).toBe(true);
    vi.advanceTimersByTime(1);
    expect(isLive(store)).toBe(false);
  });
});
