import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

import {
  createPluginHarness,
  type PluginHarness,
} from "@altnautica/plugin-sdk/harness";

import { hudLines } from "../src/overlay-draw";
import { mountSiyi } from "../src/mount";
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
 * Host delivery: the sidecar is polled every 750 ms (with jitter) and every
 * 5th tick is skipped, leaving a 1500 ms gap. Each entry is already about
 * 1000 ms old when it arrives (agent write throttle plus poll latency).
 */
function hostGaps(n = 24): number[] {
  const gaps: number[] = [];
  for (let i = 1; i <= n; i++) gaps.push(i % 5 === 0 ? 1500 : 750 + ((i * 37) % 60) - 30);
  return gaps;
}

const ENTRY_AGE_MS = 1_000;

describe("staleness at the real host cadence", () => {
  beforeEach(() => {
    vi.useFakeTimers();
    vi.setSystemTime(T0);
  });
  afterEach(() => vi.useRealTimers());

  it("uses the shared 2000 ms cutoff", () => {
    expect(STALE_AFTER_MS).toBe(2_000);
  });

  for (const skewMs of [2_000, 5_000]) {
    it(`stays live with the drone clock ${skewMs / 1000} s behind (panel, HUD, pending Range confirm)`, async () => {
      const harness = await withHarness();
      const store = new PodStateStore();
      // The drone stamps ts_ms with its own clock, behind the browser's.
      const deliver = () =>
        store.ingest({ ...ZT30, ts_ms: Date.now() - skewMs - ENTRY_AGE_MS });
      deliver();
      const root = document.createElement("div");
      document.body.appendChild(root);
      const handle = mountPanel(harness.ctx, root, store);
      const btn = root.querySelector<HTMLButtonElement>('[data-testid="siyi-range-btn"]')!;
      const link = root.querySelector('[data-testid="siyi-link"]')!;
      const gaps = hostGaps();
      vi.advanceTimersByTime(gaps[0]!);
      deliver();
      // The panel repaints on its 1 s tick; from then on it must read live.
      const mountAt = Date.now() - gaps[0]!;
      let clickAt: number | null = null;
      let next = Date.now();
      const due: number[] = gaps.slice(1).map((g) => (next += g));
      const end = due[due.length - 1]!;
      // From the second sample on, check every 50 ms.
      while (Date.now() < end) {
        vi.advanceTimersByTime(50);
        while (due.length && due[0]! <= Date.now()) {
          due.shift();
          deliver();
        }
        expect(isLive(store)).toBe(true);
        expect(hudLines(store, Date.now(), t)).not.toEqual(["No pod data"]);
        if (Date.now() - mountAt < 1_000) continue;
        expect(link.textContent).not.toBe("No pod data");
        if (clickAt === null) {
          btn.click();
          clickAt = Date.now();
        }
        if (Date.now() - clickAt < 5_000) expect(btn.textContent).toBe("Confirm: fire laser");
      }
      expect(clickAt).not.toBeNull();
      handle.destroy();
    });
  }

  it("a stopped feed reads stale at 2000 ms after the last sample", () => {
    const store = new PodStateStore();
    for (const gap of hostGaps(6)) {
      store.ingest({ ...ZT30, ts_ms: Date.now() - 2_000 - ENTRY_AGE_MS });
      vi.advanceTimersByTime(gap);
    }
    store.ingest({ ...ZT30, ts_ms: Date.now() - 2_000 - ENTRY_AGE_MS });
    vi.advanceTimersByTime(1_999);
    expect(isLive(store)).toBe(true);
    vi.advanceTimersByTime(1);
    expect(isLive(store)).toBe(false);
  });

  for (const order of [
    ["telemetry", "event"],
    ["event", "telemetry"],
  ] as const) {
    it(`keeps an old report stale on mount when both channels carry it with their own stamps (${order[0]} first)`, async () => {
      // The drone stamps the telemetry entry and the throttled event entry
      // separately, so one report arrives twice with different ts_ms.
      const harness = createPluginHarness({
        grantedCapabilities: ["command.send", "telemetry.subscribe.siyi"],
        mount: () => undefined,
      });
      await harness.start();
      harness.ctx.i18n.t = t;
      const root = document.createElement("div");
      document.body.appendChild(root);
      const handles = await mountSiyi(harness.ctx, root);
      const T = Date.now() - 5_000;
      const stamps = { telemetry: T + 3, event: T };
      for (const channel of order) {
        const payload = { ...ZT30, ts_ms: stamps[channel] };
        if (channel === "telemetry") harness.pushTelemetry("siyi", payload);
        else harness.pushEvent("siyi.pod.state", payload);
        await Promise.resolve();
        vi.advanceTimersByTime(5);
      }
      const link = root.querySelector('[data-testid="siyi-link"]')!;
      const btn = root.querySelector<HTMLButtonElement>('[data-testid="siyi-range-btn"]')!;
      // Through the panel's next 1 s repaint, the old pair reads as old.
      for (let i = 0; i < 30; i++) {
        vi.advanceTimersByTime(50);
        expect(link.textContent).toBe("No pod data");
        expect(btn.disabled).toBe(true);
      }
      for (const h of handles) h.destroy();
      root.remove();
      await harness.teardown();
    });
  }

  it("keeps the old pair stale for the HUD too, and reads live from each channel's next report", () => {
    const store = new PodStateStore();
    const T = Date.now() - 5_000;
    store.ingest({ ...ZT30, ts_ms: T + 3 }, "telemetry");
    vi.advanceTimersByTime(5);
    store.ingest({ ...ZT30, ts_ms: T }, "event");
    expect(isLive(store)).toBe(false);
    expect(dataAgeS(store)).toBe(5);
    expect(hudLines(store, Date.now(), t)).toEqual(["No pod data"]);
    vi.advanceTimersByTime(745);
    store.ingest({ ...ZT30, ts_ms: T + 753 }, "telemetry");
    expect(isLive(store)).toBe(true);
  });

  it("stays live, then stale at 2000 ms, when each channel carries its own stamps at the host cadence", () => {
    // Telemetry is stamped as each report is written; the event is throttled
    // to 2 Hz and stamped later, so it often repeats or holds an earlier
    // report than the telemetry entry beside it.
    const store = new PodStateStore();
    let eventTs = 0;
    let lastEventWrite = -Infinity;
    for (const gap of hostGaps(20)) {
      const telemetryTs = Date.now() - 2_000 - ENTRY_AGE_MS;
      if (telemetryTs - lastEventWrite >= 500) {
        lastEventWrite = telemetryTs;
        eventTs = telemetryTs + 137;
      }
      store.ingest({ ...ZT30, zoom: telemetryTs, ts_ms: telemetryTs }, "telemetry");
      store.ingest({ ...ZT30, ts_ms: eventTs }, "event");
      for (let waited = 0; waited < gap; waited += 50) {
        vi.advanceTimersByTime(Math.min(50, gap - waited));
        if (Date.now() - T0 > 1_000) expect(isLive(store)).toBe(true);
      }
    }
    const lastAt = store.receivedAt()!;
    vi.setSystemTime(lastAt + 1_999);
    expect(isLive(store)).toBe(true);
    vi.setSystemTime(lastAt + 2_000);
    expect(isLive(store)).toBe(false);
  });

  it("ignores a report stamped older than the one shown on the same channel", () => {
    const store = new PodStateStore();
    store.ingest({ ...ZT30, zoom: 1, ts_ms: Date.now() - 1_000 }, "telemetry");
    vi.advanceTimersByTime(750);
    const newest = Date.now() - 1_000;
    store.ingest({ ...ZT30, zoom: 2, ts_ms: newest }, "telemetry");
    const shownAt = store.receivedAt()!;
    vi.advanceTimersByTime(750);
    // A failed poll: the host falls back to a source stamped 4 s earlier.
    store.ingest({ ...ZT30, zoom: 9, ts_ms: newest - 4_000 }, "telemetry");
    expect(store.get()!.zoom).toBe(2);
    expect(store.receivedAt()).toBe(shownAt);
    vi.setSystemTime(shownAt + 2_000);
    expect(isLive(store)).toBe(false);
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
