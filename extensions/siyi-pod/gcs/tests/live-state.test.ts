import { afterEach, beforeEach, describe, expect, it, vi } from "vitest";

import {
  createPluginHarness,
  type PluginHarness,
} from "@altnautica/plugin-sdk/harness";

import { hudLines } from "../src/overlay-draw";
import { mountPanel } from "../src/panel";
import {
  isLive,
  PodStateStore,
  rangeReading,
  RANGE_SHOW_MS,
  STALE_AFTER_MS,
} from "../src/pod-state";
import enLocale from "../../locales/en.json";

const T0 = 1_800_000_000_000;

const ZT30 = {
  model: "ZT30",
  known: true,
  connected: true,
  link_ok: true,
  laser_enabled: true,
  gimbal_mode: "follow",
  palette: 3,
  zoom: 2,
  capabilities: {
    gimbal: true,
    zoom: true,
    thermal: true,
    laser: true,
    max_zoom: 180,
    sensors: ["eo_zoom", "ir"],
  },
  assignment: { main: "eo_zoom", sub: "ir" },
};

const t = (key: string, params?: Record<string, string | number>): string => {
  let s = (enLocale as Record<string, string>)[key] ?? key;
  for (const [k, v] of Object.entries(params ?? {})) s = s.replace(`{${k}}`, String(v));
  return s;
};

async function setup(raw: unknown) {
  const harness = createPluginHarness({
    grantedCapabilities: ["command.send"],
    mount: () => undefined,
  });
  await harness.start();
  harness.ctx.i18n.t = t;
  const store = new PodStateStore();
  store.ingest(raw);
  const root = document.createElement("div");
  document.body.appendChild(root);
  const handle = mountPanel(harness.ctx, root, store);
  return { harness, store, root, handle };
}

async function done(h: { harness: PluginHarness; root: HTMLElement; handle: { destroy(): void } }) {
  h.handle.destroy();
  h.root.remove();
  await h.harness.teardown();
}

const q = (root: HTMLElement, id: string) =>
  root.querySelector(`[data-testid="${id}"]`) as HTMLElement;

function configWrites(harness: PluginHarness, key: string) {
  return harness.calls.filter(
    (c) => c.method === "command.send" && JSON.stringify(c.args).includes(`"${key}"`),
  );
}

beforeEach(() => {
  vi.useFakeTimers();
  vi.setSystemTime(T0);
});
afterEach(() => {
  vi.useRealTimers();
});

describe("pod link header", () => {
  it("reads Connected only while the pod answers", async () => {
    const h = await setup(ZT30);
    expect(q(h.root, "siyi-link").textContent).toBe("Connected");
    h.store.ingest({ ...ZT30, link_ok: false });
    expect(q(h.root, "siyi-link").textContent).toBe("Pod not answering");
    expect(q(h.root, "siyi-body").classList.contains("siyi-lost")).toBe(true);
    expect(q(h.root, "siyi-body").style.opacity).toBe("0.45");
    await done(h);
  });
});

describe("pod data age", () => {
  it("stamps a receive time and goes stale past one second", () => {
    const store = new PodStateStore();
    store.ingest(ZT30);
    expect(store.receivedAt()).toBe(T0);
    expect(isLive(store, T0 + STALE_AFTER_MS - 1)).toBe(true);
    expect(isLive(store, T0 + STALE_AFTER_MS)).toBe(false);
  });

  it("shows No pod data and drops live values once the feed stops", async () => {
    const h = await setup({ ...ZT30, laser_range_m: 182, laser_range_at: T0 });
    expect(q(h.root, "siyi-range").textContent).toContain("182 m");
    vi.advanceTimersByTime(2_000);
    expect(q(h.root, "siyi-link").textContent).toBe("No pod data");
    expect(q(h.root, "siyi-age").textContent).toBe("Last update 2 s ago");
    expect(q(h.root, "siyi-range").textContent).toBe("—");
    expect(q(h.root, "siyi-body").classList.contains("siyi-lost")).toBe(true);
    await done(h);
  });
});

describe("laser range age", () => {
  it("shows the range with its age and hides it after the window", async () => {
    const h = await setup({ ...ZT30, laser_range_m: 182, laser_range_at: T0 - 3_000 });
    expect(q(h.root, "siyi-range").textContent).toBe("182 m · 3 s ago");
    // Keep the feed live while the range ages out.
    for (let i = 0; i < 8; i++) {
      vi.advanceTimersByTime(1_000);
      h.store.ingest({ ...ZT30, laser_range_m: 182, laser_range_at: T0 - 3_000, frames_received: i });
    }
    expect(q(h.root, "siyi-range").textContent).toBe("—");
    await done(h);
  });

  it("drops a range with no measurement time and one past the window", () => {
    const store = new PodStateStore();
    store.ingest({ ...ZT30, laser_range_m: 50 });
    expect(rangeReading(store.get(), T0)).toBeNull();
    store.ingest({ ...ZT30, laser_range_m: 50, laser_range_at: T0 - RANGE_SHOW_MS });
    expect(rangeReading(store.get(), T0)).toBeNull();
    store.ingest({ ...ZT30, laser_range_m: 50, laser_range_at: T0 - 4_000 });
    expect(rangeReading(store.get(), T0)).toEqual({ range_m: 50, age_s: 4 });
  });

  it("paints the HUD range with its age and only while fresh and live", () => {
    const store = new PodStateStore();
    store.ingest({ ...ZT30, zoom: 1, laser_range_m: 182, laser_range_at: T0 - 12_000 });
    expect(hudLines(store, T0, t).join("|")).not.toContain("182");
    store.ingest({ ...ZT30, zoom: 1, laser_range_m: 182, laser_range_at: T0 - 2_000 });
    expect(hudLines(store, T0, t)).toEqual(["Range 182 m · 2 s ago"]);
    expect(hudLines(store, T0 + STALE_AFTER_MS, t)).toEqual(["No pod data"]);
  });
});

describe("in-place updates", () => {
  it("skips notify when the state is unchanged", () => {
    const store = new PodStateStore();
    let n = 0;
    store.subscribe(() => n++);
    store.ingest(ZT30);
    store.ingest(ZT30);
    expect(n).toBe(1);
    store.ingest({ ...ZT30, zoom: 3 });
    expect(n).toBe(2);
  });

  it("keeps the same controls across updates and leaves a focused control alone", async () => {
    const h = await setup(ZT30);
    const mode = q(h.root, "siyi-mode") as HTMLSelectElement;
    const zoom = h.root.querySelector("input[type=range]") as HTMLInputElement;
    mode.focus();
    mode.value = "lock";
    h.store.ingest({ ...ZT30, zoom: 5 });
    expect(q(h.root, "siyi-mode")).toBe(mode);
    expect(h.root.querySelector("input[type=range]")).toBe(zoom);
    expect(zoom.value).toBe("5");
    expect(document.activeElement).toBe(mode);
    expect(mode.value).toBe("lock");
    await done(h);
  });
});

describe("range button", () => {
  it("is disabled with a reason while the rangefinder is not armed", async () => {
    const h = await setup({ ...ZT30, laser_enabled: false });
    const btn = q(h.root, "siyi-range-btn") as HTMLButtonElement;
    expect(btn.disabled).toBe(true);
    expect(q(h.root, "siyi-laser-note").textContent).toBe(
      "Arm the rangefinder in the pod settings to measure range.",
    );
    btn.click();
    expect(configWrites(h.harness, "laser_fire_nonce")).toHaveLength(0);
    await done(h);
  });

  it("asks for confirmation before it fires", async () => {
    const h = await setup(ZT30);
    const btn = q(h.root, "siyi-range-btn") as HTMLButtonElement;
    btn.click();
    expect(configWrites(h.harness, "laser_fire_nonce")).toHaveLength(0);
    expect(btn.textContent).toBe("Confirm: fire laser");
    btn.click();
    await vi.waitFor(() => expect(configWrites(h.harness, "laser_fire_nonce")).toHaveLength(1));
    expect(btn.textContent).toBe("Range");
    await done(h);
  });

  it("drops an unanswered confirmation", async () => {
    const h = await setup(ZT30);
    const btn = q(h.root, "siyi-range-btn") as HTMLButtonElement;
    btn.click();
    vi.advanceTimersByTime(5_000);
    expect(btn.textContent).toBe("Range");
    btn.click();
    expect(configWrites(h.harness, "laser_fire_nonce")).toHaveLength(0);
    await done(h);
  });
});

describe("operator words", () => {
  it("names modes, palettes and the recentre action", async () => {
    const h = await setup(ZT30);
    const modes = Array.from((q(h.root, "siyi-mode") as HTMLSelectElement).options);
    expect(modes.map((o) => o.textContent)).toEqual(["Lock", "Follow", "FPV"]);
    const pal = q(h.root, "siyi-palette") as HTMLSelectElement;
    expect(pal.tagName).toBe("SELECT");
    expect(Array.from(pal.options).map((o) => o.textContent)).toEqual([
      "White hot", "Sepia", "Ironbow", "Rainbow", "Night", "Aurora", "Red hot",
    ]);
    expect(Math.max(...Array.from(pal.options).map((o) => Number(o.value)))).toBe(7);
    expect(pal.value).toBe("3");
    const labels = Array.from(h.root.querySelectorAll("button")).map((b) => b.textContent);
    expect(labels).toContain("Recenter");
    expect(labels).not.toContain("Center");
    await done(h);
  });

  it("explains a refused command in a sentence", async () => {
    const h = await setup(ZT30);
    h.harness.failNext("command.send", "refused", "capability_denied:token_missing");
    (Array.from(h.root.querySelectorAll("button")).find((b) => b.textContent === "Photo"))!.click();
    const status = q(h.root, "siyi-command-status");
    await vi.waitFor(() =>
      expect(status.textContent).toBe("The command was not sent: the ground station refused it."),
    );
    await done(h);
  });

  it("uses the locale for every panel string", async () => {
    const h = await setup({ ...ZT30, known: false });
    const keys = Object.keys(enLocale);
    for (const k of [
      "panel.connected", "panel.notAnswering", "panel.noData", "panel.connecting",
      "panel.unknownModel", "panel.recenter", "panel.photo", "panel.record",
      "panel.stopRecord", "panel.highGain", "panel.lowGain", "panel.range",
      "panel.rangeConfirm", "mode.lock", "mode.follow", "mode.fpv", "palette.0",
      "palette.7", "error.refused", "error.timeout", "error.failed",
    ]) {
      expect(keys).toContain(k);
    }
    expect(h.root.textContent).toContain("This pod model is not recognised");
    await done(h);
  });
});
