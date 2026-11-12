import { readFileSync } from "node:fs";
import { resolve } from "node:path";

import { afterEach, describe, expect, it, vi } from "vitest";
import { act, cleanup, render, screen } from "@testing-library/react";

import { createPluginHarness, type PluginHarness } from "@altnautica/plugin-sdk/harness";

import { FollowMeTab } from "../src/FollowMeTab";
import { phaseColor } from "../src/style";
import { PLUGIN_VERSION } from "../src/version";
import enLocale from "../../locales/en.json";
import {
  EMPTY_FOLLOW_STATE,
  FOLLOW_STATE_TOPIC,
  modeSummary,
  normalizeFollowState,
  type FollowState,
} from "../src/types";

afterEach(() => {
  cleanup();
  vi.useRealTimers();
});

function harness() {
  return createPluginHarness({
    mount: () => undefined,
    locale: enLocale as Record<string, string>,
  });
}

const t = (key: string, params?: Record<string, string | number>): string =>
  harness().ctx.i18n.t(key, params);

const following: FollowState = {
  ...EMPTY_FOLLOW_STATE,
  active: true,
  phase: "following",
  lockState: "locked",
  commanding: true,
  fcArmed: true,
  fcGuided: true,
  distanceSetpointM: 8,
  heightSetpointM: 4,
  setpointRateHz: 20,
};

describe("live read-back freshness", () => {
  it("reads Stale with the age and blanks values once the report is 2 s old", async () => {
    vi.useFakeTimers();
    vi.setSystemTime(new Date("2026-01-01T00:00:00Z"));
    const h = harness();
    render(<FollowMeTab ctx={h.ctx} />);
    act(() => {
      h.pushEvent(FOLLOW_STATE_TOPIC, {
        active: true,
        phase: "following",
        commanding: true,
        setpoint_rate_hz: 20,
      });
    });
    await act(async () => {
      await vi.advanceTimersByTimeAsync(1000);
    });
    expect(screen.getByTestId("fm-phase").textContent).toBe("Following");
    expect(screen.queryByTestId("fm-status")).toBeNull();
    await act(async () => {
      await vi.advanceTimersByTimeAsync(2000);
    });
    expect(screen.getByTestId("fm-status").textContent).toBe(
      "Stale · last report 3 s ago",
    );
    expect(screen.getByTestId("fm-phase").textContent).toBe("—");
    expect(screen.getByTestId("fm-commanding").textContent).toBe("—");
    expect(screen.getByTestId("fm-setpoint-rate").textContent).toBe("—");
  });

  it("ages a report from its own ts_ms when the payload carries one", async () => {
    vi.useFakeTimers();
    vi.setSystemTime(new Date("2026-01-01T00:00:10Z"));
    const h = harness();
    render(<FollowMeTab ctx={h.ctx} />);
    act(() => {
      h.pushEvent(FOLLOW_STATE_TOPIC, {
        phase: "following",
        ts_ms: Date.now() - 5000,
      });
    });
    expect(screen.getByTestId("fm-status").textContent).toBe(
      "Stale · last report 5 s ago",
    );
  });

  // The host delivers every ~750 ms with jitter, drops every fifth tick (a
  // 1500 ms gap), and each entry is already ~1000 ms old when it arrives,
  // stamped with the drone's own clock.
  async function runHostCadence(
    h: PluginHarness,
    driftMs: number,
    samples: number,
    onCheck: (afterSecond: boolean) => void,
  ): Promise<void> {
    const jitter = [0, 40, -30, 25, -45, 10];
    let delivered = 0;
    for (let tick = 0; delivered < samples; tick += 1) {
      if (tick % 5 !== 4) {
        act(() => {
          h.pushEvent(FOLLOW_STATE_TOPIC, {
            active: true,
            phase: "following",
            ts_ms: Date.now() - 1000 - driftMs,
          });
        });
        delivered += 1;
      }
      const step = 750 + (jitter[tick % jitter.length] ?? 0);
      for (let elapsed = 0; elapsed < step; elapsed += 50) {
        await act(async () => {
          await vi.advanceTimersByTimeAsync(Math.min(50, step - elapsed));
        });
        onCheck(delivered >= 2);
      }
    }
  }

  for (const driftMs of [2000, 5000]) {
    it(`stays live while the drone clock runs ${driftMs / 1000} s behind`, async () => {
      vi.useFakeTimers();
      vi.setSystemTime(new Date("2026-01-01T00:00:10Z"));
      const h = harness();
      render(<FollowMeTab ctx={h.ctx} />);
      const staleAt: number[] = [];
      await runHostCadence(h, driftMs, 20, (afterSecond) => {
        if (afterSecond && screen.queryByTestId("fm-status") !== null) {
          staleAt.push(Date.now());
        }
      });
      expect(staleAt).toEqual([]);
      expect(screen.getByTestId("fm-phase").textContent).toBe("Following");
    });
  }

  it("reads Stale on mount when the first report is 5 s old", async () => {
    vi.useFakeTimers();
    vi.setSystemTime(new Date("2026-01-01T00:00:10Z"));
    const h = harness();
    render(<FollowMeTab ctx={h.ctx} />);
    act(() => {
      h.pushEvent(FOLLOW_STATE_TOPIC, { phase: "following", ts_ms: Date.now() - 5000 });
    });
    expect(screen.getByTestId("fm-status").textContent).toBe(
      "Stale · last report 5 s ago",
    );
    expect(screen.getByTestId("fm-phase").textContent).toBe("—");
  });

  it("reads Stale exactly 2 s after a live feed stops", async () => {
    vi.useFakeTimers();
    vi.setSystemTime(new Date("2026-01-01T00:00:10Z"));
    const h = harness();
    render(<FollowMeTab ctx={h.ctx} />);
    await runHostCadence(h, 0, 6, () => undefined);
    act(() => {
      h.pushEvent(FOLLOW_STATE_TOPIC, {
        active: true,
        phase: "following",
        ts_ms: Date.now() - 1000,
      });
    });
    await act(async () => {
      await vi.advanceTimersByTimeAsync(1999);
    });
    expect(screen.queryByTestId("fm-status")).toBeNull();
    expect(screen.getByTestId("fm-phase").textContent).toBe("Following");
    await act(async () => {
      await vi.advanceTimersByTimeAsync(1);
    });
    expect(screen.getByTestId("fm-status").textContent).toBe(
      "Stale · last report 2 s ago",
    );
  });

  it("ignores the same report delivered again", async () => {
    vi.useFakeTimers();
    vi.setSystemTime(new Date("2026-01-01T00:00:10Z"));
    const h = harness();
    render(<FollowMeTab ctx={h.ctx} />);
    const report = { active: true, phase: "following", ts_ms: Date.now() };
    act(() => h.pushEvent(FOLLOW_STATE_TOPIC, report));
    for (let i = 0; i < 4; i += 1) {
      await act(async () => {
        await vi.advanceTimersByTimeAsync(750);
      });
      act(() => h.pushEvent(FOLLOW_STATE_TOPIC, report));
    }
    expect(screen.getByTestId("fm-status").textContent).toBe(
      "Stale · last report 3 s ago",
    );
  });
});

describe("before the first report", () => {
  it("says it is waiting and shows dashes rather than real-looking readings", () => {
    const h = harness();
    render(<FollowMeTab ctx={h.ctx} />);
    expect(screen.getByTestId("fm-status").textContent).toBe(
      "Waiting for the drone's follow report",
    );
    for (const id of [
      "fm-phase",
      "fm-reacquire-mode",
      "fm-commanding",
      "fm-fc-armed",
      "fm-fc-guided",
      "fm-mode",
      "fm-setpoint-rate",
      "fm-lease",
    ]) {
      expect(screen.getByTestId(id).textContent).toBe("—");
    }
  });
});

describe("pilot lease", () => {
  it("accepts the integer lease id the pilot issues and reads Held", () => {
    const s = normalizeFollowState({ lease_id: 7, commanding: true });
    expect(s.leaseId).toBe(7);
    const h = harness();
    render(<FollowMeTab ctx={h.ctx} followOverride={s} />);
    expect(screen.getByTestId("fm-lease").textContent).toBe("Held");
  });

  it("reads None without a lease", () => {
    const h = harness();
    render(<FollowMeTab ctx={h.ctx} followOverride={following} />);
    expect(screen.getByTestId("fm-lease").textContent).toBe("None");
  });
});

describe("orbit geometry", () => {
  it("shows the orbit radius instead of a follow distance setpoint", () => {
    const h = harness();
    render(
      <FollowMeTab
        ctx={h.ctx}
        followOverride={{
          ...following,
          followMode: "orbit",
          orbitRadiusM: 10.5,
          distanceSetpointM: 8,
        }}
      />,
    );
    expect(screen.queryByTestId("fm-distance-setpoint")).toBeNull();
    expect(screen.getByTestId("fm-orbit-radius").textContent).toBe("10.5 m");
    expect(screen.getByText("Orbit radius")).toBeTruthy();
  });
});

describe("words through the locale", () => {
  it("builds the mode summary from keyed words", () => {
    const orbit = normalizeFollowState({
      follow_mode: "orbit",
      orbit_radius_m: 10.5,
      orbit_rate_dps: 12,
      orbit_direction: "ccw",
    });
    expect(modeSummary(orbit, null, t)).toBe(
      "Orbit 10.5 m · 12°/s · Counter-clockwise",
    );
    expect(modeSummary({ ...orbit, followMode: "follow" }, 8, t)).toBe(
      "Follow 180° · 8 m",
    );
  });

  it("names the auto camera instead of the raw config value", () => {
    const h = harness();
    render(<FollowMeTab ctx={h.ctx} followOverride={following} />);
    expect(screen.getByTestId("fm-camera").textContent).toBe(
      "Auto (first detect camera)",
    );
  });

  it("names the configured camera from the host roster as Name (id)", () => {
    const h = harness();
    render(<FollowMeTab ctx={h.ctx} followOverride={following} />);
    act(() => {
      h.pushEvent("config.changed", { designate_camera: "cam-front-0" });
      h.pushEvent("cameras.changed", {
        cameras: [
          { id: "cam-front-0", name: "Front camera" },
          { id: "cam-down-1", name: null },
        ],
      });
    });
    expect(screen.getByTestId("fm-camera").textContent).toBe(
      "Front camera (cam-front-0)",
    );
  });

  it("falls back to the camera id when the roster has no name for it", () => {
    const h = harness();
    render(<FollowMeTab ctx={h.ctx} followOverride={following} />);
    act(() => {
      h.pushEvent("config.changed", { designate_camera: "cam-down-1" });
      h.pushEvent("cameras.changed", {
        cameras: [{ id: "cam-down-1", name: null }],
      });
    });
    expect(screen.getByTestId("fm-camera").textContent).toBe("cam-down-1");
  });

  it("keys every unit template", () => {
    const en = enLocale as Record<string, string>;
    for (const key of [
      "unit.m",
      "unit.mps",
      "unit.ms",
      "unit.hz",
      "unit.s",
      "unit.dps",
      "unit.deg",
      "unit.rangeSigma",
      "unit.targetId",
    ]) {
      expect(en[key], key).toContain("{value}");
    }
  });
});

describe("tone and style", () => {
  it("draws holding as a warning and acquiring in the track tone", () => {
    expect(phaseColor("holding")).toContain("--hud-warn");
    expect(phaseColor("acquiring")).toContain("--hud-track");
  });

  it("uses the host UI font and muted labels on every row", () => {
    const h = harness();
    render(<FollowMeTab ctx={h.ctx} followOverride={following} />);
    const tab = screen.getByTestId("fm-tab") as HTMLElement;
    expect(tab.style.fontFamily).toContain("system-ui");
    for (const label of ["Phase", "Lock state"]) {
      const el = screen.getByText(label) as HTMLElement;
      expect(el.getAttribute("style") ?? "", label).toContain("--fm-text-muted");
    }
  });
});

describe("plugin version", () => {
  it("matches the manifest version", () => {
    const manifest = readFileSync(
      resolve(__dirname, "../../manifest.yaml"),
      "utf8",
    );
    const m = /^version:\s*"?([^"\s]+)"?/m.exec(manifest);
    expect(PLUGIN_VERSION).toBe(m?.[1]);
  });
});
