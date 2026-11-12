import { act, cleanup, render, screen, within } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";

import { NavigationTab } from "../src/components/NavigationTab";
import type { VisionNavTelemetry } from "../src/types";

import { harness, type Harness } from "./harness";

const T0 = Date.UTC(2026, 0, 1, 12, 0, 0);

const SNAPSHOT = {
  opticalFlowSupported: true,
  companionState: "active",
  mode: "off",
  availableEstimators: ["off", "external_nav"],
  estimatorState: "off",
  flowQuality: 186,
};
const NAV = { mode: "external_nav", active_set: 1, extnav_health: "good", odom_rate_hz: 10 };

afterEach(() => {
  cleanup();
  vi.useRealTimers();
});

function mount(): Harness {
  vi.useFakeTimers();
  vi.setSystemTime(T0);
  const h = harness();
  render(<NavigationTab ctx={h.ctx} firmware="ardupilot" />);
  return h;
}

const navStale = (): string | undefined => screen.getByTestId("vn-nav-state").dataset.stale;

describe("a payload carrying an old agent timestamp", () => {
  it("shows the navigation block stale on first paint", () => {
    const h = mount();
    act(() => h.emitTelemetry("navigation", { ...SNAPSHOT, ts_ms: T0 - 5000 }));
    expect(screen.getByTestId("vn-feed-stale").textContent).toContain("5");
    expect(screen.getByTestId("vn-companion-pill").textContent).not.toContain("Active");
  });

  it("shows nav.state stale on first paint", () => {
    const h = mount();
    act(() => h.emitEvent("nav.state", { ...NAV, ts_ms: T0 - 5000 }));
    expect(navStale()).toBe("true");
  });

  it("does not take an old engage state as current on first paint", () => {
    const h = mount();
    act(() => {
      h.emitTelemetry("navigation", { ...SNAPSHOT, ts_ms: T0 });
      h.emitEvent("navigation.engage", { state: "idle", ts_ms: T0 - 5000 });
    });
    expect(screen.queryByTestId("vn-mode-disengaged")).toBeNull();
  });

  it("never ages a payload stamped ahead of this clock past its arrival", () => {
    const h = mount();
    act(() => h.emitTelemetry("navigation", { ...SNAPSHOT, ts_ms: T0 + 60_000 }));
    expect(screen.queryByTestId("vn-feed-stale")).toBeNull();
    act(() => vi.advanceTimersByTime(2000));
    expect(screen.getByTestId("vn-feed-stale")).toBeTruthy();
  });

  it("does not reset the age when the same agent timestamp is delivered again", () => {
    const h = mount();
    const nav = { ...SNAPSHOT, ts_ms: T0 };
    act(() => h.emitTelemetry("navigation", nav));
    for (let i = 0; i < 4; i += 1) {
      act(() => vi.advanceTimersByTime(750));
      act(() => h.emitTelemetry("navigation", { ...nav }));
    }
    expect(screen.getByTestId("vn-feed-stale")).toBeTruthy();
  });
});

describe("the host's delivery cadence", () => {
  // MC polls the sidecar every 750 ms with jitter and skips every 5th
  // tick (a 1500 ms gap); each entry is already ~1000 ms old on arrival.
  const JITTER = [0, 60, -60, 40, -30, 20, -50, 30];
  function runCadence(h: Harness, skewMs: number, ticks: number): void {
    const deliver = (): void => {
      const ts = Date.now() - 1000 - skewMs;
      act(() => {
        h.emitTelemetry("navigation", { ...SNAPSHOT, ts_ms: ts });
        h.emitEvent("nav.state", { ...NAV, ts_ms: ts });
        h.emitEvent("navigation.engage", { state: "idle", ts_ms: ts });
      });
    };
    deliver();
    act(() => vi.advanceTimersByTime(750));
    deliver();
    for (let tick = 2; tick < ticks; tick += 1) {
      const gap = tick % 5 === 4 ? 1500 : 750 + (JITTER[tick % JITTER.length] ?? 0);
      for (let step = 0; step < gap; step += 50) {
        act(() => vi.advanceTimersByTime(Math.min(50, gap - step)));
        expect(screen.queryByTestId("vn-feed-stale")).toBeNull();
        expect(navStale()).toBe("false");
        expect(screen.getByTestId("vn-mode-disengaged")).toBeTruthy();
      }
      deliver();
    }
  }

  it("stays live with the drone clock 2 s behind", () => {
    runCadence(mount(), 2000, 30);
  });

  it("stays live with the drone clock 5 s behind", () => {
    runCadence(mount(), 5000, 30);
  });

  it("reads an old first sample stale on mount", () => {
    const h = mount();
    act(() => {
      h.emitTelemetry("navigation", { ...SNAPSHOT, ts_ms: T0 - 5000 });
      h.emitEvent("nav.state", { ...NAV, ts_ms: T0 - 5000 });
      h.emitEvent("navigation.engage", { state: "idle", ts_ms: T0 - 5000 });
    });
    expect(screen.getByTestId("vn-feed-stale")).toBeTruthy();
    expect(navStale()).toBe("true");
    expect(screen.queryByTestId("vn-mode-disengaged")).toBeNull();
  });

  it("reads a stopped feed stale 2000 ms after its last sample", () => {
    const h = mount();
    const deliver = (): void => {
      const ts = Date.now() - 1000 - 2000;
      act(() => {
        h.emitTelemetry("navigation", { ...SNAPSHOT, ts_ms: ts });
        h.emitEvent("nav.state", { ...NAV, ts_ms: ts });
      });
    };
    deliver();
    act(() => vi.advanceTimersByTime(1000));
    deliver();
    act(() => vi.advanceTimersByTime(1999));
    expect(screen.queryByTestId("vn-feed-stale")).toBeNull();
    expect(navStale()).toBe("false");
    act(() => vi.advanceTimersByTime(1));
    expect(screen.getByTestId("vn-feed-stale")).toBeTruthy();
    expect(navStale()).toBe("true");
  });
});

describe("trend samples", () => {
  it("adds one sample per agent report, not per delivery", () => {
    const h = mount();
    act(() => h.emitTelemetry("navigation", { ...SNAPSHOT, flowQuality: 100, ts_ms: T0 }));
    act(() => h.emitTelemetry("navigation", { ...SNAPSHOT, flowQuality: 100, ts_ms: T0 }));
    const spark = screen.getByTestId("vn-sparkline-flow-quality");
    expect(spark.querySelector("polyline")).toBeNull();
    act(() => vi.advanceTimersByTime(500));
    act(() => h.emitTelemetry("navigation", { ...SNAPSHOT, flowQuality: 120, ts_ms: T0 + 500 }));
    const points = spark.querySelector("polyline")?.getAttribute("points") ?? "";
    expect(points.split(" ")).toHaveLength(2);
  });
});

describe("rangefinder pill against the pre-arm report", () => {
  it("is red when pre-arm blocks the rangefinder even with a reading", () => {
    const telemetry: VisionNavTelemetry = {
      opticalFlowSupported: true,
      mode: "optical_flow",
      rangefinderTopology: "companion",
      flowDistanceM: 42,
      preArmReport: {
        mode: "optical_flow",
        armable: false,
        checks: [{ id: "rangefinder", severity: "blocking", detail: "outside range" }],
      },
    };
    render(<NavigationTab ctx={harness().ctx} firmware="ardupilot" telemetryOverride={telemetry} />);
    const pill = within(screen.getByTestId("vn-sensors-rangefinder")).getByTestId("vn-rangefinder-pill");
    expect(pill.dataset.kind).toBe("error");
  });
});

describe("subscription failures", () => {
  it("says the subscription could not be made when the host fails it for another reason", async () => {
    render(<NavigationTab ctx={harness({ failTelemetry: "timeout" }).ctx} firmware="ardupilot" />);
    await act(async () => {
      await Promise.resolve();
    });
    expect(screen.queryByTestId("vn-pre-arm-denied")).toBeNull();
    expect(screen.getByTestId("vn-pre-arm-subscribe-failed").textContent).toBe(
      "Could not subscribe to navigation telemetry.",
    );
  });
});
