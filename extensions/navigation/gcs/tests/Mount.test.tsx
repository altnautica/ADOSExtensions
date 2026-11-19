import { act, cleanup, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";

import { AGENT_PAGE_ID, HEALTH_PANEL_ID, Page } from "../src/components/Page";

import { harness } from "./harness";

const T0 = Date.UTC(2026, 0, 1, 12, 0, 0);
const NAV = { mode: "external_nav", active_set: 3, extnav_health: "good", odom_rate_hz: 10 };

afterEach(() => {
  cleanup();
  vi.useRealTimers();
});

describe("the bundle routes on the contribution it was mounted as", () => {
  it("the cockpit panel says there is no navigation state until one arrives, then shows it", () => {
    vi.useFakeTimers();
    vi.setSystemTime(T0);
    const h = harness({ panelId: HEALTH_PANEL_ID });
    render(<Page ctx={h.ctx} firmware="ardupilot" />);
    expect(screen.getByTestId("vn-health-none").textContent).toBe("No navigation state");
    act(() => h.emitEvent("plugin.navigation.state", { ...NAV, ts_ms: T0 }));
    expect(screen.queryByTestId("vn-health-none")).toBeNull();
    expect(screen.getByTestId("vn-nav-state").textContent).toContain("Set 3");
  });

  it("the cockpit panel drops old numbers once plugin.navigation.state goes stale", () => {
    vi.useFakeTimers();
    vi.setSystemTime(T0);
    const h = harness({ panelId: HEALTH_PANEL_ID });
    render(<Page ctx={h.ctx} firmware="ardupilot" />);
    act(() => h.emitEvent("plugin.navigation.state", { ...NAV, ts_ms: T0 }));
    act(() => {
      vi.advanceTimersByTime(4000);
    });
    expect(screen.getByTestId("vn-health-none").textContent).toBe("No navigation state · 4 s");
    expect(screen.queryByTestId("vn-nav-state")).toBeNull();
  });

  it("the Agent page is the full Navigation page", () => {
    const h = harness({ panelId: AGENT_PAGE_ID });
    render(<Page ctx={h.ctx} firmware="ardupilot" />);
    expect(screen.getByTestId("vn-navigation-tab")).toBeTruthy();
  });

  it("an unknown contribution says so instead of guessing", () => {
    const h = harness({ panelId: "vision-nav-tab" });
    render(<Page ctx={h.ctx} firmware="ardupilot" />);
    expect(screen.getByTestId("vn-unknown-page").textContent).toBe("Unknown Navigation page vision-nav-tab");
  });
});
