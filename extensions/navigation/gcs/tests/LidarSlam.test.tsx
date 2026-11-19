import { act, cleanup, render, screen } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";

import { AGENT_PAGE_ID, Page } from "../src/components/Page";

import { harness } from "./harness";

const T0 = Date.UTC(2026, 0, 1, 12, 0, 0);

afterEach(() => {
  cleanup();
  vi.useRealTimers();
});

describe("the Agent page shows the lidar-slam service", () => {
  it("names the provider, the LiDAR link and the live map from their topics", () => {
    vi.useFakeTimers();
    vi.setSystemTime(T0);
    const h = harness({ panelId: AGENT_PAGE_ID });
    render(<Page ctx={h.ctx} firmware="ardupilot" />);
    expect(screen.getByTestId("vn-slam-provider").textContent).toBe("No data");
    expect(screen.getByTestId("vn-slam-no-lidar").textContent).toBe("The rig declares no LiDAR.");

    act(() =>
      h.emitEvent("plugin.navigation.map_status", {
        placement: "stream",
        memory_used_mb: 101.6,
        memory_budget_mb: 256,
        voxel_m: 0.2,
        sweep_points: 9000,
        area_m2: 412.4,
        pose_provider: "lidar_planar",
        imu_feed: null,
      }),
    );
    act(() =>
      h.emitEvent("plugin.navigation.lidar", {
        v: 2,
        device: "/dev/ttyUSB0",
        connected: true,
        health: "good",
        rate_hz: 10.04,
        returns_last_scan: 812,
        scans: 40,
      }),
    );
    expect(screen.getByTestId("vn-slam-provider").textContent).toBe("LiDAR");
    expect(screen.getByTestId("vn-slam-map-memory").textContent).toBe("102 of 256 MB");
    expect(screen.getByTestId("vn-slam-map-placement").textContent).toBe("On a node, live");
    expect(screen.getByTestId("vn-slam-map-area").textContent).toBe("412 m²");
    expect(screen.getByTestId("vn-slam-lidar-link").textContent).toBe("Connected");
    expect(screen.getByTestId("vn-slam-lidar-rate").textContent).toBe("10.0 Hz");
    expect(screen.getByTestId("vn-slam-lidar-returns").textContent).toBe("812");
  });

  it("says when no provider runs rather than leaving the row blank", () => {
    const h = harness({ panelId: AGENT_PAGE_ID });
    render(<Page ctx={h.ctx} firmware="ardupilot" />);
    act(() => h.emitEvent("plugin.navigation.map_status", { placement: "hybrid", pose_provider: null }));
    expect(screen.getByTestId("vn-slam-provider").textContent).toBe("None (no LiDAR or VIO rig)");
  });

  it("on a node without a flight controller one sentence stands in for the FC cards", () => {
    vi.useFakeTimers();
    vi.setSystemTime(T0);
    const h = harness({ panelId: AGENT_PAGE_ID });
    render(<Page ctx={h.ctx} firmware="ardupilot" />);
    expect(screen.getByTestId("vn-telemetry-body")).toBeTruthy();
    act(() =>
      h.emitTelemetry("navigation", { ts_ms: T0, opticalFlowSupported: true, noFlightController: true }),
    );
    expect(screen.getByTestId("vn-no-fc").textContent).toBe(
      "No flight controller on this node: the pose feeds capture and the live map only.",
    );
    expect(screen.queryByTestId("vn-telemetry-body")).toBeNull();
    expect(screen.queryByTestId("vn-nav-state")).toBeNull();
    expect(screen.getByTestId("vn-lidar-slam")).toBeTruthy();
  });
});
