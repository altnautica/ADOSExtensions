import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";

import { PluginClient, type PluginContext } from "@altnautica/plugin-sdk";
import { MemoryTransport } from "@altnautica/plugin-sdk";

import { NavigationTab } from "../src/components/NavigationTab";
import type { VisionNavTelemetry } from "../src/types";

function fakeCtx(locale: Record<string, string> = {}): PluginContext {
  const subs: Array<() => void> = [];
  const transport = new MemoryTransport();
  const client = new PluginClient({ transport });
  return {
    client,
    telemetry: {
      subscribe: async () => {
        const off = (): void => undefined;
        subs.push(off);
        return off;
      },
    },
    command: {
      send: async () => ({ ok: true }),
    },
    notifications: {
      publish: async () => ({ ok: true }),
    },
    recording: {
      mark: async () => ({ ok: true }),
    },
    mission: {
      read: async () => ({}),
      write: async () => ({ ok: true }),
    },
    perception: {
      readTier: async () => ({ tier: null, offloadTarget: null }),
      subscribeDetections: async () => () => undefined,
      readSessionHealth: async () => ({
        session: "closed" as const,
        feed: "idle" as const,
        ageMs: null,
        batchesPerSecond: null,
        boundNode: null,
      }),
    },
    events: {
      subscribe: () => () => undefined,
      listen: async () => () => undefined,
      publish: async () => undefined,
    },
    config: {
      onChange: () => () => undefined,
    },
    theme: {
      onChange: () => () => undefined,
    },
    i18n: {
      t: (key) => locale[key] ?? key,
    },
  };
}

function mkTelemetry(over: Partial<VisionNavTelemetry> = {}): VisionNavTelemetry {
  return {
    opticalFlowSupported: true,
    flowQuality: 180,
    flowRateHz: 30,
    flowDistanceM: 1.25,
    companionState: "active",
    ...over,
  };
}

describe("NavigationTab", () => {
  afterEach(() => {
    cleanup();
  });

  it("renders the tab heading from the locale bundle", () => {
    const ctx = fakeCtx({ "navigation.tabTitle": "Vision Nav" });
    render(
      <NavigationTab
        ctx={ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry()}
      />,
    );
    expect(screen.getByRole("heading", { level: 2 }).textContent).toContain(
      "Vision Nav",
    );
  });

  it("flags critical companion state on the status pill", () => {
    const ctx = fakeCtx();
    render(
      <NavigationTab
        ctx={ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry({ companionState: "critical" })}
      />,
    );
    const pill = screen.getByTestId("vn-companion-pill");
    expect(pill.dataset.state).toBe("critical");
    expect(pill.textContent).toContain("Critical");
  });

  it("uses the green band on flow quality 200", () => {
    const ctx = fakeCtx();
    render(
      <NavigationTab
        ctx={ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry({ flowQuality: 200 })}
      />,
    );
    const bar = screen.getByTestId("vn-flow-quality-bar");
    expect(bar.dataset.band).toBe("high");
  });

  it("renders the ArduPilot params panel when firmware is ardupilot", () => {
    const ctx = fakeCtx();
    render(
      <NavigationTab
        ctx={ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry({ flowQuality: 200 })}
      />,
    );
    expect(screen.getByTestId("vn-ardupilot-params")).toBeTruthy();
    expect(screen.queryByTestId("vn-px4-params")).toBeNull();
  });

  it("renders the PX4 params panel when firmware is px4", () => {
    const ctx = fakeCtx();
    render(
      <NavigationTab ctx={ctx} firmware="px4" telemetryOverride={mkTelemetry()} />,
    );
    expect(screen.getByTestId("vn-px4-params")).toBeTruthy();
    expect(screen.queryByTestId("vn-ardupilot-params")).toBeNull();
  });

  it("offers no control that sends a request the plugin cannot serve", () => {
    const ctx = fakeCtx();
    render(
      <NavigationTab
        ctx={ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry({ cameraIntrinsicsLoaded: false })}
      />,
    );
    const sensors = screen.getByTestId("vn-sensors-card");
    expect(sensors.querySelectorAll("button")).toHaveLength(0);
    expect(screen.getByTestId("vn-sensors-camera").textContent).toContain(
      "Not calibrated",
    );
  });

  it("renders the iNav params panel when firmware is inav", () => {
    const ctx = fakeCtx();
    render(
      <NavigationTab
        ctx={ctx}
        firmware="inav"
        telemetryOverride={mkTelemetry({
          flowQuality: 180,
        })}
      />,
    );
    expect(screen.getByTestId("vn-inav-params")).toBeTruthy();
    expect(screen.queryByTestId("vn-ardupilot-params")).toBeNull();
    expect(screen.queryByTestId("vn-px4-params")).toBeNull();
    expect(screen.queryByTestId("vn-betaflight-unsupported")).toBeNull();
  });

  it("renders the Betaflight unsupported card when firmware is betaflight", () => {
    const ctx = fakeCtx();
    render(
      <NavigationTab
        ctx={ctx}
        firmware="betaflight"
        telemetryOverride={mkTelemetry({})}
      />,
    );
    const banner = screen.getByTestId("vn-betaflight-unsupported");
    expect(banner).toBeTruthy();
    // The banner names the firmware-upstream limitation explicitly so
    // the operator does not blame the plugin.
    expect(banner.textContent).toMatch(/no position estimator|EKF|cross-flash/i);
    expect(screen.queryByTestId("vn-inav-params")).toBeNull();
    expect(screen.queryByTestId("vn-ardupilot-params")).toBeNull();
    expect(screen.queryByTestId("vn-px4-params")).toBeNull();
  });
});
