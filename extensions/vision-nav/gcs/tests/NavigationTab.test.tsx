import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";

import { PluginClient, type PluginContext } from "@altnautica/plugin-sdk";
import { MemoryTransport } from "@altnautica/plugin-sdk";

import { NavigationTab } from "../src/components/NavigationTab";
import { firmwareFromConfig } from "../src/firmware";
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
    records: {
      list: async () => [],
      get: async () => null,
      put: async () => undefined,
      remove: async () => undefined,
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

  it("shows the static-scale banner with the source-set suggestion", () => {
    const ctx = fakeCtx();
    render(
      <NavigationTab
        ctx={ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry({
          estimatorState: "degraded",
          flowScaleSource: "static",
          flowQuality: 120,
        })}
      />,
    );
    const banner = screen.getByTestId("vn-fallback-banner-degraded");
    expect(banner.textContent).toContain(
      "No live altitude source; the flow scale is a fixed 1.5 m estimate.",
    );
    expect(banner.textContent).toContain(
      "Land, or switch the flight controller back to GPS with your RC source-set switch.",
    );
    expect(banner.textContent).not.toContain("Switch to GPS mode");
  });

  it("describes the degraded mode's baro / GPS (outdoor) ladder", () => {
    const ctx = fakeCtx();
    render(
      <NavigationTab
        ctx={ctx}
        firmware="ardupilot"
        telemetryOverride={mkTelemetry({
          availableEstimators: ["off", "optical_flow", "optical_flow_degraded"],
        })}
      />,
    );
    expect(screen.getByTestId("vn-mode-optical_flow_degraded").title).toContain(
      "baro / GPS (outdoor) ladder",
    );
  });

  it("px4 params render the EKF2 rows the flight controller reports", () => {
    const ctx = fakeCtx();
    render(
      <NavigationTab
        ctx={ctx}
        firmware="px4"
        telemetryOverride={mkTelemetry({ fcParams: ["EKF2_OF_CTRL"] })}
      />,
    );
    const card = screen.getByTestId("vn-px4-params");
    expect(card.textContent).toContain("EKF2_OF_CTRL");
    expect(card.textContent).not.toContain("EKF2_AID_MASK");
    expect(card.textContent).toContain("SENS_FLOW_MAXR");
    cleanup();

    render(
      <NavigationTab
        ctx={fakeCtx()}
        firmware="px4"
        telemetryOverride={mkTelemetry({ fcParams: ["EKF2_AID_MASK"] })}
      />,
    );
    const legacy = screen.getByTestId("vn-px4-params");
    expect(legacy.textContent).toContain("EKF2_AID_MASK");
    expect(legacy.textContent).not.toContain("EKF2_OF_CTRL");
  });

  it("firmware.type px4 renders Px4VisionParams", () => {
    const firmware = firmwareFromConfig({ firmware: { type: "px4" } });
    expect(firmware).toBe("px4");
    expect(firmwareFromConfig({ firmware: { type: "betafly" } })).toBeNull();
    expect(firmwareFromConfig({})).toBeNull();
    render(
      <NavigationTab
        ctx={fakeCtx()}
        firmware={firmware ?? "ardupilot"}
        telemetryOverride={mkTelemetry()}
      />,
    );
    expect(screen.getByTestId("vn-px4-params")).toBeTruthy();
    expect(screen.queryByTestId("vn-ardupilot-params")).toBeNull();
  });
});
