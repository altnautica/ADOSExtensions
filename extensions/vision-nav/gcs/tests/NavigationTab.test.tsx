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
    vioSupported: false,
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
        telemetryOverride={mkTelemetry({
          vioSupported: true,
          flowQuality: 200,
        })}
      />,
    );
    expect(screen.getByTestId("vn-ardupilot-params")).toBeTruthy();
    expect(screen.queryByTestId("vn-px4-params")).toBeNull();
    // Switcher enabled on ArduPilot.
    const vio = screen.getByTestId("vn-ekf-button-vio") as HTMLButtonElement;
    expect(vio.disabled).toBe(false);
    const of = screen.getByTestId("vn-ekf-button-of") as HTMLButtonElement;
    expect(of.disabled).toBe(false);
  });

  it("renders the PX4 params panel and disables the switcher on PX4", () => {
    const ctx = fakeCtx();
    render(
      <NavigationTab
        ctx={ctx}
        firmware="px4"
        telemetryOverride={mkTelemetry({ vioSupported: true })}
      />,
    );
    expect(screen.getByTestId("vn-px4-params")).toBeTruthy();
    expect(screen.queryByTestId("vn-ardupilot-params")).toBeNull();
    expect(screen.getByTestId("vn-ekf-px4-note")).toBeTruthy();
    const vio = screen.getByTestId("vn-ekf-button-vio") as HTMLButtonElement;
    expect(vio.disabled).toBe(true);
    const of = screen.getByTestId("vn-ekf-button-of") as HTMLButtonElement;
    expect(of.disabled).toBe(true);
  });
});
