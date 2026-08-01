import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";

import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

import { FollowMeTab } from "../src/FollowMeTab";
import enLocale from "../../locales/en.json";
import type { FollowState } from "../src/types";

afterEach(cleanup);

function harnessCtx() {
  return createPluginHarness({
    mount: () => undefined,
    locale: enLocale as Record<string, string>,
    grantedCapabilities: ["command.send"],
  });
}

const idleFollow: FollowState = {
  active: false,
  lockState: null,
  targetId: null,
  rangeM: null,
  distanceSetpointM: null,
  heightSetpointM: null,
  commanding: false,
  fcArmed: false,
  fcGuided: false,
  holdReason: "inactive",
};

const commandingFollow: FollowState = {
  active: true,
  lockState: "locked",
  targetId: 42,
  rangeM: 12.3,
  distanceSetpointM: 8,
  heightSetpointM: 4,
  commanding: true,
  fcArmed: true,
  fcGuided: true,
  holdReason: null,
};

describe("FollowMeTab", () => {
  it("renders specs and live metrics with the generic detector", () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={idleFollow} />);
    expect(screen.getByTestId("fm-specs")).toBeTruthy();
    expect(screen.getByTestId("fm-metrics")).toBeTruthy();
    // The detector is a generic person/COCO model, never named weights.
    expect(screen.getByText("Generic person / COCO model")).toBeTruthy();
  });

  it("reflects the honest commanding flag and lock state from the read-back", () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={commandingFollow} />);
    expect(screen.getByTestId("fm-commanding").textContent).toBe("Yes");
    expect(screen.getByTestId("fm-fc-armed").textContent).toBe("Yes");
    expect(screen.getByTestId("fm-fc-guided").textContent).toBe("Yes");
    expect(screen.getByTestId("fm-lock-state").textContent).toContain("Locked");
    expect(screen.getByTestId("fm-target-id").textContent).toBe("#42");
    expect(screen.getByTestId("fm-range").textContent).toBe("12.3 m");
  });

  it("shows the honest FC state when locked but the FC is not guided", () => {
    const h = harnessCtx();
    render(
      <FollowMeTab
        ctx={h.ctx}
        followOverride={{
          ...commandingFollow,
          commanding: false,
          fcArmed: true,
          fcGuided: false,
          holdReason: "fc-not-guided",
        }}
      />,
    );
    // Locked but not commanding, and the FC rows explain why.
    expect(screen.getByTestId("fm-commanding").textContent).toBe("No");
    expect(screen.getByTestId("fm-fc-armed").textContent).toBe("Yes");
    expect(screen.getByTestId("fm-fc-guided").textContent).toBe("No");
  });

  it("names the gate that is holding instead of only saying not commanding", () => {
    const h = harnessCtx();
    render(
      <FollowMeTab
        ctx={h.ctx}
        followOverride={{
          ...commandingFollow,
          commanding: false,
          fcArmed: false,
          fcGuided: false,
          holdReason: "pose-stale",
        }}
      />,
    );
    expect(screen.getByTestId("fm-commanding").textContent).toBe("No");
    // A stalled pose is a fault, and must read as one rather than looking
    // like an aircraft that is simply sitting on the ground.
    expect(screen.getByTestId("fm-hold-reason").textContent).toBe(
      "Vehicle telemetry stopped",
    );
  });

  it("distinguishes a lost heartbeat from a disarmed flight controller", () => {
    // Both show FC armed: No. Only the read-back's reason separates them.
    const h = harnessCtx();
    render(
      <FollowMeTab
        ctx={h.ctx}
        followOverride={{
          ...commandingFollow,
          commanding: false,
          fcArmed: false,
          fcGuided: false,
          holdReason: "fc-stale",
        }}
      />,
    );
    expect(screen.getByTestId("fm-fc-armed").textContent).toBe("No");
    expect(screen.getByTestId("fm-hold-reason").textContent).toBe(
      "No flight controller heartbeat",
    );
  });

  it("shows no hold reason while the follow is commanding", () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={commandingFollow} />);
    expect(screen.queryByTestId("fm-hold-reason-row")).toBeNull();
  });

  it("points to the native settings controls and edits no config from the iframe", () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={idleFollow} />);
    // The settings moved to native `contributes.parameters`; the tab shows a
    // hint and exposes no editable settings inputs.
    expect(screen.getByTestId("fm-settings-hint")).toBeTruthy();
    expect(screen.queryByTestId("fm-settings")).toBeNull();
    expect(screen.queryByTestId("fm-follow-distance")).toBeNull();
    // No config write command is ever issued from the tab.
    const write = h.calls.find((c) => c.method === "command.send");
    expect(write).toBeUndefined();
  });
});
