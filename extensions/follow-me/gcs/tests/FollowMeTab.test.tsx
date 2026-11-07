import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";

import { createPluginHarness } from "@altnautica/plugin-sdk/harness";

import { FollowMeTab } from "../src/FollowMeTab";
import enLocale from "../../locales/en.json";
import { EMPTY_FOLLOW_STATE, type FollowState } from "../src/types";

afterEach(cleanup);

function harnessCtx() {
  return createPluginHarness({
    mount: () => undefined,
    locale: enLocale as Record<string, string>,
    grantedCapabilities: ["command.send"],
  });
}

const idleFollow: FollowState = {
  ...EMPTY_FOLLOW_STATE,
  holdReason: "inactive",
};

const commandingFollow: FollowState = {
  ...EMPTY_FOLLOW_STATE,
  active: true,
  phase: "following",
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
  it("shows the phase, the re-acquire mode, range with uncertainty and the coast window", () => {
    const h = harnessCtx();
    render(
      <FollowMeTab
        ctx={h.ctx}
        followOverride={{
          ...commandingFollow,
          phase: "coasting",
          lockState: "lost",
          reacquireMode: "smart",
          rangeM: 10.2,
          rangeSigmaM: 0.6,
          rangeCues: [{ kind: "ground", rangeM: 10.1, sigmaM: 0.7 }],
          aglM: 4.1,
          aglSource: "rng",
          coastRemainingS: 2.5,
          pipelineLatencyMs: 85,
          setpointRateHz: 20,
        }}
      />,
    );
    expect(screen.getByTestId("fm-phase").textContent).toBe("Coasting");
    expect(screen.getByTestId("fm-reacquire-mode").textContent).toBe("Smart");
    expect(screen.getByTestId("fm-range").textContent).toBe("10.2 ± 0.6 m");
    expect(screen.getByTestId("fm-range-cues").textContent).toBe("Ground");
    expect(screen.getByTestId("fm-agl").textContent).toBe("4.1 m · Rangefinder");
    expect(screen.getByTestId("fm-coast-remaining").textContent).toBe("2.5 s");
    expect(screen.getByTestId("fm-latency").textContent).toBe("85 ms");
    expect(screen.getByTestId("fm-setpoint-rate").textContent).toBe("20.0 Hz");
  });

  it("hides the coast window outside a coast", () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={commandingFollow} />);
    expect(screen.getByTestId("fm-phase").textContent).toBe("Following");
    expect(screen.queryByTestId("fm-coast-remaining")).toBeNull();
  });

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

  it("shows the honest FC state when the pilot refuses the lease", () => {
    const h = harnessCtx();
    render(
      <FollowMeTab
        ctx={h.ctx}
        followOverride={{
          ...commandingFollow,
          commanding: false,
          fcArmed: true,
          fcGuided: false,
          holdReason: "lease-refused",
        }}
      />,
    );
    expect(screen.getByTestId("fm-hold-reason").textContent).toBe(
      "The pilot cannot hand over the vehicle",
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
