import { afterEach, describe, expect, it } from "vitest";
import { act, cleanup, fireEvent, render, screen } from "@testing-library/react";

import { createPluginHarness, type PluginHarness } from "@altnautica/plugin-sdk/harness";

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
  it("shows the phase, the re-acquire mode, range with uncertainty and the search", () => {
    const h = harnessCtx();
    render(
      <FollowMeTab
        ctx={h.ctx}
        followOverride={{
          ...commandingFollow,
          phase: "searching",
          lockState: "lost",
          reacquireMode: "smart",
          rangeM: 10.2,
          rangeSigmaM: 0.6,
          rangeCues: [{ kind: "ground", rangeM: 10.1, sigmaM: 0.7 }],
          aglM: 4.1,
          aglSource: "rng",
          reacquireRemainingS: 18.4,
          searchBudgetS: 25,
          searchStep: 1,
          vantageState: "arrived",
          occluder: { ne: [11, -1], kind: "narrow" },
          pipelineLatencyMs: 85,
          setpointRateHz: 20,
          cameraHfovDeg: 120,
          cameraMountPitchDeg: -90,
          cameraGeometrySource: "settings",
        }}
      />,
    );
    expect(screen.getByTestId("fm-phase").textContent).toBe("Searching");
    expect(screen.getByTestId("fm-reacquire-mode").textContent).toBe("Smart");
    expect(screen.getByTestId("fm-range").textContent).toBe("10.2 ± 0.6 m");
    expect(screen.getByTestId("fm-range-cues").textContent).toBe("Ground");
    expect(screen.getByTestId("fm-agl").textContent).toBe("4.1 m · Rangefinder");
    expect(screen.getByTestId("fm-search-remaining").textContent).toBe("18.4 of 25 s");
    expect(screen.getByTestId("fm-search-step").textContent).toBe("2");
    expect(screen.getByTestId("fm-vantage").textContent).toBe("Arrived");
    expect(screen.getByTestId("fm-occluder").textContent).toBe("A narrow obstacle (tree, column)");
    expect(screen.getByTestId("fm-latency").textContent).toBe("85 ms");
    expect(screen.getByTestId("fm-setpoint-rate").textContent).toBe("20.0 Hz");
    expect(screen.getByTestId("fm-camera-geometry").textContent).toBe(
      "120° · -90° · Settings",
    );
    expect(screen.queryByTestId("fm-prompt")).toBeNull();
  });

  it("shows the search budget while predicting and repositioning, without a step", () => {
    const h = harnessCtx();
    render(
      <FollowMeTab
        ctx={h.ctx}
        followOverride={{ ...commandingFollow, phase: "repositioning", reacquireRemainingS: 21, searchBudgetS: 25, vantageState: "enroute" }}
      />,
    );
    expect(screen.getByTestId("fm-phase").textContent).toBe("Repositioning");
    expect(screen.getByTestId("fm-search-remaining").textContent).toBe("21.0 of 25 s");
    expect(screen.getByTestId("fm-vantage").textContent).toBe("On the way");
    expect(screen.queryByTestId("fm-search-step")).toBeNull();
    expect(screen.queryByTestId("fm-occluder")).toBeNull();
  });

  it("hides the search outside a recovery", () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={commandingFollow} />);
    expect(screen.getByTestId("fm-phase").textContent).toBe("Following");
    expect(screen.queryByTestId("fm-search-remaining")).toBeNull();
    expect(screen.queryByTestId("fm-vantage")).toBeNull();
  });

  it("says when the subject is faster than the follow can fly", () => {
    const h = harnessCtx();
    render(
      <FollowMeTab
        ctx={h.ctx}
        followOverride={{ ...commandingFollow, outpaced: true, speedLimitMps: 2 }}
      />,
    );
    expect(screen.getByTestId("fm-outpaced").textContent).toBe("Subject faster than the 2 m/s limit");
  });

  it("shows no speed warning while the follow keeps up", () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={{ ...commandingFollow, speedLimitMps: 6 }} />);
    expect(screen.queryByTestId("fm-outpaced")).toBeNull();
  });

  it("says the height is kept above head height when a lower one was set", () => {
    const h = harnessCtx();
    render(
      <FollowMeTab
        ctx={h.ctx}
        followOverride={{ ...commandingFollow, heightSetpointM: 3.4, heightRaised: true }}
      />,
    );
    expect(screen.getByTestId("fm-height-setpoint").textContent).toBe(
      "3.4 m · kept above head height for safety",
    );
    cleanup();
    render(<FollowMeTab ctx={h.ctx} followOverride={commandingFollow} />);
    expect(screen.getByTestId("fm-height-setpoint").textContent).toBe("4.0 m");
  });

  it.each([
    ["camera-range", "5.0 m · at least 5.0 m with camera-only range"],
    ["range-unknown", "5.0 m · held at 5.0 m until the range is measured"],
    ["subject-standing", "5.0 m · held at 5.0 m while the subject stands"],
    [null, "5.0 m · held at 5.0 m"],
  ] as const)("says why a shorter follow distance is flown farther (%s)", (reason, text) => {
    const h = harnessCtx();
    render(
      <FollowMeTab
        ctx={h.ctx}
        followOverride={{
          ...commandingFollow,
          distanceSetpointM: 5,
          standoffMinM: 5,
          standoffRaised: true,
          standoffReason: reason,
        }}
      />,
    );
    expect(screen.getByTestId("fm-distance-setpoint").textContent).toBe(text);
  });

  it("says the vehicle is closing in, to a tenth of a metre", () => {
    const h = harnessCtx();
    render(
      <FollowMeTab
        ctx={h.ctx}
        followOverride={{
          ...commandingFollow,
          distanceSetpointM: 3.34,
          standoffMinM: 3.34,
          standoffRaised: true,
          standoffReason: "closing-in",
        }}
      />,
    );
    expect(screen.getByTestId("fm-distance-setpoint").textContent).toBe("3.3 m · closing in from 3.3 m");
    cleanup();
    render(<FollowMeTab ctx={h.ctx} followOverride={{ ...commandingFollow, standoffMinM: 5 }} />);
    expect(screen.getByTestId("fm-distance-setpoint").textContent).not.toContain("·");
  });

  const waiting: FollowState = {
    ...EMPTY_FOLLOW_STATE,
    active: true,
    phase: "holding-await-operator",
    lockState: "lost",
    holdReason: "search-exhausted",
    holdNe: [7.9, 2.4],
    reacquireMode: "smart",
    operatorResumeSeq: 2,
    fcArmed: true,
    fcGuided: true,
  };

  it("asks the operator what to do while waiting, with the reason", () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={waiting} />);
    expect(screen.getByTestId("fm-phase").textContent).toBe("Holding, waiting for you");
    expect(screen.getByTestId("fm-prompt-title").textContent).toBe(
      "Target lost: holding, waiting for instructions",
    );
    expect(screen.getByTestId("fm-prompt-body").textContent).toBe(
      "The search ran out of time without finding the subject.",
    );
    expect(screen.getByTestId("fm-prompt-redesignate").textContent).toContain("click the subject");
    for (const id of ["resume", "rth", "land", "stop"]) {
      expect(screen.getByTestId(`fm-prompt-${id}`)).toBeTruthy();
    }
    // Showing the prompt sends nothing.
    expect(h.calls.find((c) => c.method === "command.send")).toBeUndefined();
  });

  const sent = (h: PluginHarness) =>
    h.calls.filter((c) => c.method === "command.send").map((c) => c.args);

  it("resumes the search by writing the next resume number", async () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={waiting} />);
    await act(async () => {
      fireEvent.click(screen.getByTestId("fm-prompt-resume"));
    });
    expect(sent(h)).toEqual([
      { command: "plugin.config.write", args: { key: "operator_resume_seq", value: 3 } },
    ]);
  });

  it("stops the follow, returns home or lands on the operator's word", async () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={waiting} />);
    await act(async () => {
      fireEvent.click(screen.getByTestId("fm-prompt-stop"));
      fireEvent.click(screen.getByTestId("fm-prompt-rth"));
      fireEvent.click(screen.getByTestId("fm-prompt-land"));
    });
    expect(sent(h)).toEqual([
      { command: "plugin.config.write", args: { key: "active", value: false } },
      { command: "rtl", args: {} },
      { command: "land", args: {} },
    ]);
  });

  it("offers no resume when nothing is left to search from", () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={{ ...waiting, holdReason: "no-range" }} />);
    expect(screen.getByTestId("fm-prompt-body").textContent).toBe(
      "There is no prediction left to search from. Designate the subject again.",
    );
    expect(screen.queryByTestId("fm-prompt-resume")).toBeNull();
  });

  it("offers no resume in strict re-acquire, where a resumed search gives up at once", () => {
    const h = harnessCtx();
    render(
      <FollowMeTab
        ctx={h.ctx}
        followOverride={{ ...waiting, holdReason: "awaiting-operator", reacquireMode: "strict" }}
      />,
    );
    expect(screen.getByTestId("fm-prompt")).toBeTruthy();
    expect(screen.queryByTestId("fm-prompt-resume")).toBeNull();
  });

  it("keeps the prompt through a camera loss, without resume", () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={{ ...waiting, holdReason: "camera-lost" }} />);
    expect(screen.getByTestId("fm-prompt-body").textContent).toBe(
      "The follow camera stopped sending frames. The follow holds at the last point.",
    );
    expect(screen.queryByTestId("fm-prompt-resume")).toBeNull();
    expect(screen.getByTestId("fm-prompt-stop")).toBeTruthy();
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
    // Outside the hold prompt the tab sends nothing.
    const write = h.calls.find((c) => c.method === "command.send");
    expect(write).toBeUndefined();
  });

  it("shows the recovery settings in effect", () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={idleFollow} />);
    expect(screen.getByTestId("fm-uncertain-mode").textContent).toBe("Keep following");
    expect(screen.getByTestId("fm-predictive-search").textContent).toBe("On");
  });
});
