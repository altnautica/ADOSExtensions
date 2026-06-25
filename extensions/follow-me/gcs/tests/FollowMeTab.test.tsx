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
};

const commandingFollow: FollowState = {
  active: true,
  lockState: "locked",
  targetId: 42,
  rangeM: 12.3,
  distanceSetpointM: 8,
  heightSetpointM: 4,
  commanding: true,
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
    expect(screen.getByTestId("fm-lock-state").textContent).toContain("Locked");
    expect(screen.getByTestId("fm-target-id").textContent).toBe("#42");
    expect(screen.getByTestId("fm-range").textContent).toBe("12.3 m");
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
