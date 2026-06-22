import { afterEach, describe, expect, it } from "vitest";
import { cleanup, fireEvent, render, screen } from "@testing-library/react";

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
  it("renders specs, settings, and live metrics with the generic detector", () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={idleFollow} />);
    expect(screen.getByTestId("fm-specs")).toBeTruthy();
    expect(screen.getByTestId("fm-settings")).toBeTruthy();
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

  it("writes a settings change through ctx.command", async () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={idleFollow} />);
    const input = screen.getByTestId("fm-follow-distance") as HTMLInputElement;
    fireEvent.change(input, { target: { value: "15" } });
    // Let the async command.send round-trip settle.
    await Promise.resolve();
    await Promise.resolve();
    const write = h.calls.find(
      (c) =>
        c.method === "command.send" &&
        (c.args as { command?: string }).command === "plugin.config.write",
    );
    expect(write).toBeTruthy();
    const args = (write!.args as { args: { key: string; value: number } }).args;
    expect(args.key).toBe("follow_distance_m");
    expect(args.value).toBe(15);
  });

  it("locks the settings inputs while a follow is commanding", () => {
    const h = harnessCtx();
    render(<FollowMeTab ctx={h.ctx} followOverride={commandingFollow} />);
    const input = screen.getByTestId("fm-follow-distance") as HTMLInputElement;
    expect(input.disabled).toBe(true);
    expect(screen.getByTestId("fm-settings-locked")).toBeTruthy();
  });
});
