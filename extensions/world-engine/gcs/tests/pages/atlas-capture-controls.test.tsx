/**
 * The capture bar refuses commands to a node that stopped answering, says
 * why a control is disabled as visible text (not only a tooltip), and its
 * buttons grow to a touch-sized target under a coarse pointer.
 */

import { afterEach, describe, expect, it } from "vitest";
import { cleanup, render, screen } from "@testing-library/react";

import messages from "../../../locales/en.json";
import { ToastProvider } from "../../src/components/ui/Toast";
import type { AtlasControl } from "../../src/hooks/use-atlas-control";
import type { AtlasReadiness } from "../../src/lib/agent/atlas-control-client";
import { AtlasCaptureControls } from "../../src/pages/drone/atlas/AtlasCaptureControls";

afterEach(cleanup);

const c = messages.atlas.capture;
const noop = () => Promise.resolve({ ok: true as const, status: {} as never });

function control(patch: Partial<AtlasControl>): AtlasControl {
  return {
    deviceId: "d1",
    readiness: null,
    offline: false,
    configError: null,
    busy: false,
    live: true,
    enable: () => Promise.resolve(true),
    disable: () => Promise.resolve(true),
    setCaptureProfile: () => Promise.resolve(true),
    setReconstructSteps: () => Promise.resolve(true),
    start: noop,
    stop: noop,
    pause: noop,
    resume: noop,
    ...patch,
  } as AtlasControl;
}

function show(ctl: AtlasControl, extra: { canStart?: boolean; startBlockedKey?: string | null } = {}) {
  render(
    <ToastProvider>
      <AtlasCaptureControls
        control={ctl}
        canStart={extra.canStart ?? true}
        startBlockedKey={extra.startBlockedKey ?? null}
        reconstruct={{ available: false, disabledKey: "capture.reconstructDisabledNoSession", submit: () => Promise.resolve(false) }}
      />
    </ToastProvider>,
  );
}

describe("AtlasCaptureControls", () => {
  it("disables Start for a node that stopped answering, with the reason as text", () => {
    show(control({ offline: true }));
    const start = screen.getByRole("button", { name: c.startCapture });
    expect((start as HTMLButtonElement).disabled).toBe(true);
    expect(screen.getByText(c.nodeNotAnswering)).toBeTruthy();
  });

  it("disables Pause and Stop for a node that stopped answering", () => {
    const readiness = { capturing: true, state: "capturing" } as AtlasReadiness;
    show(control({ offline: true, readiness }));
    expect((screen.getByRole("button", { name: c.pauseCapture }) as HTMLButtonElement).disabled).toBe(true);
    expect((screen.getByRole("button", { name: c.stopCapture }) as HTMLButtonElement).disabled).toBe(true);
  });

  it("keeps Start enabled for an answering node", () => {
    // An answering node has reported a readiness; a null readiness is a node
    // that has not answered yet, which the page gates through `canStart`.
    const readiness = { capturing: false, state: "idle" } as AtlasReadiness;
    show(control({ readiness }));
    expect((screen.getByRole("button", { name: c.startCapture }) as HTMLButtonElement).disabled).toBe(false);
    expect(screen.queryByText(c.nodeNotAnswering)).toBeNull();
  });

  it("shows the start and reconstruct disabled reasons as visible text", () => {
    show(control({}), { canStart: false, startBlockedKey: "capture.reconstructDisabledNoNode" });
    expect(screen.getByText(c.reconstructDisabledNoNode)).toBeTruthy();
    expect(screen.getByText(c.reconstructDisabledNoSession)).toBeTruthy();
  });

  it("gives its buttons a touch-sized target under a coarse pointer, with a focus ring", () => {
    show(control({}));
    const cls = screen.getByRole("button", { name: c.startCapture }).className;
    expect(cls).toContain("we:pointer-coarse:min-h-12");
    expect(cls).toContain("we:focus-visible:outline-2");
  });
});
