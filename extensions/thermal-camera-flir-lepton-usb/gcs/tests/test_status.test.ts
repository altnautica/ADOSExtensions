import { describe, it, expect } from "vitest";

import { parseThermalState, readout, reasonText } from "../src/status";

describe("readout", () => {
  it("reports an unknown state before the agent has said anything", () => {
    // Nothing has been received on either channel. "Awaiting thermal frames"
    // would claim frames are on their way, which is not known.
    const view = readout({ state: null, spotC: null });
    expect(view.kind).toBe("unknown");
    expect(view.text).not.toMatch(/awaiting/i);
    expect(view.text).toMatch(/unknown/i);
  });

  it("names a missing sensor instead of awaiting frames", () => {
    const view = readout({
      state: { connected: false, reason: "no-device" },
      spotC: null,
    });
    expect(view.kind).toBe("unavailable");
    expect(view.text).toBe("No thermal sensor detected");
  });

  it("names the reason the camera is unavailable", () => {
    const view = readout({
      state: { connected: false, reason: "no-backend" },
      spotC: null,
    });
    expect(view.kind).toBe("unavailable");
    expect(view.text).toBe(
      "Thermal camera unavailable — no UVC backend is installed on the agent",
    );
  });

  it("stays unavailable when the agent reports no reason", () => {
    const view = readout({ state: { connected: false }, spotC: null });
    expect(view.kind).toBe("unavailable");
    expect(view.text).toMatch(/^Thermal camera unavailable — /);
  });

  it("awaits frames only once the agent reports an open session", () => {
    const view = readout({ state: { connected: true }, spotC: null });
    expect(view.kind).toBe("awaiting");
    expect(view.text).toBe("Awaiting thermal frames...");
  });

  it("renders the spot temperature once a frame read-back arrives", () => {
    const view = readout({ state: { connected: true }, spotC: 42.46 });
    expect(view.kind).toBe("spot");
    expect(view.text).toBe("Spot 42.5 °C");
  });

  it("never shows a temperature for a disconnected camera", () => {
    // A stale reading must not survive the camera going away.
    const view = readout({
      state: { connected: false, reason: "open-failed" },
      spotC: 42.5,
    });
    expect(view.kind).toBe("unavailable");
    expect(view.text).not.toMatch(/42\.5/);
  });
});

describe("reasonText", () => {
  it("shows an unrecognised reason verbatim", () => {
    expect(reasonText("thermal-fuse-blown")).toBe("thermal-fuse-blown");
  });

  it("says the reason is missing rather than inventing one", () => {
    expect(reasonText(undefined)).toBe("reason not reported");
  });
});

describe("parseThermalState", () => {
  it("reads connectivity and reason off the state payload", () => {
    expect(
      parseThermalState({
        connected: false,
        reason: "no-device",
        palette: "ironbow",
        gain: true,
      }),
    ).toEqual({
      connected: false,
      reason: "no-device",
      palette: "ironbow",
      gain: true,
    });
  });

  it("rejects a payload with no connectivity flag", () => {
    // An unreadable payload must not be taken for a connected camera.
    expect(parseThermalState({ palette: "ironbow" })).toBeNull();
    expect(parseThermalState({ connected: "yes" })).toBeNull();
    expect(parseThermalState(null)).toBeNull();
    expect(parseThermalState("thermal")).toBeNull();
  });

  it("drops a non-string reason rather than rendering it", () => {
    const state = parseThermalState({ connected: false, reason: 7 });
    expect(state).toEqual({ connected: false });
    expect(readout({ state, spotC: null }).text).toMatch(
      /reason not reported/,
    );
  });
});
