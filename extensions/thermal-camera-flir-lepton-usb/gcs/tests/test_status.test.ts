import { describe, it, expect } from "vitest";

import { createPluginContext } from "@altnautica/plugin-sdk";

import enLocale from "../../locales/en.json";
import {
  STALE_AFTER_MS,
  paletteText,
  parseThermalState,
  readout,
  reasonText,
} from "../src/status";

const { t } = createPluginContext({
  locale: enLocale as Record<string, string>,
}).i18n;

const live = (spotC: number | null, ageMs: number | null = 0) =>
  readout({ state: { connected: true }, spotC, ageMs, t });

describe("readout", () => {
  it("reports an unknown state before the agent has said anything", () => {
    // Nothing has been received on either channel. "Awaiting thermal frames"
    // would claim frames are on their way, which is not known.
    const view = readout({ state: null, spotC: null, ageMs: null, t });
    expect(view.kind).toBe("unknown");
    expect(view.text).not.toMatch(/awaiting/i);
    expect(view.text).toMatch(/unknown/i);
  });

  it("names a missing sensor with the one locale no-device wording", () => {
    const view = readout({
      state: { connected: false, reason: "no-device" },
      spotC: null,
      ageMs: null,
      t,
    });
    expect(view.kind).toBe("unavailable");
    expect(view.text).toBe(enLocale["panel.noDevice"]);
  });

  it("names the reason the camera is unavailable from the locale", () => {
    const view = readout({
      state: { connected: false, reason: "no-backend" },
      spotC: null,
      ageMs: null,
      t,
    });
    expect(view.kind).toBe("unavailable");
    expect(view.text).toBe(
      "Thermal camera unavailable — no UVC backend is installed on the agent",
    );
  });

  it("stays unavailable when the agent reports no reason", () => {
    const view = readout({
      state: { connected: false },
      spotC: null,
      ageMs: null,
      t,
    });
    expect(view.kind).toBe("unavailable");
    expect(view.text).toMatch(/^Thermal camera unavailable — reason not reported/);
  });

  it("awaits frames only once the agent reports an open session", () => {
    const view = live(null, null);
    expect(view.kind).toBe("awaiting");
    expect(view.text).toBe(enLocale["panel.awaitingFrames"]);
  });

  it("renders a fresh spot temperature through the locale", () => {
    const view = live(42.46, 100);
    expect(view.kind).toBe("spot");
    expect(view.text).toBe("Spot 42.5 °C");
  });

  it("stops showing the value once the reading is stale, and shows its age", () => {
    expect(STALE_AFTER_MS).toBe(2000);
    expect(live(42.46, STALE_AFTER_MS - 1).kind).toBe("spot");
    const view = live(42.46, 3400);
    expect(view.kind).toBe("stale");
    expect(view.text).toBe("Thermal frames stopped — last reading 3 s ago");
    expect(view.text).not.toMatch(/42/);
  });

  it("never shows a temperature for a disconnected camera", () => {
    // A stale reading must not survive the camera going away.
    const view = readout({
      state: { connected: false, reason: "open-failed" },
      spotC: 42.5,
      ageMs: 0,
      t,
    });
    expect(view.kind).toBe("unavailable");
    expect(view.text).not.toMatch(/42\.5/);
  });
});

describe("reasonText", () => {
  it("never shows a raw reason id it does not know", () => {
    expect(reasonText("thermal-fuse-blown", t)).toBe(
      "reason not recognised by this overlay version",
    );
  });

  it("says the reason is missing rather than inventing one", () => {
    expect(reasonText(undefined, t)).toBe("reason not reported");
  });

  it("resolves every reason through the locale bundle", () => {
    for (const reason of ["no-backend", "open-failed", "stream-failed", "not-started"]) {
      const text = reasonText(reason, t);
      expect(text).not.toMatch(/^overlay\./);
      expect(text).not.toBe(reason);
    }
  });
});

describe("paletteText", () => {
  it("names the agent-applied palette read-only", () => {
    expect(paletteText({ connected: true, palette: "ironbow" }, t)).toBe(
      "Palette: Ironbow",
    );
  });

  it("shows nothing without a session or a known palette", () => {
    expect(paletteText({ connected: false, palette: "ironbow" }, t)).toBeNull();
    expect(paletteText({ connected: true }, t)).toBeNull();
    expect(paletteText({ connected: true, palette: "plasma" }, t)).toBeNull();
    expect(paletteText(null, t)).toBeNull();
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
    expect(readout({ state, spotC: null, ageMs: null, t }).text).toMatch(
      /reason not reported/,
    );
  });
});
