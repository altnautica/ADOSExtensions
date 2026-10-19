/**
 * The capture health chips: exactly the six ids in display order, each level
 * mapped to its tone, an unreported chip grey (never green), readings in the
 * agent's units, and a prompt only on a red chip.
 */

import { describe, expect, it } from "vitest";

import { coerceHealth, HEALTH_IDS } from "../../../src/lib/capture/capture-state";
import { healthChips, worstHealth } from "../../../src/lib/capture/health-chips";

describe("coerceHealth", () => {
  it("keeps the six known ids in display order and drops unknown ids and levels", () => {
    const raw = [
      { id: "clock", level: "ok", value: 0.42 },
      { id: "vio", level: "bad", value: 1 },
      { id: "blur", level: "warn", value: 182 },
      { id: "texture", level: "purple", value: 54 },
      { id: "storage", level: "bad" },
    ];
    expect(coerceHealth(raw)).toEqual([
      { id: "blur", level: "warn", value: 182 },
      { id: "storage", level: "bad", value: null },
      { id: "clock", level: "ok", value: 0.42 },
    ]);
    expect(coerceHealth("nope")).toEqual([]);
  });
});

describe("healthChips", () => {
  it("returns all six chips, tones by level, unknown when not reported", () => {
    const chips = healthChips([
      { id: "blur", level: "ok", value: 182 },
      { id: "texture", level: "warn", value: 54 },
      { id: "flicker", level: "bad", value: 3.4 },
      { id: "clock", level: "ok", value: 0.42 },
    ]);
    expect(chips.map((c) => c.id)).toEqual([...HEALTH_IDS]);
    expect(chips.map((c) => c.tone)).toEqual(["good", "warning", "critical", "offline", "offline", "good"]);
    expect(chips.map((c) => c.level)).toEqual(["ok", "warn", "bad", null, null, "ok"]);
  });

  it("formats each reading in its unit", () => {
    const chips = healthChips([
      { id: "blur", level: "ok", value: 182.4 },
      { id: "texture", level: "ok", value: 54 },
      { id: "flicker", level: "ok", value: 0.8 },
      { id: "corridor", level: "ok", value: 0.123 },
      { id: "storage", level: "ok", value: 61 },
      { id: "clock", level: "ok", value: 0.42 },
    ]);
    expect(chips.map((c) => c.reading)).toEqual(["182", "54 pts", "0.8 %", "0.12", "61 %", "σ 0.42 ms"]);
  });

  it("raises the operator prompt only on a red chip", () => {
    const chips = healthChips([
      { id: "blur", level: "bad", value: 20 },
      { id: "corridor", level: "bad", value: 0.05 },
      { id: "texture", level: "warn", value: 90 },
    ]);
    const prompts = Object.fromEntries(chips.map((c) => [c.id, c.promptKey]));
    expect(prompts).toEqual({
      blur: "slowDown",
      texture: null,
      flicker: null,
      corridor: "turnToCorner",
      storage: null,
      clock: null,
    });
  });
});

describe("worstHealth", () => {
  it("is the worst reported level, or null when none", () => {
    expect(worstHealth([])).toBeNull();
    expect(worstHealth([{ id: "blur", level: "ok", value: 1 }])).toBe("ok");
    expect(
      worstHealth([
        { id: "blur", level: "warn", value: 1 },
        { id: "clock", level: "bad", value: 1 },
      ]),
    ).toBe("bad");
  });
});
