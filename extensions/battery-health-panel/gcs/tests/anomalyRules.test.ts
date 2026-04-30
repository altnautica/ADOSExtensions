import { describe, it, expect } from "vitest";

import { evaluateAnomalies } from "../src/anomalyRules";
import {
  DEFAULT_CONFIG,
  type BatterySample,
  type PredictiveState,
  type ThresholdConfig,
} from "../src/types";

const T = DEFAULT_CONFIG.thresholds;

function mk(overrides: Partial<BatterySample> = {}): BatterySample {
  return {
    timestampMs: 1000,
    packId: 0,
    cellVoltagesV: [3.9, 3.9, 3.9, 3.9],
    totalVoltageV: 15.6,
    currentA: -10,
    consumedAh: 0,
    consumedWh: 0,
    remainingPercent: 80,
    temperatureC: 30,
    cellCount: 4,
    chemistry: "lipo",
    ...overrides,
  };
}

describe("evaluateAnomalies", () => {
  it("fires nothing on a healthy sample", () => {
    const events = evaluateAnomalies(mk(), mk({ timestampMs: 2000 }), T, null);
    expect(events).toHaveLength(0);
  });

  it("fires cell_low when min cell drops below the warning threshold", () => {
    const sample = mk({ cellVoltagesV: [3.9, 3.9, 3.9, 3.45] });
    const events = evaluateAnomalies(null, sample, T, null);
    const ids = events.map((e) => e.ruleId);
    expect(ids).toContain("cell_low");
    expect(ids).not.toContain("cell_critical");
  });

  it("escalates to cell_critical without double-firing cell_low", () => {
    const sample = mk({ cellVoltagesV: [3.9, 3.9, 3.9, 3.2] });
    const events = evaluateAnomalies(null, sample, T, null);
    const ids = events.map((e) => e.ruleId);
    expect(ids).toContain("cell_critical");
    expect(ids).not.toContain("cell_low");
  });

  it("fires cell_divergence when spread crosses the threshold", () => {
    const sample = mk({ cellVoltagesV: [3.9, 3.9, 3.9, 3.83] });
    const events = evaluateAnomalies(null, sample, T, null);
    expect(events.find((e) => e.ruleId === "cell_divergence")).toBeDefined();
  });

  it("ignores divergence when one cell is unknown (NaN)", () => {
    const sample = mk({ cellVoltagesV: [3.9, 3.9, Number.NaN, 3.85] });
    const events = evaluateAnomalies(null, sample, T, null);
    expect(events.find((e) => e.ruleId === "cell_divergence")).toBeUndefined();
  });

  it("fires voltage_drop when total drops faster than configured", () => {
    const prev = mk({ timestampMs: 0, totalVoltageV: 16.0 });
    const curr = mk({ timestampMs: 1000, totalVoltageV: 14.8 });
    const events = evaluateAnomalies(prev, curr, T, null);
    expect(events.find((e) => e.ruleId === "voltage_drop")).toBeDefined();
  });

  it("does not fire voltage_drop when dt is too large to be a real drop", () => {
    const prev = mk({ timestampMs: 0, totalVoltageV: 16.0 });
    const curr = mk({ timestampMs: 60000, totalVoltageV: 14.8 });
    const events = evaluateAnomalies(prev, curr, T, null);
    expect(events.find((e) => e.ruleId === "voltage_drop")).toBeUndefined();
  });

  it("fires temp_spike on sudden temperature rise", () => {
    const prev = mk({ timestampMs: 0, temperatureC: 30 });
    const curr = mk({ timestampMs: 1000, temperatureC: 38 });
    const events = evaluateAnomalies(prev, curr, T, null);
    expect(events.find((e) => e.ruleId === "temp_spike")).toBeDefined();
  });

  it("ignores temp_spike when temperature is unknown", () => {
    const prev = mk({ timestampMs: 0, temperatureC: 30 });
    const curr = mk({ timestampMs: 1000, temperatureC: null });
    const events = evaluateAnomalies(prev, curr, T, null);
    expect(events.find((e) => e.ruleId === "temp_spike")).toBeUndefined();
  });

  it("fires predictive_low when ETA falls under 60 seconds", () => {
    const predictive: PredictiveState = {
      etaSec: 45,
      rateLikely: "high",
      meanCurrentA: 25,
    };
    const events = evaluateAnomalies(null, mk(), T, predictive);
    expect(events.find((e) => e.ruleId === "predictive_low")).toBeDefined();
  });

  it("does not fire predictive_low while idle", () => {
    const predictive: PredictiveState = {
      etaSec: null,
      rateLikely: "idle",
      meanCurrentA: 0,
    };
    const events = evaluateAnomalies(null, mk(), T, predictive);
    expect(
      events.find((e) => e.ruleId === "predictive_low"),
    ).toBeUndefined();
  });

  it("respects custom thresholds via the config arg", () => {
    const strict: ThresholdConfig = {
      ...T,
      lowCellVoltageV: 3.95,
      criticalCellVoltageV: 3.9,
    };
    // Healthy under defaults, low under strict (between strict critical
    // and strict low: 3.9 <= min < 3.95).
    const sample = mk({ cellVoltagesV: [3.92, 3.92, 3.92, 3.92] });
    const lax = evaluateAnomalies(null, sample, T, null);
    expect(lax.find((e) => e.ruleId === "cell_low")).toBeUndefined();
    const tight = evaluateAnomalies(null, sample, strict, null);
    const ids = tight.map((e) => e.ruleId);
    expect(ids).toContain("cell_low");
    expect(ids).not.toContain("cell_critical");
  });
});
