import { describe, it, expect } from "vitest";

import { computePredictive } from "../src/predictive";
import type { BatterySample, PredictiveConfig } from "../src/types";

function mk(t: number, percent: number, current = -10): BatterySample {
  return {
    timestampMs: t,
    packId: 0,
    cellVoltagesV: [3.9, 3.9, 3.9, 3.9],
    totalVoltageV: 15.6,
    currentA: current,
    consumedAh: 0,
    consumedWh: 0,
    remainingPercent: percent,
    temperatureC: 25,
    cellCount: 4,
    chemistry: "lipo",
  };
}

const cfg: PredictiveConfig = { windowSeconds: 30, minimumPercent: 25 };

describe("computePredictive", () => {
  it("returns null on insufficient samples", () => {
    expect(computePredictive([], cfg)).toBeNull();
    expect(computePredictive([mk(0, 90)], cfg)).toBeNull();
  });

  it("returns null when elapsed window is sub-second", () => {
    const samples = [mk(0, 90), mk(500, 89.5)];
    expect(computePredictive(samples, cfg)).toBeNull();
  });

  it("reports idle when remaining percent does not drop", () => {
    const samples = [mk(0, 90, 0), mk(2000, 90, 0), mk(4000, 90, 0)];
    const res = computePredictive(samples, cfg);
    expect(res).not.toBeNull();
    expect(res!.rateLikely).toBe("idle");
    expect(res!.etaSec).toBeNull();
  });

  it("reports past when already below the target minimum", () => {
    const samples = [mk(0, 30), mk(5000, 24, -20)];
    const res = computePredictive(samples, cfg);
    expect(res).not.toBeNull();
    expect(res!.rateLikely).toBe("past");
    expect(res!.etaSec).toBe(0);
  });

  it("projects ETA linearly from sliding window", () => {
    // 90 -> 80 over 10s = 1%/s. Target 25 -> 55 percent points away = 55s.
    const samples = [
      mk(0, 90),
      mk(2000, 88),
      mk(4000, 86),
      mk(6000, 84),
      mk(8000, 82),
      mk(10000, 80),
    ];
    const res = computePredictive(samples, cfg);
    expect(res).not.toBeNull();
    expect(res!.etaSec).toBe(55);
    expect(res!.rateLikely).toBe("high");
    expect(res!.meanCurrentA).toBeGreaterThan(0);
  });

  it("respects the configured window so older samples do not skew the rate", () => {
    // 1%/s for 30s, then 0.1%/s after. Default 30s window should pick
    // up the slow tail near the end.
    const samples: BatterySample[] = [];
    for (let t = 0; t <= 30000; t += 1000) {
      samples.push(mk(t, 100 - t / 1000));
    }
    for (let t = 31000; t <= 60000; t += 1000) {
      const seconds = (t - 30000) / 1000;
      samples.push(mk(t, 70 - seconds * 0.1));
    }
    const res = computePredictive(samples, cfg);
    expect(res).not.toBeNull();
    // At t=60000 we're at ~67%. Window covers 30000-60000 with linear
    // 0.1%/s -> ETA = (67-25)/0.1 = ~420s.
    expect(res!.etaSec).toBeGreaterThan(380);
    expect(res!.etaSec).toBeLessThan(460);
  });
});
