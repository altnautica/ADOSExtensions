import { describe, expect, it } from "vitest";

import { scoreFrame } from "../src/calibration/qualityScore";
import {
  isPoseSetDiverse,
  poseFromCorners,
} from "../src/calibration/poseCluster";

describe("scoreFrame", () => {
  it("verdicts GOOD on all signals high", () => {
    const r = scoreFrame({
      sharpness: 1500,
      tagCount: 34,
      tagAreaSpan: 0.8,
      meanExposure: 128,
    });
    expect(r.verdict).toBe("good");
    expect(r.reasons).toEqual([]);
  });

  it("drops on blurry frames", () => {
    const r = scoreFrame({
      sharpness: 100,
      tagCount: 34,
      tagAreaSpan: 0.8,
      meanExposure: 128,
    });
    expect(r.verdict).toBe("drop");
    expect(r.reasons).toContain("blurry");
  });

  it("drops on too few tags", () => {
    const r = scoreFrame({
      sharpness: 1500,
      tagCount: 10,
      tagAreaSpan: 0.8,
      meanExposure: 128,
    });
    expect(r.verdict).toBe("drop");
    expect(r.reasons).toContain("too few tags");
  });

  it("drops on tag area span too small", () => {
    const r = scoreFrame({
      sharpness: 1500,
      tagCount: 34,
      tagAreaSpan: 0.2,
      meanExposure: 128,
    });
    expect(r.verdict).toBe("drop");
    expect(r.reasons).toContain("target too small in frame");
  });

  it("drops on under-exposed frame", () => {
    const r = scoreFrame({
      sharpness: 1500,
      tagCount: 34,
      tagAreaSpan: 0.8,
      meanExposure: 10,
    });
    expect(r.verdict).toBe("drop");
    expect(r.reasons).toContain("exposure out of range");
  });

  it("verdicts OK on mid-range signals", () => {
    const r = scoreFrame({
      sharpness: 400,
      tagCount: 26,
      tagAreaSpan: 0.5,
      meanExposure: 128,
    });
    expect(r.verdict).toBe("ok");
  });
});

describe("poseFromCorners", () => {
  it("returns near-zero tilt for a square front-on tag", () => {
    const corners = {
      topLeft: { x: 100, y: 100 },
      topRight: { x: 200, y: 100 },
      bottomRight: { x: 200, y: 200 },
      bottomLeft: { x: 100, y: 200 },
    };
    const p = poseFromCorners(corners);
    expect(p.tiltDeg).toBeLessThan(5);
  });

  it("returns large tilt for a skewed tag", () => {
    const corners = {
      topLeft: { x: 100, y: 100 },
      topRight: { x: 200, y: 100 },
      bottomRight: { x: 180, y: 120 },
      bottomLeft: { x: 110, y: 120 },
    };
    const p = poseFromCorners(corners);
    expect(p.tiltDeg).toBeGreaterThan(10);
  });
});

describe("isPoseSetDiverse", () => {
  it("rejects an empty set", () => {
    expect(isPoseSetDiverse([])).toBe(false);
  });

  it("rejects clustered samples", () => {
    const samples = Array.from({ length: 10 }, () => ({
      tiltDeg: 5,
      rotationDeg: 90,
    }));
    expect(isPoseSetDiverse(samples)).toBe(false);
  });

  it("accepts a diverse set across multiple buckets", () => {
    const samples = [
      { tiltDeg: 5, rotationDeg: 30 },
      { tiltDeg: 20, rotationDeg: 90 },
      { tiltDeg: 40, rotationDeg: 150 },
      { tiltDeg: 60, rotationDeg: 210 },
      { tiltDeg: 80, rotationDeg: 270 },
    ];
    expect(isPoseSetDiverse(samples)).toBe(true);
  });
});
