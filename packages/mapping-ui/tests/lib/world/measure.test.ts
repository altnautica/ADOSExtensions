/**
 * Measure tool math: 3D distance, plan point-to-wall and wall-to-wall
 * distances (parallel walls give the room width, crossing walls 0), and the
 * metric + imperial readouts.
 */

import { describe, expect, it } from "vitest";

import {
  distance3,
  FT2_PER_M2,
  formatArea,
  formatLength,
  pointToSegment,
  segmentToSegment,
} from "../../../src/lib/world/measure";

describe("distance3", () => {
  it("is the Euclidean distance", () => {
    expect(distance3([0, 0, 0], [3, 4, 12])).toBe(13);
    expect(distance3([1, 1, 1], [1, 1, 1])).toBe(0);
  });
});

describe("pointToSegment", () => {
  it("projects onto the segment and clamps to its ends", () => {
    expect(pointToSegment([2, 3], [0, 0, 4, 0])).toBe(3);
    expect(pointToSegment([-3, 4], [0, 0, 4, 0])).toBe(5);
    expect(pointToSegment([7, 4], [0, 0, 4, 0])).toBe(5);
    expect(pointToSegment([1, 1], [2, 2, 2, 2])).toBeCloseTo(Math.SQRT2, 12);
  });
});

describe("segmentToSegment", () => {
  it("gives the gap between two parallel walls (room width)", () => {
    expect(segmentToSegment([0, 0, 5, 0], [0, 3.6, 5, 3.6])).toBeCloseTo(3.6, 12);
  });

  it("is zero for crossing walls and the end gap for offset ones", () => {
    expect(segmentToSegment([0, 0, 4, 4], [0, 4, 4, 0])).toBe(0);
    expect(segmentToSegment([0, 0, 1, 0], [4, 0, 6, 0])).toBe(3);
  });
});

describe("readouts", () => {
  it("formats a length in metres and feet + inches", () => {
    expect(formatLength(3.42)).toBe("3.42 m · 11 ft 3 in");
    expect(formatLength(0.3048)).toBe("0.30 m · 1 ft 0 in");
  });

  it("formats an area in m² and ft²", () => {
    expect(FT2_PER_M2).toBeCloseTo(10.7639, 4);
    expect(formatArea(18.6)).toBe("18.6 m² · 200 ft²");
  });
});
