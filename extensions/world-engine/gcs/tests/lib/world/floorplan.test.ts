/**
 * The floorplan overlay: coercion of `floorplan.json`, the north-up 2D plan
 * transform (uniform scale, centred, north at the top, invertible), the 3D
 * floor lines in viewer coordinates, room areas and label points.
 */

import { describe, expect, it } from "vitest";

import {
  coerceFloorplan,
  floorBounds,
  floorLinesViewer,
  planTransform,
  polygonArea,
  roomLabelPoint,
  type FloorplanFloor,
} from "../../../src/lib/world/floorplan";

const FLOOR: FloorplanFloor = {
  floor: 0,
  zM: 1.5,
  walls: [
    [0, 0, 10, 0],
    [10, 0, 10, 5],
    [10, 5, 0, 5],
    [0, 5, 0, 0],
  ],
  doors: [[4, 0, 5, 0]],
  rooms: [{ name: "Hall", polygon: [[0, 0], [10, 0], [10, 5], [0, 5]], areaM2: 50, areaFt2: 538, hotspot: null }],
};

describe("coerceFloorplan", () => {
  it("reads floors, walls, doors and rooms; drops malformed entries", () => {
    const plan = coerceFloorplan({
      contract: "ados.capture/1",
      units: "m",
      frame: "enu_local",
      floors: [
        {
          floor: 1,
          z_m: 3.1,
          walls: [[0, 0, 1, 0], [1, 2], "x"],
          doors: [],
          rooms: [{ name: "Kitchen", polygon: [[0, 0], [1, 0], [1, 1], [0]], area_m2: 1, area_ft2: 10.8, hotspot: [0.5, 0.5, 4.7] }],
        },
        { z_m: 0 },
      ],
    });
    expect(plan?.floors).toHaveLength(1);
    const f = plan!.floors[0]!;
    expect(f.zM).toBe(3.1);
    expect(f.walls).toEqual([[0, 0, 1, 0]]);
    expect(f.rooms[0]).toEqual({
      name: "Kitchen",
      polygon: [[0, 0], [1, 0], [1, 1]],
      areaM2: 1,
      areaFt2: 10.8,
      hotspot: [0.5, 0.5, 4.7],
    });
    expect(coerceFloorplan({ floors: [] })).toBeNull();
  });
});

describe("planTransform (2D overlay)", () => {
  const b = floorBounds(FLOOR)!;
  const tf = planTransform(b, 220, 120, 10);

  it("fits the plan with one uniform scale and centres it", () => {
    // 10 m × 5 m into 200 × 100 usable px → 20 px/m, exactly filling the box.
    expect(tf.scale).toBe(20);
    expect(tf.toPx(0, 0)).toEqual([10, 110]);
    expect(tf.toPx(10, 5)).toEqual([210, 10]);
  });

  it("puts north at the top (y up in metres is y down in pixels)", () => {
    const [, southY] = tf.toPx(5, 0);
    const [, northY] = tf.toPx(5, 5);
    expect(northY).toBeLessThan(southY);
  });

  it("is invertible", () => {
    const [px, py] = tf.toPx(3.25, 1.75);
    const [x, y] = tf.toEnu(px, py);
    expect(x).toBeCloseTo(3.25, 12);
    expect(y).toBeCloseTo(1.75, 12);
  });

  it("centres the short axis when the box is wider than the plan", () => {
    const wide = planTransform(b, 420, 120, 10);
    expect(wide.scale).toBe(20);
    // 10 m = 200 px centred in 420 px → 110 px margin on each side.
    expect(wide.toPx(0, 0)[0]).toBe(110);
  });
});

describe("floorLinesViewer (3D overlay)", () => {
  it("draws each wall at floor height in viewer axes (x east, y up, z −north)", () => {
    const lines = floorLinesViewer(FLOOR, 0.02);
    expect(lines).toHaveLength(FLOOR.walls.length * 6);
    // Wall 2: (10,0) → (10,5) at z = 1.52 m.
    expect(Array.from(lines.slice(6, 12))).toEqual([10, Math.fround(1.52), -0, 10, Math.fround(1.52), -5]);
  });
});

describe("rooms", () => {
  it("computes the shoelace area regardless of winding", () => {
    expect(polygonArea([[0, 0], [4, 0], [4, 3], [0, 3]])).toBe(12);
    expect(polygonArea([[0, 0], [0, 3], [4, 3], [4, 0]])).toBe(12);
  });

  it("labels at the hotspot, else at the vertex mean", () => {
    expect(roomLabelPoint(FLOOR.rooms[0]!)).toEqual([5, 2.5]);
    expect(roomLabelPoint({ ...FLOOR.rooms[0]!, hotspot: [1, 2, 3] })).toEqual([1, 2]);
  });
});
