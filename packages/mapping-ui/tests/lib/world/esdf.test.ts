/**
 * Walk collision against the node's occupancy ESDF: the header and grid are
 * read as the node writes them, the walker stops a radius short of a wall
 * and slides along it.
 */

import { PerspectiveCamera } from "three";
import { describe, expect, it } from "vitest";

import { WalkControls } from "../../../src/components/world/walk-controls";
import { esdfDistance, esdfField, slideStep, WALK_RADIUS_M, type EsdfField } from "../../../src/lib/world/esdf";

const RES = 0.1;
const ORIGIN = [-5, -5, -1];
const DIMS = [101, 101, 41];

/** A wall on the plane east = 2 m: `occupancy.json` and the grid bytes
 * (`f32` little-endian, x fastest), truncated at 4 m. */
function wallEast2(): { header: unknown; bytes: ArrayBuffer } {
  const [nx, ny, nz] = DIMS as [number, number, number];
  const bytes = new ArrayBuffer(nx * ny * nz * 4);
  const view = new DataView(bytes);
  for (let k = 0; k < nz; k++)
    for (let j = 0; j < ny; j++)
      for (let i = 0; i < nx; i++) {
        const x = ORIGIN[0]! + i * RES;
        view.setFloat32(((k * ny + j) * nx + i) * 4, Math.min(Math.abs(x - 2), 4), true);
      }
  const header = { file: "occupancy.esdf", layout: "f32le, x fastest", origin: ORIGIN, resolution_m: RES, dims: DIMS, truncation_m: 4 };
  return { header, bytes };
}

function field(): EsdfField {
  const { header, bytes } = wallEast2();
  const f = esdfField(header, bytes);
  if (!f) throw new Error("field not read");
  return f;
}

describe("esdf", () => {
  it("reads the node's header and grid, and refuses a size mismatch", () => {
    const f = field();
    expect(esdfDistance(f, [0, 0, 1])).toBeCloseTo(2, 5);
    expect(esdfDistance(f, [2, 3, 0.5])).toBeCloseTo(0, 5);
    expect(esdfDistance(f, [50, 0, 0])).toBeNull();
    const { header } = wallEast2();
    expect(esdfField(header, new ArrayBuffer(16))).toBeNull();
    expect(esdfField({ ...(header as object), dims: [1, 2] }, new ArrayBuffer(8))).toBeNull();
  });

  it("walk_is_blocked_by_esdf_wall", () => {
    const f = field();
    // Viewer x is east; walk east from the origin at eye height.
    let eye: [number, number, number] = [0, 1.6, 0];
    for (let i = 0; i < 100; i++) {
      const [dx, dz] = slideStep(f, eye, 0, [0.05, 0]);
      eye = [eye[0] + dx, eye[1], eye[2] + dz];
    }
    expect(eye[0]).toBeLessThanOrEqual(2 - WALK_RADIUS_M + RES / 2);
    expect(eye[0]).toBeGreaterThan(2 - WALK_RADIUS_M - 2 * RES);

    // Pushing diagonally into the wall slides along it (viewer −z is north).
    const [dx, dz] = slideStep(f, eye, 0, [0.05, -0.05]);
    expect(dx).toBeCloseTo(0, 6);
    expect(dz).toBeCloseTo(-0.05, 6);
    // Walking away is always allowed.
    expect(slideStep(f, eye, 0, [-0.05, 0])).toEqual([-0.05, 0]);
  });

  it("keeps the walk controls' camera off the wall", () => {
    const f = field();
    const camera = new PerspectiveCamera();
    const controls = new WalkControls(camera, document.createElement("div"));
    controls.collide = (eye, floorY, step) => slideStep(f, eye, floorY, step);
    // Facing east: yaw −π/2 turns forward (−z) onto +x.
    controls.yaw = -Math.PI / 2;
    controls.stick = { forward: 1, strafe: 0 };
    for (let i = 0; i < 100; i++) controls.update(0.1);
    expect(camera.position.x).toBeLessThanOrEqual(2 - WALK_RADIUS_M + RES / 2);
    expect(camera.position.x).toBeGreaterThan(1.4);
    controls.dispose();
  });
});
