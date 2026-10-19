/**
 * Placement: the contract's `p_enu = s·R·p + t` and the ENU → viewer axes
 * (x east, y up, z −north) every overlay shares, so a splat, its mesh and its
 * floorplan land in the same place.
 */

import { describe, expect, it } from "vitest";
import * as THREE from "three";

import {
  artifactToEnu,
  artifactToViewerMatrix,
  coercePlacement,
  enuToViewer,
  viewerToEnu,
} from "../../../src/lib/world/placement";

/** A RUB artifact (x right, y up, z back) whose right is east: R maps
 * x→E, y→U, z→−N. */
const RUB = {
  anchor: null,
  R_enu_artifact: [1, 0, 0, 0, 0, -1, 0, 1, 0],
  t_enu_artifact: [10, 20, 1.5],
  scale: 2,
};

describe("coercePlacement", () => {
  it("accepts a placement or a .place.json descriptor; rejects malformed ones", () => {
    expect(coercePlacement({ ...RUB, contract: "ados.capture/1", kind: "splat" })).toEqual(RUB);
    expect(coercePlacement({ ...RUB, scale: 0 })).toBeNull();
    expect(coercePlacement({ ...RUB, R_enu_artifact: [1, 0, 0] })).toBeNull();
    expect(coercePlacement(null)).toBeNull();
  });
});

describe("ENU ↔ viewer", () => {
  it("maps east to x, up to y and north to −z, and back", () => {
    expect(enuToViewer([1, 2, 3])).toEqual([1, 3, -2]);
    expect(viewerToEnu(enuToViewer([1, 2, 3]))).toEqual([1, 2, 3]);
  });
});

describe("artifactToViewerMatrix", () => {
  it("agrees with artifactToEnu followed by enuToViewer", () => {
    const m = new THREE.Matrix4().set(...(artifactToViewerMatrix(RUB) as Parameters<THREE.Matrix4["set"]>));
    for (const p of [
      [0, 0, 0],
      [1, 0, 0],
      [0, 1, 0],
      [0.3, -2, 5],
    ] as [number, number, number][]) {
      const viaMatrix = new THREE.Vector3(...p).applyMatrix4(m).toArray();
      const expected = enuToViewer(artifactToEnu(RUB, p));
      viaMatrix.forEach((v, i) => expect(v).toBeCloseTo(expected[i]!, 9));
    }
  });

  it("is the bare ENU → viewer swap for a null placement", () => {
    const m = new THREE.Matrix4().set(...(artifactToViewerMatrix(null) as Parameters<THREE.Matrix4["set"]>));
    const v = new THREE.Vector3(1, 2, 3).applyMatrix4(m).toArray();
    expect(v).toEqual([1, 3, -2]);
  });
});
