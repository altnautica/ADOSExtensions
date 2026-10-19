/**
 * Viewer GPU bookkeeping: disposing an object frees each geometry and
 * material under it exactly once (shared ones too, arrays too) and detaches
 * it; replaced loader materials are freed as they are swapped.
 */

import { describe, expect, it, vi } from "vitest";
import * as THREE from "three";

import { disposeMaterials, disposeObject } from "../../../src/components/world/scene-resources";

describe("disposeObject", () => {
  it("frees every geometry and material once and detaches the root", () => {
    const parent = new THREE.Group();
    const root = new THREE.Group();
    parent.add(root);
    const sharedGeom = new THREE.BoxGeometry();
    const sharedMat = new THREE.MeshBasicMaterial();
    const other = new THREE.MeshBasicMaterial();
    root.add(new THREE.Mesh(sharedGeom, sharedMat));
    root.add(new THREE.Mesh(sharedGeom, [sharedMat, other]));
    const line = new THREE.LineSegments(new THREE.BufferGeometry(), new THREE.LineBasicMaterial());
    root.add(line);
    const spies = [sharedGeom, sharedMat, other, line.geometry, line.material as THREE.Material].map((r) =>
      vi.spyOn(r, "dispose"),
    );

    expect(disposeObject(root)).toEqual({ geometries: 2, materials: 3 });
    for (const s of spies) expect(s).toHaveBeenCalledTimes(1);
    expect(root.parent).toBeNull();
  });

  it("is a no-op on null", () => {
    expect(disposeObject(null)).toEqual({ geometries: 0, materials: 0 });
  });
});

describe("disposeMaterials", () => {
  it("frees a single material or an array", () => {
    const a = new THREE.MeshBasicMaterial();
    const b = new THREE.MeshBasicMaterial();
    const sa = vi.spyOn(a, "dispose");
    const sb = vi.spyOn(b, "dispose");
    expect(disposeMaterials([a, b])).toBe(2);
    expect(disposeMaterials(a)).toBe(1);
    expect(disposeMaterials(undefined)).toBe(0);
    expect(sa).toHaveBeenCalledTimes(2);
    expect(sb).toHaveBeenCalledTimes(1);
  });
});
