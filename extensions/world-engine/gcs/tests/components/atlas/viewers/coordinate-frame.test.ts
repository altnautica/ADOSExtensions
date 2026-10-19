import { describe, it, expect } from "vitest";
import * as THREE from "three";
import { orientCloudToYUp } from "../../../../src/components/atlas/viewers/coordinate-frame";

// A legacy `.ply` cloud is in the COLMAP/OpenCV world frame (Y-down,
// Z-forward); the viewer is Y-up, Z-back. The convention transform is a 180°
// rotation about X = diag(1,-1,-1): X is preserved, Y and Z are negated.

describe("orientCloudToYUp (point-cloud geometry)", () => {
  it("negates Y and Z, preserves X, on the position attribute", () => {
    const geom = new THREE.BufferGeometry();
    geom.setAttribute("position", new THREE.BufferAttribute(new Float32Array([2, 3, 5]), 3));
    orientCloudToYUp(geom);
    const p = geom.getAttribute("position");
    expect(p.getX(0)).toBeCloseTo(2, 5);
    expect(p.getY(0)).toBeCloseTo(-3, 5);
    expect(p.getZ(0)).toBeCloseTo(-5, 5);
  });

  it("is the 180°-about-X rotation", () => {
    const geom = new THREE.BufferGeometry();
    geom.setAttribute("position", new THREE.BufferAttribute(new Float32Array([1, 2, 3]), 3));
    orientCloudToYUp(geom);
    const p = geom.getAttribute("position");
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI);
    const expected = new THREE.Vector3(1, 2, 3).applyQuaternion(q);
    expect(p.getX(0)).toBeCloseTo(expected.x, 5);
    expect(p.getY(0)).toBeCloseTo(expected.y, 5);
    expect(p.getZ(0)).toBeCloseTo(expected.z, 5);
  });
});
