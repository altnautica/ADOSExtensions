/**
 * @module atlas/viewers/coordinate-frame
 * @description Orients a legacy `.ply` point cloud, which comes out of the
 * COLMAP / OpenCV pipeline in its Y-DOWN, Z-FORWARD world frame, into the
 * three.js Y-UP, Z-BACK frame: a 180° rotation about X — negate Y and Z,
 * `diag(1, -1, -1)`. Artifacts that carry a contract placement (splats, LAZ
 * clouds, meshes) are placed through `lib/world/placement` instead.
 */

import type { BufferGeometry } from "three";

/**
 * Rotate a parsed point-cloud geometry in place from the COLMAP Y-down frame into
 * the viewer's Y-up frame (180° about X). Call BEFORE `computeBoundingSphere` so
 * the framing sphere is computed on the corrected coordinates.
 */
export function orientCloudToYUp(geom: BufferGeometry): void {
  geom.rotateX(Math.PI);
}
