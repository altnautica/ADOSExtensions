/**
 * @module world/esdf
 * @description The node's occupancy ESDF (`occupancy.esdf`, `f32`
 * little-endian, x fastest, then y, then z; metres to the nearest surface,
 * truncated) with its header (`occupancy.json`: origin of voxel (0,0,0)'s
 * centre in local ENU, resolution, dims), and the walker's collision against
 * it: the eye's capsule keeps a radius of clearance and slides along walls.
 */

import { arr, num, objOrNull } from "../capture/coerce";
import { viewerToEnu, type Vec3 } from "./placement";

export interface EsdfField {
  /** Centre of voxel (0,0,0), local ENU metres. */
  origin: Vec3;
  resolutionM: number;
  dims: [number, number, number];
  distances: Float32Array;
}

/** Clearance the walker keeps from every surface, metres. */
export const WALK_RADIUS_M = 0.25;
/** The capsule's lowest axis point above the floor, metres: low enough for a
 * table edge, high enough to stay clear of the floor itself. */
const CAPSULE_BOTTOM_M = 0.6;
/** Spacing of the clearance samples along the capsule's axis, metres. */
const CAPSULE_STEP_M = 0.2;

/** The ESDF from its `occupancy.json` header and the grid's bytes; null when
 * either is malformed or the byte count does not match the dims. */
export function esdfField(header: unknown, bytes: ArrayBuffer): EsdfField | null {
  const h = objOrNull(header);
  if (!h) return null;
  const origin = arr(h.origin).map(num);
  const dims = arr(h.dims).map(num);
  const res = num(h.resolution_m);
  if (origin.length !== 3 || dims.length !== 3 || origin.some((v) => v === null) || !res || res <= 0) return null;
  if (dims.some((d) => d === null || d < 1 || !Number.isInteger(d))) return null;
  const [nx, ny, nz] = dims as [number, number, number];
  if (bytes.byteLength !== nx * ny * nz * 4) return null;
  // Copy into an aligned, little-endian view.
  const view = new DataView(bytes);
  const distances = new Float32Array(nx * ny * nz);
  for (let i = 0; i < distances.length; i++) distances[i] = view.getFloat32(i * 4, true);
  return { origin: origin as Vec3, resolutionM: res, dims: [nx, ny, nz], distances };
}

/** Distance to the nearest surface at an ENU point (nearest voxel), or null
 * outside the grid (nothing is known there, so nothing blocks). */
export function esdfDistance(f: EsdfField, [x, y, z]: Vec3): number | null {
  const i = Math.round((x - f.origin[0]) / f.resolutionM);
  const j = Math.round((y - f.origin[1]) / f.resolutionM);
  const k = Math.round((z - f.origin[2]) / f.resolutionM);
  const [nx, ny, nz] = f.dims;
  if (i < 0 || j < 0 || k < 0 || i >= nx || j >= ny || k >= nz) return null;
  return f.distances[(k * ny + j) * nx + i] ?? null;
}

/** The smallest clearance along the walker's vertical capsule axis, from
 * just above the floor to the eye (viewer coordinates); Infinity where the
 * field knows nothing. */
export function capsuleClearance(f: EsdfField, eye: Vec3, floorY: number): number {
  let min = Infinity;
  for (let y = floorY + CAPSULE_BOTTOM_M; y <= eye[1] + 1e-9; y += CAPSULE_STEP_M) {
    const d = esdfDistance(f, viewerToEnu([eye[0], y, eye[2]]));
    if (d !== null) min = Math.min(min, d);
  }
  const top = esdfDistance(f, viewerToEnu(eye));
  return top === null ? min : Math.min(min, top);
}

/** The horizontal move `[dx, dz]` (viewer coordinates) the walker at `eye`
 * may make: the whole step when it keeps `radius` of clearance; else the step
 * with its component into the wall removed, so the walker slides along it;
 * nothing when even that would bring it closer to a surface. A walker already
 * too close may always move away. */
export function slideStep(
  f: EsdfField,
  eye: Vec3,
  floorY: number,
  [dx, dz]: [number, number],
  radius = WALK_RADIUS_M,
): [number, number] {
  if (dx === 0 && dz === 0) return [0, 0];
  const at = (x: number, z: number) => capsuleClearance(f, [x, eye[1], z], floorY);
  const here = at(eye[0], eye[2]);
  const allowed = (x: number, z: number) => {
    const d = at(x, z);
    return d >= radius || d > here;
  };
  if (allowed(eye[0] + dx, eye[2] + dz)) return [dx, dz];
  // The wall normal: the clearance gradient where the step would land.
  const h = f.resolutionM;
  const [qx, qz] = [eye[0] + dx, eye[2] + dz];
  const gx = at(qx + h, qz) - at(qx - h, qz);
  const gz = at(qx, qz + h) - at(qx, qz - h);
  const len = Math.hypot(gx, gz);
  if (!Number.isFinite(len) || len === 0) return [0, 0];
  const [nx, nz] = [gx / len, gz / len];
  const into = dx * nx + dz * nz;
  if (into >= 0) return [0, 0];
  const slide: [number, number] = [dx - into * nx, dz - into * nz];
  return allowed(eye[0] + slide[0], eye[2] + slide[1]) ? slide : [0, 0];
}
