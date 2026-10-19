/**
 * @module world/placement
 * @description Where a reconstructed artifact sits in the world, and how the
 * viewer draws it. The contract places every artifact in a local ENU frame
 * (metres, Z up) with a placement `p_enu = scale · R · p_artifact + t`; the
 * viewer is three.js (Y up, Z towards the viewer), so ENU maps to it as
 * `x = east, y = up, z = −north`. Every overlay (floorplan, hotspots,
 * measurements, the collision mesh) goes through the same two functions, so
 * splat, mesh and plan always agree.
 */

import type { Placement } from "../contract/generated";

export type Vec3 = [number, number, number];

/** The identity placement: the artifact is already in local ENU metres. */
export const IDENTITY_PLACEMENT: Placement = {
  anchor: null,
  R_enu_artifact: [1, 0, 0, 0, 1, 0, 0, 0, 1],
  t_enu_artifact: [0, 0, 0],
  scale: 1,
};

function finiteArray(v: unknown, n: number): v is number[] {
  return Array.isArray(v) && v.length === n && v.every((x) => typeof x === "number" && Number.isFinite(x));
}

/** A placement (or a `.place.json` artifact descriptor, which carries the same
 * fields), or null when the rotation, translation or scale is malformed. */
export function coercePlacement(raw: unknown): Placement | null {
  if (!raw || typeof raw !== "object") return null;
  const e = raw as Record<string, unknown>;
  if (!finiteArray(e.R_enu_artifact, 9) || !finiteArray(e.t_enu_artifact, 3)) return null;
  const scale = typeof e.scale === "number" && Number.isFinite(e.scale) && e.scale > 0 ? e.scale : null;
  if (scale === null) return null;
  const anchor = e.anchor && typeof e.anchor === "object" ? (e.anchor as Placement["anchor"]) : null;
  return { anchor, R_enu_artifact: e.R_enu_artifact, t_enu_artifact: e.t_enu_artifact, scale };
}

/** ENU metres → viewer coordinates. */
export function enuToViewer(p: readonly number[]): Vec3 {
  return [p[0] ?? 0, p[2] ?? 0, -(p[1] ?? 0)];
}

/** Viewer coordinates → ENU metres (the inverse of {@link enuToViewer}). */
export function viewerToEnu(p: readonly number[]): Vec3 {
  return [p[0] ?? 0, -(p[2] ?? 0), p[1] ?? 0];
}

/** A point in the artifact's own frame → ENU metres. */
export function artifactToEnu(p: Placement, [x, y, z]: Vec3): Vec3 {
  const R = p.R_enu_artifact;
  const t = p.t_enu_artifact;
  const s = p.scale;
  return [
    s * (R[0]! * x + R[1]! * y + R[2]! * z) + t[0]!,
    s * (R[3]! * x + R[4]! * y + R[5]! * z) + t[1]!,
    s * (R[6]! * x + R[7]! * y + R[8]! * z) + t[2]!,
  ];
}

/**
 * The row-major 4×4 that takes artifact coordinates into viewer coordinates:
 * `M_viewer_enu · [s·R | t]`. Feed it to three's `Matrix4.set(...)` (row-major).
 */
export function artifactToViewerMatrix(p: Placement | null): number[] {
  const { R_enu_artifact: R, t_enu_artifact: t, scale: s } = p ?? IDENTITY_PLACEMENT;
  // ENU rows (east, north, up) of s·R and t.
  const east = [s * R[0]!, s * R[1]!, s * R[2]!, t[0]!];
  const north = [s * R[3]!, s * R[4]!, s * R[5]!, t[1]!];
  const up = [s * R[6]!, s * R[7]!, s * R[8]!, t[2]!];
  // Viewer rows: x = east, y = up, z = −north.
  return [...east, ...up, ...north.map((v) => -v), 0, 0, 0, 1];
}
