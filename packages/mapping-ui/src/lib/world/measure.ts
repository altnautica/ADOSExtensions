/**
 * @module world/measure
 * @description Measurement math for the viewer's measure tool: point-to-point
 * distances in ENU metres, wall-to-wall distances on the plan, and the metric
 * plus imperial readout listings need.
 */

import type { Vec3 } from "./placement";
import type { Segment } from "./floorplan";

export const M_PER_FT = 0.3048;
export const FT2_PER_M2 = 1 / (M_PER_FT * M_PER_FT);

export function distance3(a: Vec3, b: Vec3): number {
  return Math.hypot(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
}

/** Shortest distance from plan point `p` to segment `s` (metres). */
export function pointToSegment(p: [number, number], [x0, y0, x1, y1]: Segment): number {
  const dx = x1 - x0;
  const dy = y1 - y0;
  const len2 = dx * dx + dy * dy;
  const t = len2 === 0 ? 0 : Math.max(0, Math.min(1, ((p[0] - x0) * dx + (p[1] - y0) * dy) / len2));
  return Math.hypot(p[0] - (x0 + t * dx), p[1] - (y0 + t * dy));
}

function segmentsCross(a: Segment, b: Segment): boolean {
  const cross = (ax: number, ay: number, bx: number, by: number, cx: number, cy: number) =>
    (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
  const d1 = cross(b[0], b[1], b[2], b[3], a[0], a[1]);
  const d2 = cross(b[0], b[1], b[2], b[3], a[2], a[3]);
  const d3 = cross(a[0], a[1], a[2], a[3], b[0], b[1]);
  const d4 = cross(a[0], a[1], a[2], a[3], b[2], b[3]);
  return d1 * d2 < 0 && d3 * d4 < 0;
}

/** Shortest distance between two wall segments on the plan (0 when they
 * cross); for two parallel walls this is the room width between them. */
export function segmentToSegment(a: Segment, b: Segment): number {
  if (segmentsCross(a, b)) return 0;
  return Math.min(
    pointToSegment([a[0], a[1]], b),
    pointToSegment([a[2], a[3]], b),
    pointToSegment([b[0], b[1]], a),
    pointToSegment([b[2], b[3]], a),
  );
}

/** "3.42 m · 11 ft 3 in". */
export function formatLength(m: number): string {
  const totalIn = Math.round((m / M_PER_FT) * 12);
  const ft = Math.floor(totalIn / 12);
  const inch = totalIn % 12;
  return `${m.toFixed(2)} m · ${ft} ft ${inch} in`;
}

/** "18.6 m² · 200 ft²". */
export function formatArea(m2: number): string {
  return `${m2.toFixed(1)} m² · ${Math.round(m2 * FT2_PER_M2)} ft²`;
}
