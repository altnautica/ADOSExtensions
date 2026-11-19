/**
 * @module editor/geometry
 * @description The editor's ground-plane geometry, the same rules the node
 * applies: point in polygon (ray casting), a polygon that crosses itself, the
 * patch placement (rotate about the source box centre, then translate), and
 * the measurements (distance, area, volume above a plane). All coordinates
 * are the product's local frame in metres (x east, y north, z up).
 */

export type Xy = [number, number];
export type Xyz = [number, number, number];

/** Most corners a selection polygon may have (the node's limit). */
export const MAX_POLYGON_VERTICES = 256;

export function pointInPolygon(polygon: readonly Xy[], x: number, y: number): boolean {
  let inside = false;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    const [ax, ay] = polygon[i]!;
    const [bx, by] = polygon[j]!;
    if (ay > y !== by > y && x < ((bx - ax) * (y - ay)) / (by - ay) + ax) inside = !inside;
  }
  return inside;
}

function orient(a: Xy, b: Xy, c: Xy): number {
  return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
}

function onSegment(a: Xy, b: Xy, p: Xy): boolean {
  return (
    p[0] >= Math.min(a[0], b[0]) &&
    p[0] <= Math.max(a[0], b[0]) &&
    p[1] >= Math.min(a[1], b[1]) &&
    p[1] <= Math.max(a[1], b[1])
  );
}

function segmentsTouch(a: Xy, b: Xy, c: Xy, d: Xy): boolean {
  const d1 = orient(c, d, a);
  const d2 = orient(c, d, b);
  const d3 = orient(a, b, c);
  const d4 = orient(a, b, d);
  if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) return true;
  return (
    (d1 === 0 && onSegment(c, d, a)) ||
    (d2 === 0 && onSegment(c, d, b)) ||
    (d3 === 0 && onSegment(a, b, c)) ||
    (d4 === 0 && onSegment(a, b, d))
  );
}

/** Whether two non-adjacent edges of the closed polygon cross or touch. */
export function selfIntersects(polygon: readonly Xy[]): boolean {
  const n = polygon.length;
  for (let i = 0; i < n; i++) {
    for (let j = i + 1; j < n; j++) {
      if (j === i + 1 || (i === 0 && j === n - 1)) continue;
      if (segmentsTouch(polygon[i]!, polygon[(i + 1) % n]!, polygon[j]!, polygon[(j + 1) % n]!)) return true;
    }
  }
  return false;
}

/** The axis-aligned rectangle with opposite corners `a` and `b`. */
export function rectPolygon(a: Xy, b: Xy): Xy[] {
  const [x0, x1] = [Math.min(a[0], b[0]), Math.max(a[0], b[0])];
  const [y0, y1] = [Math.min(a[1], b[1]), Math.max(a[1], b[1])];
  return [
    [x0, y0],
    [x1, y0],
    [x1, y1],
    [x0, y1],
  ];
}

/** Shoelace area of a simple polygon, m². */
export function polygonArea(polygon: readonly Xy[]): number {
  let s = 0;
  for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
    s += polygon[j]![0] * polygon[i]![1] - polygon[i]![0] * polygon[j]![1];
  }
  return Math.abs(s) / 2;
}

export function distance(a: Xyz, b: Xyz): number {
  return Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
}

/** A patch placement: rotate about the source box centre, then translate. */
export function place(box: [Xy, Xy], rotateDeg: number, translate: Xy, p: Xy): Xy {
  const cx = (box[0][0] + box[1][0]) / 2;
  const cy = (box[0][1] + box[1][1]) / 2;
  const r = (rotateDeg * Math.PI) / 180;
  const [s, c] = [Math.sin(r), Math.cos(r)];
  const [dx, dy] = [p[0] - cx, p[1] - cy];
  return [cx + c * dx - s * dy + translate[0], cy + s * dx + c * dy + translate[1]];
}

/**
 * Volume above a horizontal base plane inside a ground polygon: per grid
 * cell, the highest point above the plane times the cell area (the node's
 * rule, run on whatever points are given).
 */
export function volumeAbove(
  positions: ArrayLike<number>,
  include: (i: number) => boolean,
  polygon: readonly Xy[],
  baseZ: number,
  cellM: number,
): { volumeM3: number; cells: number } {
  const top = new Map<string, number>();
  for (let i = 0; i * 3 + 2 < positions.length; i++) {
    if (!include(i)) continue;
    const x = positions[i * 3]!;
    const y = positions[i * 3 + 1]!;
    if (!pointInPolygon(polygon, x, y)) continue;
    const key = `${Math.floor(x / cellM)},${Math.floor(y / cellM)}`;
    const z = positions[i * 3 + 2]!;
    const prev = top.get(key);
    if (prev === undefined || z > prev) top.set(key, z);
  }
  let v = 0;
  for (const z of top.values()) v += Math.max(0, z - baseZ);
  return { volumeM3: v * cellM * cellM, cells: top.size };
}
