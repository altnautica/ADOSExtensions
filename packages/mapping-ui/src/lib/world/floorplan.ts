/**
 * @module world/floorplan
 * @description The node's `floorplan.json` (walls, doors, rooms per floor in
 * local ENU metres) and the two ways it is drawn: as a north-up 2D plan (SVG
 * pixels) and as lines on the floor of the 3D view (viewer coordinates).
 */

import { arr, num, obj, str, vec } from "../capture/coerce";
import { enuToViewer, type Vec3 } from "./placement";

/** A wall or door segment `[x0, y0, x1, y1]` in ENU metres. */
export type Segment = [number, number, number, number];

export interface FloorRoom {
  name: string;
  polygon: [number, number][];
  areaM2: number | null;
  areaFt2: number | null;
  hotspot: Vec3 | null;
}

export interface FloorplanFloor {
  floor: number;
  zM: number;
  walls: Segment[];
  doors: Segment[];
  rooms: FloorRoom[];
}

export interface Floorplan {
  floors: FloorplanFloor[];
}

function segments(raw: unknown): Segment[] {
  return arr(raw).flatMap((s) => {
    const v = Array.isArray(s) && s.length === 4 && s.every((x) => typeof x === "number") ? s : null;
    return v ? [v as Segment] : [];
  });
}

/** Coerce a `floorplan.json`, or null when it carries no floor. */
export function coerceFloorplan(raw: unknown): Floorplan | null {
  const floors = arr(obj(raw).floors).flatMap((f): FloorplanFloor[] => {
    const e = obj(f);
    const floor = num(e.floor);
    if (floor === null) return [];
    return [
      {
        floor,
        zM: num(e.z_m) ?? 0,
        walls: segments(e.walls),
        doors: segments(e.doors),
        rooms: arr(e.rooms).flatMap((r): FloorRoom[] => {
          const room = obj(r);
          const polygon = arr(room.polygon).flatMap((p) => {
            const v = vec(p, 2);
            return v ? [v] : [];
          });
          return [
            {
              name: str(room.name) ?? "",
              polygon,
              areaM2: num(room.area_m2),
              areaFt2: num(room.area_ft2),
              hotspot: vec(room.hotspot, 3),
            },
          ];
        }),
      },
    ];
  });
  return floors.length > 0 ? { floors } : null;
}

export interface Bounds {
  minX: number;
  minY: number;
  maxX: number;
  maxY: number;
}

/** The plan extent of one floor (walls, doors and room outlines). */
export function floorBounds(f: FloorplanFloor): Bounds | null {
  const xs: number[] = [];
  const ys: number[] = [];
  for (const [x0, y0, x1, y1] of [...f.walls, ...f.doors]) xs.push(x0, x1), ys.push(y0, y1);
  for (const r of f.rooms) for (const [x, y] of r.polygon) xs.push(x), ys.push(y);
  if (xs.length === 0) return null;
  return { minX: Math.min(...xs), minY: Math.min(...ys), maxX: Math.max(...xs), maxY: Math.max(...ys) };
}

/** Maps ENU metres onto a `width × height` pixel box, north up, one uniform
 * scale, centred, with `pad` pixels of margin. */
export interface PlanTransform {
  /** Pixels per metre. */
  scale: number;
  toPx(x: number, y: number): [number, number];
  toEnu(px: number, py: number): [number, number];
}

export function planTransform(b: Bounds, width: number, height: number, pad = 12): PlanTransform {
  const spanX = Math.max(b.maxX - b.minX, 1e-6);
  const spanY = Math.max(b.maxY - b.minY, 1e-6);
  const scale = Math.min((width - 2 * pad) / spanX, (height - 2 * pad) / spanY);
  // Centre the plan in the box; screen y grows down, north grows up.
  const ox = (width - spanX * scale) / 2;
  const oy = (height - spanY * scale) / 2;
  return {
    scale,
    toPx: (x, y) => [ox + (x - b.minX) * scale, oy + (b.maxY - y) * scale],
    toEnu: (px, py) => [b.minX + (px - ox) / scale, b.maxY - (py - oy) / scale],
  };
}

/** A floor's walls as line-segment vertex pairs in viewer coordinates, drawn
 * `lift` metres above the floor so they do not z-fight the splat floor. */
export function floorLinesViewer(f: FloorplanFloor, lift = 0.02): Float32Array {
  const out = new Float32Array(f.walls.length * 6);
  f.walls.forEach(([x0, y0, x1, y1], i) => {
    out.set([...enuToViewer([x0, y0, f.zM + lift]), ...enuToViewer([x1, y1, f.zM + lift])], i * 6);
  });
  return out;
}

/** Shoelace area of a plan polygon in m² (absolute). */
export function polygonArea(poly: readonly [number, number][]): number {
  let twice = 0;
  for (let i = 0; i < poly.length; i++) {
    const [x0, y0] = poly[i]!;
    const [x1, y1] = poly[(i + 1) % poly.length]!;
    twice += x0 * y1 - x1 * y0;
  }
  return Math.abs(twice) / 2;
}

/** A room's label point: its hotspot when set, else its vertex mean. */
export function roomLabelPoint(r: FloorRoom): [number, number] | null {
  if (r.hotspot) return [r.hotspot[0], r.hotspot[1]];
  if (r.polygon.length === 0) return null;
  const sx = r.polygon.reduce((s, p) => s + p[0], 0);
  const sy = r.polygon.reduce((s, p) => s + p[1], 0);
  return [sx / r.polygon.length, sy / r.polygon.length];
}
