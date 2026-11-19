/**
 * @module world/aisles
 * @description The node's warehouse `aisles.geojson`: one LineString per aisle
 * centreline in local ENU metres (`frame: "enu_local"`), with its id, width
 * and length. Drawn as floor strips in the viewer.
 */

import { arr, num, obj, str, vec } from "../capture/coerce";
import { enuToViewer } from "./placement";

export interface Aisle {
  id: string;
  /** Centreline vertices `[east, north]`, metres. */
  line: [number, number][];
  widthM: number | null;
  lengthM: number | null;
}

/** Coerce an `aisles.geojson` FeatureCollection; non-LineString features and
 * lines with fewer than two vertices are dropped. */
export function coerceAisles(raw: unknown): Aisle[] {
  return arr(obj(raw).features).flatMap((f, i): Aisle[] => {
    const feature = obj(f);
    const geom = obj(feature.geometry);
    if (geom.type !== "LineString") return [];
    const line = arr(geom.coordinates).flatMap((c) => {
      const v = vec(Array.isArray(c) ? c.slice(0, 2) : c, 2);
      return v ? [v] : [];
    });
    if (line.length < 2) return [];
    const p = obj(feature.properties);
    return [{ id: str(p.id) ?? String(num(p.id) ?? i + 1), line, widthM: num(p.width_m), lengthM: num(p.length_m) }];
  });
}

/** Each aisle as line-segment vertex pairs in viewer coordinates at height
 * `z` (ENU up, metres): the centreline, plus the two edges `width/2` either
 * side of every centreline segment when the width is known. */
export function aisleLinesViewer(aisles: readonly Aisle[], z: number): Float32Array {
  const out: number[] = [];
  const push = (x0: number, y0: number, x1: number, y1: number) =>
    out.push(...enuToViewer([x0, y0, z]), ...enuToViewer([x1, y1, z]));
  for (const a of aisles) {
    for (let i = 0; i + 1 < a.line.length; i++) {
      const [x0, y0] = a.line[i]!;
      const [x1, y1] = a.line[i + 1]!;
      push(x0, y0, x1, y1);
      const len = Math.hypot(x1 - x0, y1 - y0);
      if (!a.widthM || len === 0) continue;
      // Unit normal to the segment, scaled to half the aisle width.
      const nx = (-(y1 - y0) / len) * (a.widthM / 2);
      const ny = ((x1 - x0) / len) * (a.widthM / 2);
      push(x0 + nx, y0 + ny, x1 + nx, y1 + ny);
      push(x0 - nx, y0 - ny, x1 - nx, y1 - ny);
    }
  }
  return new Float32Array(out);
}
