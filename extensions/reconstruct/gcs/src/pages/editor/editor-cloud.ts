/**
 * @module editor/editor-cloud
 * @description The cloud the editor shows: a version decoded in its local
 * frame, reduced to a point budget for the canvas, with each kept point's
 * layer. Layers are contiguous runs in file order (the node keeps them so),
 * and the provenance gives their counts, so a point's layer follows from its
 * index. Selection counts run over the shown points.
 */

import { decimateCloud } from "@altnautica/mapping-ui/components/model/viewers/decimate-cloud";

import type { LayerInfo } from "./editor-api";
import { pointInPolygon, type Xy } from "./geometry";

/** Points the canvas holds at most. */
export const EDITOR_POINT_BUDGET = 1_500_000;

export interface EditorCloud {
  /** Shown points, local frame (x east, y north, z up). */
  positions: Float32Array;
  colors: Float32Array | null;
  /** Layer id of each shown point. */
  layers: Uint16Array;
  /** Points in the version itself. */
  total: number;
  /** Whether the canvas shows fewer points than the version holds. */
  reduced: boolean;
  zRange: [number, number];
}

/** A selection's footprint: a ground polygon and an optional height band. */
export interface Region {
  polygon: Xy[];
  z: [number, number] | null;
}

/** The layer id of each of `total` points from the layer table; every point
 * is the base layer when the table does not add up to the count. */
export function layerIds(total: number, table: readonly LayerInfo[]): Uint16Array {
  const out = new Uint16Array(total);
  const sum = table.reduce((s, l) => s + l.points, 0);
  if (sum !== total) return out;
  let at = 0;
  for (const l of table) {
    out.fill(l.id, at, at + l.points);
    at += l.points;
  }
  return out;
}

export function buildEditorCloud(
  positions: Float32Array,
  colors: Float32Array | null,
  table: readonly LayerInfo[],
  budget = EDITOR_POINT_BUDGET,
): EditorCloud {
  const total = Math.floor(positions.length / 3);
  const ids = layerIds(total, table);
  const dec = decimateCloud(positions, colors, budget);
  const layers = dec.indices ? Uint16Array.from(dec.indices, (i) => ids[i]!) : ids;
  let lo = Infinity;
  let hi = -Infinity;
  for (let i = 2; i < dec.positions.length; i += 3) {
    const z = dec.positions[i]!;
    if (z < lo) lo = z;
    if (z > hi) hi = z;
  }
  return {
    positions: dec.positions,
    colors: dec.colors,
    layers,
    total,
    reduced: dec.decimated,
    zRange: [Number.isFinite(lo) ? lo : 0, Number.isFinite(hi) ? hi : 0],
  };
}

export interface RegionCount {
  count: number;
  /** `[min, max]` corners of the selected points, or null for none. */
  bounds: [[number, number, number], [number, number, number]] | null;
}

/** The shown points of visible layers inside `region`. */
export function countRegion(cloud: EditorCloud, region: Region, hidden: ReadonlySet<number>): RegionCount {
  const p = cloud.positions;
  let count = 0;
  const min: [number, number, number] = [Infinity, Infinity, Infinity];
  const max: [number, number, number] = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < cloud.layers.length; i++) {
    if (hidden.has(cloud.layers[i]!)) continue;
    const z = p[i * 3 + 2]!;
    if (region.z && (z < region.z[0] || z > region.z[1])) continue;
    const x = p[i * 3]!;
    const y = p[i * 3 + 1]!;
    if (!pointInPolygon(region.polygon, x, y)) continue;
    count++;
    for (const [k, v] of [x, y, z].entries()) {
      if (v < min[k]!) min[k] = v;
      if (v > max[k]!) max[k] = v;
    }
  }
  return { count, bounds: count > 0 ? [min, max] : null };
}
