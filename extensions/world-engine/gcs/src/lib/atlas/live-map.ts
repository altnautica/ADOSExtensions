/**
 * @module lib/atlas/live-map
 * @description The live map a drone (or, while it streams, its compute node)
 * builds as it flies: occupancy tiles and sweep-cloud deltas on the
 * occupancy and point-cloud topics with `kind: "live"`. Decodes them and
 * folds them into one accumulated map per device.
 *
 * Frame: `map_ned` (north/east metres from the first scan). A tile `(ix, iy)`
 * covers x in `[ix, ix+1) * tile_cells * res_m`, y likewise; its PNG row 0 is
 * the largest x (north up), column 0 the smallest y (west left).
 */

import { asMsgpackMap, decodeMsgpack, type MsgpackValue } from "./msgpack";
import {
  PLUGIN_ATLAS_OCCUPANCY_TOPIC,
  PLUGIN_ATLAS_POINTCLOUD_TOPIC,
} from "./world-contract";

export type LiveMapPayload =
  | {
      kind: "occupancy";
      seq: number;
      resM: number;
      tileCells: number;
      tiles: { ix: number; iy: number; pngB64: string }[];
    }
  | { kind: "pointcloud"; seq: number; voxelM: number; points: Float32Array };

/** The map accumulated from every live payload of one device. */
export interface LiveMap {
  resM: number;
  tileCells: number;
  /** Latest PNG (base64) per tile, keyed `"ix,iy"`. */
  tiles: Record<string, { ix: number; iy: number; pngB64: string }>;
  voxelM: number;
  /** Every sweep centroid so far, xyz interleaved. */
  points: Float32Array;
  occSeq: number;
  cloudSeq: number;
}

export const EMPTY_LIVE_MAP: LiveMap = {
  resM: 0.05,
  tileCells: 64,
  tiles: {},
  voxelM: 0,
  points: new Float32Array(0),
  occSeq: 0,
  cloudSeq: 0,
};

const num = (v: MsgpackValue | undefined): number | null =>
  typeof v === "number" ? v : typeof v === "bigint" ? Number(v) : null;
const str = (v: MsgpackValue | undefined): string | null =>
  typeof v === "string" ? v : null;

function base64Bytes(b64: string): Uint8Array {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i += 1) out[i] = bin.charCodeAt(i);
  return out;
}

/** Decode a live payload on `topic`, or null for anything else (a product
 * descriptor, another topic, a malformed map). */
export function decodeLiveMapPayload(
  topic: string,
  payload: Uint8Array,
): LiveMapPayload | null {
  let fields: { [k: string]: MsgpackValue } | null;
  try {
    fields = asMsgpackMap(decodeMsgpack(payload));
  } catch {
    return null;
  }
  if (!fields || fields.kind !== "live") return null;
  const seq = num(fields.seq);
  if (seq === null) return null;
  if (topic === PLUGIN_ATLAS_OCCUPANCY_TOPIC) {
    const resM = num(fields.res_m);
    const tileCells = num(fields.tile_cells);
    if (resM === null || tileCells === null || !Array.isArray(fields.tiles)) return null;
    const tiles = [];
    for (const t of fields.tiles) {
      const m = asMsgpackMap(t);
      const ix = num(m?.ix);
      const iy = num(m?.iy);
      const pngB64 = str(m?.png_b64);
      if (ix === null || iy === null || pngB64 === null) return null;
      tiles.push({ ix, iy, pngB64 });
    }
    return { kind: "occupancy", seq, resM, tileCells, tiles };
  }
  if (topic === PLUGIN_ATLAS_POINTCLOUD_TOPIC) {
    const voxelM = num(fields.voxel_m);
    const count = num(fields.count);
    const b64 = str(fields.points_b64);
    if (voxelM === null || count === null || b64 === null) return null;
    const bytes = base64Bytes(b64);
    if (bytes.length !== count * 12) return null;
    const points = new Float32Array(count * 3);
    const view = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
    for (let i = 0; i < count * 3; i += 1) points[i] = view.getFloat32(i * 4, true);
    return { kind: "pointcloud", seq, voxelM, points };
  }
  return null;
}

/** Fold one payload into `map`. A changed voxel edge supersedes every earlier
 * cloud delta, so the cloud restarts from this one. */
export function applyLivePayload(map: LiveMap, p: LiveMapPayload): LiveMap {
  if (p.kind === "occupancy") {
    const tiles = { ...map.tiles };
    for (const t of p.tiles) tiles[`${t.ix},${t.iy}`] = t;
    return { ...map, resM: p.resM, tileCells: p.tileCells, tiles, occSeq: p.seq };
  }
  const restart = map.voxelM !== 0 && p.voxelM !== map.voxelM;
  const base = restart ? new Float32Array(0) : map.points;
  const points = new Float32Array(base.length + p.points.length);
  points.set(base, 0);
  points.set(p.points, base.length);
  return { ...map, voxelM: p.voxelM, points, cloudSeq: p.seq };
}
