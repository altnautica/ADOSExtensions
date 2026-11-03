import { describe, expect, it } from "vitest";

import {
  applyLivePayload,
  decodeLiveMapPayload,
  EMPTY_LIVE_MAP,
} from "../../../src/lib/atlas/live-map";
import { decodeWorldDescriptor } from "../../../src/lib/atlas/world-descriptors";
import {
  PLUGIN_ATLAS_OCCUPANCY_TOPIC,
  PLUGIN_ATLAS_POINTCLOUD_TOPIC,
} from "../../../src/lib/atlas/world-contract";
import { encodeTestMap } from "./golden-atlas-frames";

function pointB64(x: number, y: number, z: number): string {
  const buf = new Uint8Array(12);
  const v = new DataView(buf.buffer);
  [x, y, z].forEach((c, i) => v.setFloat32(i * 4, c, true));
  return btoa(String.fromCharCode(...buf));
}

const occ = encodeTestMap({
  kind: "live",
  seq: 1,
  frame: "map_frd",
  res_m: 0.05,
  tile_cells: 64,
  tiles: [{ ix: 0, iy: 1, png_b64: "AA==" }],
});

const cloud = (voxel: number, seq: number) =>
  encodeTestMap({
    kind: "live",
    seq,
    frame: "map_frd",
    voxel_m: voxel,
    count: 1,
    points_b64: pointB64(1.5, -2, 0.25),
  });

describe("live map payloads", () => {
  it("decode the tiles and points and are not artifact descriptors", () => {
    const o = decodeLiveMapPayload(PLUGIN_ATLAS_OCCUPANCY_TOPIC, occ);
    expect(o).toMatchObject({ kind: "occupancy", seq: 1, tileCells: 64 });
    expect(decodeWorldDescriptor(PLUGIN_ATLAS_OCCUPANCY_TOPIC, occ)).toBeNull();
    const c = decodeLiveMapPayload(PLUGIN_ATLAS_POINTCLOUD_TOPIC, cloud(0.05, 1));
    expect(c?.kind).toBe("pointcloud");
    if (c?.kind !== "pointcloud") return;
    expect(Array.from(c.points)).toEqual([1.5, -2, 0.25]);
    expect(
      decodeLiveMapPayload(PLUGIN_ATLAS_OCCUPANCY_TOPIC, encodeTestMap({ kind: "product" })),
    ).toBeNull();
  });

  it("accumulate, and a coarsened voxel restarts the cloud", () => {
    let map = EMPTY_LIVE_MAP;
    for (const p of [
      decodeLiveMapPayload(PLUGIN_ATLAS_OCCUPANCY_TOPIC, occ),
      decodeLiveMapPayload(PLUGIN_ATLAS_POINTCLOUD_TOPIC, cloud(0.05, 1)),
      decodeLiveMapPayload(PLUGIN_ATLAS_POINTCLOUD_TOPIC, cloud(0.05, 2)),
    ]) {
      if (p) map = applyLivePayload(map, p);
    }
    expect(Object.keys(map.tiles)).toEqual(["0,1"]);
    expect(map.points.length).toBe(6);
    const coarse = decodeLiveMapPayload(PLUGIN_ATLAS_POINTCLOUD_TOPIC, cloud(0.1, 3));
    if (coarse) map = applyLivePayload(map, coarse);
    expect(map.points.length).toBe(3);
    expect(map.voxelM).toBe(0.1);
  });
});
