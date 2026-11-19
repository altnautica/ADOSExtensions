/**
 * The tour bundle manifest: paths stay inside the bundle, the placement is
 * read through the contract coercion, rooms keep their hotspot and still.
 */

import { describe, expect, it } from "vitest";

import { coerceTourManifest } from "../../../src/lib/world/tour";

describe("coerceTourManifest", () => {
  it("reads the bundle manifest", () => {
    const m = coerceTourManifest({
      contract: "ados.capture/1",
      title: "Flat 4B",
      splat: "scene.spz",
      lod: "scene-lod-0.rad",
      mesh: "collision.glb",
      floorplan: "floorplan.json",
      anchor: null,
      placement: { anchor: null, R_enu_artifact: [1, 0, 0, 0, 1, 0, 0, 0, 1], t_enu_artifact: [0, 0, 0], scale: 1 },
      rooms: [{ name: "Kitchen", floor: 0, hotspot: [1, 2, 1.6], still: "stills/0.jpg" }, { floor: 1 }],
    })!;
    expect(m.title).toBe("Flat 4B");
    expect(m.lod).toBe("scene-lod-0.rad");
    expect(m.placement?.scale).toBe(1);
    expect(m.rooms).toEqual([{ name: "Kitchen", floor: 0, hotspot: [1, 2, 1.6], still: "stills/0.jpg" }]);
  });

  it("refuses paths that leave the bundle and a manifest without a splat", () => {
    const m = coerceTourManifest({
      splat: "scene.spz",
      mesh: "../../etc/passwd",
      floorplan: "https://example.com/plan.json",
      lod: "/abs.rad",
    })!;
    expect([m.mesh, m.floorplan, m.lod]).toEqual([null, null, null]);
    expect(coerceTourManifest({ splat: "../x.spz" })).toBeNull();
    expect(coerceTourManifest({})).toBeNull();
  });

  it("reads the ESDF and the navigation nodes, dropping malformed nodes", () => {
    const m = coerceTourManifest({
      splat: "scene.spz",
      esdf: "occupancy.esdf",
      nodes: [{ id: "n0", position: [1, 2, 1.6], yaw_deg: 90 }, { id: "n1", position: [1, 2] }, { position: [0, 0, 0] }],
    })!;
    expect(m.esdf).toBe("occupancy.esdf");
    expect(m.nodes).toEqual([{ id: "n0", position: [1, 2, 1.6], yawDeg: 90 }]);
    const old = coerceTourManifest({ splat: "scene.spz" })!;
    expect([old.esdf, old.nodes]).toEqual([null, []]);
  });
});
