/**
 * @module world/tour
 * @description The tour bundle's `tour.json`: the splat (and optional LOD
 * file), collision mesh, floorplan and room hotspots of one reconstruction,
 * every path relative to the bundle's `index.html`.
 */

import type { GeoAnchor, Placement } from "../contract/generated";
import { arr, num, obj, objOrNull, str, vec } from "../capture/coerce";
import { coercePlacement, type Vec3 } from "./placement";

export interface TourRoom {
  name: string;
  floor: number;
  hotspot: Vec3 | null;
  still: string | null;
}

export interface TourManifest {
  title: string;
  splat: string;
  lod: string | null;
  mesh: string | null;
  floorplan: string | null;
  anchor: GeoAnchor | null;
  /** Where the splat sits in local ENU; null = already in local ENU. */
  placement: Placement | null;
  rooms: TourRoom[];
}

/** A bundle-relative path: no scheme, no absolute path, no parent hops. */
function relPath(v: unknown): string | null {
  const s = str(v);
  if (!s || /^[a-z][a-z0-9+.-]*:/i.test(s) || s.startsWith("/") || s.split("/").includes("..")) return null;
  return s;
}

/** Coerce a `tour.json`, or null when it names no splat. */
export function coerceTourManifest(raw: unknown): TourManifest | null {
  const e = objOrNull(raw);
  const splat = relPath(e?.splat);
  if (!e || !splat) return null;
  return {
    title: str(e.title) ?? "Tour",
    splat,
    lod: relPath(e.lod),
    mesh: relPath(e.mesh),
    floorplan: relPath(e.floorplan),
    anchor: (objOrNull(e.anchor) as GeoAnchor | null) ?? null,
    placement: coercePlacement(e.placement),
    rooms: arr(e.rooms).flatMap((r) => {
      const room = obj(r);
      const name = str(room.name);
      return name
        ? [{ name, floor: num(room.floor) ?? 0, hotspot: vec(room.hotspot, 3), still: relPath(room.still) }]
        : [];
    }),
  };
}
