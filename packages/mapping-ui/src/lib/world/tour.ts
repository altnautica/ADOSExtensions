/**
 * @module world/tour
 * @description The tour bundle's `tour.json`: the splat (and optional LOD
 * file), collision mesh, floorplan, occupancy ESDF, room hotspots and the
 * navigation nodes of one reconstruction, every path relative to the
 * bundle's `index.html`.
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

/** A place on the walked path the viewer offers as a jump target. */
export interface TourNode {
  id: string;
  /** Camera position, local ENU metres. */
  position: Vec3;
  /** Heading, degrees counter-clockwise from east. */
  yawDeg: number;
}

export interface TourManifest {
  title: string;
  splat: string;
  lod: string | null;
  mesh: string | null;
  floorplan: string | null;
  /** The occupancy ESDF grid; its header is the `.json` of the same stem. */
  esdf: string | null;
  anchor: GeoAnchor | null;
  /** Where the splat sits in local ENU; null = already in local ENU. */
  placement: Placement | null;
  rooms: TourRoom[];
  nodes: TourNode[];
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
    esdf: relPath(e.esdf),
    anchor: (objOrNull(e.anchor) as GeoAnchor | null) ?? null,
    placement: coercePlacement(e.placement),
    rooms: arr(e.rooms).flatMap((r) => {
      const room = obj(r);
      const name = str(room.name);
      return name
        ? [{ name, floor: num(room.floor) ?? 0, hotspot: vec(room.hotspot, 3), still: relPath(room.still) }]
        : [];
    }),
    nodes: arr(e.nodes).flatMap((n) => {
      const node = obj(n);
      const id = str(node.id);
      const position = vec(node.position, 3);
      return id && position ? [{ id, position, yawDeg: num(node.yaw_deg) ?? 0 }] : [];
    }),
  };
}
