/**
 * @module atlas/viewers/splat-format
 * @description Derive a splat artifact's real format from its file name. The
 * splat loader is handed a stand-in URL (see `sentinel-fetch`), so its own
 * `endsWith('.ply')` sniffing cannot be trusted; the viewer passes an
 * explicit format instead.
 */

/** A splat artifact's file kind. */
export type SplatExt = "ply" | "splat" | "ksplat" | "spz";

/**
 * The artifact's file kind from its name (a query string or fragment is
 * ignored). Defaults to `ply` (the only format the compute node emits today)
 * when nothing matches.
 */
export function splatArtifactExt(name: string): SplatExt {
  const path = name.split(/[?#]/)[0] ?? name;
  const ext = path.toLowerCase().split(".").pop() ?? "";
  if (ext === "splat" || ext === "ksplat" || ext === "spz") return ext;
  return "ply";
}
