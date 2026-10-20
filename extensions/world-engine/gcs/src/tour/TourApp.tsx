/**
 * @module tour/TourApp
 * @description The standalone tour viewer: reads `tour.json` beside the page,
 * then the floorplan it names, and shows the splat in the shared world viewer
 * (walk, dollhouse, floorplan, room hotspots, measure on the collision mesh)
 * with a room list of hero stills. Static files only, no backend.
 */

import { useEffect, useState } from "react";

import { SparkViewer } from "../components/world/SparkViewer";
import type { WorldScene } from "../components/world/use-spark-scene";
import { translator } from "../host/i18n";
import { urlSource } from "../lib/net/artifact-source";
import { coerceFloorplan } from "../lib/world/floorplan";
import { coerceTourManifest, type TourManifest } from "../lib/world/tour";

const t = translator("tour");

type TourState =
  | { status: "loading" }
  | { status: "failed" }
  | { status: "ready"; manifest: TourManifest; scene: WorldScene };

async function fetchJson(url: string): Promise<unknown | null> {
  try {
    const res = await fetch(url);
    return res.ok ? await res.json() : null;
  } catch {
    return null;
  }
}

/** Load the tour next to `pageUrl` into a world scene. */
export async function loadTour(pageUrl: string): Promise<{ manifest: TourManifest; scene: WorldScene } | null> {
  const at = (rel: string) => new URL(rel, pageUrl).href;
  const manifest = coerceTourManifest(await fetchJson(at("tour.json")));
  if (!manifest) return null;
  const floorplan = manifest.floorplan ? coerceFloorplan(await fetchJson(at(manifest.floorplan))) : null;
  // The ESDF's header is the `.json` of the grid's stem, as the node writes it.
  const esdfHeader = manifest.esdf ? await fetchJson(at(manifest.esdf.replace(/\.[^./]+$/, "") + ".json")) : null;
  return {
    manifest,
    scene: {
      splat: urlSource(at(manifest.lod ?? manifest.splat)),
      placement: manifest.placement,
      collision: manifest.mesh ? urlSource(at(manifest.mesh)) : null,
      floorplan,
      rooms: manifest.rooms.map((r) => ({ name: r.name, floor: r.floor, hotspot: r.hotspot })),
      esdf: manifest.esdf && esdfHeader ? { header: esdfHeader, grid: urlSource(at(manifest.esdf)) } : null,
      nodes: manifest.nodes,
    },
  };
}

export function TourApp() {
  const [state, setState] = useState<TourState>({ status: "loading" });

  useEffect(() => {
    void loadTour(window.location.href).then((tour) => {
      setState(tour ? { status: "ready", ...tour } : { status: "failed" });
      if (tour) document.title = tour.manifest.title;
    });
  }, []);

  if (state.status === "loading") {
    return <p className="we:p-6 we:text-sm we:text-text-tertiary">{t("loading")}</p>;
  }
  if (state.status === "failed") {
    return <p className="we:p-6 we:text-sm we:text-status-error">{t("failed")}</p>;
  }
  const { manifest, scene } = state;
  const stills = manifest.rooms.filter((r) => r.still);

  return (
    <div className="we:flex we:h-full we:flex-col">
      <header className="we:flex we:items-center we:gap-3 we:border-b we:border-border-default we:px-4 we:py-2">
        <h1 className="we:text-sm we:font-medium">{manifest.title}</h1>
        <span className="we:text-[11px] we:text-text-tertiary">{t("rooms", { n: manifest.rooms.length })}</span>
      </header>
      <main className="we:relative we:flex-1">
        <SparkViewer scene={scene} className="we:absolute we:inset-0" />
      </main>
      {stills.length > 0 && (
        <nav aria-label={t("roomList")} className="we:flex we:gap-2 we:overflow-x-auto we:border-t we:border-border-default we:p-2">
          {stills.map((r) => (
            <figure key={`${r.floor}-${r.name}`} className="we:w-40 we:shrink-0">
              <img src={r.still ?? undefined} alt={r.name} loading="lazy" className="we:aspect-video we:w-full we:rounded we:object-cover" />
              <figcaption className="we:mt-1 we:text-[11px] we:text-text-secondary">{r.name}</figcaption>
            </figure>
          ))}
        </nav>
      )}
    </div>
  );
}
