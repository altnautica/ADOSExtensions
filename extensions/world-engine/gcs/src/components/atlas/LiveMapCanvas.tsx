/**
 * @module atlas/LiveMapCanvas
 * @description A top-down view of the live map a device builds as it flies:
 * the occupancy tiles (grey PNG, north up) and the sweep-cloud points over
 * them, in the `map_ned` frame (north up, east right). Draws nothing until a
 * live payload has arrived.
 */

import { useEffect, useRef } from "react";

import { translator } from "../../host/i18n";
import type { LiveMap } from "../../lib/atlas/live-map";

const t = translator("atlas");

/** Pixels the canvas is drawn at (CSS scales it). */
const SIZE = 512;

/** The map's north/east bounds over tiles and points, or null when empty. */
function bounds(map: LiveMap): [number, number, number, number] | null {
  let [n0, e0, n1, e1] = [Infinity, Infinity, -Infinity, -Infinity];
  const tileM = map.tileCells * map.resM;
  for (const tile of Object.values(map.tiles)) {
    n0 = Math.min(n0, tile.ix * tileM);
    n1 = Math.max(n1, (tile.ix + 1) * tileM);
    e0 = Math.min(e0, tile.iy * tileM);
    e1 = Math.max(e1, (tile.iy + 1) * tileM);
  }
  for (let i = 0; i < map.points.length; i += 3) {
    n0 = Math.min(n0, (map.points[i] ?? 0));
    n1 = Math.max(n1, (map.points[i] ?? 0));
    e0 = Math.min(e0, (map.points[i + 1] ?? 0));
    e1 = Math.max(e1, (map.points[i + 1] ?? 0));
  }
  return Number.isFinite(n0) ? [n0, e0, n1, e1] : null;
}

export function LiveMapCanvas({ map }: { map: LiveMap }) {
  const ref = useRef<HTMLCanvasElement>(null);
  const box = bounds(map);

  useEffect(() => {
    const ctx = ref.current?.getContext("2d");
    if (!ctx || !box) return;
    const [n0, e0, n1, e1] = box;
    const scale = SIZE / Math.max(n1 - n0, e1 - e0, 1);
    // North up, east right.
    const px = (e: number) => (e - e0) * scale;
    const py = (n: number) => (n1 - n) * scale;
    ctx.fillStyle = "rgb(205,205,205)";
    ctx.fillRect(0, 0, SIZE, SIZE);
    const tileM = map.tileCells * map.resM;
    let cancelled = false;
    const drawPoints = () => {
      ctx.fillStyle = "rgba(56,189,248,0.8)";
      for (let i = 0; i < map.points.length; i += 3) {
        ctx.fillRect(px((map.points[i + 1] ?? 0)), py((map.points[i] ?? 0)), 1.5, 1.5);
      }
    };
    const tiles = Object.values(map.tiles);
    let pending = tiles.length;
    if (pending === 0) drawPoints();
    for (const tile of tiles) {
      const img = new Image();
      img.onload = img.onerror = () => {
        if (cancelled) return;
        if (img.complete && img.naturalWidth > 0) {
          ctx.imageSmoothingEnabled = false;
          ctx.drawImage(
            img,
            px(tile.iy * tileM),
            py((tile.ix + 1) * tileM),
            tileM * scale,
            tileM * scale,
          );
        }
        pending -= 1;
        if (pending === 0) drawPoints();
      };
      img.src = `data:image/png;base64,${tile.pngB64}`;
    }
    return () => {
      cancelled = true;
    };
  }, [map, box]);

  if (!box) return null;
  return (
    <figure className="we:flex we:flex-col we:gap-1">
      <canvas
        ref={ref}
        width={SIZE}
        height={SIZE}
        className="we:w-full we:max-w-[512px] we:aspect-square we:rounded we:bg-black"
        aria-label={t("liveMapHeading")}
      />
      <figcaption className="we:text-xs we:opacity-70">
        {t("liveMapCaption", {
          tiles: Object.keys(map.tiles).length,
          points: map.points.length / 3,
          voxel: Math.round(map.voxelM * 100),
        })}
      </figcaption>
    </figure>
  );
}
