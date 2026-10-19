/**
 * @module world/use-spark-scene
 * @description Owns one {@link SparkStage} on a canvas and loads a
 * {@link WorldScene} into it: the splat (bytes, or a `.rad` LOD root streamed
 * through a sentinel tree route so chunk requests carry the host's auth), the
 * collision mesh and the floorplan floor. Reports load progress and failure.
 */

import { useEffect, useState, type RefObject } from "react";

import type { Placement } from "../../lib/contract/generated";
import type { ArtifactSource } from "../../lib/net/artifact-source";
import { fetchArrayBufferWithProgress } from "../../lib/net/fetch-with-progress";
import { honourRange, serveTreeAtSentinel, type SentinelRoute } from "../../lib/net/sentinel-fetch";
import type { Floorplan } from "../../lib/world/floorplan";
import type { Vec3 } from "../../lib/world/placement";
import { SparkStage } from "./spark-stage";

/** Everything the world viewer shows for one reconstruction. */
export interface WorldScene {
  /** The splat: `.spz` / `.ply` read whole, `.rad` streamed. */
  splat: ArtifactSource | null;
  /** The splat's placement in local ENU (null = already local ENU). */
  placement: Placement | null;
  /** Collision mesh (GLB, local ENU) for picking and measuring. */
  collision: ArtifactSource | null;
  floorplan: Floorplan | null;
  rooms: { name: string; floor: number; hotspot: Vec3 | null }[];
}

export type SceneLoad =
  | { phase: "loading"; percent: number | null }
  | { phase: "ready" }
  | { phase: "failed" };

function isLod(name: string): boolean {
  return name.toLowerCase().endsWith(".rad");
}

/** Serve a `.rad` root and its chunk siblings to the renderer, honouring
 * `Range` requests even when the store answers a plain 200. */
function serveLod(source: ArtifactSource): SentinelRoute {
  return serveTreeAtSentinel(source.name, async ({ rel, headers, signal }) => {
    const target = rel === source.name ? source : source.sibling?.(rel);
    if (!target) return new Response(null, { status: 404 });
    const range = headers.get("Range");
    const res = await target.fetch(signal ?? new AbortController().signal, range ? { headers: { Range: range } } : {});
    return honourRange(res, range);
  });
}

export function useSparkScene(
  canvasRef: RefObject<HTMLCanvasElement | null>,
  scene: WorldScene,
): { stage: SparkStage | null; load: SceneLoad } {
  const [stage, setStage] = useState<SparkStage | null>(null);
  const [load, setLoad] = useState<SceneLoad>({ phase: "loading", percent: null });

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    let live: SparkStage | null = null;
    let disposed = false;
    SparkStage.create(canvas).then(
      (s) => {
        if (disposed) s.dispose();
        else {
          live = s;
          setStage(s);
        }
      },
      () => !disposed && setLoad({ phase: "failed" }),
    );
    return () => {
      disposed = true;
      live?.dispose();
      setStage(null);
    };
  }, [canvasRef]);

  const splat = scene.splat;
  useEffect(() => {
    if (!stage || !splat) return;
    const abort = new AbortController();
    let route: SentinelRoute | null = null;
    setLoad({ phase: "loading", percent: null });
    void (async () => {
      try {
        if (isLod(splat.name)) {
          route = serveLod(splat);
          await stage.loadSplat({ url: route.url, fileName: splat.name }, scene.placement);
        } else {
          const buf = await fetchArrayBufferWithProgress(splat, {
            signal: abort.signal,
            onProgress: (p) => setLoad({ phase: "loading", percent: p.percent }),
          });
          if (abort.signal.aborted) return;
          await stage.loadSplat({ bytes: new Uint8Array(buf), fileName: splat.name }, scene.placement);
        }
        if (!abort.signal.aborted) setLoad({ phase: "ready" });
      } catch {
        if (!abort.signal.aborted) setLoad({ phase: "failed" });
      }
    })();
    return () => {
      abort.abort();
      route?.release();
    };
    // The placement belongs to the splat; both change together with its key.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [stage, splat?.key]);

  const collision = scene.collision;
  useEffect(() => {
    if (!stage || !collision) return;
    const abort = new AbortController();
    fetchArrayBufferWithProgress(collision, { signal: abort.signal })
      .then((buf) => (abort.signal.aborted ? undefined : stage.loadCollision(buf)))
      // A missing collision mesh only disables surface picking.
      .catch(() => undefined);
    return () => abort.abort();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [stage, collision?.key]);

  return { stage, load };
}
