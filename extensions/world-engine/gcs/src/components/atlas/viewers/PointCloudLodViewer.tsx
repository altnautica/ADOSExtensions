/**
 * @module atlas/viewers/PointCloudLodViewer
 * @description The cloud viewer that survives very large `.ply` reconstructions.
 * `PointCloudViewer` puts every vertex on the GPU as one THREE.Points, so a
 * multi-million-point cloud exhausts browser memory; this loads the cloud, then
 * decimates it to a point budget (a single-level voxel-grid pass, `decimateCloud`)
 * so the retained buffer never exceeds the budget no matter how dense the
 * source. Parsing and decimation run in a Web Worker (`ply-decimate-worker`)
 * with the file buffer transferred in and the kept points transferred back, so
 * a huge cloud never stalls the main thread. The decimation keeps a spatially
 * uniform subset (one representative per occupied cell) rather than a clumped
 * slice, and a badge states "showing X of Y" so the operator knows the view is
 * decimated. The worker is terminated, and the renderer, orbit controls,
 * geometry and material are disposed, on unmount / source change.
 *
 * The worker's transient peak still includes the full parsed file (PLYLoader
 * has no streaming entry point); out-of-core octree streaming for clouds that
 * don't fit in memory at all is not implemented.
 */

import { useEffect, useRef, useState } from "react";
import type { BufferGeometry, Material, WebGLRenderer } from "three";
import type { OrbitControls } from "three/examples/jsm/controls/OrbitControls.js";
import { ViewerError } from "./ViewerError";
import { ViewerLoading } from "./ViewerLoading";
import { startDecimateWorker, type DecimateWorker } from "./decimate-in-worker";
import { orientCloudToYUp } from "./coordinate-frame";
import { followCanvasSize, frameCloud } from "./cloud-view";
import {
  fetchArrayBufferWithProgress,
  type FetchProgress,
} from "../../../lib/net/fetch-with-progress";
import type { ArtifactSource } from "../../../lib/net/artifact-source";

/** Retained-point cap. ~1.5M points = ~18 MB of float positions, GPU-comfortable. */
const LOD_POINT_BUDGET = 1_500_000;

interface CloudStats {
  total: number;
  kept: number;
  decimated: boolean;
}

export function PointCloudLodViewer({ source }: { source: ArtifactSource }) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [failed, setFailed] = useState(false);
  const [loading, setLoading] = useState(true);
  const [stats, setStats] = useState<CloudStats | null>(null);
  const [progress, setProgress] = useState<FetchProgress | null>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    setFailed(false);
    setLoading(true);
    setStats(null);
    setProgress(null);
    const abort = new AbortController();
    let worker: DecimateWorker | null = null;
    let raf = 0;
    let disposed = false;
    // Hoisted so the cleanup can release the WebGL context + GPU buffers.
    let renderer: WebGLRenderer | null = null;
    let controls: OrbitControls | null = null;
    let geometry: BufferGeometry | null = null;
    let material: Material | null = null;
    let stopResize: (() => void) | null = null;

    void (async () => {
      try {
        const THREE = await import("three");
        const { OrbitControls: Orbit } = await import(
          "three/examples/jsm/controls/OrbitControls.js"
        );
        if (disposed || !canvasRef.current) return;

        // Load the full cloud, then parse + decimate it off the main thread.
        const buffer = await fetchArrayBufferWithProgress(source, {
          signal: abort.signal,
          onProgress: (p) => {
            if (!disposed) setProgress(p);
          },
        });
        if (disposed) return;
        worker = startDecimateWorker();
        const dec = await worker.run(buffer, LOD_POINT_BUDGET);
        worker.stop();
        worker = null;
        if (disposed) return;
        if (dec.kept === 0) throw new Error("empty cloud");

        const geom = new THREE.BufferGeometry();
        geom.setAttribute(
          "position",
          new THREE.BufferAttribute(dec.positions, 3),
        );
        if (dec.colors) {
          geom.setAttribute("color", new THREE.BufferAttribute(dec.colors, 3));
        }
        // COLMAP Y-down world frame → viewer Y-up (before the framing sphere).
        orientCloudToYUp(geom);
        geom.computeBoundingSphere();
        geometry = geom;
        setStats({ total: dec.total, kept: dec.kept, decimated: dec.decimated });

        const width = canvas.clientWidth || 640;
        const height = canvas.clientHeight || 360;
        const r = new THREE.WebGLRenderer({ canvas, antialias: false });
        r.setSize(width, height, false);
        renderer = r;

        const scene = new THREE.Scene();
        scene.background = new THREE.Color(0x0a0a0a);
        const camera = new THREE.PerspectiveCamera(60, width / height, 0.01, 5000);
        stopResize = followCanvasSize(canvas, r, camera);
        const ctrl = new Orbit(camera, canvas);
        controls = ctrl;

        const hasColor = !!dec.colors;
        const mat = new THREE.PointsMaterial({
          size: 0.012,
          sizeAttenuation: true,
          vertexColors: hasColor,
          color: hasColor ? 0xffffff : 0x88ccff,
        });
        material = mat;
        scene.add(new THREE.Points(geom, mat));

        // Frame the cloud from its bounding sphere so it fills the view, with
        // clip planes fitted to its size.
        const bs = geom.boundingSphere;
        if (bs) {
          const fit = frameCloud(bs.radius);
          camera.near = fit.near;
          camera.far = fit.far;
          camera.updateProjectionMatrix();
          ctrl.maxDistance = fit.maxDistance;
          camera.position.set(bs.center.x, bs.center.y, bs.center.z + fit.distance);
          ctrl.target.copy(bs.center);
        }
        ctrl.update();
        setLoading(false);

        const frame = () => {
          ctrl.update();
          r.render(scene, camera);
          raf = requestAnimationFrame(frame);
        };
        raf = requestAnimationFrame(frame);
      } catch {
        if (!disposed) {
          setLoading(false);
          setFailed(true);
        }
      }
    })();

    return () => {
      disposed = true;
      abort.abort();
      worker?.stop();
      cancelAnimationFrame(raf);
      stopResize?.();
      controls?.dispose();
      geometry?.dispose();
      material?.dispose();
      renderer?.dispose();
    };
    // Keyed by the source's identity, not the object: a re-render that hands
    // over an equal source must not restart the download.
  }, [source.key]);

  return (
    <div className="we:relative we:w-full we:h-full we:min-h-[320px]">
      <canvas ref={canvasRef} className="we:w-full we:h-full" />
      {loading && !failed && (
        <ViewerLoading
          percent={progress?.percent ?? undefined}
          receivedBytes={progress?.receivedBytes}
          totalBytes={progress?.totalBytes ?? undefined}
          label="Downloading cloud"
        />
      )}
      {failed && <ViewerError what="point cloud" />}
      {stats?.decimated && !failed && (
        <div className="we:absolute we:bottom-2 we:left-2 we:rounded we:bg-bg-primary/70 we:px-2 we:py-1 we:text-[10px] we:font-mono we:text-text-tertiary we:tabular-nums">
          {`showing ${stats.kept.toLocaleString()} / ${stats.total.toLocaleString()} points`}
        </div>
      )}
    </div>
  );
}
