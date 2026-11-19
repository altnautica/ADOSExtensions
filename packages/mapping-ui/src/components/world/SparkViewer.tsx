/**
 * @module world/SparkViewer
 * @description The world viewer: a Gaussian splat rendered by Spark with
 * orbit, dollhouse and first-person walk modes (WASD, or the touch stick),
 * the floorplan drawn on the floor plus a north-up mini plan, room hotspots
 * that jump the walker into a room, and a two-click measure tool (picked on
 * the collision mesh, optionally refined by the node's on-surface measure).
 *
 * Self-contained: it reads no host API, so the plugin pages and the static
 * tour viewer render the same component.
 */

import { useEffect, useMemo, useRef, useState, type MouseEvent } from "react";

import { translator } from "../../host/i18n";
import { cn } from "../../lib/utils";
import { distance3, formatLength } from "../../lib/world/measure";
import { enuToViewer, viewerToEnu, type Vec3 } from "../../lib/world/placement";
import { ViewerError } from "../model/viewers/ViewerError";
import { ViewerLoading } from "../model/viewers/ViewerLoading";
import { FloorplanSvg } from "./FloorplanSvg";
import type { ViewMode } from "./spark-stage";
import { TouchJoystick } from "./TouchJoystick";
import { useSparkScene, type WorldScene } from "./use-spark-scene";
import { EYE_HEIGHT_M } from "./walk-controls";

const t = translator("world");

const MODES: readonly ViewMode[] = ["orbit", "dollhouse", "walk"];

const FOCUS = "map:focus-visible:outline-2 map:focus-visible:outline-on-media";
/** Toolbar controls grow to a touch-sized target under a coarse pointer. */
const CONTROL = `${FOCUS} map:pointer-coarse:min-h-12 map:pointer-coarse:min-w-12`;
/** Hotspots keep their drawn size (they sit on a projected point) and get an
 * invisible touch-sized hit area around it under a coarse pointer. */
const HIT_AREA =
  "map:pointer-coarse:before:absolute map:pointer-coarse:before:-inset-[18px] map:pointer-coarse:before:content-['']";

export interface SparkViewerProps {
  scene: WorldScene;
  /** On-surface distance between two ENU points from the node, or null. */
  measureRemote?: (a: Vec3, b: Vec3) => Promise<number | null>;
  className?: string;
}

export function SparkViewer({ scene, measureRemote, className }: SparkViewerProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const hotspotRefs = useRef<(HTMLButtonElement | null)[]>([]);
  const nodeRefs = useRef<(HTMLButtonElement | null)[]>([]);
  const { stage, load } = useSparkScene(canvasRef, scene);
  const [mode, setMode] = useState<ViewMode>("orbit");
  const floors = scene.floorplan?.floors ?? [];
  const [floorIdx, setFloorIdx] = useState(0);
  const floor = floors[floorIdx] ?? null;
  const [showPlan, setShowPlan] = useState(true);
  const [measuring, setMeasuring] = useState(false);
  const [points, setPoints] = useState<Vec3[]>([]);
  const [remote, setRemote] = useState<number | null>(null);
  const [here, setHere] = useState<{ x: number; y: number; heading: number } | null>(null);
  const rooms = useMemo(
    () => scene.rooms.filter((r) => r.hotspot && (!floor || r.floor === floor.floor)),
    [scene.rooms, floor],
  );
  // Navigation nodes on the shown floor, each with the floor point its disc
  // sits on (the floor plane, else eye height below the node).
  const nodes = useMemo(
    () =>
      (scene.nodes ?? []).flatMap((n) => {
        const ground = floor ? floor.zM : n.position[2] - EYE_HEIGHT_M;
        if (floor && Math.abs(n.position[2] - EYE_HEIGHT_M - floor.zM) > 1.2) return [];
        return [{ ...n, disc: enuToViewer([n.position[0], n.position[1], ground]) }];
      }),
    [scene.nodes, floor],
  );

  useEffect(() => {
    stage?.setFloor(showPlan ? floor : null);
    stage?.setAisles(showPlan ? (scene.aisles ?? []) : []);
  }, [stage, floor, showPlan, scene.aisles]);
  useEffect(() => stage?.setMode(mode), [stage, mode]);
  useEffect(() => stage?.setMeasure(points), [stage, points]);

  // Hotspot labels and the mini-plan marker follow the camera every frame;
  // labels move by direct style writes, the marker at a few Hz.
  useEffect(() => {
    if (!stage) return;
    let lastHere = 0;
    return stage.onFrame(() => {
      rooms.forEach((r, i) => {
        const el = hotspotRefs.current[i];
        if (!el || !r.hotspot) return;
        const p = stage.project(enuToViewer(r.hotspot));
        el.style.display = p.visible ? "" : "none";
        el.style.transform = `translate(${p.x}px, ${p.y}px) translate(-50%, -100%)`;
      });
      nodes.forEach((n, i) => {
        const el = nodeRefs.current[i];
        if (!el) return;
        const p = stage.project(n.disc);
        el.style.display = p.visible ? "" : "none";
        el.style.transform = `translate(${p.x}px, ${p.y}px) translate(-50%, -50%)`;
      });
      const now = performance.now();
      if (now - lastHere > 200) {
        lastHere = now;
        const [e, n] = viewerToEnu(stage.camera.position.toArray());
        setHere({ x: e, y: n, heading: stage.camera.rotation.y });
      }
    });
  }, [stage, rooms, nodes]);

  const jumpTo = (hotspot: Vec3) => {
    if (!stage) return;
    const [x, , z] = enuToViewer(hotspot);
    setMode("walk");
    stage.setMode("walk");
    stage.camera.position.set(x, (floor?.zM ?? 0) + EYE_HEIGHT_M, z);
  };

  const onCanvasClick = async (e: MouseEvent<HTMLCanvasElement>) => {
    if (!measuring || !stage) return;
    const p = stage.pick(e.clientX, e.clientY);
    if (!p) return;
    const next = points.length >= 2 ? [p] : [...points, p];
    setPoints(next);
    setRemote(null);
    if (next.length === 2 && measureRemote) {
      setRemote(await measureRemote(viewerToEnu(next[0]!), viewerToEnu(next[1]!)));
    }
  };

  if (!scene.splat) return <ViewerError what={t("splat")} />;

  return (
    <div className={cn("map:relative map:w-full map:h-full map:min-h-[320px] map:overflow-hidden", className)}>
      <canvas
        ref={canvasRef}
        className={cn("map:absolute map:inset-0 map:w-full map:h-full", measuring && "map:cursor-crosshair")}
        onClick={(e) => void onCanvasClick(e)}
      />
      {mode !== "dollhouse" &&
        nodes.map((n, i) => (
          <button
            key={n.id}
            ref={(el) => {
              nodeRefs.current[i] = el;
            }}
            type="button"
            aria-label={t("navNode", { id: n.id })}
            title={t("navNode", { id: n.id })}
            onClick={() => jumpTo(n.position)}
            className={cn(
              "map:absolute map:left-0 map:top-0 map:h-3 map:w-5 map:rounded-[50%] map:bg-on-media/40 map:border map:border-on-media/70 map:hover:bg-on-media/70",
              FOCUS,
              HIT_AREA,
            )}
          />
        ))}
      {rooms.map((r, i) => (
        <button
          key={`${r.name}-${i}`}
          ref={(el) => {
            hotspotRefs.current[i] = el;
          }}
          type="button"
          onClick={() => r.hotspot && jumpTo(r.hotspot)}
          className={cn(
            "map:absolute map:left-0 map:top-0 map:px-2 map:py-0.5 map:rounded-full map:text-[11px] map:bg-scrim/60 map:text-on-media map:border map:border-on-media/40",
            FOCUS,
            HIT_AREA,
          )}
        >
          {r.name}
        </button>
      ))}
      <div className="map:absolute map:top-2 map:left-2 map:flex map:flex-wrap map:gap-1" role="toolbar" aria-label={t("toolbar")}>
        {MODES.map((m) => (
          <button
            key={m}
            type="button"
            aria-pressed={mode === m}
            onClick={() => setMode(m)}
            className={cn(
              "map:text-[11px] map:px-2 map:py-1 map:rounded map:bg-scrim/60 map:text-on-media",
              CONTROL,
              mode === m && "map:ring-1 map:ring-on-media",
            )}
          >
            {t(`mode.${m}`)}
          </button>
        ))}
        {floors.length > 0 && (
          <button
            type="button"
            aria-pressed={showPlan}
            onClick={() => setShowPlan((v) => !v)}
            className={cn("map:text-[11px] map:px-2 map:py-1 map:rounded map:bg-scrim/60 map:text-on-media", CONTROL)}
          >
            {t("plan")}
          </button>
        )}
        {floors.length > 1 && (
          <select
            aria-label={t("floor")}
            value={floorIdx}
            onChange={(e) => setFloorIdx(Number(e.target.value))}
            className={cn("map:text-[11px] map:px-1 map:rounded map:bg-scrim/60 map:text-on-media", CONTROL)}
          >
            {floors.map((f, i) => (
              <option key={f.floor} value={i}>
                {t("floorN", { n: f.floor })}
              </option>
            ))}
          </select>
        )}
        <button
          type="button"
          aria-pressed={measuring}
          onClick={() => {
            setMeasuring((v) => !v);
            setPoints([]);
            setRemote(null);
          }}
          className={cn("map:text-[11px] map:px-2 map:py-1 map:rounded map:bg-scrim/60 map:text-on-media", CONTROL)}
        >
          {t("measure")}
        </button>
      </div>
      {measuring && (
        <div className="map:absolute map:top-10 map:left-2 map:text-[11px] map:px-2 map:py-1 map:rounded map:bg-scrim/70 map:text-on-media">
          {points.length < 2 ? t("measureHint") : formatLength(distance3(points[0]!, points[1]!))}
          {remote !== null && <span className="map:ml-2 map:opacity-70">{t("onSurface", { d: formatLength(remote) })}</span>}
        </div>
      )}
      {floor && showPlan && (
        <div className="map:absolute map:top-2 map:right-2 map:rounded map:bg-scrim/60 map:text-on-media map:p-1">
          <FloorplanSvg floor={floor} width={180} height={140} here={here} onRoom={(r) => r.hotspot && jumpTo(r.hotspot)} />
        </div>
      )}
      {mode === "walk" && <TouchJoystick label={t("joystick")} onChange={(f, s) => stage?.setStick(f, s)} />}
      {load.phase === "loading" && <ViewerLoading percent={load.percent ?? undefined} label={t("loading")} />}
      {load.phase === "failed" && <ViewerError what={t("splat")} />}
      {load.phase === "empty" && <ViewerError message={t("empty")} />}
    </div>
  );
}
