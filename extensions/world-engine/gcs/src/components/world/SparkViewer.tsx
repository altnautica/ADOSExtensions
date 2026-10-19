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
import { ViewerError } from "../atlas/viewers/ViewerError";
import { ViewerLoading } from "../atlas/viewers/ViewerLoading";
import { FloorplanSvg } from "./FloorplanSvg";
import type { ViewMode } from "./spark-stage";
import { TouchJoystick } from "./TouchJoystick";
import { useSparkScene, type WorldScene } from "./use-spark-scene";
import { EYE_HEIGHT_M } from "./walk-controls";

const t = translator("world");

const MODES: readonly ViewMode[] = ["orbit", "dollhouse", "walk"];

export interface SparkViewerProps {
  scene: WorldScene;
  /** On-surface distance between two ENU points from the node, or null. */
  measureRemote?: (a: Vec3, b: Vec3) => Promise<number | null>;
  className?: string;
}

export function SparkViewer({ scene, measureRemote, className }: SparkViewerProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const hotspotRefs = useRef<(HTMLButtonElement | null)[]>([]);
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

  useEffect(() => stage?.setFloor(showPlan ? floor : null), [stage, floor, showPlan]);
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
      const now = performance.now();
      if (now - lastHere > 200) {
        lastHere = now;
        const [e, n] = viewerToEnu(stage.camera.position.toArray());
        setHere({ x: e, y: n, heading: stage.camera.rotation.y });
      }
    });
  }, [stage, rooms]);

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
    <div className={cn("we:relative we:w-full we:h-full we:min-h-[320px] we:overflow-hidden", className)}>
      <canvas
        ref={canvasRef}
        className={cn("we:absolute we:inset-0 we:w-full we:h-full", measuring && "we:cursor-crosshair")}
        onClick={(e) => void onCanvasClick(e)}
      />
      {rooms.map((r, i) => (
        <button
          key={`${r.name}-${i}`}
          ref={(el) => {
            hotspotRefs.current[i] = el;
          }}
          type="button"
          onClick={() => r.hotspot && jumpTo(r.hotspot)}
          className="we:absolute we:left-0 we:top-0 we:px-2 we:py-0.5 we:rounded-full we:text-[11px] we:bg-scrim/60 we:text-on-media we:border we:border-on-media/40"
        >
          {r.name}
        </button>
      ))}
      <div className="we:absolute we:top-2 we:left-2 we:flex we:flex-wrap we:gap-1" role="toolbar" aria-label={t("toolbar")}>
        {MODES.map((m) => (
          <button
            key={m}
            type="button"
            aria-pressed={mode === m}
            onClick={() => setMode(m)}
            className={cn(
              "we:text-[11px] we:px-2 we:py-1 we:rounded we:bg-scrim/60 we:text-on-media",
              mode === m && "we:ring-1 we:ring-on-media",
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
            className="we:text-[11px] we:px-2 we:py-1 we:rounded we:bg-scrim/60 we:text-on-media"
          >
            {t("plan")}
          </button>
        )}
        {floors.length > 1 && (
          <select
            aria-label={t("floor")}
            value={floorIdx}
            onChange={(e) => setFloorIdx(Number(e.target.value))}
            className="we:text-[11px] we:px-1 we:rounded we:bg-scrim/60 we:text-on-media"
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
          className="we:text-[11px] we:px-2 we:py-1 we:rounded we:bg-scrim/60 we:text-on-media"
        >
          {t("measure")}
        </button>
      </div>
      {measuring && (
        <div className="we:absolute we:top-10 we:left-2 we:text-[11px] we:px-2 we:py-1 we:rounded we:bg-scrim/70 we:text-on-media">
          {points.length < 2 ? t("measureHint") : formatLength(distance3(points[0]!, points[1]!))}
          {remote !== null && <span className="we:ml-2 we:opacity-70">{t("onSurface", { d: formatLength(remote) })}</span>}
        </div>
      )}
      {floor && showPlan && (
        <div className="we:absolute we:top-2 we:right-2 we:rounded we:bg-scrim/60 we:text-on-media we:p-1">
          <FloorplanSvg floor={floor} width={180} height={140} here={here} onRoom={(r) => r.hotspot && jumpTo(r.hotspot)} />
        </div>
      )}
      {mode === "walk" && <TouchJoystick label={t("joystick")} onChange={(f, s) => stage?.setStick(f, s)} />}
      {load.phase === "loading" && <ViewerLoading percent={load.percent ?? undefined} label={t("loading")} />}
      {load.phase === "failed" && <ViewerError what={t("splat")} />}
    </div>
  );
}
