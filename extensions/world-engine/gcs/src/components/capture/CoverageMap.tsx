/**
 * @module capture/CoverageMap
 * @description The capture's top-down coverage: covered voxel columns shaded
 * by hit count, the walked trajectory, the start point and the operator's
 * current position and heading, north up. Drawn to one canvas, once per
 * coverage poll, so a warehouse-sized grid (10,000 m² at 0.25 m is 160k
 * cells) costs one pass and no DOM nodes.
 */

import { useEffect, useMemo, useRef } from "react";

import type { CaptureCoverage } from "../../lib/capture/capture-state";
import { planTransform, type Bounds } from "../../lib/world/floorplan";

const W = 320;
const H = 220;

/** The extent of the covered cells and the trajectory, in ENU metres, found
 * in one pass (no argument spreading, safe for any cell count). Null when
 * there is nothing to draw. */
export function coverageBounds(c: CaptureCoverage): Bounds | null {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const k of c.cells) {
    if (k.ix * c.cellM < minX) minX = k.ix * c.cellM;
    if (k.iy * c.cellM < minY) minY = k.iy * c.cellM;
    if ((k.ix + 1) * c.cellM > maxX) maxX = (k.ix + 1) * c.cellM;
    if ((k.iy + 1) * c.cellM > maxY) maxY = (k.iy + 1) * c.cellM;
  }
  for (const [x, y] of c.trajectory) {
    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
  }
  return Number.isFinite(minX) ? { minX, minY, maxX, maxY } : null;
}

function draw(ctx: CanvasRenderingContext2D, c: CaptureCoverage, b: Bounds, accent: string): void {
  const tf = planTransform(b, W, H, 10);
  const side = Math.max(1, c.cellM * tf.scale);
  let maxHits = 1;
  for (const k of c.cells) if (k.hits > maxHits) maxHits = k.hits;
  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = accent;
  for (const k of c.cells) {
    const [x, y] = tf.toPx(k.ix * c.cellM, (k.iy + 1) * c.cellM);
    ctx.globalAlpha = 0.2 + 0.6 * Math.min(1, k.hits / maxHits);
    ctx.fillRect(x, y, side, side);
  }
  ctx.globalAlpha = 1;
  if (c.trajectory.length > 1) {
    ctx.strokeStyle = "#facc15";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    c.trajectory.forEach(([x, y], i) => {
      const [px, py] = tf.toPx(x, y);
      if (i === 0) ctx.moveTo(px, py);
      else ctx.lineTo(px, py);
    });
    ctx.stroke();
  }
  if (c.start) {
    const [sx, sy] = tf.toPx(c.start[0], c.start[1]);
    ctx.strokeStyle = "#22c55e";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(sx, sy, 4, 0, Math.PI * 2);
    ctx.stroke();
  }
  if (c.current) {
    const [cx, cy] = tf.toPx(c.current[0], c.current[1]);
    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(((c.headingDeg ?? 0) * Math.PI) / 180);
    ctx.fillStyle = "#f8fafc";
    ctx.beginPath();
    ctx.moveTo(0, -7);
    ctx.lineTo(5, 5);
    ctx.lineTo(0, 2);
    ctx.lineTo(-5, 5);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
}

export function CoverageMap({ coverage, emptyLabel, label }: { coverage: CaptureCoverage | null; emptyLabel: string; label: string }) {
  const ref = useRef<HTMLCanvasElement>(null);
  const bounds = useMemo(() => (coverage ? coverageBounds(coverage) : null), [coverage]);

  useEffect(() => {
    const canvas = ref.current;
    const ctx = canvas?.getContext("2d");
    if (!canvas || !ctx || !coverage || !bounds) return;
    draw(ctx, coverage, bounds, getComputedStyle(canvas).color || "#38bdf8");
  }, [coverage, bounds]);

  if (!coverage || !bounds) {
    return (
      <div className="we:flex we:items-center we:justify-center we:rounded we:bg-bg-tertiary we:text-[11px] we:text-text-tertiary" style={{ width: W, height: H }}>
        {emptyLabel}
      </div>
    );
  }
  return (
    <canvas ref={ref} width={W} height={H} role="img" aria-label={label} className="we:rounded we:bg-bg-tertiary we:text-accent-primary" />
  );
}
