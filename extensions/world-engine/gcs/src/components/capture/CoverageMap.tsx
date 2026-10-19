/**
 * @module capture/CoverageMap
 * @description The capture's top-down coverage: covered voxel columns shaded
 * by hit count, the walked trajectory, the start point and the operator's
 * current position and heading, north up.
 */

import { useMemo } from "react";

import type { CaptureCoverage } from "../../lib/capture/capture-state";
import { planTransform } from "../../lib/world/floorplan";

const W = 320;
const H = 220;

export function CoverageMap({ coverage, emptyLabel }: { coverage: CaptureCoverage | null; emptyLabel: string }) {
  const view = useMemo(() => {
    if (!coverage || (coverage.cells.length === 0 && coverage.trajectory.length === 0)) return null;
    const c = coverage.cellM;
    const xs = [...coverage.cells.flatMap((k) => [k.ix * c, (k.ix + 1) * c]), ...coverage.trajectory.map((p) => p[0])];
    const ys = [...coverage.cells.flatMap((k) => [k.iy * c, (k.iy + 1) * c]), ...coverage.trajectory.map((p) => p[1])];
    const tf = planTransform(
      { minX: Math.min(...xs), minY: Math.min(...ys), maxX: Math.max(...xs), maxY: Math.max(...ys) },
      W,
      H,
      10,
    );
    const maxHits = Math.max(1, ...coverage.cells.map((k) => k.hits));
    return { tf, maxHits };
  }, [coverage]);

  if (!coverage || !view) {
    return (
      <div className="we:flex we:items-center we:justify-center we:rounded we:bg-bg-tertiary we:text-[11px] we:text-text-tertiary" style={{ width: W, height: H }}>
        {emptyLabel}
      </div>
    );
  }
  const { tf, maxHits } = view;
  const c = coverage.cellM;
  const side = Math.max(1, c * tf.scale);
  const path = coverage.trajectory.map((p) => tf.toPx(p[0], p[1]).join(",")).join(" ");
  const start = coverage.start ? tf.toPx(coverage.start[0], coverage.start[1]) : null;
  const cur = coverage.current ? tf.toPx(coverage.current[0], coverage.current[1]) : null;

  return (
    <svg width={W} height={H} className="we:rounded we:bg-bg-tertiary we:text-accent-primary" role="img">
      {coverage.cells.map((k) => {
        const [x, y] = tf.toPx(k.ix * c, (k.iy + 1) * c);
        return (
          <rect
            key={`${k.ix}:${k.iy}`}
            x={x}
            y={y}
            width={side}
            height={side}
            fill="currentColor"
            fillOpacity={0.2 + 0.6 * Math.min(1, k.hits / maxHits)}
          />
        );
      })}
      {path && <polyline points={path} fill="none" stroke="#facc15" strokeWidth={1.5} />}
      {start && <circle cx={start[0]} cy={start[1]} r={4} fill="none" stroke="#22c55e" strokeWidth={2} />}
      {cur && (
        <g transform={`translate(${cur[0]} ${cur[1]}) rotate(${coverage.headingDeg ?? 0})`}>
          <path d="M0,-7 L5,5 L0,2 L-5,5 Z" fill="#f8fafc" />
        </g>
      )}
    </svg>
  );
}
