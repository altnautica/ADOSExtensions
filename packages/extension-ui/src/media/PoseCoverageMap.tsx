import type { CSSProperties } from "react";

import { TOKENS } from "../theme/tokens";

/**
 * One captured pose. ``tiltDeg`` is the angle off-axis (0 = straight
 * at the target, 60 = oblique). ``rotationDeg`` is the rotation
 * around the optical axis (0 = upright, 90 = sideways).
 */
export interface PoseSample {
  tiltDeg: number;
  rotationDeg: number;
}

interface Props {
  /** Captured poses so far. */
  samples: PoseSample[];
  /** Optional label rendered above the map. */
  label?: string;
  /** Grid resolution. Default 5 (5x5 buckets). */
  gridSize?: number;
}

/**
 * 2D coverage heatmap. The X-axis maps rotation 0-360deg into N
 * buckets; the Y-axis maps tilt 0-90deg into N buckets. Each bucket
 * lights up brighter as more poses land in it. The operator scans
 * the map and tops up under-represented zones before submitting.
 */
export function PoseCoverageMap({
  samples,
  label,
  gridSize = 5,
}: Props): JSX.Element {
  // Build the bucket counts.
  const buckets = new Array<number>(gridSize * gridSize).fill(0);
  let max = 0;
  for (const sample of samples) {
    const rIdx = clampBucket(sample.rotationDeg, 0, 360, gridSize);
    const tIdx = clampBucket(sample.tiltDeg, 0, 90, gridSize);
    const idx = tIdx * gridSize + rIdx;
    const next = (buckets[idx] ?? 0) + 1;
    buckets[idx] = next;
    if (next > max) max = next;
  }

  return (
    <div style={wrapper} data-testid="ext-ui-pose-coverage-map">
      {label !== undefined ? <span style={labelStyle}>{label}</span> : null}
      <div
        style={{
          ...grid,
          gridTemplateColumns: `repeat(${gridSize}, 1fr)`,
          gridTemplateRows: `repeat(${gridSize}, 1fr)`,
        }}
      >
        {buckets.map((count, idx) => {
          const intensity = max === 0 ? 0 : count / max;
          return (
            <div
              key={idx}
              style={cell(intensity)}
              data-testid={`ext-ui-pose-coverage-cell-${idx}`}
              title={`${count} pose${count === 1 ? "" : "s"}`}
            />
          );
        })}
      </div>
      <div style={legend}>
        <span style={axisLabel}>tilt 0</span>
        <span style={axisLabel}>rot 0–360</span>
        <span style={axisLabel}>tilt 90</span>
      </div>
    </div>
  );
}

function clampBucket(
  value: number,
  min: number,
  max: number,
  buckets: number,
): number {
  const span = max - min;
  if (span <= 0) return 0;
  const norm = (value - min) / span;
  const clamped = Math.max(0, Math.min(0.9999, norm));
  return Math.floor(clamped * buckets);
}

const wrapper: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.375rem",
};
const labelStyle: CSSProperties = {
  fontSize: "0.75rem",
  color: TOKENS.textMuted,
  fontWeight: 600,
};
const grid: CSSProperties = {
  display: "grid",
  gap: "2px",
  aspectRatio: "1 / 1",
  width: "100%",
  maxWidth: "180px",
};
const cell = (intensity: number): CSSProperties => ({
  background:
    intensity === 0
      ? TOKENS.surface2
      : `color-mix(in srgb, ${TOKENS.accent} ${Math.round(intensity * 100)}%, ${TOKENS.surface2})`,
  borderRadius: "2px",
  minHeight: "12px",
});
const legend: CSSProperties = {
  display: "flex",
  justifyContent: "space-between",
  fontSize: "0.65rem",
  color: TOKENS.textMuted,
};
const axisLabel: CSSProperties = {
  fontFamily: "system-ui, sans-serif",
};
