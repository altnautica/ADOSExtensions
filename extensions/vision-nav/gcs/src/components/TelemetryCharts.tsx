import { useEffect, useRef, useState, type CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { tr, type TFn } from "../i18n";
import type { VisionNavTelemetry } from "../types";

interface Props {
  ctx: PluginContext;
  telemetry: VisionNavTelemetry;
}

// 60 samples at the 1 Hz heartbeat cadence covers a minute of history,
// which is the right window for "is this estimator stable right now"
// without being so large the sparkline goes unreadable.
const BUFFER_LIMIT = 60;

interface Series {
  id: string;
  label: string;
  values: number[];
  format: (v: number) => string;
  colorVar: string;
  yMin: number;
  yMax: number;
}

/**
 * Inline-SVG sparklines for the live estimator telemetry. Each series
 * keeps its own rolling buffer; the plugin iframe is sandboxed and reads
 * only through ``ctx.telemetry``, so the host's ring buffer is out of
 * reach.
 *
 * Every received navigation block adds one sample, whether or not the
 * value moved, so a steady value still draws a line and the window is
 * a minute of heartbeats. A series with one sample shows it as the
 * current value.
 */
export function TelemetryCharts({ ctx, telemetry }: Props): JSX.Element {
  const t = ctx.i18n.t;
  const flowQualityRef = useRef<number[]>([]);
  const syncOffsetRef = useRef<number[]>([]);
  const [, setTick] = useState(0);

  // Keyed on the snapshot object: the store builds a new one per
  // received block, and re-renders between blocks keep the same one.
  useEffect(() => {
    let changed = false;
    if (typeof telemetry.flowQuality === "number") {
      flowQualityRef.current = pushAndCap(flowQualityRef.current, telemetry.flowQuality);
      changed = true;
    }
    if (typeof telemetry.cameraImuSyncOffsetMs === "number") {
      syncOffsetRef.current = pushAndCap(
        syncOffsetRef.current,
        Math.abs(telemetry.cameraImuSyncOffsetMs),
      );
      changed = true;
    }
    if (changed) {
      setTick((n) => n + 1);
    }
  }, [telemetry]);

  const series: Series[] = [
    {
      id: "flow-quality",
      label: tr(t, "navigation.charts.flowQuality"),
      values: flowQualityRef.current,
      format: (v) => tr(t, "navigation.format.quality", { v: Math.round(v) }),
      colorVar: "var(--vn-accent, #3a82ff)",
      yMin: 0,
      yMax: 255,
    },
    {
      id: "sync-offset",
      label: tr(t, "navigation.charts.syncOffset"),
      values: syncOffsetRef.current,
      format: (v) => tr(t, "navigation.format.ms", { v: v.toFixed(2) }),
      colorVar: "var(--vn-warn, #f59e0b)",
      yMin: 0,
      yMax: 40,
    },
  ];

  return (
    <section style={card} data-testid="vn-telemetry-charts">
      <h3 style={heading}>{tr(t, "navigation.charts.title")}</h3>
      <div style={grid}>
        {series.map((s) => (
          <Sparkline key={s.id} series={s} t={t} />
        ))}
      </div>
    </section>
  );
}

const W = 180;
const H = 36;

function Sparkline({ series, t }: { series: Series; t: TFn }): JSX.Element {
  const { id, label, values, format, colorVar, yMin, yMax } = series;
  const last = values.length > 0 ? values[values.length - 1] : undefined;
  const span = Math.max(yMax - yMin, 1e-6);
  const y = (v: number): number => H - ((Math.min(Math.max(v, yMin), yMax) - yMin) / span) * H;

  let plot: JSX.Element;
  if (last === undefined) {
    plot = (
      <line
        x1={0}
        x2={W}
        y1={H / 2}
        y2={H / 2}
        stroke="var(--vn-border, rgba(255,255,255,0.08))"
        strokeDasharray="2 2"
      />
    );
  } else if (values.length === 1) {
    plot = <circle cx={W} cy={y(last)} r={2} fill={colorVar} />;
  } else {
    const points = values
      .map((v, i) => `${((i / (values.length - 1)) * W).toFixed(1)},${y(v).toFixed(1)}`)
      .join(" ");
    plot = <polyline points={points} fill="none" stroke={colorVar} strokeWidth={1.5} />;
  }

  return (
    <div style={cell} data-testid={`vn-sparkline-${id}`}>
      <div style={labelRow}>
        <span style={labelText}>{label}</span>
        <span style={valueText}>
          {last === undefined ? tr(t, "navigation.noData") : format(last)}
        </span>
      </div>
      <svg width={W} height={H} style={{ display: "block", overflow: "visible" }} aria-hidden>
        {plot}
      </svg>
    </div>
  );
}

function pushAndCap(buf: number[], value: number): number[] {
  const next = buf.length >= BUFFER_LIMIT ? buf.slice(1) : buf.slice();
  next.push(value);
  return next;
}

const card: CSSProperties = {
  background: "var(--vn-surface, rgba(255,255,255,0.02))",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.5rem",
  padding: "0.875rem 1rem",
  color: "var(--vn-text, #ededed)",
  display: "flex",
  flexDirection: "column",
  gap: "0.625rem",
};
const heading: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
  margin: 0,
  color: "var(--vn-text-2, #b4b4b4)",
};
const grid: CSSProperties = {
  display: "grid",
  gridTemplateColumns: "repeat(auto-fit, minmax(180px, 1fr))",
  gap: "0.75rem",
};
const cell: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.25rem",
};
const labelRow: CSSProperties = {
  display: "flex",
  alignItems: "center",
  justifyContent: "space-between",
};
const labelText: CSSProperties = {
  fontSize: "0.7rem",
  color: "var(--vn-text-muted, #94a3b8)",
};
const valueText: CSSProperties = {
  fontSize: "0.75rem",
  fontWeight: 600,
  fontVariantNumeric: "tabular-nums",
};
