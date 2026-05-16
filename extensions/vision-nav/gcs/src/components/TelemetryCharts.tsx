import { useEffect, useRef, useState, type CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import type { VisionNavTelemetry } from "../types";

type TFn = PluginContext["i18n"]["t"];

function tr(t: TFn, key: string, fallback: string): string {
  const result = t(key);
  return result === key ? fallback : result;
}

interface Props {
  ctx: PluginContext;
  telemetry: VisionNavTelemetry;
}

// 60 samples at the 1 Hz heartbeat cadence covers a minute of history,
// which is the right window for "is this estimator stable right now"
// without being so large the sparkline goes unreadable.
const BUFFER_LIMIT = 60;

interface Series {
  label: string;
  unit: string;
  values: number[];
  colorVar: string;
  yMin?: number;
  yMax?: number;
}

/**
 * Inline-SVG sparklines for the live estimator telemetry. Each
 * sparkline maintains its own local rolling buffer; we do not depend
 * on the host's telemetry-store ring buffer here because the plugin
 * iframe is sandboxed and reads only through ``ctx.telemetry``.
 *
 * The component subscribes to the same telemetry stream the rest of
 * the cards use; on each new heartbeat we push the matching field
 * into each series' buffer and re-render. Empty series render as a
 * muted "no data" placeholder so the card doesn't collapse mid-flight
 * when a metric goes ``null`` for one tick.
 */
export function TelemetryCharts({ ctx, telemetry }: Props): JSX.Element {
  const t = ctx.i18n.t;
  const flowQualityRef = useRef<number[]>([]);
  const syncOffsetRef = useRef<number[]>([]);
  const featureCountRef = useRef<number[]>([]);
  const driftRef = useRef<number[]>([]);
  const [, setTick] = useState(0);

  useEffect(() => {
    let changed = false;
    if (typeof telemetry.flowQuality === "number") {
      flowQualityRef.current = pushAndCap(
        flowQualityRef.current,
        telemetry.flowQuality,
      );
      changed = true;
    }
    if (typeof telemetry.cameraImuSyncOffsetMs === "number") {
      syncOffsetRef.current = pushAndCap(
        syncOffsetRef.current,
        Math.abs(telemetry.cameraImuSyncOffsetMs),
      );
      changed = true;
    }
    if (typeof telemetry.estimatorFeatureCount === "number") {
      featureCountRef.current = pushAndCap(
        featureCountRef.current,
        telemetry.estimatorFeatureCount,
      );
      changed = true;
    }
    if (typeof telemetry.estimatorDriftEstimateM === "number") {
      driftRef.current = pushAndCap(
        driftRef.current,
        telemetry.estimatorDriftEstimateM,
      );
      changed = true;
    }
    if (changed) {
      setTick((n) => n + 1);
    }
  }, [
    telemetry.flowQuality,
    telemetry.cameraImuSyncOffsetMs,
    telemetry.estimatorFeatureCount,
    telemetry.estimatorDriftEstimateM,
  ]);

  const series: Series[] = [
    {
      label: tr(t, "navigation.charts.flowQuality", "Flow quality"),
      unit: "/255",
      values: flowQualityRef.current,
      colorVar: "var(--vn-accent, #2563eb)",
      yMin: 0,
      yMax: 255,
    },
    {
      label: tr(t, "navigation.charts.syncOffset", "Sync offset"),
      unit: "ms",
      values: syncOffsetRef.current,
      colorVar: "var(--vn-warn, #f59e0b)",
      yMin: 0,
      yMax: 40,
    },
    {
      label: tr(t, "navigation.charts.features", "Features"),
      unit: "",
      values: featureCountRef.current,
      colorVar: "var(--vn-ok, #34d399)",
      yMin: 0,
    },
    {
      label: tr(t, "navigation.charts.drift", "Drift"),
      unit: "m",
      values: driftRef.current,
      colorVar: "var(--vn-warn, #f59e0b)",
      yMin: 0,
    },
  ];

  return (
    <section style={card} data-testid="vn-telemetry-charts">
      <h3 style={heading}>
        {tr(t, "navigation.charts.title", "Telemetry trends")}
      </h3>
      <div style={grid}>
        {series.map((s) => (
          <Sparkline key={s.label} {...s} t={t} />
        ))}
      </div>
    </section>
  );
}

interface SparklineProps extends Series {
  t: TFn;
}

function Sparkline({
  label,
  unit,
  values,
  colorVar,
  yMin,
  yMax,
  t,
}: SparklineProps): JSX.Element {
  const w = 180;
  const h = 36;
  const last = values.length > 0 ? values[values.length - 1] : null;

  if (values.length < 2) {
    return (
      <div style={cell} data-testid={`vn-sparkline-${slug(label)}`}>
        <div style={labelRow}>
          <span style={labelText}>{label}</span>
          <span style={valueText}>
            {tr(t, "navigation.charts.noData", "no data")}
          </span>
        </div>
        <svg width={w} height={h} style={{ display: "block" }} aria-hidden>
          <line
            x1={0}
            x2={w}
            y1={h / 2}
            y2={h / 2}
            stroke="var(--vn-border, rgba(255,255,255,0.08))"
            strokeDasharray="2 2"
          />
        </svg>
      </div>
    );
  }

  const min = yMin ?? Math.min(...values);
  const max = yMax ?? Math.max(...values);
  const span = Math.max(max - min, 1e-6);
  const points = values
    .map((v, i) => {
      const x = (i / Math.max(values.length - 1, 1)) * w;
      const y = h - ((v - min) / span) * h;
      return `${x.toFixed(1)},${y.toFixed(1)}`;
    })
    .join(" ");

  return (
    <div style={cell} data-testid={`vn-sparkline-${slug(label)}`}>
      <div style={labelRow}>
        <span style={labelText}>{label}</span>
        <span style={valueText}>
          {typeof last === "number"
            ? `${formatValue(last, unit)}${unit ? " " + unit : ""}`
            : "—"}
        </span>
      </div>
      <svg width={w} height={h} style={{ display: "block" }} aria-hidden>
        <polyline
          points={points}
          fill="none"
          stroke={colorVar}
          strokeWidth={1.5}
        />
      </svg>
    </div>
  );
}

function pushAndCap(buf: number[], value: number): number[] {
  const next = buf.length >= BUFFER_LIMIT ? buf.slice(1) : buf.slice();
  next.push(value);
  return next;
}

function formatValue(v: number, unit: string): string {
  if (unit === "ms" || unit === "m") {
    return v.toFixed(2);
  }
  return Number.isInteger(v) ? v.toString() : v.toFixed(2);
}

function slug(label: string): string {
  return label.toLowerCase().replace(/[^a-z0-9]+/g, "-");
}

const card: CSSProperties = {
  background: "var(--vn-surface, rgba(255,255,255,0.02))",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.5rem",
  padding: "0.875rem 1rem",
  color: "var(--vn-text, #e5e7eb)",
  display: "flex",
  flexDirection: "column",
  gap: "0.625rem",
};
const heading: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
  margin: 0,
  color: "var(--vn-text-2, #cbd5e1)",
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
