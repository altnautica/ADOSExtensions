import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { DASH, tr } from "../i18n";
import type { VisionNavTelemetry } from "../types";

interface Props {
  ctx: PluginContext;
  telemetry: VisionNavTelemetry;
}

function bandClassFor(quality: number | null | undefined): string {
  if (typeof quality !== "number") return "unknown";
  if (quality < 50) return "low";
  if (quality < 150) return "mid";
  return "high";
}

const BAND_FILL: Record<string, string> = {
  low: "var(--vn-error, #ef4444)",
  mid: "var(--vn-warn, #f59e0b)",
  high: "var(--vn-ok, #34d399)",
  unknown: "var(--vn-muted, #6b7280)",
};

export function FlowHealthCard({ ctx, telemetry }: Props): JSX.Element {
  const t = ctx.i18n.t;
  const quality = telemetry.flowQuality;
  const band = bandClassFor(quality);
  const pct = typeof quality !== "number" ? 0 : Math.max(0, Math.min(100, (quality / 255) * 100));
  const rate = telemetry.flowRateHz;
  const distance = telemetry.flowDistanceM;

  return (
    <section style={card} data-testid="vn-flow-health">
      <h3 style={heading}>{tr(t, "navigation.flowHealth.title")}</h3>
      <div>
        <div style={row}>
          <span style={label}>{tr(t, "navigation.flowHealth.quality")}</span>
          <span style={value}>
            {typeof quality === "number" ? tr(t, "navigation.format.quality", { v: quality }) : DASH}
          </span>
        </div>
        <div
          style={barOuter}
          role="progressbar"
          aria-valuemin={0}
          aria-valuemax={255}
          aria-valuenow={quality ?? 0}
        >
          <div
            style={{ ...barInner, width: `${pct}%`, background: BAND_FILL[band] }}
            data-testid="vn-flow-quality-bar"
            data-band={band}
          />
        </div>
      </div>
      <div style={row}>
        <span style={label}>{tr(t, "navigation.flowHealth.rate")}</span>
        <span style={value} data-testid="vn-flow-rate">
          {typeof rate === "number" ? tr(t, "navigation.format.hz", { v: rate.toFixed(1) }) : DASH}
        </span>
      </div>
      <div style={row}>
        <span style={label}>{tr(t, "navigation.flowHealth.distance")}</span>
        <span style={value} data-testid="vn-flow-distance">
          {typeof distance === "number" ? tr(t, "navigation.format.m", { v: distance.toFixed(2) }) : DASH}
        </span>
      </div>
    </section>
  );
}

const card: CSSProperties = {
  background: "var(--vn-surface, rgba(255,255,255,0.02))",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.5rem",
  padding: "0.875rem 1rem",
  display: "flex",
  flexDirection: "column",
  gap: "0.625rem",
  color: "var(--vn-text, #ededed)",
};
const heading: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
  color: "var(--vn-text-2, #b4b4b4)",
  margin: 0,
};
const barOuter: CSSProperties = {
  width: "100%",
  height: "0.5rem",
  background: "var(--vn-surface-2, rgba(255,255,255,0.06))",
  borderRadius: "9999px",
  overflow: "hidden",
};
const barInner: CSSProperties = {
  height: "100%",
  transition: "width 200ms linear",
};
const row: CSSProperties = {
  display: "flex",
  justifyContent: "space-between",
  alignItems: "baseline",
  fontSize: "0.8125rem",
};
const value: CSSProperties = {
  fontVariantNumeric: "tabular-nums",
  color: "var(--vn-text, #ededed)",
};
const label: CSSProperties = {
  color: "var(--vn-text-3, #94a3b8)",
};
