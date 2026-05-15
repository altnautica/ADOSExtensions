import type { CSSProperties } from "react";

import type { VisionNavTelemetry } from "../types";

interface Props {
  telemetry: VisionNavTelemetry;
}

function bandClassFor(quality: number | undefined): string {
  if (quality === undefined) return "unknown";
  if (quality < 50) return "low";
  if (quality < 150) return "mid";
  return "high";
}

function bandFill(band: string): string {
  switch (band) {
    case "low":
      return "var(--vn-error, #ef4444)";
    case "mid":
      return "var(--vn-warn, #f59e0b)";
    case "high":
      return "var(--vn-ok, #34d399)";
    default:
      return "var(--vn-muted, #6b7280)";
  }
}

export function FlowHealthCard({ telemetry }: Props): JSX.Element {
  const quality = telemetry.flowQuality;
  const band = bandClassFor(quality);
  const pct = quality === undefined ? 0 : Math.max(0, Math.min(100, (quality / 255) * 100));
  const rate = telemetry.flowRateHz;
  const distance = telemetry.flowDistanceM;

  const card: CSSProperties = {
    background: "var(--vn-surface, rgba(255,255,255,0.02))",
    border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
    borderRadius: "0.5rem",
    padding: "0.875rem 1rem",
    display: "flex",
    flexDirection: "column",
    gap: "0.625rem",
    color: "var(--vn-text, #e5e7eb)",
  };
  const heading: CSSProperties = {
    fontSize: "0.8125rem",
    fontWeight: 600,
    color: "var(--vn-text-2, #cbd5e1)",
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
    width: `${pct}%`,
    height: "100%",
    background: bandFill(band),
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
    color: "var(--vn-text, #e5e7eb)",
  };
  const label: CSSProperties = {
    color: "var(--vn-text-3, #94a3b8)",
  };

  return (
    <section style={card} data-testid="vn-flow-health">
      <h3 style={heading}>Flow health</h3>
      <div>
        <div style={row}>
          <span style={label}>Flow quality</span>
          <span style={value}>
            {quality === undefined ? "—" : `${quality}/255`}
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
            style={barInner}
            data-testid="vn-flow-quality-bar"
            data-band={band}
          />
        </div>
      </div>
      <div style={row}>
        <span style={label}>Flow rate</span>
        <span style={value} data-testid="vn-flow-rate">
          {rate === undefined ? "—" : `${rate.toFixed(1)} Hz`}
        </span>
      </div>
      <div style={row}>
        <span style={label}>Flow distance</span>
        <span style={value} data-testid="vn-flow-distance">
          {distance === null || distance === undefined
            ? "—"
            : `${distance.toFixed(2)} m`}
        </span>
      </div>
    </section>
  );
}
