import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import type { Freshness } from "../freshness";
import { DASH, tr } from "../i18n";
import type { VisionNavTelemetry } from "../types";

import { EstimatorStatePill } from "./EstimatorStatePill";

interface Props {
  ctx: PluginContext;
  telemetry: VisionNavTelemetry;
  freshness: Freshness;
}

const ENGINES = ["optical_flow", "optical_flow_degraded", "external_nav"];

/**
 * Estimator status card. Shows the engine name, the current state, the
 * flow quality, and the camera-IMU sync offset the aligner measures.
 */
export function EstimatorCard({ ctx, telemetry, freshness }: Props): JSX.Element {
  const t = ctx.i18n.t;
  const mode = telemetry.mode;
  const engine =
    typeof mode === "string" && ENGINES.includes(mode) ? tr(t, `navigation.engine.${mode}`) : DASH;
  const sync = telemetry.cameraImuSyncOffsetMs;

  return (
    <section style={card} data-testid="vn-estimator-card">
      <header style={header}>
        <h3 style={heading}>{tr(t, "navigation.estimatorCard.title")}</h3>
        <EstimatorStatePill
          t={t}
          state={telemetry.estimatorState}
          freshness={freshness}
          testId="vn-estimator-pill"
        />
      </header>
      <div style={rows}>
        <Row label={tr(t, "navigation.estimatorCard.engine")} value={engine} />
        <Row
          label={tr(t, "navigation.estimatorCard.flowQuality")}
          value={
            typeof telemetry.flowQuality === "number"
              ? tr(t, "navigation.format.quality", { v: telemetry.flowQuality })
              : DASH
          }
        />
        <Row
          label={tr(t, "navigation.estimatorCard.syncOffset")}
          value={typeof sync === "number" ? tr(t, "navigation.format.ms", { v: sync.toFixed(1) }) : DASH}
        />
      </div>
    </section>
  );
}

interface RowProps {
  label: string;
  value: string;
}

function Row({ label, value }: RowProps): JSX.Element {
  return (
    <div style={rowItem}>
      <span style={rowLabel}>{label}</span>
      <span style={rowValue}>{value}</span>
    </div>
  );
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
const header: CSSProperties = {
  display: "flex",
  alignItems: "center",
  justifyContent: "space-between",
};
const heading: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
  margin: 0,
  color: "var(--vn-text-2, #b4b4b4)",
};
const rows: CSSProperties = {
  display: "grid",
  gridTemplateColumns: "1fr 1fr",
  gap: "0.5rem 1rem",
};
const rowItem: CSSProperties = {
  display: "flex",
  justifyContent: "space-between",
  alignItems: "center",
  padding: "0.25rem 0",
  borderBottom: "1px solid var(--vn-border, rgba(255,255,255,0.04))",
};
const rowLabel: CSSProperties = {
  fontSize: "0.75rem",
  color: "var(--vn-text-muted, #94a3b8)",
};
const rowValue: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
  fontVariantNumeric: "tabular-nums",
};
