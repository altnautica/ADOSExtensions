import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import type {
  EstimatorMode,
  EstimatorState,
  VisionNavTelemetry,
} from "../types";

type TFn = PluginContext["i18n"]["t"];

function tr(t: TFn, key: string, fallback: string): string {
  const result = t(key);
  return result === key ? fallback : result;
}

interface Props {
  ctx: PluginContext;
  telemetry: VisionNavTelemetry;
}

/**
 * Estimator status card. Shows the engine name, current state, and
 * the VIO-specific telemetry the agent reports back: feature count,
 * drift estimate, reset counter, IMU-camera sync offset trend.
 *
 * The card renders for every estimator mode; for non-VIO modes the
 * VIO rows are hidden so the operator does not see "—" rows that
 * never populate.
 */
export function EstimatorCard({ ctx, telemetry }: Props): JSX.Element {
  const t = ctx.i18n.t;
  const mode = (telemetry.mode as EstimatorMode | undefined) ?? "off";
  const state = (telemetry.estimatorState as EstimatorState | undefined) ?? "off";
  const isVio = mode === "vio_openvins" || mode === "vio_vins_fusion" || mode === "hybrid_of_plus_vio";

  const engineLabel = ENGINE_LABELS[mode] ?? "—";

  return (
    <section style={card} data-testid="vn-estimator-card">
      <header style={header}>
        <h3 style={heading}>
          {tr(t, "navigation.estimatorCard.title", "Estimator")}
        </h3>
        <StatePill state={state} t={t} />
      </header>
      <div style={rows}>
        <Row
          label={tr(t, "navigation.estimatorCard.engine", "Engine")}
          value={engineLabel}
        />
        <Row
          label={tr(t, "navigation.estimatorCard.flowQuality", "Flow quality")}
          value={
            typeof telemetry.flowQuality === "number"
              ? `${telemetry.flowQuality}/255`
              : "—"
          }
        />
        {isVio && (
          <>
            <Row
              label={tr(t, "navigation.estimatorCard.features", "Features")}
              value={
                typeof telemetry.estimatorFeatureCount === "number"
                  ? String(telemetry.estimatorFeatureCount)
                  : "—"
              }
            />
            <Row
              label={tr(t, "navigation.estimatorCard.drift", "Drift")}
              value={
                typeof telemetry.estimatorDriftEstimateM === "number"
                  ? `${telemetry.estimatorDriftEstimateM.toFixed(2)} m`
                  : "—"
              }
            />
            <Row
              label={tr(t, "navigation.estimatorCard.resets", "Resets")}
              value={
                typeof telemetry.vioResetCounter === "number"
                  ? String(telemetry.vioResetCounter)
                  : "—"
              }
            />
          </>
        )}
        <Row
          label={tr(t, "navigation.estimatorCard.syncOffset", "Sync offset")}
          value={
            typeof telemetry.cameraImuSyncOffsetMs === "number"
              ? `${telemetry.cameraImuSyncOffsetMs.toFixed(1)} ms`
              : "—"
          }
        />
      </div>
    </section>
  );
}

const ENGINE_LABELS: Partial<Record<EstimatorMode, string>> = {
  off: "—",
  optical_flow: "Lucas-Kanade",
  optical_flow_degraded: "Lucas-Kanade (degraded)",
  vio_openvins: "OpenVINS",
  vio_vins_fusion: "VINS-Fusion",
  hybrid_of_plus_vio: "OpenVINS + Lucas-Kanade",
};

interface PillProps {
  state: EstimatorState;
  t: TFn;
}

function StatePill({ state, t }: PillProps): JSX.Element {
  const tone = STATE_TONES[state] ?? STATE_TONES.off;
  return (
    <span style={pill(tone.color)} data-testid={`vn-estimator-pill-${state}`}>
      <span style={dot(tone.color)} aria-hidden />
      {tr(t, `navigation.estimatorState.${state}`, tone.label)}
    </span>
  );
}

const STATE_TONES: Record<EstimatorState, { color: string; label: string }> = {
  off: { color: "var(--vn-muted, #6b7280)", label: "Off" },
  init: { color: "var(--vn-warn, #f59e0b)", label: "Initializing" },
  converging: { color: "var(--vn-warn, #f59e0b)", label: "Converging" },
  converged: { color: "var(--vn-ok, #34d399)", label: "Converged" },
  degraded: { color: "var(--vn-warn, #f59e0b)", label: "Degraded" },
  failed: { color: "var(--vn-error, #ef4444)", label: "Failed" },
};

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
  color: "var(--vn-text, #e5e7eb)",
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
  color: "var(--vn-text-2, #cbd5e1)",
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
const pill = (color: string): CSSProperties => ({
  display: "inline-flex",
  alignItems: "center",
  gap: "0.375rem",
  padding: "0.25rem 0.5rem",
  borderRadius: "999px",
  border: `1px solid ${color}`,
  color,
  fontSize: "0.7rem",
  fontWeight: 600,
});
const dot = (color: string): CSSProperties => ({
  width: "0.5rem",
  height: "0.5rem",
  borderRadius: "50%",
  background: color,
  display: "inline-block",
});
