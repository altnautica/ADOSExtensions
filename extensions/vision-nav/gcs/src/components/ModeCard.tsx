import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import type {
  EstimatorMode,
  EstimatorState,
  VisionNavTelemetry,
} from "../types";

// The plugin SDK's i18n.tr(t,) returns the key itself when the host has
// no translation for it. The pattern in this codebase is to call tr(t,)
// and substitute a literal fallback when the result equals the key,
// keeping the call sites readable while still honouring locale
// overrides when the host has them.
type TFn = PluginContext["i18n"]["t"];
function tr(t: TFn, key: string, fallback: string): string {
  const result = t(key);
  return result === key ? fallback : result;
}

interface Props {
  ctx: PluginContext;
  telemetry: VisionNavTelemetry;
}

interface ModeOption {
  id: EstimatorMode;
  label: string;
  description: string;
  needs: string;
}

/**
 * The six modes the estimator framework can host. The card filters
 * this list against ``availableEstimators`` from the heartbeat so an
 * operator never sees an option the agent cannot actually run.
 */
const ALL_MODES: ModeOption[] = [
  {
    id: "off",
    label: "Off",
    description:
      "Sensors only. The estimator emits no MAVLink and the EKF is " +
      "untouched. Useful for hardware diagnostics and calibration runs.",
    needs: "No hardware required.",
  },
  {
    id: "optical_flow",
    label: "Optical Flow",
    description:
      "Downward camera plus a rangefinder. Lucas-Kanade tracker " +
      "feeds OPTICAL_FLOW_RAD to the EKF. The default GPS-denied path.",
    needs: "Downward USB or CSI camera and a rangefinder.",
  },
  {
    id: "optical_flow_degraded",
    label: "Optical Flow (degraded)",
    description:
      "Same tracker as Optical Flow, but the scale comes from the " +
      "baro / GPS ladder instead of a rangefinder. Reduced accuracy; " +
      "the GCS marks the estimator state as degraded.",
    needs: "Downward camera. No rangefinder required.",
  },
  {
    id: "vio_openvins",
    label: "VIO (OpenVINS)",
    description:
      "Forward camera plus the FC IMU, fused by OpenVINS. Reports a " +
      "full 6-DOF pose. Best fit for low-cost NPU boards.",
    needs: "Forward global-shutter camera and an NPU-capable SBC.",
  },
  {
    id: "vio_vins_fusion",
    label: "VIO (VINS-Fusion)",
    description:
      "Forward camera plus the FC IMU, fused by VINS-Fusion. Higher " +
      "CPU cost than OpenVINS, more accurate on fast motion.",
    needs: "Forward camera and a higher-end SBC (Rock 5C / CM4 class).",
  },
  {
    id: "hybrid_of_plus_vio",
    label: "Hybrid (OF + VIO)",
    description:
      "Both estimators running in parallel: a downward camera feeds " +
      "OF and a forward camera feeds VIO. The EKF fuses both inputs.",
    needs: "Two cameras, an NPU board, and the headroom for two cores.",
  },
];

export function ModeCard({ ctx, telemetry }: Props): JSX.Element {
  const t = ctx.i18n.t;
  const available = telemetry.availableEstimators ?? [];
  const current = telemetry.mode;
  const estimatorState =
    (telemetry.estimatorState as EstimatorState | undefined) ?? "off";

  const options = ALL_MODES.filter((opt) => available.includes(opt.id));
  const renderable = options.length > 0 ? options : ALL_MODES.slice(0, 2);

  return (
    <section style={card} data-testid="vn-mode-card">
      <header style={header}>
        <h3 style={heading}>{tr(t,"navigation.modeCard.title", "Mode")}</h3>
        <StatePill state={estimatorState} t={t} />
      </header>
      <p style={subhead}>
        {tr(t,
          "navigation.modeCard.subhead",
          "Choose the estimator the plugin feeds to the flight controller.",
        )}
      </p>
      <div style={row}>
        {renderable.map((opt) => {
          const selected = opt.id === current;
          return (
            <button
              key={opt.id}
              type="button"
              style={button(selected)}
              data-testid={`vn-mode-button-${opt.id}`}
              aria-pressed={selected}
              title={`${opt.description}\n\n${opt.needs}`}
            >
              <div style={btnLabel}>{opt.label}</div>
              <div style={btnNeeds}>{opt.needs}</div>
            </button>
          );
        })}
      </div>
      {options.length === 0 ? (
        <p style={hint} data-testid="vn-mode-empty-hint">
          {tr(t,
            "navigation.modeCard.awaitingAgent",
            "The agent has not advertised its available estimators yet. " +
              "Reconnect or wait for the next heartbeat.",
          )}
        </p>
      ) : null}
    </section>
  );
}

interface PillProps {
  state: EstimatorState;
  t: PluginContext["i18n"]["t"];
}

function StatePill({ state, t }: PillProps): JSX.Element {
  const tone = STATE_TONES[state] ?? STATE_TONES.off;
  return (
    <span style={pill(tone.color)} data-testid={`vn-estimator-state-${state}`}>
      <span style={dot(tone.color)} aria-hidden />
      {tr(t,`navigation.estimatorState.${state}`, tone.label)}
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
const subhead: CSSProperties = {
  fontSize: "0.75rem",
  margin: 0,
  color: "var(--vn-text-muted, #94a3b8)",
};
const row: CSSProperties = {
  display: "grid",
  gridTemplateColumns: "repeatr(t,auto-fit, minmax(180px, 1fr))",
  gap: "0.5rem",
};
const button = (active: boolean): CSSProperties => ({
  padding: "0.625rem 0.75rem",
  background: active
    ? "var(--vn-accent, #2563eb)"
    : "var(--vn-surface-2, rgba(255,255,255,0.06))",
  color: "var(--vn-text, #e5e7eb)",
  border: active
    ? "1px solid var(--vn-accent, #2563eb)"
    : "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.375rem",
  cursor: "pointer",
  display: "flex",
  flexDirection: "column",
  alignItems: "flex-start",
  gap: "0.25rem",
  textAlign: "left",
  fontSize: "0.8125rem",
});
const btnLabel: CSSProperties = { fontWeight: 600 };
const btnNeeds: CSSProperties = {
  fontSize: "0.7rem",
  color: "var(--vn-text-muted, #94a3b8)",
};
const hint: CSSProperties = {
  fontSize: "0.75rem",
  margin: 0,
  color: "var(--vn-text-muted, #94a3b8)",
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
