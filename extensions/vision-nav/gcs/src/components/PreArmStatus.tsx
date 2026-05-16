import type { CSSProperties } from "react";

import type {
  PreArmCheck,
  PreArmCheckSeverity,
  VisionNavTelemetry,
} from "../types";

interface Props {
  telemetry: VisionNavTelemetry;
}

/**
 * Mode-aware pre-arm status. Reads ``telemetry.preArmReport.checks``
 * from the agent and renders one row per check. Each check carries an
 * id, a severity (``ok`` / ``pending`` / ``blocking``), and a free-form
 * detail string. Severity drives the indicator colour; the detail
 * appears as a sub-row when present.
 *
 * Backward compat: when ``preArmReport`` is absent (older agent that
 * predates the runtime-wired pipeline) the card falls back to the
 * legacy three-check render derived from the flat telemetry fields.
 */
export function PreArmStatus({ telemetry }: Props): JSX.Element {
  const report = telemetry.preArmReport;
  const checks = report ? report.checks : legacyChecks(telemetry);
  const armable = report
    ? report.armable
    : checks.every((c) => c.severity === "ok");

  return (
    <section style={card} data-testid="vn-pre-arm">
      <header style={headerRow}>
        <h3 style={heading}>Pre-arm</h3>
        <span
          style={armablePill(armable)}
          data-testid={`vn-pre-arm-${armable ? "armable" : "blocked"}`}
        >
          {armable ? "Ready" : "Blocked"}
        </span>
      </header>
      <ul style={list}>
        {checks.map((check) => (
          <li
            key={check.id}
            style={item}
            data-state={check.severity}
            data-check-id={check.id}
          >
            <span style={row}>
              <span style={indicator(check.severity)} aria-hidden="true">
                {iconFor(check.severity)}
              </span>
              <span>{titleFor(check.id, check.detail)}</span>
            </span>
            {check.detail ? <span style={hintStyle}>{check.detail}</span> : null}
          </li>
        ))}
      </ul>
    </section>
  );
}

function legacyChecks(telemetry: VisionNavTelemetry): PreArmCheck[] {
  // Fallback for agents that don't yet emit preArmReport. Surface the
  // same three checks the old card showed so a half-upgraded fleet
  // does not render an empty pre-arm card.
  const companionOk = telemetry.companionState === "active";
  const flowOk = (telemetry.flowQuality ?? 0) > 50;
  return [
    {
      id: "companion_active",
      severity: companionOk ? "ok" : "blocking",
      detail: companionOk
        ? ""
        : "Vision companion is not running. Check agent logs.",
    },
    {
      id: "flow_quality",
      severity: flowOk ? "ok" : "blocking",
      detail: flowOk
        ? `Quality ${telemetry.flowQuality ?? 0}/255.`
        : "Flow quality is below the pre-arm gate (50/255).",
    },
    {
      id: "ekf_source",
      severity: "pending",
      detail: "Configure EKF source set manually in the parameter editor.",
    },
  ];
}

function titleFor(id: string, _detail: string): string {
  return TITLES[id] ?? humanise(id);
}

const TITLES: Record<string, string> = {
  companion_active: "Companion process active",
  flow_quality: "Optical flow quality",
  rangefinder: "Rangefinder healthy",
  scale_source: "Altitude scale source",
  estimator_converged: "Estimator converged",
  intrinsics_loaded: "Camera intrinsics loaded",
  extrinsics_loaded: "Camera-IMU extrinsics loaded",
  sync_offset: "Camera-IMU sync offset",
  feature_count: "VIO feature count",
  ekf_source: "EKF source set is vision",
  mode_unknown: "Mode recognised",
};

function humanise(id: string): string {
  return id
    .split("_")
    .map((s) => s.charAt(0).toUpperCase() + s.slice(1))
    .join(" ");
}

function iconFor(severity: PreArmCheckSeverity): string {
  if (severity === "ok") return "✓";
  if (severity === "pending") return "…";
  return "✕";
}

const card: CSSProperties = {
  background: "var(--vn-surface, rgba(255,255,255,0.02))",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.5rem",
  padding: "0.875rem 1rem",
  color: "var(--vn-text, #e5e7eb)",
  display: "flex",
  flexDirection: "column",
  gap: "0.5rem",
};
const headerRow: CSSProperties = {
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
const armablePill = (ready: boolean): CSSProperties => ({
  display: "inline-flex",
  alignItems: "center",
  padding: "0.2rem 0.5rem",
  borderRadius: "999px",
  border: `1px solid ${ready ? "var(--vn-ok, #34d399)" : "var(--vn-error, #ef4444)"}`,
  color: ready ? "var(--vn-ok, #34d399)" : "var(--vn-error, #ef4444)",
  fontSize: "0.7rem",
  fontWeight: 600,
});
const list: CSSProperties = {
  listStyle: "none",
  margin: 0,
  padding: 0,
  display: "flex",
  flexDirection: "column",
  gap: "0.375rem",
};
const item: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.125rem",
  fontSize: "0.8125rem",
};
const row: CSSProperties = {
  display: "flex",
  gap: "0.5rem",
  alignItems: "center",
};
const hintStyle: CSSProperties = {
  color: "var(--vn-text-3, #94a3b8)",
  fontSize: "0.75rem",
  marginLeft: "1.25rem",
};
const indicator = (severity: PreArmCheckSeverity): CSSProperties => ({
  display: "inline-flex",
  width: "0.875rem",
  height: "0.875rem",
  borderRadius: "9999px",
  background:
    severity === "ok"
      ? "var(--vn-ok, #34d399)"
      : severity === "pending"
        ? "var(--vn-warn, #f59e0b)"
        : "var(--vn-error, #ef4444)",
  color: "#0b1220",
  alignItems: "center",
  justifyContent: "center",
  fontSize: "0.625rem",
  fontWeight: 700,
  flexShrink: 0,
});
