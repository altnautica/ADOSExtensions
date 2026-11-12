import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { noDataLabel, type Freshness } from "../freshness";
import { FLAT_LOCALE, tr, type TFn } from "../i18n";
import type { PreArmCheck, PreArmCheckSeverity, VisionNavTelemetry } from "../types";

interface Props {
  ctx: PluginContext;
  telemetry: VisionNavTelemetry;
  freshness: Freshness;
  /** The host refused the navigation telemetry subscription. */
  denied: boolean;
}

/** The flow-quality pre-arm minimum, as the agent's gate applies it (`>=`). */
const FLOW_QUALITY_GATE = 50;

/**
 * Mode-aware pre-arm status. Reads ``telemetry.preArmReport.checks``
 * from the agent and renders one row per check. Each check carries an
 * id, a severity (``ok`` / ``pending`` / ``blocking``), and a detail
 * string. Severity drives the indicator colour; the detail appears as a
 * sub-row when present.
 *
 * Before the first navigation block the card says it is waiting, and
 * when the host refused the telemetry it says so; neither claims a
 * fault. A stale block keeps its rows (dimmed by the tab) but the
 * Ready / Blocked pill reads "No data" with the age.
 *
 * Backward compat: a block that has data but no ``preArmReport`` (an
 * older agent) renders the legacy three-check list derived from the
 * flat telemetry fields.
 */
export function PreArmStatus({ ctx, telemetry, freshness, denied }: Props): JSX.Element {
  const t = ctx.i18n.t;
  const head = (pill: JSX.Element | null): JSX.Element => (
    <header style={headerRow}>
      <h3 style={heading}>{tr(t, "navigation.preArm.title")}</h3>
      {pill}
    </header>
  );

  if (denied || freshness.kind === "none") {
    return (
      <section style={card} data-testid="vn-pre-arm">
        {head(null)}
        <p style={hintStyle} data-testid={denied ? "vn-pre-arm-denied" : "vn-pre-arm-waiting"}>
          {tr(t, denied ? "navigation.preArm.denied" : "navigation.preArm.waiting")}
        </p>
      </section>
    );
  }

  const report = telemetry.preArmReport;
  const checks = report ? report.checks : legacyChecks(telemetry, t);
  const armable = report ? report.armable : checks.every((c) => c.severity === "ok");
  const pill =
    freshness.kind === "stale" ? (
      <span style={armablePill(MUTED)} data-testid="vn-pre-arm-stale">
        {noDataLabel(t, freshness)}
      </span>
    ) : (
      <span
        style={armablePill(armable ? OK : ERROR)}
        data-testid={`vn-pre-arm-${armable ? "armable" : "blocked"}`}
      >
        {tr(t, armable ? "navigation.preArm.ready" : "navigation.preArm.blocked")}
      </span>
    );

  return (
    <section style={card} data-testid="vn-pre-arm">
      {head(pill)}
      <ul style={list}>
        {checks.map((check) => (
          <li key={check.id} style={item} data-state={check.severity} data-check-id={check.id}>
            <span style={row}>
              <span style={indicator(check.severity)} aria-hidden="true">
                {ICONS[check.severity]}
              </span>
              <span>{titleFor(check.id, t)}</span>
            </span>
            {check.detail ? <span style={detailStyle}>{check.detail}</span> : null}
          </li>
        ))}
      </ul>
    </section>
  );
}

function legacyChecks(telemetry: VisionNavTelemetry, t: TFn): PreArmCheck[] {
  // Fallback for agents that don't yet emit preArmReport. Surface the
  // same three checks the old card showed so a half-upgraded fleet
  // does not render an empty pre-arm card.
  const companionOk = telemetry.companionState === "active";
  const quality = telemetry.flowQuality ?? 0;
  const flowOk = quality >= FLOW_QUALITY_GATE;
  return [
    {
      id: "companion_active",
      severity: companionOk ? "ok" : "blocking",
      detail: companionOk ? "" : tr(t, "navigation.preArm.legacy.companionDown"),
    },
    {
      id: "flow_quality",
      severity: flowOk ? "ok" : "blocking",
      detail: flowOk
        ? tr(t, "navigation.preArm.legacy.flowOk", { q: quality })
        : tr(t, "navigation.preArm.legacy.flowLow"),
    },
    {
      id: "ekf_source",
      severity: "pending",
      detail: tr(t, "navigation.preArm.legacy.ekfSource"),
    },
  ];
}

// Keyed by the check ids the agent's pre-arm gate emits. An id with no
// locale entry falls back to its words with the underscores removed, so
// a new check still renders a label.
function titleFor(id: string, t: TFn): string {
  const key = `navigation.preArm.checks.${id}`;
  if (key in FLAT_LOCALE) return tr(t, key);
  const words = id.split("_").join(" ");
  return words.charAt(0).toUpperCase() + words.slice(1);
}

const ICONS: Record<PreArmCheckSeverity, string> = {
  ok: "✓",
  pending: "…",
  blocking: "✕",
};

const OK = "var(--vn-ok, #34d399)";
const WARN = "var(--vn-warn, #f59e0b)";
const ERROR = "var(--vn-error, #ef4444)";
const MUTED = "var(--vn-muted, #6b7280)";

const card: CSSProperties = {
  background: "var(--vn-surface, rgba(255,255,255,0.02))",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.5rem",
  padding: "0.875rem 1rem",
  color: "var(--vn-text, #ededed)",
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
  color: "var(--vn-text-2, #b4b4b4)",
};
const armablePill = (color: string): CSSProperties => ({
  display: "inline-flex",
  alignItems: "center",
  padding: "0.2rem 0.5rem",
  borderRadius: "999px",
  border: `1px solid ${color}`,
  color,
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
  margin: 0,
  color: "var(--vn-text-3, #94a3b8)",
  fontSize: "0.8125rem",
};
const detailStyle: CSSProperties = {
  color: "var(--vn-text-3, #94a3b8)",
  fontSize: "0.75rem",
  marginLeft: "1.25rem",
};
const indicator = (severity: PreArmCheckSeverity): CSSProperties => ({
  display: "inline-flex",
  width: "0.875rem",
  height: "0.875rem",
  borderRadius: "9999px",
  background: severity === "ok" ? OK : severity === "pending" ? WARN : ERROR,
  color: "#0a0a0a",
  alignItems: "center",
  justifyContent: "center",
  fontSize: "0.625rem",
  fontWeight: 700,
  flexShrink: 0,
});
