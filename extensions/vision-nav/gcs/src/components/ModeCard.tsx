import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import type { Freshness } from "../freshness";
import { tr } from "../i18n";
import type { EstimatorMode, VisionNavTelemetry } from "../types";

import { EstimatorStatePill } from "./EstimatorStatePill";

interface Props {
  ctx: PluginContext;
  telemetry: VisionNavTelemetry;
  freshness: Freshness;
  /** The agent's `navigation.engage` state: `false` while disengaged,
   * `null` when unknown or stale. */
  engaged: boolean | null;
}

/**
 * The modes the estimator framework can host. The card filters this
 * list against ``availableEstimators`` from the heartbeat so an
 * operator never sees an option the agent cannot actually run.
 */
const ALL_MODES: EstimatorMode[] = ["off", "optical_flow", "optical_flow_degraded", "external_nav"];

/**
 * The modes the agent can run, with the current (effective) one marked.
 * A read-only list: the plugin's per-drone settings choose the mode and
 * the Engage skill starts and stops it.
 */
export function ModeCard({ ctx, telemetry, freshness, engaged }: Props): JSX.Element {
  const t = ctx.i18n.t;
  const available = telemetry.availableEstimators ?? [];
  const current = telemetry.mode;

  // Only the modes the agent has advertised are listed. Until the first
  // heartbeat arrives the card shows the hint below and no rows, rather
  // than guessing at a mode set.
  const renderable = ALL_MODES.filter((id) => available.includes(id));

  return (
    <section style={card} data-testid="vn-mode-card">
      <header style={header}>
        <h3 style={heading}>{tr(t, "navigation.modeCard.title")}</h3>
        <EstimatorStatePill
          t={t}
          state={telemetry.estimatorState}
          freshness={freshness}
          testId="vn-estimator-state"
        />
      </header>
      {engaged === false ? (
        <p style={disengaged} role="status" data-testid="vn-mode-disengaged">
          {tr(t, "navigation.modeCard.disengaged")}
        </p>
      ) : null}
      <p style={subhead}>{tr(t, "navigation.modeCard.subhead")}</p>
      {renderable.length > 0 ? (
        <ul style={list}>
          {renderable.map((id) => {
            const selected = id === current;
            return (
              <li
                key={id}
                style={tile(selected)}
                data-testid={`vn-mode-${id}`}
                aria-current={selected ? "true" : undefined}
              >
                <span style={tileLabel}>
                  {tr(t, `navigation.modes.${id}.label`)}
                  {selected ? (
                    <span style={currentBadge}>{tr(t, "navigation.modeCard.current")}</span>
                  ) : null}
                </span>
                <span style={selected ? selectedText : mutedText}>
                  {tr(t, `navigation.modes.${id}.description`)}
                </span>
                <span style={selected ? selectedText : mutedText}>
                  {tr(t, `navigation.modes.${id}.needs`)}
                </span>
              </li>
            );
          })}
        </ul>
      ) : (
        <p style={subhead} data-testid="vn-mode-empty-hint">
          {tr(t, "navigation.modeCard.awaitingAgent")}
        </p>
      )}
    </section>
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
const subhead: CSSProperties = {
  fontSize: "0.75rem",
  margin: 0,
  color: "var(--vn-text-muted, #94a3b8)",
};
const disengaged: CSSProperties = {
  fontSize: "0.8125rem",
  margin: 0,
  padding: "0.5rem 0.75rem",
  borderRadius: "0.375rem",
  border: "1px solid var(--vn-warn, #f59e0b)",
  color: "var(--vn-text, #ededed)",
};
const list: CSSProperties = {
  listStyle: "none",
  margin: 0,
  padding: 0,
  display: "grid",
  gridTemplateColumns: "repeat(auto-fit, minmax(180px, 1fr))",
  gap: "0.5rem",
};
// A read-only tile: no pointer cursor and no button chrome. The current
// mode is filled with the accent and carries a "Current" word, so colour
// is not the only cue.
const tile = (selected: boolean): CSSProperties => ({
  padding: "0.625rem 0.75rem",
  background: selected
    ? "var(--vn-accent, #3a82ff)"
    : "transparent",
  color: "var(--vn-text, #ededed)",
  border: selected
    ? "1px solid var(--vn-accent, #3a82ff)"
    : "1px dashed var(--vn-border, rgba(255,255,255,0.12))",
  borderRadius: "0.375rem",
  display: "flex",
  flexDirection: "column",
  gap: "0.25rem",
  fontSize: "0.8125rem",
});
const tileLabel: CSSProperties = {
  fontWeight: 600,
  display: "inline-flex",
  alignItems: "center",
  gap: "0.375rem",
  flexWrap: "wrap",
};
const currentBadge: CSSProperties = {
  fontSize: "0.65rem",
  fontWeight: 700,
  textTransform: "uppercase",
  letterSpacing: "0.05em",
  color: "var(--vn-text, #ededed)",
};
const mutedText: CSSProperties = {
  fontSize: "0.7rem",
  color: "var(--vn-text-muted, #94a3b8)",
};
const selectedText: CSSProperties = {
  fontSize: "0.7rem",
  color: "var(--vn-text, #ededed)",
};
