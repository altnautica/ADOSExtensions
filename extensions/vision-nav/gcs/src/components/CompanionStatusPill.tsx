import type { CSSProperties } from "react";

import { noDataLabel, type Freshness } from "../freshness";
import { tr, type TFn } from "../i18n";
import type { VisionNavTelemetry } from "../types";

type CompanionState = NonNullable<VisionNavTelemetry["companionState"]>;

interface Props {
  t: TFn;
  state: VisionNavTelemetry["companionState"];
  /** The navigation block's freshness; only a live block names a state. */
  freshness: Freshness;
}

const DOTS: Record<CompanionState, string> = {
  active: "var(--vn-ok, #34d399)",
  critical: "var(--vn-error, #ef4444)",
  terminating: "var(--vn-error, #ef4444)",
  inactive: "var(--vn-warn, #f59e0b)",
};
const MUTED = "var(--vn-muted, #6b7280)";

export function CompanionStatusPill({ t, state, freshness }: Props): JSX.Element {
  const known = freshness.kind === "live" && state !== undefined && state in DOTS;
  const dataState = known ? state : freshness.kind === "stale" ? "stale" : "none";
  const word = known
    ? tr(t, `navigation.companion.state.${state}`)
    : noDataLabel(t, freshness);
  const dotStyle: CSSProperties = {
    width: "0.5rem",
    height: "0.5rem",
    borderRadius: "9999px",
    background: known ? DOTS[state] : MUTED,
  };
  return (
    <span style={pillStyle} data-testid="vn-companion-pill" data-state={dataState}>
      <span style={dotStyle} aria-hidden="true" />
      <span>{tr(t, "navigation.companion.pill", { state: word })}</span>
    </span>
  );
}

const pillStyle: CSSProperties = {
  display: "inline-flex",
  alignItems: "center",
  gap: "0.5rem",
  padding: "0.25rem 0.75rem",
  borderRadius: "9999px",
  background: "var(--vn-surface-2, rgba(255,255,255,0.04))",
  color: "var(--vn-text, #ededed)",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  fontSize: "0.8125rem",
  fontWeight: 500,
};
