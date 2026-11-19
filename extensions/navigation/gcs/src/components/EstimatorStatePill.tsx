import type { CSSProperties } from "react";

import { noDataLabel, type Freshness } from "../freshness";
import { tr, type TFn } from "../i18n";
import type { EstimatorState } from "../types";

interface Props {
  t: TFn;
  state: EstimatorState | string | undefined;
  freshness: Freshness;
  /** `data-testid` prefix; the state (or `stale` / `none`) is appended. */
  testId: string;
}

const COLORS: Record<EstimatorState, string> = {
  off: "var(--vn-muted, #6b7280)",
  init: "var(--vn-warn, #f59e0b)",
  converging: "var(--vn-warn, #f59e0b)",
  converged: "var(--vn-ok, #34d399)",
  degraded: "var(--vn-warn, #f59e0b)",
  failed: "var(--vn-error, #ef4444)",
};
const MUTED = "var(--vn-muted, #6b7280)";

/** The estimator state, or "No data" with its age when the navigation
 * block is stale or has not arrived. */
export function EstimatorStatePill({ t, state, freshness, testId }: Props): JSX.Element {
  if (freshness.kind !== "live") {
    return (
      <span style={pill(MUTED)} data-testid={`${testId}-${freshness.kind}`}>
        <span style={dot(MUTED)} aria-hidden />
        {noDataLabel(t, freshness)}
      </span>
    );
  }
  const known: EstimatorState =
    state !== undefined && state in COLORS ? (state as EstimatorState) : "off";
  return (
    <span style={pill(COLORS[known])} data-testid={`${testId}-${known}`}>
      <span style={dot(COLORS[known])} aria-hidden />
      {tr(t, `navigation.estimatorState.${known}`)}
    </span>
  );
}

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
