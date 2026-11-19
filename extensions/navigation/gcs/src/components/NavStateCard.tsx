import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { noDataLabel, type Freshness } from "../freshness";
import { DASH, tr } from "../i18n";
import type { ExtnavHealth, NavState } from "../types";

interface Props {
  ctx: PluginContext;
  state: NavState;
  /** How current the `nav.state` event is. */
  freshness: Freshness;
  /** `navigation.poseAgeMs`: median age of a pose when its ODOMETRY is
   * sent, or null before the first one (or while the block is stale). */
  poseAgeMs?: number | null;
}

/** Symbol plus word for each tracking health, so colour is never the only cue. */
const HEALTH: Record<ExtnavHealth, { symbol: string; color: string }> = {
  good: { symbol: "●", color: "var(--vn-ok, #34d399)" },
  degraded: { symbol: "◐", color: "var(--vn-warn, #f59e0b)" },
  lost: { symbol: "✕", color: "var(--vn-error, #ef4444)" },
};

/** GPS_FIX_TYPE values with a word in the shared nav spec (0-8). */
const MAX_FIX_TYPE = 8;

/**
 * The live navigation feed: which EKF source set is active, how the
 * external pose is tracking, the odometry rate and pose age, the obstacle
 * rate, the nearest obstacle and the GPS fix, in the words and units the
 * agent cockpit and the TUI use for the same `nav.state` fields. Past the
 * shared stale period the rows dim and the tracking health reads
 * "No data" with the age.
 */
export function NavStateCard({ ctx, state, freshness, poseAgeMs = null }: Props): JSX.Element {
  const t = ctx.i18n.t;
  const live = freshness.kind === "live";
  const health = state.extnavHealth ? HEALTH[state.extnavHealth] : null;
  const fix = state.gpsFix;
  const rows: Array<{ id: string; key: string; text: string; color?: string }> = [
    {
      id: "active-set",
      key: "activeSet",
      text: state.activeSet === null ? DASH : tr(t, "navigation.navState.setN", { n: state.activeSet }),
    },
    {
      id: "extnav-health",
      key: "extnavHealth",
      text: !live
        ? noDataLabel(t, freshness)
        : health && state.extnavHealth
          ? `${health.symbol} ${tr(t, `navigation.navState.health.${state.extnavHealth}`)}`
          : DASH,
      color: live ? health?.color : undefined,
    },
    {
      id: "odom-rate",
      key: "odomRate",
      text:
        state.odomRateHz === null
          ? DASH
          : tr(t, "navigation.format.hz", { v: state.odomRateHz.toFixed(0) }),
    },
    {
      id: "pose-age",
      key: "poseAge",
      text:
        poseAgeMs === null || !Number.isFinite(poseAgeMs)
          ? DASH
          : tr(t, "navigation.format.ms", { v: Math.round(poseAgeMs) }),
    },
    {
      id: "reset-counter",
      key: "resetCounter",
      text: state.lastResetCounter === null ? DASH : `${state.lastResetCounter}`,
    },
    {
      id: "obstacle-rate",
      key: "obstacleRate",
      text:
        state.obstacleRateHz === null
          ? DASH
          : tr(t, "navigation.format.hz", { v: state.obstacleRateHz.toFixed(0) }),
    },
    {
      id: "min-obstacle",
      key: "minObstacle",
      text:
        state.minObstacleM === null
          ? DASH
          : tr(t, "navigation.format.m", { v: state.minObstacleM.toFixed(2) }),
    },
    {
      id: "gps-fix",
      key: "gpsFix",
      text:
        fix === null
          ? DASH
          : Number.isInteger(fix) && fix >= 0 && fix <= MAX_FIX_TYPE
            ? tr(t, `navigation.navState.fix.${fix}`)
            : tr(t, "navigation.navState.fix.unknown"),
    },
  ];

  return (
    <section
      style={freshness.kind === "stale" ? { ...card, opacity: 0.5 } : card}
      data-testid="vn-nav-state"
      data-stale={freshness.kind === "stale" ? "true" : "false"}
    >
      <h3 style={heading}>{tr(t, "navigation.navState.title")}</h3>
      {state.mode === null ? (
        <span style={label} data-testid="vn-nav-state-waiting">
          {tr(t, "navigation.navState.waiting")}
        </span>
      ) : (
        rows.map((r) => (
          <div style={row} key={r.id}>
            <span style={label}>{tr(t, `navigation.navState.${r.key}`)}</span>
            <span
              style={r.color ? { ...value, color: r.color } : value}
              data-testid={`vn-nav-${r.id}`}
            >
              {r.text}
            </span>
          </div>
        ))
      )}
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
  gap: "0.5rem",
  color: "var(--vn-text, #ededed)",
};
const heading: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
  color: "var(--vn-text-2, #b4b4b4)",
  margin: 0,
};
const row: CSSProperties = {
  display: "flex",
  justifyContent: "space-between",
  alignItems: "baseline",
  fontSize: "0.8125rem",
};
const label: CSSProperties = { color: "var(--vn-text-3, #94a3b8)" };
const value: CSSProperties = {
  fontVariantNumeric: "tabular-nums",
  color: "var(--vn-text, #ededed)",
};
