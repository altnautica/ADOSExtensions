import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import type { ExtnavHealth, NavState } from "../types";

type TFn = PluginContext["i18n"]["t"];
function tr(t: TFn, key: string, fallback: string): string {
  const result = t(key);
  return result === key ? fallback : result;
}

interface Props {
  ctx: PluginContext;
  state: NavState;
}

/** Symbol plus word for each tracking health, so colour is never the only cue. */
const HEALTH: Record<ExtnavHealth, { symbol: string; key: string; word: string; color: string }> = {
  good: { symbol: "●", key: "good", word: "Good", color: "var(--vn-ok, #34d399)" },
  degraded: { symbol: "◐", key: "degraded", word: "Degraded", color: "var(--vn-warn, #f59e0b)" },
  lost: { symbol: "✕", key: "lost", word: "Lost", color: "var(--vn-error, #ef4444)" },
};

/** GPS_FIX_TYPE words. */
const FIX_WORDS = ["No GPS", "No fix", "2D fix", "3D fix", "DGPS", "RTK float", "RTK fixed"];

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

const DASH = "—";

/**
 * The live navigation feed: which EKF source set is active, how the
 * external pose is tracking, the ODOMETRY and obstacle rates, the nearest
 * obstacle and the GPS fix. Reads the agent's `nav.state` event.
 */
export function NavStateCard({ ctx, state }: Props): JSX.Element {
  const t = ctx.i18n.t;
  const health = state.extnavHealth ? HEALTH[state.extnavHealth] : null;
  const rows: Array<{ id: string; key: string; fallback: string; text: string; color?: string }> = [
    {
      id: "active-set",
      key: "activeSet",
      fallback: "EKF source set",
      text: state.activeSet === null ? DASH : `${state.activeSet}`,
    },
    {
      id: "extnav-health",
      key: "extnavHealth",
      fallback: "External nav",
      text: health
        ? `${health.symbol} ${tr(t, `navigation.navState.health.${health.key}`, health.word)}`
        : DASH,
      color: health?.color,
    },
    {
      id: "odom-rate",
      key: "odomRate",
      fallback: "ODOMETRY rate",
      text: state.odomRateHz === null ? DASH : `${state.odomRateHz.toFixed(1)} Hz`,
    },
    {
      id: "reset-counter",
      key: "resetCounter",
      fallback: "Pose resets",
      text: state.lastResetCounter === null ? DASH : `${state.lastResetCounter}`,
    },
    {
      id: "obstacle-rate",
      key: "obstacleRate",
      fallback: "Obstacle feed",
      text: state.obstacleRateHz === null ? DASH : `${state.obstacleRateHz.toFixed(1)} Hz`,
    },
    {
      id: "min-obstacle",
      key: "minObstacle",
      fallback: "Nearest obstacle",
      text: state.minObstacleM === null ? DASH : `${state.minObstacleM.toFixed(2)} m`,
    },
    {
      id: "gps-fix",
      key: "gpsFix",
      fallback: "GPS",
      text: state.gpsFix === null ? DASH : (FIX_WORDS[state.gpsFix] ?? `Fix ${state.gpsFix}`),
    },
  ];

  return (
    <section style={card} data-testid="vn-nav-state">
      <h3 style={heading}>{tr(t, "navigation.navState.title", "Navigation feed")}</h3>
      {state.mode === null ? (
        <span style={label} data-testid="vn-nav-state-waiting">
          {tr(t, "navigation.navState.waiting", "Waiting for the agent's navigation state.")}
        </span>
      ) : (
        rows.map((r) => (
          <div style={row} key={r.id}>
            <span style={label}>{tr(t, `navigation.navState.${r.key}`, r.fallback)}</span>
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
