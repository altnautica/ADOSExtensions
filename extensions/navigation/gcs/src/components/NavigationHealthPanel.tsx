import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { freshnessOf, LIVE, useNow, type Freshness } from "../freshness";
import { tr } from "../i18n";
import { useNavState, useNavigationTelemetry } from "../store";
import type { NavState, NavigationTelemetry } from "../types";

import { NavStateCard } from "./NavStateCard";

export interface NavigationHealthPanelProps {
  ctx: PluginContext;
  /** Overrides for tests; an override counts as live. */
  navStateOverride?: NavState;
  telemetryOverride?: NavigationTelemetry;
}

/**
 * The cockpit panel: what the aircraft holds position on, and how well.
 * The mode in use, then the `nav.state` rows (EKF source set, tracking,
 * odometry rate and pose age, obstacle feed, nearest obstacle, GPS). A
 * stale `nav.state` reads as one line instead of old numbers, because a
 * pilot glances at this panel in flight.
 */
export function NavigationHealthPanel(props: NavigationHealthPanelProps): JSX.Element {
  const t = props.ctx.i18n.t;
  const now = useNow();
  const liveNav = useNavState(props.ctx);
  const live = useNavigationTelemetry(props.ctx);
  const navState = props.navStateOverride ?? liveNav.value;
  const navFresh: Freshness = props.navStateOverride ? LIVE : freshnessOf(liveNav.receivedAt, now);
  const telemetry = props.telemetryOverride ?? live.snapshot;
  const telemetryFresh: Freshness = props.telemetryOverride ? LIVE : freshnessOf(live.receivedAt, now);

  if (navFresh.kind !== "live") {
    return (
      <div style={panel} data-testid="vn-health-panel">
        <p style={quiet} role="status" data-testid="vn-health-none">
          {navFresh.kind === "stale"
            ? tr(t, "navigation.health.noStateAge", { s: navFresh.ageS })
            : tr(t, "navigation.health.noState")}
        </p>
      </div>
    );
  }
  const mode = telemetryFresh.kind === "live" ? telemetry.mode : null;
  return (
    <div style={panel} data-testid="vn-health-panel">
      <p style={modeLine} data-testid="vn-health-mode">
        {mode ? tr(t, `navigation.modes.${mode}.label`) : tr(t, "navigation.noData")}
      </p>
      <NavStateCard
        ctx={props.ctx}
        state={navState}
        freshness={navFresh}
        poseAgeMs={telemetryFresh.kind === "live" ? telemetry.poseAgeMs : null}
      />
    </div>
  );
}

const panel: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.5rem",
  padding: "0.5rem",
  background: "var(--vn-bg, #0a0a0a)",
  color: "var(--vn-text, #ededed)",
  minHeight: "100%",
  boxSizing: "border-box",
};
const modeLine: CSSProperties = { margin: 0, fontSize: "0.875rem", fontWeight: 600 };
const quiet: CSSProperties = { margin: 0, fontSize: "0.8125rem", color: "var(--vn-text-3, #94a3b8)" };
