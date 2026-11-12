import { useEffect, useState, type CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { freshnessOf, LIVE, useNow, type Freshness } from "../freshness";
import { tr } from "../i18n";
import { useEngage, useNavState, useVisionNavTelemetry } from "../store";
import type { FirmwareType, NavState, VisionNavTelemetry } from "../types";

import { ArduPilotVisionParams } from "./ArduPilotVisionParams";
import { BetaflightUnsupported } from "./BetaflightUnsupported";
import { CompanionStatusPill } from "./CompanionStatusPill";
import { EstimatorCard } from "./EstimatorCard";
import { FallbackBanner } from "./FallbackBanner";
import { FlowHealthCard } from "./FlowHealthCard";
import { InavVisionParams } from "./InavVisionParams";
import { ModeCard } from "./ModeCard";
import { NavStateCard } from "./NavStateCard";
import { PreArmStatus } from "./PreArmStatus";
import { Px4VisionParams } from "./Px4VisionParams";
import { SensorsCard } from "./SensorsCard";
import { TelemetryCharts } from "./TelemetryCharts";
import { TourOrchestrator } from "./TourOrchestrator";

export interface NavigationTabProps {
  ctx: PluginContext;
  firmware: FirmwareType;
  /**
   * Optional override for tests that want to drive the UI without
   * pumping the telemetry transport. An override counts as live.
   */
  telemetryOverride?: VisionNavTelemetry;
  /** Optional `nav.state` override for tests. An override counts as live. */
  navStateOverride?: NavState;
}

export function NavigationTab(props: NavigationTabProps): JSX.Element {
  const t = props.ctx.i18n.t;
  const now = useNow();
  const live = useVisionNavTelemetry(props.ctx);
  const liveNav = useNavState(props.ctx);
  const engage = useEngage(props.ctx);
  const [themeVars, setThemeVars] = useState<Record<string, string>>({});
  const [tourReplay, setTourReplay] = useState(false);

  const telemetry = props.telemetryOverride ?? live.snapshot;
  const telemetryFresh: Freshness = props.telemetryOverride
    ? LIVE
    : freshnessOf(live.receivedAt, now);
  const denied = !props.telemetryOverride && live.denied;
  const navState = props.navStateOverride ?? liveNav.value;
  const navFresh: Freshness = props.navStateOverride
    ? LIVE
    : freshnessOf(liveNav.receivedAt, now);
  const engaged = freshnessOf(engage.receivedAt, now).kind === "live" ? engage.value : null;
  const stale = telemetryFresh.kind === "stale";

  useEffect(() => {
    const off = props.ctx.theme.onChange((vars) => {
      setThemeVars(vars);
    });
    return off;
  }, [props.ctx]);

  const container: CSSProperties = {
    display: "flex",
    flexDirection: "column",
    gap: "0.75rem",
    padding: "0.875rem",
    background: "var(--vn-bg, #0a0a0a)",
    color: "var(--vn-text, #ededed)",
    minHeight: "100%",
    boxSizing: "border-box",
    ...(themeVars as CSSProperties),
  };

  return (
    <div style={container} data-testid="vn-navigation-tab">
      <header style={header}>
        <h2 style={title}>{tr(t, "navigation.tabTitle")}</h2>
        <div style={{ display: "flex", alignItems: "center", gap: "0.5rem" }}>
          <button
            type="button"
            onClick={() => setTourReplay(true)}
            style={replayButton}
            data-testid="vn-tour-replay"
          >
            {tr(t, "navigation.tour.replay")}
          </button>
          <CompanionStatusPill t={t} state={telemetry.companionState} freshness={telemetryFresh} />
        </div>
      </header>
      <TourOrchestrator
        ctx={props.ctx}
        telemetry={telemetry}
        replay={tourReplay}
        onClose={() => setTourReplay(false)}
      />
      {telemetryFresh.kind === "stale" ? (
        <p style={staleNotice} role="status" data-testid="vn-feed-stale">
          {tr(t, "navigation.feedStale", { s: telemetryFresh.ageS })}
        </p>
      ) : null}
      <div
        style={stale ? { ...body, opacity: 0.5 } : body}
        data-testid="vn-telemetry-body"
        data-stale={stale ? "true" : "false"}
      >
        {telemetryFresh.kind === "live" ? (
          <FallbackBanner ctx={props.ctx} telemetry={telemetry} />
        ) : null}
        <ModeCard ctx={props.ctx} telemetry={telemetry} freshness={telemetryFresh} engaged={engaged} />
        <SensorsCard ctx={props.ctx} telemetry={telemetry} freshness={telemetryFresh} />
        <EstimatorCard ctx={props.ctx} telemetry={telemetry} freshness={telemetryFresh} />
        <TelemetryCharts ctx={props.ctx} telemetry={telemetry} />
        <FlowHealthCard ctx={props.ctx} telemetry={telemetry} />
        <PreArmStatus ctx={props.ctx} telemetry={telemetry} freshness={telemetryFresh} denied={denied} />
      </div>
      <NavStateCard
        ctx={props.ctx}
        state={navState}
        freshness={navFresh}
        poseAgeMs={telemetryFresh.kind === "live" ? telemetry.poseAgeMs : null}
      />
      {props.firmware === "ardupilot" ? (
        <ArduPilotVisionParams ctx={props.ctx} />
      ) : props.firmware === "px4" ? (
        <Px4VisionParams ctx={props.ctx} fcParams={telemetry.fcParams} />
      ) : props.firmware === "inav" ? (
        <InavVisionParams ctx={props.ctx} />
      ) : props.firmware === "betaflight" ? (
        <BetaflightUnsupported ctx={props.ctx} />
      ) : (
        <section style={unsupportedCard} data-testid="vn-unsupported-firmware">
          {tr(t, "navigation.params.unsupported", { firmware: props.firmware })}
        </section>
      )}
    </div>
  );
}

const header: CSSProperties = {
  display: "flex",
  justifyContent: "space-between",
  alignItems: "center",
  gap: "0.75rem",
};
const title: CSSProperties = {
  margin: 0,
  fontSize: "1rem",
  fontWeight: 600,
};
const replayButton: CSSProperties = {
  background: "transparent",
  border: "none",
  color: "var(--vn-text-muted, #94a3b8)",
  fontSize: "0.8125rem",
  cursor: "pointer",
  padding: "0 0.5rem",
  minHeight: "48px",
  minWidth: "48px",
  textDecoration: "underline",
};
const staleNotice: CSSProperties = {
  margin: 0,
  padding: "0.5rem 0.75rem",
  borderRadius: "0.375rem",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  color: "var(--vn-text-2, #b4b4b4)",
  fontSize: "0.8125rem",
};
const body: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.75rem",
};
const unsupportedCard: CSSProperties = {
  background: "var(--vn-surface, rgba(255,255,255,0.02))",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.5rem",
  padding: "0.875rem 1rem",
  color: "var(--vn-text-3, #94a3b8)",
  fontSize: "0.8125rem",
};
