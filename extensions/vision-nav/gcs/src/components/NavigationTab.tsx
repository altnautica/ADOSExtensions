import { useEffect, useState, type CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { useVisionNavTelemetry } from "../store";
import type { FirmwareType, VisionNavTelemetry } from "../types";

import { ArduPilotVisionParams } from "./ArduPilotVisionParams";
import { CompanionStatusPill } from "./CompanionStatusPill";
import { EkfSourceSwitcher } from "./EkfSourceSwitcher";
import { FlowHealthCard } from "./FlowHealthCard";
import { PreArmStatus } from "./PreArmStatus";
import { Px4VisionParams } from "./Px4VisionParams";

export interface NavigationTabProps {
  ctx: PluginContext;
  firmware: FirmwareType;
  /**
   * Optional override for tests that want to drive the UI without
   * pumping the telemetry transport.
   */
  telemetryOverride?: VisionNavTelemetry;
}

export function NavigationTab(props: NavigationTabProps): JSX.Element {
  const live = useVisionNavTelemetry(props.ctx);
  const telemetry = props.telemetryOverride ?? live;
  const [themeVars, setThemeVars] = useState<Record<string, string>>({});

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
    background: "var(--vn-bg, #0b1220)",
    color: "var(--vn-text, #e5e7eb)",
    minHeight: "100%",
    boxSizing: "border-box",
    ...(themeVars as CSSProperties),
  };
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

  const tabTitle = props.ctx.i18n.t("navigation.tabTitle");
  const renderedTitle = tabTitle === "navigation.tabTitle" ? "Vision Nav" : tabTitle;

  return (
    <div style={container} data-testid="vn-navigation-tab">
      <header style={header}>
        <h2 style={title}>{renderedTitle}</h2>
        <CompanionStatusPill state={telemetry.companionState} />
      </header>
      <FlowHealthCard telemetry={telemetry} />
      <PreArmStatus telemetry={telemetry} />
      <EkfSourceSwitcher firmware={props.firmware} ctx={props.ctx} />
      {props.firmware === "ardupilot" ? (
        <ArduPilotVisionParams />
      ) : props.firmware === "px4" ? (
        <Px4VisionParams />
      ) : (
        <UnsupportedFirmware firmware={props.firmware} />
      )}
    </div>
  );
}

function UnsupportedFirmware({
  firmware,
}: {
  firmware: FirmwareType;
}): JSX.Element {
  const card: CSSProperties = {
    background: "var(--vn-surface, rgba(255,255,255,0.02))",
    border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
    borderRadius: "0.5rem",
    padding: "0.875rem 1rem",
    color: "var(--vn-text-3, #94a3b8)",
    fontSize: "0.8125rem",
  };
  return (
    <section style={card} data-testid="vn-unsupported-firmware">
      Vision navigation parameters are only surfaced for ArduPilot and PX4.
      Detected firmware: {firmware}.
    </section>
  );
}
