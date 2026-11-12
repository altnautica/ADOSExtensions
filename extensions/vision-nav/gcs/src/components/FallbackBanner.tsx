import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { tr } from "../i18n";
import type { VisionNavTelemetry } from "../types";

interface Props {
  ctx: PluginContext;
  telemetry: VisionNavTelemetry;
}

/**
 * Inline banner shown when the estimator is degraded or has failed.
 * Explains what tripped the state in operator-readable terms and
 * suggests a concrete next action — slow the yaw, improve lighting,
 * switch mode, or land. Stays hidden when the estimator is healthy
 * so the card layout does not flicker in normal flight.
 */
export function FallbackBanner({ ctx, telemetry }: Props): JSX.Element | null {
  const t = ctx.i18n.t;
  const state = telemetry.estimatorState;
  if (state !== "degraded" && state !== "failed") {
    return null;
  }
  const isFailed = state === "failed";

  return (
    <section style={banner(isFailed)} data-testid={`vn-fallback-banner-${state}`} role="alert">
      <div style={iconBlock(isFailed)} aria-hidden>
        {isFailed ? "✕" : "!"}
      </div>
      <div style={textBlock}>
        <div style={title}>
          {tr(t, isFailed ? "navigation.fallback.failedTitle" : "navigation.fallback.degradedTitle")}
        </div>
        <div style={body}>
          {tr(t, reasonKey(telemetry))} {tr(t, suggestionKey(telemetry))}
        </div>
      </div>
    </section>
  );
}

const syncDrifted = (telemetry: VisionNavTelemetry): boolean =>
  typeof telemetry.cameraImuSyncOffsetMs === "number" &&
  Math.abs(telemetry.cameraImuSyncOffsetMs) > 30;
const lowQuality = (telemetry: VisionNavTelemetry): boolean =>
  typeof telemetry.flowQuality === "number" && telemetry.flowQuality < 50;

/** The most likely reason, first match only, so the banner stays short;
 * the estimator and sensor cards carry the full context. */
function reasonKey(telemetry: VisionNavTelemetry): string {
  if (telemetry.flowScaleSource === "static") return "navigation.fallback.reasonStaticScale";
  if (lowQuality(telemetry)) return "navigation.fallback.reasonLowQuality";
  if (syncDrifted(telemetry)) return "navigation.fallback.reasonSyncDrift";
  if (telemetry.flowScaleSource === null || telemetry.flowScaleSource === undefined) {
    return "navigation.fallback.reasonNoScale";
  }
  return "navigation.fallback.reasonGeneric";
}

function suggestionKey(telemetry: VisionNavTelemetry): string {
  const scale = telemetry.flowScaleSource;
  if (scale === null || scale === undefined || scale === "static") {
    return "navigation.fallback.suggestionLand";
  }
  if (syncDrifted(telemetry)) return "navigation.fallback.suggestionRecalibrate";
  if (lowQuality(telemetry)) return "navigation.fallback.suggestionSlowYaw";
  return "navigation.fallback.suggestionWatch";
}

const banner = (isFailed: boolean): CSSProperties => ({
  display: "flex",
  alignItems: "flex-start",
  gap: "0.75rem",
  padding: "0.625rem 0.875rem",
  borderRadius: "0.5rem",
  border: `1px solid ${isFailed ? "var(--vn-error, #ef4444)" : "var(--vn-warn, #f59e0b)"}`,
  background: isFailed
    ? "color-mix(in srgb, var(--vn-error, #ef4444) 12%, transparent)"
    : "color-mix(in srgb, var(--vn-warn, #f59e0b) 12%, transparent)",
  color: "var(--vn-text, #ededed)",
});
const iconBlock = (isFailed: boolean): CSSProperties => ({
  width: "1.75rem",
  height: "1.75rem",
  display: "flex",
  alignItems: "center",
  justifyContent: "center",
  borderRadius: "50%",
  background: isFailed ? "var(--vn-error, #ef4444)" : "var(--vn-warn, #f59e0b)",
  color: "white",
  fontWeight: 700,
  fontSize: "0.95rem",
  flexShrink: 0,
});
const textBlock: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.25rem",
};
const title: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
};
const body: CSSProperties = {
  fontSize: "0.75rem",
  color: "var(--vn-text-2, #b4b4b4)",
};
