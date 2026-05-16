import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import type {
  EstimatorState,
  VisionNavTelemetry,
} from "../types";

type TFn = PluginContext["i18n"]["t"];

function tr(t: TFn, key: string, fallback: string): string {
  const result = t(key);
  return result === key ? fallback : result;
}

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
  const state = (telemetry.estimatorState as EstimatorState | undefined) ?? "off";

  if (state !== "degraded" && state !== "failed") {
    return null;
  }

  const isFailed = state === "failed";
  const reason = describeDegradation(telemetry, t);
  const suggestion = suggestionFor(telemetry, t);

  return (
    <section
      style={banner(isFailed)}
      data-testid={`vn-fallback-banner-${state}`}
      role="alert"
    >
      <div style={iconBlock(isFailed)}>{isFailed ? "✕" : "!"}</div>
      <div style={textBlock}>
        <div style={title}>
          {isFailed
            ? tr(
                t,
                "navigation.fallback.failedTitle",
                "Vision estimator failed",
              )
            : tr(
                t,
                "navigation.fallback.degradedTitle",
                "Vision estimator degraded",
              )}
        </div>
        <div style={body}>
          {reason} {suggestion}
        </div>
      </div>
    </section>
  );
}

function describeDegradation(
  telemetry: VisionNavTelemetry,
  t: TFn,
): string {
  // Order the reasons by likelihood; only return the first match so
  // the banner stays short. The estimator card carries the full
  // diagnostic context for operators who want details.
  if (typeof telemetry.flowQuality === "number" && telemetry.flowQuality < 50) {
    return tr(
      t,
      "navigation.fallback.reasonLowQuality",
      "Optical-flow quality dropped below the safe gate.",
    );
  }
  if (
    typeof telemetry.cameraImuSyncOffsetMs === "number" &&
    Math.abs(telemetry.cameraImuSyncOffsetMs) > 30
  ) {
    return tr(
      t,
      "navigation.fallback.reasonSyncDrift",
      "Camera-IMU time sync drifted beyond 30 ms.",
    );
  }
  if (telemetry.flowScaleSource === null || telemetry.flowScaleSource === undefined) {
    return tr(
      t,
      "navigation.fallback.reasonNoScale",
      "No altitude or depth source is healthy for OF scaling.",
    );
  }
  if (
    typeof telemetry.estimatorFeatureCount === "number" &&
    telemetry.estimatorFeatureCount < 20
  ) {
    return tr(
      t,
      "navigation.fallback.reasonFewFeatures",
      "VIO is tracking too few visual features.",
    );
  }
  return tr(
    t,
    "navigation.fallback.reasonGeneric",
    "Estimator inputs are below the safe operating envelope.",
  );
}

function suggestionFor(
  telemetry: VisionNavTelemetry,
  t: TFn,
): string {
  if (telemetry.flowScaleSource === null || telemetry.flowScaleSource === undefined) {
    return tr(
      t,
      "navigation.fallback.suggestionLand",
      "Switch to GPS mode or land.",
    );
  }
  if (
    typeof telemetry.cameraImuSyncOffsetMs === "number" &&
    Math.abs(telemetry.cameraImuSyncOffsetMs) > 30
  ) {
    return tr(
      t,
      "navigation.fallback.suggestionRecalibrate",
      "Re-run the camera-IMU calibration before the next flight.",
    );
  }
  if (typeof telemetry.flowQuality === "number" && telemetry.flowQuality < 50) {
    return tr(
      t,
      "navigation.fallback.suggestionSlowYaw",
      "Slow yaw rate and fly over a more textured surface.",
    );
  }
  return tr(
    t,
    "navigation.fallback.suggestionWatch",
    "Watch closely and prepare to hand back to manual.",
  );
}

const banner = (isFailed: boolean): CSSProperties => ({
  display: "flex",
  alignItems: "flex-start",
  gap: "0.75rem",
  padding: "0.625rem 0.875rem",
  borderRadius: "0.5rem",
  border: `1px solid ${
    isFailed
      ? "var(--vn-error, #ef4444)"
      : "var(--vn-warn, #f59e0b)"
  }`,
  background: isFailed
    ? "color-mix(in srgb, var(--vn-error, #ef4444) 12%, transparent)"
    : "color-mix(in srgb, var(--vn-warn, #f59e0b) 12%, transparent)",
  color: "var(--vn-text, #e5e7eb)",
});
const iconBlock = (isFailed: boolean): CSSProperties => ({
  width: "1.75rem",
  height: "1.75rem",
  display: "flex",
  alignItems: "center",
  justifyContent: "center",
  borderRadius: "50%",
  background: isFailed
    ? "var(--vn-error, #ef4444)"
    : "var(--vn-warn, #f59e0b)",
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
  color: "var(--vn-text-2, #cbd5e1)",
};
