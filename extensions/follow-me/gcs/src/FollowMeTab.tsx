/**
 * Follow-Me node-detail tab: Specs + live metrics (the custom viz).
 *
 * The plain follow settings (distance, height, gimbal-point, designate
 * camera, HFOV, detector model) are declared in the manifest as native
 * `contributes.parameters`, so the GCS renders those controls itself — this
 * iframe does NOT re-implement them. The tab renders only what a native
 * control can't: the static plugin facts and the honest live read-back the
 * agent publishes (lock state, commanding flag, range, setpoints).
 *
 * @license GPL-3.0-or-later
 */

import { useEffect, useState, type CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { normalizeConfig } from "./config";
import { useFollowState } from "./follow-state";
import { card, labelRow, lockColor, sectionTitle } from "./style";
import {
  DEFAULT_CONFIG,
  holdReasonLabelKey,
  isStaleHold,
  type FollowConfig,
  type FollowState,
} from "./types";

export interface FollowMeTabProps {
  ctx: PluginContext;
  /** Test seam: drive the live metrics without a real subscription. */
  followOverride?: FollowState;
}

export function FollowMeTab({
  ctx,
  followOverride,
}: FollowMeTabProps): JSX.Element {
  const liveFollow = useFollowState(ctx);
  const follow = followOverride ?? liveFollow;
  // Read-only view of the per-drone config so Specs can show the live camera.
  // The settings are edited through the host's native parameter controls, not
  // here, so the tab never writes config.
  const [config, setConfig] = useState<FollowConfig>(DEFAULT_CONFIG);
  const [themeVars, setThemeVars] = useState<Record<string, string>>({});

  useEffect(() => {
    const off = ctx.config.onChange((next) => {
      setConfig(normalizeConfig(next));
    });
    return off;
  }, [ctx]);

  useEffect(() => {
    const off = ctx.theme.onChange((vars) => setThemeVars(vars));
    return off;
  }, [ctx]);

  const t = (key: string): string => ctx.i18n.t(key);

  const container: CSSProperties = {
    display: "flex",
    flexDirection: "column",
    gap: "0.75rem",
    padding: "0.875rem",
    background: "var(--fm-bg, #0b1220)",
    color: "var(--fm-text, #e5e7eb)",
    minHeight: "100%",
    boxSizing: "border-box",
    ...(themeVars as CSSProperties),
  };

  return (
    <div style={container} data-testid="fm-tab">
      <h2 style={{ margin: 0, fontSize: "1rem", fontWeight: 600 }}>
        {t("tab.title")}
      </h2>

      <SpecsSection ctx={ctx} config={config} />

      <MetricsSection ctx={ctx} follow={follow} />
    </div>
  );
}

function SpecsSection({
  ctx,
  config,
}: {
  ctx: PluginContext;
  config: FollowConfig;
}): JSX.Element {
  const t = (key: string): string => ctx.i18n.t(key);
  return (
    <section style={card} data-testid="fm-specs">
      <h3 style={sectionTitle}>{t("specs.title")}</h3>
      <SpecRow label={t("specs.detector")} value={t("specs.detectorValue")} />
      <SpecRow label={t("specs.camera")} value={config.designate_camera} />
      <SpecRow label={t("specs.fcMode")} value={t("specs.fcModeValue")} />
      <SpecRow label={t("specs.resource")} value={t("specs.resourceValue")} />
      <p style={{ margin: "0.4rem 0 0", fontSize: "0.7rem", color: "var(--fm-text-muted, #94a3b8)" }} data-testid="fm-settings-hint">
        {t("specs.settingsHint")}
      </p>
    </section>
  );
}

function MetricsSection({
  ctx,
  follow,
}: {
  ctx: PluginContext;
  follow: FollowState;
}): JSX.Element {
  const t = (key: string): string => ctx.i18n.t(key);
  const lockLabel = follow.lockState
    ? t(`lock.${follow.lockState}`)
    : t("lock.none");
  return (
    <section style={card} data-testid="fm-metrics">
      <h3 style={sectionTitle}>{t("metrics.title")}</h3>
      <div style={labelRow}>
        <span>{t("metrics.lockState")}</span>
        <span
          style={{
            display: "inline-flex",
            alignItems: "center",
            gap: "0.4rem",
            color: lockColor(follow.lockState),
          }}
          data-testid="fm-lock-state"
        >
          <span
            style={{
              width: "0.55rem",
              height: "0.55rem",
              borderRadius: "999px",
              background: lockColor(follow.lockState),
            }}
          />
          {lockLabel}
        </span>
      </div>
      <MetricRow
        label={t("metrics.commanding")}
        value={follow.commanding ? t("metrics.yes") : t("metrics.no")}
        testId="fm-commanding"
      />
      {!follow.commanding && follow.holdReason !== null ? (
        // "Not commanding" on its own reads the same whether the aircraft is
        // sitting disarmed on the ground or its telemetry died mid-follow.
        // Name the gate, and colour the two telemetry faults as faults.
        <div style={labelRow} data-testid="fm-hold-reason-row">
          <span>{t("metrics.holdReason")}</span>
          <span
            style={{
              color: isStaleHold(follow.holdReason)
                ? "var(--fm-error, #ef4444)"
                : "var(--fm-text-muted, #94a3b8)",
            }}
            data-testid="fm-hold-reason"
          >
            {t(holdReasonLabelKey(follow.holdReason))}
          </span>
        </div>
      ) : null}
      <MetricRow
        label={t("metrics.fcArmed")}
        value={follow.fcArmed ? t("metrics.yes") : t("metrics.no")}
        testId="fm-fc-armed"
      />
      <MetricRow
        label={t("metrics.guidedMode")}
        value={follow.fcGuided ? t("metrics.yes") : t("metrics.no")}
        testId="fm-fc-guided"
      />
      <MetricRow
        label={t("metrics.targetId")}
        value={follow.targetId != null ? `#${follow.targetId}` : "—"}
        testId="fm-target-id"
      />
      <MetricRow
        label={t("metrics.range")}
        value={fmtMeters(follow.rangeM)}
        testId="fm-range"
      />
      <MetricRow
        label={t("metrics.distanceSetpoint")}
        value={fmtMeters(follow.distanceSetpointM)}
        testId="fm-distance-setpoint"
      />
      <MetricRow
        label={t("metrics.heightSetpoint")}
        value={fmtMeters(follow.heightSetpointM)}
        testId="fm-height-setpoint"
      />
    </section>
  );
}

function SpecRow({
  label,
  value,
}: {
  label: string;
  value: string;
}): JSX.Element {
  return (
    <div style={labelRow}>
      <span style={{ color: "var(--fm-text-muted, #94a3b8)" }}>{label}</span>
      <span>{value}</span>
    </div>
  );
}

function MetricRow({
  label,
  value,
  testId,
}: {
  label: string;
  value: string;
  testId: string;
}): JSX.Element {
  return (
    <div style={labelRow}>
      <span style={{ color: "var(--fm-text-muted, #94a3b8)" }}>{label}</span>
      <span style={{ fontFamily: "monospace" }} data-testid={testId}>
        {value}
      </span>
    </div>
  );
}

function fmtMeters(v: number | null): string {
  if (v == null || !Number.isFinite(v)) return "—";
  return `${v.toFixed(1)} m`;
}
