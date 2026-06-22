/**
 * Follow-Me drone-detail tab: Specs, Settings, Live metrics.
 *
 * Renders the static plugin facts, the four per-drone settings the agent
 * reads live (distance, height, gimbal-point, designate camera), and the
 * honest live read-back the agent publishes. The settings inputs are
 * disabled while the agent is actively commanding so a follow in progress
 * cannot have its standoff yanked from under it; the operator disarms the
 * skill first.
 *
 * @license GPL-3.0-or-later
 */

import { useEffect, useState, type CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { normalizeConfig, writeConfigKey } from "./config";
import { useFollowState } from "./follow-state";
import { card, inputStyle, labelRow, lockColor, sectionTitle } from "./style";
import {
  DEFAULT_CONFIG,
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

  async function update<K extends keyof FollowConfig>(
    key: K,
    value: FollowConfig[K],
  ): Promise<void> {
    setConfig((prev) => ({ ...prev, [key]: value }));
    await writeConfigKey(ctx, key, value);
  }

  const settingsLocked = follow.commanding;

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

      <section style={card} data-testid="fm-settings">
        <h3 style={sectionTitle}>{t("settings.title")}</h3>
        {settingsLocked ? (
          <p
            style={{
              margin: "0 0 0.5rem 0",
              fontSize: "0.75rem",
              color: lockColor("uncertain"),
            }}
            data-testid="fm-settings-locked"
          >
            {t("settings.lockedWhileCommanding")}
          </p>
        ) : null}
        <NumberField
          label={t("settings.followDistance")}
          value={config.follow_distance_m}
          min={3}
          max={30}
          step={0.5}
          disabled={settingsLocked}
          onCommit={(v) => void update("follow_distance_m", v)}
          testId="fm-follow-distance"
        />
        <NumberField
          label={t("settings.followHeight")}
          value={config.follow_height_m}
          min={0}
          max={20}
          step={0.5}
          disabled={settingsLocked}
          onCommit={(v) => void update("follow_height_m", v)}
          testId="fm-follow-height"
        />
        <div style={labelRow}>
          <span>{t("settings.gimbalPoint")}</span>
          <input
            type="checkbox"
            checked={config.gimbal_point}
            disabled={settingsLocked}
            onChange={(e) => void update("gimbal_point", e.target.checked)}
            data-testid="fm-gimbal-point"
          />
        </div>
        <div style={labelRow}>
          <span>{t("settings.designateCamera")}</span>
          <input
            type="text"
            style={inputStyle}
            value={config.designate_camera}
            disabled={settingsLocked}
            onChange={(e) => void update("designate_camera", e.target.value)}
            data-testid="fm-designate-camera"
          />
        </div>
      </section>

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

function NumberField({
  label,
  value,
  min,
  max,
  step,
  disabled,
  onCommit,
  testId,
}: {
  label: string;
  value: number;
  min: number;
  max: number;
  step: number;
  disabled: boolean;
  onCommit: (v: number) => void;
  testId: string;
}): JSX.Element {
  return (
    <div style={labelRow}>
      <span>{label}</span>
      <input
        type="number"
        style={inputStyle}
        value={value}
        min={min}
        max={max}
        step={step}
        disabled={disabled}
        onChange={(e) => {
          const next = Number(e.target.value);
          if (Number.isFinite(next)) {
            onCommit(clamp(next, min, max));
          }
        }}
        data-testid={testId}
      />
    </div>
  );
}

function fmtMeters(v: number | null): string {
  if (v == null || !Number.isFinite(v)) return "—";
  return `${v.toFixed(1)} m`;
}

function clamp(v: number, min: number, max: number): number {
  return v < min ? min : v > max ? max : v;
}
