/**
 * Follow-Me node-detail tab: Specs + live metrics (the custom viz).
 *
 * The plain follow settings (distance, height, gimbal-point, designate
 * camera, HFOV, detector model) are declared in the manifest as native
 * `contributes.parameters`, so the GCS renders those controls itself — this
 * iframe does NOT re-implement them. The tab renders only what a native
 * control can't: the static plugin facts and the honest live read-back the
 * agent publishes (phase, lock state, commanding flag, range with its
 * uncertainty and cues, height source, re-acquire state, latency). The
 * re-acquire mode itself is a native parameter (rendered as a Strict | Smart
 * toggle); the tab shows the mode in effect.
 *
 * Before the first report, and once the latest report is FOLLOW_STALE_MS
 * old, every live value reads as a dash under a status line that says why
 * (waiting, or stale with the report's age), so a silent drone never looks
 * like a live one.
 *
 * @license GPL-3.0-or-later
 */

import { useEffect, useState, type CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { CAMERAS_CHANGED_TOPIC, cameraNameMap, normalizeConfig } from "./config";
import { FOLLOW_STALE_MS, useFollowState } from "./follow-state";
import {
  card,
  labelRow,
  lockColor,
  mutedLabel,
  phaseColor,
  sectionTitle,
} from "./style";
import {
  DEFAULT_CONFIG,
  holdReasonLabelKey,
  isStaleHold,
  modeSummary,
  type FollowConfig,
  type FollowState,
} from "./types";

type Translate = (key: string, params?: Record<string, string | number>) => string;

const DASH = "—";

export interface FollowMeTabProps {
  ctx: PluginContext;
  /** Test seam: drive the live metrics without a real subscription. The
   * override is treated as a fresh report. */
  followOverride?: FollowState;
}

export function FollowMeTab({
  ctx,
  followOverride,
}: FollowMeTabProps): JSX.Element {
  const live = useFollowState(ctx);
  const reading = followOverride
    ? { state: followOverride, ageMs: 0 }
    : live;
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

  // Camera display names from the host's node roster; absent until the host
  // knows the roster, in which case the About row shows the raw id.
  const [cameraNames, setCameraNames] = useState<ReadonlyMap<string, string>>(
    () => new Map(),
  );
  useEffect(() => {
    const off = ctx.events.subscribe(CAMERAS_CHANGED_TOPIC, (payload) => {
      setCameraNames(cameraNameMap(payload));
    });
    return off;
  }, [ctx]);

  const t: Translate = (key, params) => ctx.i18n.t(key, params);

  const container: CSSProperties = {
    display: "flex",
    flexDirection: "column",
    gap: "0.75rem",
    padding: "0.875rem",
    background: "var(--fm-bg, #0a0a0a)",
    color: "var(--fm-text, #ededed)",
    fontFamily:
      "var(--fm-font, var(--font-sans, system-ui, -apple-system, 'Segoe UI', Roboto, sans-serif))",
    minHeight: "100%",
    boxSizing: "border-box",
    ["--fm-text-muted" as string]: "#94a3b8",
    ...(themeVars as CSSProperties),
  };

  const stale = reading.ageMs != null && reading.ageMs >= FOLLOW_STALE_MS;
  const status =
    reading.state == null
      ? t("status.waiting")
      : stale
        ? t("status.stale", { age: Math.floor((reading.ageMs ?? 0) / 1000) })
        : null;

  return (
    <div style={container} data-testid="fm-tab">
      <h2 style={{ margin: 0, fontSize: "1rem", fontWeight: 600 }}>
        {t("tab.title")}
      </h2>

      <SpecsSection t={t} config={config} cameraNames={cameraNames} />

      <MetricsSection
        t={t}
        follow={stale ? null : reading.state}
        status={status}
      />
    </div>
  );
}

function SpecsSection({
  t,
  config,
  cameraNames,
}: {
  t: Translate;
  config: FollowConfig;
  cameraNames: ReadonlyMap<string, string>;
}): JSX.Element {
  const cameraId = config.designate_camera;
  const cameraName = cameraNames.get(cameraId);
  return (
    <section style={card} data-testid="fm-specs">
      <h3 style={sectionTitle}>{t("specs.title")}</h3>
      <SpecRow label={t("specs.detector")} value={t("specs.detectorValue")} />
      <SpecRow
        label={t("specs.camera")}
        value={
          cameraId === "auto"
            ? t("specs.cameraAuto")
            : cameraName
              ? t("specs.cameraNamed", { name: cameraName, id: cameraId })
              : cameraId
        }
        testId="fm-camera"
      />
      <SpecRow
        label={t("settings.reacquireMode")}
        value={t(`reacquire.${config.reacquire_mode}`)}
      />
      <SpecRow label={t("specs.fcMode")} value={t("specs.fcModeValue")} />
      <SpecRow label={t("specs.resource")} value={t("specs.resourceValue")} />
      <p style={{ margin: "0.4rem 0 0", fontSize: "0.7rem", ...mutedLabel }} data-testid="fm-settings-hint">
        {t("specs.settingsHint")}
      </p>
    </section>
  );
}

function MetricsSection({
  t,
  follow,
  status,
}: {
  t: Translate;
  /** The fresh report, or null when there is none or it is stale. */
  follow: FollowState | null;
  /** Why the values are blank (waiting / stale), or null when live. */
  status: string | null;
}): JSX.Element {
  const f = follow;
  const yesNo = (v: boolean): string => (v ? t("metrics.yes") : t("metrics.no"));
  const meters = (v: number | null): string =>
    v == null || !Number.isFinite(v) ? DASH : t("unit.m", { value: v.toFixed(1) });
  const lockLabel = !f
    ? DASH
    : f.lockState
      ? t(`lock.${f.lockState}`)
      : t("lock.none");
  const orbit = f?.followMode === "orbit";
  return (
    <section style={card} data-testid="fm-metrics">
      <h3 style={sectionTitle}>{t("metrics.title")}</h3>
      {status ? (
        <div style={{ ...labelRow, color: "var(--hud-warn, #f59e0b)" }} data-testid="fm-status">
          {status}
        </div>
      ) : null}
      <div style={labelRow}>
        <span style={mutedLabel}>{t("metrics.phase")}</span>
        <span
          style={{ color: f ? phaseColor(f.phase) : undefined, fontWeight: 600 }}
          data-testid="fm-phase"
        >
          {f ? t(`phase.${f.phase}`) : DASH}
        </span>
      </div>
      <MetricRow
        label={t("metrics.reacquireMode")}
        value={f ? t(`reacquire.${f.reacquireMode}`) : DASH}
        testId="fm-reacquire-mode"
      />
      {f && (f.phase === "coasting" || f.phase === "searching") ? (
        <MetricRow
          label={t("metrics.coastRemaining")}
          value={t("unit.s", { value: f.coastRemainingS.toFixed(1) })}
          testId="fm-coast-remaining"
        />
      ) : null}
      <div style={labelRow}>
        <span style={mutedLabel}>{t("metrics.lockState")}</span>
        <span
          style={{
            display: "inline-flex",
            alignItems: "center",
            gap: "0.4rem",
            color: f ? lockColor(f.lockState) : undefined,
          }}
          data-testid="fm-lock-state"
        >
          {f ? (
            <span
              style={{
                width: "0.55rem",
                height: "0.55rem",
                borderRadius: "999px",
                background: lockColor(f.lockState),
              }}
            />
          ) : null}
          {lockLabel}
        </span>
      </div>
      <MetricRow
        label={t("metrics.commanding")}
        value={f ? yesNo(f.commanding) : DASH}
        testId="fm-commanding"
      />
      {f && !f.commanding && f.holdReason !== null ? (
        // "Not commanding" on its own reads the same whether the aircraft is
        // sitting disarmed on the ground or its telemetry died mid-follow.
        // Name the gate, and colour the two telemetry faults as faults.
        <div style={labelRow} data-testid="fm-hold-reason-row">
          <span style={mutedLabel}>{t("metrics.holdReason")}</span>
          <span
            style={{
              color: isStaleHold(f.holdReason)
                ? "var(--fm-error, #ef4444)"
                : "var(--fm-text-muted, #94a3b8)",
            }}
            data-testid="fm-hold-reason"
          >
            {t(holdReasonLabelKey(f.holdReason))}
          </span>
        </div>
      ) : null}
      <MetricRow
        label={t("metrics.followMode")}
        value={f ? modeSummary(f, f.distanceSetpointM, t) : DASH}
        testId="fm-mode"
      />
      <MetricRow
        label={t("metrics.lease")}
        value={f ? (f.leaseId != null ? t("lease.held") : t("lease.none")) : DASH}
        testId="fm-lease"
      />
      <MetricRow
        label={t("metrics.nudged")}
        value={f ? yesNo(f.nudged) : DASH}
        testId="fm-nudged"
      />
      <MetricRow
        label={t("metrics.fcArmed")}
        value={f ? yesNo(f.fcArmed) : DASH}
        testId="fm-fc-armed"
      />
      <MetricRow
        label={t("metrics.guidedMode")}
        value={f ? yesNo(f.fcGuided) : DASH}
        testId="fm-fc-guided"
      />
      <MetricRow
        label={t("metrics.targetId")}
        value={
          f?.targetId != null ? t("unit.targetId", { value: f.targetId }) : DASH
        }
        testId="fm-target-id"
      />
      <MetricRow
        label={t("metrics.range")}
        value={
          f?.rangeM == null || !Number.isFinite(f.rangeM)
            ? DASH
            : f.rangeSigmaM == null || !Number.isFinite(f.rangeSigmaM)
              ? meters(f.rangeM)
              : t("unit.rangeSigma", {
                  value: f.rangeM.toFixed(1),
                  sigma: f.rangeSigmaM.toFixed(1),
                })
        }
        testId="fm-range"
      />
      <MetricRow
        label={t("metrics.rangeCues")}
        value={
          f?.rangeCues.length
            ? f.rangeCues.map((c) => t(`cue.${c.kind}`)).join(" · ")
            : DASH
        }
        testId="fm-range-cues"
      />
      <MetricRow
        label={t("metrics.agl")}
        value={
          f?.aglM == null
            ? DASH
            : `${meters(f.aglM)} · ${t(`agl.${f.aglSource ?? "baro"}`)}`
        }
        testId="fm-agl"
      />
      <MetricRow
        label={t("metrics.cameraGeometry")}
        value={
          f?.cameraHfovDeg == null ||
          f.cameraMountPitchDeg == null ||
          f.cameraGeometrySource == null
            ? DASH
            : [
                t("unit.deg", { value: f.cameraHfovDeg.toFixed(0) }),
                t("unit.deg", { value: f.cameraMountPitchDeg.toFixed(0) }),
                t(`geometry.${f.cameraGeometrySource}`),
              ].join(" · ")
        }
        testId="fm-camera-geometry"
      />
      <MetricRow
        label={t("metrics.targetSpeed")}
        value={
          f?.targetSpeedMps == null
            ? DASH
            : t("unit.mps", { value: f.targetSpeedMps.toFixed(1) })
        }
        testId="fm-target-speed"
      />
      <MetricRow
        label={t("metrics.latency")}
        value={
          f?.pipelineLatencyMs == null
            ? DASH
            : t("unit.ms", { value: f.pipelineLatencyMs })
        }
        testId="fm-latency"
      />
      <MetricRow
        label={t("metrics.setpointRate")}
        value={f ? t("unit.hz", { value: f.setpointRateHz.toFixed(1) }) : DASH}
        testId="fm-setpoint-rate"
      />
      {orbit ? (
        <MetricRow
          label={t("metrics.orbitRadius")}
          value={meters(f?.orbitRadiusM ?? null)}
          testId="fm-orbit-radius"
        />
      ) : (
        <MetricRow
          label={t("metrics.distanceSetpoint")}
          value={meters(f?.distanceSetpointM ?? null)}
          testId="fm-distance-setpoint"
        />
      )}
      <MetricRow
        label={t("metrics.heightSetpoint")}
        value={meters(f?.heightSetpointM ?? null)}
        testId="fm-height-setpoint"
      />
    </section>
  );
}

function SpecRow({
  label,
  value,
  testId,
}: {
  label: string;
  value: string;
  testId?: string;
}): JSX.Element {
  return (
    <div style={labelRow}>
      <span style={mutedLabel}>{label}</span>
      <span data-testid={testId}>{value}</span>
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
      <span style={mutedLabel}>{label}</span>
      <span style={{ fontFamily: "monospace" }} data-testid={testId}>
        {value}
      </span>
    </div>
  );
}
