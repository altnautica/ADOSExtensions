import type { CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import { noDataLabel, type Freshness } from "../freshness";
import { DASH, tr, type TFn } from "../i18n";
import type { VisionNavTelemetry } from "../types";

interface Props {
  ctx: PluginContext;
  telemetry: VisionNavTelemetry;
  freshness: Freshness;
}

const IMU_SOURCES = ["mavlink-raw-imu", "mavlink-scaled-imu2", "direct-i2c", "direct-dronecan"];
const SCALE_SOURCES = ["rangefinder", "baro", "gps", "vision", "static"];

/**
 * Sensor health card: camera, IMU, rangefinder, and a diagnostics row
 * when the agent reports a fault. Each row shows the source in words,
 * the live rate / value, and a health pill (green / amber / red /
 * muted). The card is read-only: the camera calibration is the Kalibr
 * `camchain.yaml` the agent reads from the plugin's data directory at
 * start-up, and the IMU is the flight controller's own.
 */
export function SensorsCard({ ctx, telemetry, freshness }: Props): JSX.Element {
  const t = ctx.i18n.t;
  // A stale or missing navigation block names no health: every pill
  // reads "No data" (with the age) instead.
  const live = freshness.kind === "live";
  const gate = (tone: Tone): Tone => (live ? tone : { kind: "muted", label: noDataLabel(t, freshness) });
  return (
    <section style={card} data-testid="vn-sensors-card">
      <h3 style={heading}>{tr(t, "navigation.sensorsCard.title")}</h3>
      <div style={rows}>
        <CameraRow telemetry={telemetry} t={t} gate={gate} />
        <ImuRow telemetry={telemetry} t={t} gate={gate} />
        <RangefinderRow telemetry={telemetry} t={t} gate={gate} />
        <Diagnostics telemetry={telemetry} t={t} />
      </div>
    </section>
  );
}

interface RowProps {
  telemetry: VisionNavTelemetry;
  t: TFn;
  gate: (tone: Tone) => Tone;
}

function CameraRow({ telemetry, t, gate }: RowProps): JSX.Element {
  const loaded = telemetry.cameraIntrinsicsLoaded === true;
  const tone: Tone = loaded
    ? { kind: "ok", label: tr(t, "navigation.sensorsCard.calibrated") }
    : { kind: "warn", label: tr(t, "navigation.sensorsCard.notCalibrated") };
  return (
    <div style={rowContainer} data-testid="vn-sensors-camera">
      <RowLabel
        label={tr(t, "navigation.sensorsCard.camera")}
        sublabel={telemetry.recommendedCameraId ?? DASH}
      />
      <div style={rowRight} title={tr(t, "navigation.sensorsCard.calibrationHint")}>
        <Pill tone={gate(tone)} />
      </div>
    </div>
  );
}

function ImuRow({ telemetry, t, gate }: RowProps): JSX.Element {
  const source = telemetry.imuSource;
  const rate = telemetry.imuRateHz;
  const sourceWords =
    typeof source !== "string"
      ? DASH
      : IMU_SOURCES.includes(source)
        ? tr(t, `navigation.imuSource.${source}`)
        : tr(t, "navigation.imuSource.unknown");
  return (
    <div style={rowContainer} data-testid="vn-sensors-imu">
      <RowLabel label={tr(t, "navigation.sensorsCard.imu")} sublabel={sourceWords} />
      <div style={rowRight}>
        <Metric
          label={tr(t, "navigation.sensorsCard.rate")}
          value={typeof rate === "number" ? tr(t, "navigation.format.hz", { v: Math.round(rate) }) : DASH}
        />
        <Pill tone={gate(syncOffsetTone(telemetry.cameraImuSyncOffsetMs, t))} />
      </div>
    </div>
  );
}

/**
 * The rangefinder pill reports its health, not its wiring: red with a
 * driver error, or with no reading in the mode that needs one; amber
 * with no reading where the scale ladder can stand in; green with a
 * reading. Where it is wired goes in the sublabel.
 */
function rangefinderTone(telemetry: VisionNavTelemetry, t: TFn): Tone {
  const required = telemetry.mode === "optical_flow";
  const topology = telemetry.rangefinderTopology ?? null;
  if (topology === null) {
    return { kind: required ? "error" : "muted", label: tr(t, "navigation.sensorsCard.absent") };
  }
  if (telemetry.rangefinderError) {
    return { kind: "error", label: tr(t, "navigation.sensorsCard.error") };
  }
  if (typeof telemetry.flowDistanceM !== "number") {
    return { kind: required ? "error" : "warn", label: tr(t, "navigation.sensorsCard.noReading") };
  }
  return { kind: "ok", label: tr(t, "navigation.sensorsCard.healthy") };
}

function RangefinderRow({ telemetry, t, gate }: RowProps): JSX.Element {
  const topology = telemetry.rangefinderTopology ?? null;
  const distance = telemetry.flowDistanceM;
  const scale = telemetry.flowScaleSource ?? null;
  return (
    <div style={rowContainer} data-testid="vn-sensors-rangefinder">
      <RowLabel
        label={tr(t, "navigation.sensorsCard.rangefinder")}
        sublabel={topology === null ? DASH : tr(t, `navigation.topology.${topology}`)}
      />
      <div style={rowRight}>
        <Metric
          label={tr(t, "navigation.sensorsCard.scale")}
          value={scale !== null && SCALE_SOURCES.includes(scale) ? tr(t, `navigation.scaleSource.${scale}`) : DASH}
        />
        <Metric
          label={tr(t, "navigation.sensorsCard.distance")}
          value={typeof distance === "number" ? tr(t, "navigation.format.m", { v: distance.toFixed(2) }) : DASH}
        />
        <Pill tone={gate(rangefinderTone(telemetry, t))} testId="vn-rangefinder-pill" />
      </div>
    </div>
  );
}

/** What the agent could not wire or send, in words. */
function diagnosticLines(telemetry: VisionNavTelemetry, t: TFn): string[] {
  const lines: string[] = [];
  if (telemetry.rangefinderError) lines.push(telemetry.rangefinderError);
  if ((telemetry.emitErrors ?? 0) > 0) {
    lines.push(tr(t, "navigation.diagnostics.emitErrors", { n: telemetry.emitErrors ?? 0 }));
  }
  for (const reason of telemetry.degradedReasons ?? []) {
    const at = reason.indexOf(":");
    const kind = at < 0 ? reason : reason.slice(0, at);
    const name = at < 0 ? "" : reason.slice(at + 1);
    if (kind === "mavlink_subscribe_denied") {
      lines.push(tr(t, "navigation.diagnostics.mavlinkDenied", { msg: name }));
    } else if (kind === "event_subscribe_denied") {
      lines.push(tr(t, "navigation.diagnostics.eventDenied", { topic: name }));
    } else {
      lines.push(tr(t, "navigation.diagnostics.other", { reason }));
    }
  }
  return lines;
}

function Diagnostics({ telemetry, t }: { telemetry: VisionNavTelemetry; t: TFn }): JSX.Element | null {
  const lines = diagnosticLines(telemetry, t);
  if (lines.length === 0) return null;
  return (
    <div style={diagnostics} data-testid="vn-diagnostics">
      <span style={rowLabelTop}>{tr(t, "navigation.diagnostics.title")}</span>
      <ul style={diagnosticList}>
        {lines.map((line) => (
          <li key={line}>{line}</li>
        ))}
      </ul>
    </div>
  );
}

interface RowLabelProps {
  label: string;
  sublabel: string;
}

function RowLabel({ label, sublabel }: RowLabelProps): JSX.Element {
  return (
    <div style={rowLabelStack}>
      <span style={rowLabelTop}>{label}</span>
      <span style={rowLabelBottom}>{sublabel}</span>
    </div>
  );
}

interface MetricProps {
  label: string;
  value: string;
}

function Metric({ label, value }: MetricProps): JSX.Element {
  return (
    <span style={metric}>
      <span style={metricLabel}>{label}</span>
      <span style={metricValue}>{value}</span>
    </span>
  );
}

type Tone = {
  kind: "ok" | "warn" | "error" | "muted";
  label: string;
};

function Pill({ tone, testId }: { tone: Tone; testId?: string }): JSX.Element {
  const color = TONE_COLOR[tone.kind];
  return (
    <span style={pill(color)} data-testid={testId} data-kind={tone.kind}>
      <span style={dot(color)} aria-hidden />
      {tone.label}
    </span>
  );
}

const TONE_COLOR: Record<Tone["kind"], string> = {
  ok: "var(--vn-ok, #34d399)",
  warn: "var(--vn-warn, #f59e0b)",
  error: "var(--vn-error, #ef4444)",
  muted: "var(--vn-muted, #6b7280)",
};

function syncOffsetTone(offset: number | null | undefined, t: TFn): Tone {
  if (typeof offset !== "number" || Number.isNaN(offset)) {
    return { kind: "muted", label: DASH };
  }
  const abs = Math.abs(offset);
  if (abs <= 10) return { kind: "ok", label: tr(t, "navigation.syncBand.green") };
  if (abs <= 30) return { kind: "warn", label: tr(t, "navigation.syncBand.yellow") };
  return { kind: "error", label: tr(t, "navigation.syncBand.red") };
}

const card: CSSProperties = {
  background: "var(--vn-surface, rgba(255,255,255,0.02))",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.5rem",
  padding: "0.875rem 1rem",
  color: "var(--vn-text, #ededed)",
  display: "flex",
  flexDirection: "column",
  gap: "0.625rem",
};
const heading: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
  margin: 0,
  color: "var(--vn-text-2, #b4b4b4)",
};
const rows: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.5rem",
};
const rowContainer: CSSProperties = {
  display: "flex",
  alignItems: "center",
  justifyContent: "space-between",
  padding: "0.5rem 0",
  borderBottom: "1px solid var(--vn-border, rgba(255,255,255,0.04))",
};
const rowLabelStack: CSSProperties = {
  display: "flex",
  flexDirection: "column",
};
const rowLabelTop: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
};
const rowLabelBottom: CSSProperties = {
  fontSize: "0.7rem",
  color: "var(--vn-text-muted, #94a3b8)",
};
const rowRight: CSSProperties = {
  display: "flex",
  alignItems: "center",
  gap: "0.625rem",
};
const metric: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  alignItems: "flex-end",
  gap: "0.125rem",
};
const metricLabel: CSSProperties = {
  fontSize: "0.65rem",
  color: "var(--vn-text-muted, #94a3b8)",
};
const metricValue: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
  fontVariantNumeric: "tabular-nums",
};
const diagnostics: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.25rem",
  padding: "0.5rem 0",
};
const diagnosticList: CSSProperties = {
  margin: 0,
  paddingLeft: "1.25rem",
  fontSize: "0.75rem",
  color: "var(--vn-warn, #f59e0b)",
};
const pill = (color: string): CSSProperties => ({
  display: "inline-flex",
  alignItems: "center",
  gap: "0.375rem",
  padding: "0.25rem 0.5rem",
  borderRadius: "999px",
  border: `1px solid ${color}`,
  color,
  fontSize: "0.7rem",
  fontWeight: 600,
});
const dot = (color: string): CSSProperties => ({
  width: "0.5rem",
  height: "0.5rem",
  borderRadius: "50%",
  background: color,
  display: "inline-block",
});
