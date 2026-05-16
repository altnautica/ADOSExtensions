import { useState, type CSSProperties } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import type { VisionNavTelemetry } from "../types";
import { CalibrationWizard } from "./CalibrationWizard";

// The plugin SDK's i18n.t() returns the key itself when no host
// translation is available. ``tr`` substitutes a literal fallback so
// the call sites stay readable while still honouring locale overrides.
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
 * Three-row sensor health card: camera, IMU, rangefinder. Each row
 * shows the source identity, the live rate / value, and a colour
 * indicator (green / yellow / red / muted) that the operator reads
 * at a glance. The Calibrate CTA on the camera row opens the guided
 * calibration wizard.
 */
export function SensorsCard({ ctx, telemetry }: Props): JSX.Element {
  const t = ctx.i18n.t;
  const [wizardOpen, setWizardOpen] = useState(false);

  return (
    <section style={card} data-testid="vn-sensors-card">
      <h3 style={heading}>{tr(t, "navigation.sensorsCard.title", "Sensors")}</h3>
      <div style={rows}>
        <CameraRow
          telemetry={telemetry}
          t={t}
          onCalibrate={() => setWizardOpen(true)}
        />
        <ImuRow telemetry={telemetry} t={t} />
        <RangefinderRow telemetry={telemetry} t={t} />
      </div>
      {wizardOpen ? (
        <CalibrationWizard
          ctx={ctx}
          telemetry={telemetry}
          onClose={() => setWizardOpen(false)}
        />
      ) : null}
    </section>
  );
}

interface RowProps {
  telemetry: VisionNavTelemetry;
  t: PluginContext["i18n"]["t"];
}

interface CameraRowProps extends RowProps {
  onCalibrate: () => void;
}

function CameraRow({
  telemetry,
  t,
  onCalibrate,
}: CameraRowProps): JSX.Element {
  const device = telemetry.recommendedCameraId ?? "—";
  const loaded = telemetry.cameraIntrinsicsLoaded === true;
  const tone: Tone = loaded
    ? { kind: "ok", label: tr(t, "navigation.sensorsCard.calibrated", "Calibrated") }
    : {
        kind: "warn",
        label: tr(t, "navigation.sensorsCard.notCalibrated", "Not calibrated"),
      };

  return (
    <div style={rowContainer} data-testid="vn-sensors-camera">
      <RowLabel
        label={tr(t, "navigation.sensorsCard.camera", "Camera")}
        sublabel={device}
      />
      <div style={rowRight}>
        <Pill tone={tone} />
        <button
          type="button"
          style={cta(!loaded)}
          title={tr(t,
            "navigation.sensorsCard.calibrateHint",
            "Open the guided calibration wizard. " +
              "Captures frames + IMU motion, solves intrinsics + timeshift, applies the result.",
          )}
          data-testid="vn-camera-calibrate-cta"
          onClick={onCalibrate}
        >
          {loaded
            ? tr(t, "navigation.sensorsCard.recalibrate", "Recalibrate")
            : tr(t, "navigation.sensorsCard.calibrate", "Calibrate")}
        </button>
      </div>
    </div>
  );
}

function ImuRow({ telemetry, t }: RowProps): JSX.Element {
  const source = telemetry.imuSource ?? "—";
  const rate = telemetry.imuRateHz;
  const offset = telemetry.cameraImuSyncOffsetMs;
  const syncTone = syncOffsetTone(offset, t);
  return (
    <div style={rowContainer} data-testid="vn-sensors-imu">
      <RowLabel
        label={tr(t, "navigation.sensorsCard.imu", "IMU")}
        sublabel={typeof source === "string" ? source : "—"}
      />
      <div style={rowRight}>
        <Metric
          label={tr(t, "navigation.sensorsCard.rate", "Rate")}
          value={
            typeof rate === "number" ? `${Math.round(rate)} Hz` : "—"
          }
        />
        <Pill tone={syncTone} />
      </div>
    </div>
  );
}

function RangefinderRow({ telemetry, t }: RowProps): JSX.Element {
  const topology = telemetry.rangefinderTopology ?? null;
  const distance = telemetry.flowDistanceM;
  const scale = telemetry.flowScaleSource ?? null;
  let tone: Tone;
  if (topology === null) {
    tone = {
      kind: "muted",
      label: tr(t, "navigation.sensorsCard.absent", "Absent"),
    };
  } else {
    tone = {
      kind: "ok",
      label: topology === "companion" ? "Companion" : topology === "fc" ? "FC" : "Both",
    };
  }
  return (
    <div style={rowContainer} data-testid="vn-sensors-rangefinder">
      <RowLabel
        label={tr(t, "navigation.sensorsCard.rangefinder", "Rangefinder")}
        sublabel={
          scale
            ? tr(t, `navigation.scaleSource.${scale}`, scale)
            : tr(t, "navigation.sensorsCard.noScale", "No scale source")
        }
      />
      <div style={rowRight}>
        <Metric
          label={tr(t, "navigation.sensorsCard.distance", "Distance")}
          value={
            typeof distance === "number"
              ? `${distance.toFixed(2)} m`
              : "—"
          }
        />
        <Pill tone={tone} />
      </div>
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

function Pill({ tone }: { tone: Tone }): JSX.Element {
  const color = TONE_COLOR[tone.kind];
  return (
    <span style={pill(color)}>
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

function syncOffsetTone(
  offset: number | undefined,
  t: PluginContext["i18n"]["t"],
): Tone {
  if (typeof offset !== "number" || Number.isNaN(offset)) {
    return { kind: "muted", label: tr(t, "navigation.syncBand.unknown", "—") };
  }
  const abs = Math.abs(offset);
  if (abs <= 10) {
    return {
      kind: "ok",
      label: tr(t, "navigation.syncBand.green", "Sync ≤10 ms"),
    };
  }
  if (abs <= 30) {
    return {
      kind: "warn",
      label: tr(t, "navigation.syncBand.yellow", "Sync ≤30 ms"),
    };
  }
  return {
    kind: "error",
    label: tr(t, "navigation.syncBand.red", "Sync >30 ms"),
  };
}

const card: CSSProperties = {
  background: "var(--vn-surface, rgba(255,255,255,0.02))",
  border: "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.5rem",
  padding: "0.875rem 1rem",
  color: "var(--vn-text, #e5e7eb)",
  display: "flex",
  flexDirection: "column",
  gap: "0.625rem",
};
const heading: CSSProperties = {
  fontSize: "0.8125rem",
  fontWeight: 600,
  margin: 0,
  color: "var(--vn-text-2, #cbd5e1)",
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
const cta = (highlight: boolean): CSSProperties => ({
  padding: "0.25rem 0.5rem",
  background: highlight
    ? "var(--vn-accent, #2563eb)"
    : "var(--vn-surface-2, rgba(255,255,255,0.06))",
  color: "var(--vn-text, #e5e7eb)",
  border: highlight
    ? "1px solid var(--vn-accent, #2563eb)"
    : "1px solid var(--vn-border, rgba(255,255,255,0.08))",
  borderRadius: "0.25rem",
  fontSize: "0.7rem",
  fontWeight: 600,
  cursor: "pointer",
});
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
const errorText: CSSProperties = {
  fontSize: "0.75rem",
  margin: 0,
  color: "var(--vn-error, #ef4444)",
};
