/**
 * Shared types for the Battery Health Panel.
 *
 * `BatterySample` is the host-normalized telemetry shape; the panel
 * never sees raw MAVLink. `AnomalyEvent` and `PredictiveState` are
 * derived domain objects produced by the rules engine and the
 * predictive math respectively.
 */

export interface BatterySample {
  timestampMs: number;
  packId: number;
  cellVoltagesV: ReadonlyArray<number>;
  totalVoltageV: number;
  currentA: number;
  consumedAh: number;
  consumedWh: number;
  remainingPercent: number;
  temperatureC: number | null;
  cellCount: number;
  chemistry: "lipo" | "liion" | "lifepo4" | "unknown";
}

export interface StatustextMessage {
  timestampMs: number;
  severity: number;
  text: string;
}

export interface ThresholdConfig {
  lowCellVoltageV: number;
  criticalCellVoltageV: number;
  cellDivergenceMv: number;
  voltageDropRateVPerSec: number;
  tempSpikeRateCPerSec: number;
}

export interface PredictiveConfig {
  windowSeconds: number;
  minimumPercent: number;
}

export interface AudioConfig {
  warning: boolean;
  critical: boolean;
}

export interface BatteryHealthConfig {
  thresholds: ThresholdConfig;
  predictive: PredictiveConfig;
  audio: AudioConfig;
}

export type AnomalyRuleId =
  | "cell_low"
  | "cell_critical"
  | "cell_divergence"
  | "voltage_drop"
  | "temp_spike"
  | "predictive_low";

export type AnomalySeverity = "info" | "warning" | "critical";

export interface AnomalyEvent {
  id: string;
  ruleId: AnomalyRuleId;
  packId: number;
  severity: AnomalySeverity;
  title: string;
  body: string;
  firedAtMs: number;
  /** Set when the underlying condition has cleared for the cooldown window. */
  clearedAtMs?: number;
  meta?: Record<string, unknown>;
}

export interface PredictiveState {
  /** Seconds until remainingPercent reaches the configured min, or `null` if idle. */
  etaSec: number | null;
  rateLikely: "idle" | "normal" | "high" | "past";
  meanCurrentA: number;
}

export const DEFAULT_CONFIG: BatteryHealthConfig = {
  thresholds: {
    lowCellVoltageV: 3.5,
    criticalCellVoltageV: 3.3,
    cellDivergenceMv: 50,
    voltageDropRateVPerSec: 0.5,
    tempSpikeRateCPerSec: 5.0,
  },
  predictive: {
    windowSeconds: 30,
    minimumPercent: 25,
  },
  audio: {
    warning: true,
    critical: true,
  },
};
