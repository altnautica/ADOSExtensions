import type {
  AnomalyEvent,
  AnomalyRuleId,
  BatterySample,
  PredictiveState,
  ThresholdConfig,
} from "./types";

/**
 * Pure rule engine. Each rule takes the previous and current sample
 * (and where relevant the predictive state) and returns an
 * `AnomalyEvent` if the rule fires, otherwise null. The engine
 * collects the firing rules into a list. Hysteresis is the store's
 * job; the engine itself is stateless.
 */

interface RuleContext {
  prev: BatterySample | null;
  curr: BatterySample;
  config: ThresholdConfig;
  predictive: PredictiveState | null;
}

type Rule = (ctx: RuleContext) => AnomalyEvent | null;

const ruleCellLow: Rule = ({ curr, config }) => {
  const min = minDefined(curr.cellVoltagesV);
  if (min === null || min >= config.lowCellVoltageV) return null;
  if (min < config.criticalCellVoltageV) return null;
  return mk("cell_low", curr, "warning", "Cell low", `${min.toFixed(2)} V`);
};

const ruleCellCritical: Rule = ({ curr, config }) => {
  const min = minDefined(curr.cellVoltagesV);
  if (min === null || min >= config.criticalCellVoltageV) return null;
  return mk(
    "cell_critical",
    curr,
    "critical",
    "Cell critical",
    `${min.toFixed(2)} V`,
  );
};

const ruleCellDivergence: Rule = ({ curr, config }) => {
  const min = minDefined(curr.cellVoltagesV);
  const max = maxDefined(curr.cellVoltagesV);
  if (min === null || max === null) return null;
  const divMv = (max - min) * 1000;
  if (divMv <= config.cellDivergenceMv) return null;
  return mk(
    "cell_divergence",
    curr,
    "warning",
    "Cell divergence",
    `${Math.round(divMv)} mV`,
    { divergenceMv: Math.round(divMv) },
  );
};

const ruleVoltageDrop: Rule = ({ prev, curr, config }) => {
  if (!prev) return null;
  const dt = (curr.timestampMs - prev.timestampMs) / 1000;
  if (dt <= 0 || dt > 5) return null;
  const dV = prev.totalVoltageV - curr.totalVoltageV;
  const dropPerSec = dV / dt;
  if (dropPerSec <= config.voltageDropRateVPerSec) return null;
  return mk(
    "voltage_drop",
    curr,
    "warning",
    "Voltage drop",
    `${dropPerSec.toFixed(2)} V/s`,
    { dropPerSec },
  );
};

const ruleTempSpike: Rule = ({ prev, curr, config }) => {
  if (!prev || prev.temperatureC === null || curr.temperatureC === null) {
    return null;
  }
  const dt = (curr.timestampMs - prev.timestampMs) / 1000;
  if (dt <= 0 || dt > 5) return null;
  const dT = curr.temperatureC - prev.temperatureC;
  const ratePerSec = dT / dt;
  if (ratePerSec <= config.tempSpikeRateCPerSec) return null;
  return mk(
    "temp_spike",
    curr,
    "warning",
    "Temperature spike",
    `${ratePerSec.toFixed(1)} C/s`,
    { ratePerSec },
  );
};

const rulePredictiveLow: Rule = ({ curr, predictive }) => {
  if (!predictive) return null;
  if (predictive.rateLikely === "idle" || predictive.rateLikely === "past") {
    return null;
  }
  if (predictive.etaSec === null || predictive.etaSec >= 60) return null;
  return mk(
    "predictive_low",
    curr,
    "warning",
    "Predictive low battery",
    `${predictive.etaSec}s to reserve`,
    { etaSec: predictive.etaSec },
  );
};

const ALL_RULES: ReadonlyArray<Rule> = [
  ruleCellCritical,
  ruleCellLow,
  ruleCellDivergence,
  ruleVoltageDrop,
  ruleTempSpike,
  rulePredictiveLow,
];

export function evaluateAnomalies(
  prev: BatterySample | null,
  curr: BatterySample,
  config: ThresholdConfig,
  predictive: PredictiveState | null,
): ReadonlyArray<AnomalyEvent> {
  const out: AnomalyEvent[] = [];
  for (const rule of ALL_RULES) {
    const event = rule({ prev, curr, config, predictive });
    if (event) out.push(event);
  }
  return out;
}

// Helpers ───────────────────────────────────────────────────────────

function minDefined(values: ReadonlyArray<number>): number | null {
  let min = Infinity;
  let saw = false;
  for (const v of values) {
    if (Number.isFinite(v)) {
      if (v < min) min = v;
      saw = true;
    }
  }
  return saw ? min : null;
}

function maxDefined(values: ReadonlyArray<number>): number | null {
  let max = -Infinity;
  let saw = false;
  for (const v of values) {
    if (Number.isFinite(v)) {
      if (v > max) max = v;
      saw = true;
    }
  }
  return saw ? max : null;
}

function mk(
  ruleId: AnomalyRuleId,
  curr: BatterySample,
  severity: AnomalyEvent["severity"],
  title: string,
  body: string,
  meta?: Record<string, unknown>,
): AnomalyEvent {
  return {
    id: `${ruleId}::${curr.packId}::${curr.timestampMs}`,
    ruleId,
    packId: curr.packId,
    severity,
    title,
    body,
    firedAtMs: curr.timestampMs,
    meta,
  };
}
