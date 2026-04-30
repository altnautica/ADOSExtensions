import type {
  BatterySample,
  PredictiveConfig,
  PredictiveState,
} from "./types";

/**
 * Compute time-to-min from a window of recent battery samples.
 *
 * Pure function. The math is intentionally simple: average percent-drop
 * rate over the window, project forward to `targetMinPercent`. Good
 * within ~10% over the linear region of a LiPo discharge curve. A
 * Coulomb-counted model is reserved for v1.1.
 *
 * Returns `null` when there is not enough data to compute a rate
 * (single sample, sub-second window). Returns an `idle` state when the
 * pack is idling or charging (non-positive percent drop). Returns a
 * `past` state when remaining is already at or below the target.
 */
export function computePredictive(
  samples: ReadonlyArray<BatterySample>,
  config: PredictiveConfig,
): PredictiveState | null {
  if (samples.length < 2) return null;
  const windowMs = config.windowSeconds * 1000;
  const lastSample = samples[samples.length - 1];
  if (!lastSample) return null;
  const windowStartMs = lastSample.timestampMs - windowMs;
  const window = samples.filter((s) => s.timestampMs >= windowStartMs);
  if (window.length < 2) return null;

  const first = window[0];
  const last = window[window.length - 1];
  if (!first || !last) return null;
  const elapsedSec = (last.timestampMs - first.timestampMs) / 1000;
  if (elapsedSec < 1) return null;

  const meanCurrentA = meanAbsCurrent(window);
  const dropPercent = first.remainingPercent - last.remainingPercent;
  const dropPerSec = dropPercent / elapsedSec;

  if (dropPerSec <= 0) {
    return { etaSec: null, rateLikely: "idle", meanCurrentA };
  }

  const remainingToMin = last.remainingPercent - config.minimumPercent;
  if (remainingToMin <= 0) {
    return { etaSec: 0, rateLikely: "past", meanCurrentA };
  }

  const etaSec = remainingToMin / dropPerSec;
  return {
    etaSec: Math.round(etaSec),
    rateLikely: dropPerSec > 0.5 ? "high" : "normal",
    meanCurrentA,
  };
}

function meanAbsCurrent(samples: ReadonlyArray<BatterySample>): number {
  if (samples.length === 0) return 0;
  let sum = 0;
  for (const s of samples) sum += Math.abs(s.currentA);
  return sum / samples.length;
}
