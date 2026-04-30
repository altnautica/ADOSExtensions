import { evaluateAnomalies } from "./anomalyRules";
import { computePredictive } from "./predictive";
import {
  DEFAULT_CONFIG,
  type AnomalyEvent,
  type AnomalyRuleId,
  type BatteryHealthConfig,
  type BatterySample,
  type PredictiveState,
  type StatustextMessage,
} from "./types";

const DEFAULT_BUFFER_LIMIT = 3000;
/** How long an anomaly stays in the live list once its rule no longer fires. */
const HYSTERESIS_MS = 5000;

export interface PackState {
  packId: number;
  ringBuffer: BatterySample[];
  latestSample: BatterySample | null;
  predictive: PredictiveState | null;
  /** Live anomalies, keyed by rule id within the pack. */
  liveAnomalies: Map<AnomalyRuleId, AnomalyEvent>;
  /** Anomalies fired since the panel mounted. Newest-first. */
  history: AnomalyEvent[];
  fcAlarms: StatustextMessage[];
}

export interface BatteryStoreSnapshot {
  packs: ReadonlyArray<PackState>;
  config: BatteryHealthConfig;
}

export type AnomalyListener = (
  fired: ReadonlyArray<AnomalyEvent>,
  cleared: ReadonlyArray<AnomalyEvent>,
) => void;

export interface BatteryStore {
  ingest(sample: BatterySample): void;
  ingestStatustext(msg: StatustextMessage): void;
  setConfig(next: BatteryHealthConfig): void;
  getSnapshot(): BatteryStoreSnapshot;
  subscribe(listener: () => void): () => void;
  /** Notified when anomalies fire or clear. Used to bridge to the host. */
  onAnomaly(listener: AnomalyListener): () => void;
}

export function createBatteryStore(
  initialConfig: BatteryHealthConfig = DEFAULT_CONFIG,
  bufferLimit: number = DEFAULT_BUFFER_LIMIT,
): BatteryStore {
  const packs = new Map<number, PackState>();
  let config: BatteryHealthConfig = initialConfig;
  const listeners = new Set<() => void>();
  const anomalyListeners = new Set<AnomalyListener>();

  function getOrCreatePack(packId: number): PackState {
    const existing = packs.get(packId);
    if (existing) return existing;
    const pack: PackState = {
      packId,
      ringBuffer: [],
      latestSample: null,
      predictive: null,
      liveAnomalies: new Map(),
      history: [],
      fcAlarms: [],
    };
    packs.set(packId, pack);
    return pack;
  }

  function notify(): void {
    for (const l of listeners) l();
  }

  return {
    ingest(sample) {
      const pack = getOrCreatePack(sample.packId);
      const prev = pack.latestSample;
      pack.ringBuffer.push(sample);
      if (pack.ringBuffer.length > bufferLimit) {
        pack.ringBuffer.shift();
      }
      pack.latestSample = sample;
      pack.predictive = computePredictive(
        pack.ringBuffer,
        config.predictive,
      );

      const fired = evaluateAnomalies(
        prev,
        sample,
        config.thresholds,
        pack.predictive,
      );
      const firedRules = new Set(fired.map((e) => e.ruleId));

      // Update the live map: replace existing rows, drop those whose
      // condition no longer fires AND whose hysteresis window expired.
      const cleared: AnomalyEvent[] = [];
      for (const [ruleId, current] of pack.liveAnomalies) {
        if (firedRules.has(ruleId)) continue;
        if (
          current.clearedAtMs === undefined ||
          sample.timestampMs - current.clearedAtMs < HYSTERESIS_MS
        ) {
          if (current.clearedAtMs === undefined) {
            current.clearedAtMs = sample.timestampMs;
          }
          continue;
        }
        pack.liveAnomalies.delete(ruleId);
        cleared.push(current);
      }
      const freshFired: AnomalyEvent[] = [];
      for (const event of fired) {
        const existing = pack.liveAnomalies.get(event.ruleId);
        if (existing) {
          existing.body = event.body;
          existing.clearedAtMs = undefined;
          continue;
        }
        pack.liveAnomalies.set(event.ruleId, event);
        pack.history.unshift(event);
        if (pack.history.length > 200) pack.history.pop();
        freshFired.push(event);
      }
      if (freshFired.length > 0 || cleared.length > 0) {
        for (const l of anomalyListeners) l(freshFired, cleared);
      }
      notify();
    },
    ingestStatustext(msg) {
      // Match common FC battery alarm strings. The exact pack id is
      // not reliably encoded so we attach to pack 0 by default.
      if (!/(battery|cell|voltage)/i.test(msg.text)) return;
      const pack = getOrCreatePack(0);
      pack.fcAlarms.unshift(msg);
      if (pack.fcAlarms.length > 50) pack.fcAlarms.pop();
      notify();
    },
    setConfig(next) {
      config = next;
      // Recompute predictive state against existing buffers so the UI
      // and the anomaly engine see the new thresholds without a
      // round-trip through the next ingest.
      for (const pack of packs.values()) {
        pack.predictive = computePredictive(
          pack.ringBuffer,
          next.predictive,
        );
      }
      notify();
    },
    getSnapshot() {
      return {
        packs: Array.from(packs.values()),
        config,
      };
    },
    subscribe(listener) {
      listeners.add(listener);
      return () => {
        listeners.delete(listener);
      };
    },
    onAnomaly(listener) {
      anomalyListeners.add(listener);
      return () => {
        anomalyListeners.delete(listener);
      };
    },
  };
}
