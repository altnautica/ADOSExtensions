import { describe, it, expect, vi } from "vitest";

import { createBatteryStore } from "../src/batteryStore";
import { DEFAULT_CONFIG, type BatterySample } from "../src/types";

function mk(over: Partial<BatterySample> = {}): BatterySample {
  return {
    timestampMs: 1000,
    packId: 0,
    cellVoltagesV: [3.9, 3.9, 3.9, 3.9],
    totalVoltageV: 15.6,
    currentA: -10,
    consumedAh: 0,
    consumedWh: 0,
    remainingPercent: 80,
    temperatureC: 30,
    cellCount: 4,
    chemistry: "lipo",
    ...over,
  };
}

describe("batteryStore", () => {
  it("creates a pack on first ingest and updates its latest sample", () => {
    const store = createBatteryStore();
    store.ingest(mk());
    const snap = store.getSnapshot();
    expect(snap.packs).toHaveLength(1);
    const pack = snap.packs[0];
    expect(pack).toBeDefined();
    expect(pack!.latestSample?.totalVoltageV).toBe(15.6);
  });

  it("rotates the ring buffer at the configured limit", () => {
    const store = createBatteryStore(DEFAULT_CONFIG, 5);
    for (let i = 0; i < 12; i++) {
      store.ingest(mk({ timestampMs: 1000 + i * 1000 }));
    }
    const snap = store.getSnapshot();
    expect(snap.packs[0]?.ringBuffer).toHaveLength(5);
    expect(snap.packs[0]?.ringBuffer[0]?.timestampMs).toBe(1000 + 7 * 1000);
  });

  it("notifies subscribers on every ingest and stops after unsubscribe", () => {
    const store = createBatteryStore();
    const listener = vi.fn();
    const stop = store.subscribe(listener);
    store.ingest(mk({ timestampMs: 1000 }));
    store.ingest(mk({ timestampMs: 2000 }));
    expect(listener).toHaveBeenCalledTimes(2);
    stop();
    store.ingest(mk({ timestampMs: 3000 }));
    expect(listener).toHaveBeenCalledTimes(2);
  });

  it("emits anomaly callbacks only on fresh fires, not repeats", () => {
    const store = createBatteryStore();
    const onAnomaly = vi.fn();
    store.onAnomaly(onAnomaly);
    // First sample with cell low fires the anomaly.
    store.ingest(mk({ timestampMs: 1000, cellVoltagesV: [3.9, 3.9, 3.9, 3.4] }));
    // Second sample still in the same condition: no fresh fire.
    store.ingest(mk({ timestampMs: 2000, cellVoltagesV: [3.9, 3.9, 3.9, 3.4] }));
    expect(onAnomaly).toHaveBeenCalledTimes(1);
    const [fired] = onAnomaly.mock.calls[0]!;
    expect((fired as Array<{ ruleId: string }>)[0]?.ruleId).toBe("cell_low");
  });

  it("clears anomalies after the hysteresis window expires", () => {
    const store = createBatteryStore();
    const onAnomaly = vi.fn();
    store.onAnomaly(onAnomaly);
    store.ingest(mk({ timestampMs: 1000, cellVoltagesV: [3.9, 3.9, 3.9, 3.4] }));
    // Condition lifts.
    store.ingest(mk({ timestampMs: 2000, cellVoltagesV: [3.9, 3.9, 3.9, 3.85] }));
    // Hysteresis still active, no clear yet.
    store.ingest(mk({ timestampMs: 5000, cellVoltagesV: [3.9, 3.9, 3.9, 3.85] }));
    // Past 5s window from clearedAtMs (2000), should clear.
    store.ingest(mk({ timestampMs: 8000, cellVoltagesV: [3.9, 3.9, 3.9, 3.85] }));
    const lastCall = onAnomaly.mock.calls[onAnomaly.mock.calls.length - 1]!;
    const cleared = lastCall[1] as Array<{ ruleId: string }>;
    expect(cleared.find((e) => e.ruleId === "cell_low")).toBeDefined();
  });

  it("captures FC alarm STATUSTEXT lines into fcAlarms", () => {
    const store = createBatteryStore();
    store.ingestStatustext({
      timestampMs: 1000,
      severity: 4,
      text: "Battery low: 22.4V",
    });
    store.ingestStatustext({
      timestampMs: 2000,
      severity: 4,
      text: "Aux servos calibrated",
    });
    const snap = store.getSnapshot();
    const fc = snap.packs[0]?.fcAlarms ?? [];
    expect(fc).toHaveLength(1);
    expect(fc[0]?.text).toContain("Battery low");
  });

  it("setConfig recomputes predictive without waiting for the next sample", () => {
    const store = createBatteryStore();
    // Drop 10 percent over 10 seconds (1%/s), so remaining stays well
    // above any plausible target threshold and the math is in the
    // linear region.
    for (let t = 0; t < 10000; t += 1000) {
      store.ingest(mk({ timestampMs: t, remainingPercent: 95 - t / 1000 }));
    }
    const before = store.getSnapshot().packs[0]?.predictive;
    expect(before?.etaSec).toBeGreaterThan(0);
    store.setConfig({
      ...DEFAULT_CONFIG,
      predictive: { windowSeconds: 30, minimumPercent: 50 },
    });
    const after = store.getSnapshot().packs[0]?.predictive;
    expect(after).not.toBeNull();
    expect(after?.etaSec).not.toBe(before?.etaSec);
  });
});
