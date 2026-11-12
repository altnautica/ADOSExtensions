import { describe, expect, it } from "vitest";

import { SampleClock, STALE_AFTER_MS, agentTimestampOf } from "../src/index";

/**
 * Feed a clock the way Mission Control delivers a sidecar topic: a poll every
 * 750 ms with jitter, every fifth tick skipped (a 1500 ms gap), each entry
 * already `pipelineMs` old on the agent's clock, and the agent clock
 * `skewMs` behind the browser. Returns every instant (checked every 50 ms)
 * at which the feed read stale.
 */
function feed(clock: SampleClock, skewMs: number, pipelineMs: number, start = 1_000_000): number[] {
  const staleAt: number[] = [];
  let now = start;
  const end = start + 20_000;
  let tick = 0;
  let nextDelivery = start;
  for (; now <= end; now += 50) {
    if (now >= nextDelivery) {
      clock.ingest(now - pipelineMs - skewMs, now);
      tick += 1;
      const jitter = ((tick * 37) % 200) - 100;
      nextDelivery = now + (tick % 5 === 0 ? 1500 : 750) + jitter;
    }
    if (clock.isStale(now)) staleAt.push(now);
  }
  return staleAt;
}

describe("SampleClock", () => {
  it("uses the shared 2000 ms stale_after_ms", () => {
    expect(STALE_AFTER_MS).toBe(2000);
  });

  it.each([0, 2000, 5000])(
    "stays live through 750 ms polls and skipped ticks with the drone clock %i ms behind",
    (skewMs) => {
      const clock = new SampleClock();
      const stale = feed(clock, skewMs, 1000);
      // Only the first sample may be judged by its (lagging) stamp.
      expect(stale.every((t) => t < 1_000_000 + 750)).toBe(true);
    },
  );

  it("shows an old first sample stale on mount, then live once the run continues", () => {
    const clock = new SampleClock();
    const now = 1_000_000;
    clock.ingest(now - 5000, now);
    expect(clock.isStale(now)).toBe(true);
    expect(clock.ageMs(now)).toBe(5000);
    clock.ingest(now + 750 - 5000 + 750, now + 750);
    expect(clock.isStale(now + 750)).toBe(false);
  });

  it("goes stale 2000 ms after a feed stops", () => {
    const clock = new SampleClock();
    const now = 1_000_000;
    clock.ingest(now - 300, now);
    clock.ingest(now + 450, now + 750);
    expect(clock.isStale(now + 750 + 1999)).toBe(false);
    expect(clock.isStale(now + 750 + 2000)).toBe(true);
  });

  it("judges a feed that resumes after going stale by its stamp again", () => {
    const clock = new SampleClock();
    const now = 1_000_000;
    clock.ingest(now - 1000, now);
    // A newer report, but the run went stale before it arrived.
    expect(clock.ingest(now - 500, now + 3000)).toBe(now - 500);
    expect(clock.isStale(now + 3000)).toBe(true);
  });

  it.each([-3000, 0, 5000])(
    "ignores a report stamped older than the last one (drone clock offset %i ms)",
    (skewMs) => {
      const clock = new SampleClock();
      const stamp = (t: number) => t - 1000 - skewMs;
      let now = 1_000_000;
      clock.ingest(stamp(now), now);
      now += 750;
      clock.ingest(stamp(now), now);
      const lastLive = now;
      const newest = stamp(now);
      // A failed poll: the host falls back to a slower source whose report is
      // stamped 4 s before the last one. It must not replace the newer reading
      // or read live.
      now += 750;
      expect(clock.ingest(newest - 4000, now)).toBeNull();
      expect(clock.sampleAt).toBe(lastLive);
      // The slower source repeats while the faster one is down; the newer
      // reading still goes stale 2000 ms after it arrived.
      expect(clock.ingest(newest - 4000, now + 750)).toBeNull();
      expect(clock.isStale(lastLive + 1999)).toBe(false);
      expect(clock.isStale(lastLive + 2000)).toBe(true);
      // The faster source answers again before then: the run continues live.
      const back = lastLive + 1500;
      expect(clock.ingest(stamp(back), back)).toBe(back);
      expect(clock.isStale(back + 1999)).toBe(false);
    },
  );

  it("costs one sample, not the feed, when the drone clock steps back", () => {
    const clock = new SampleClock();
    let now = 1_000_000;
    let ts = now - 1000;
    clock.ingest(ts, now);
    now += 750;
    ts += 750;
    clock.ingest(ts, now);
    // The drone clock steps back a minute (a time-source correction).
    now += 750;
    ts -= 60_000;
    expect(clock.ingest(ts, now)).toBeNull();
    for (let i = 0; i < 10; i++) {
      now += 750;
      ts += 750;
      expect(clock.ingest(ts, now)).toBe(now);
      expect(clock.isStale(now)).toBe(false);
    }
  });

  it("forgets the last stamp on reset", () => {
    const clock = new SampleClock();
    clock.ingest(5000, 5000);
    clock.reset();
    expect(clock.ingest(1000, 5100)).toBe(1000);
  });

  it("ignores a repeated report and never ages past arrival for a clock running ahead", () => {
    const clock = new SampleClock();
    const now = 1_000_000;
    expect(clock.ingest(now + 9000, now)).toBe(now);
    expect(clock.ingest(now + 9000, now + 1500)).toBeNull();
    expect(clock.isStale(now + 2000)).toBe(true);
  });

  it("is stale before any sample and after reset", () => {
    const clock = new SampleClock();
    expect(clock.isStale(0)).toBe(true);
    clock.ingest(null, 10);
    expect(clock.isStale(10)).toBe(false);
    clock.reset();
    expect(clock.sampleAt).toBeNull();
  });

  it("reads the agent ts_ms from a payload", () => {
    expect(agentTimestampOf({ ts_ms: 5 })).toBe(5);
    expect(agentTimestampOf({ ts_ms: "5" })).toBeNull();
    expect(agentTimestampOf(null)).toBeNull();
  });
});
