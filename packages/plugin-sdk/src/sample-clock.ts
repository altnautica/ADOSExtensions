/**
 * Ageing for host-delivered state samples.
 *
 * Hosts attach the agent's `ts_ms` (wall-clock ms on the drone) to plugin
 * state payloads, and replay the last stored entry when a panel mounts,
 * whatever its age. The drone clock can run behind the browser (a board with
 * no RTC or time source), and a sample already spends part of its budget in
 * the agent's write throttle and the host's poll before it arrives. So the
 * drone timestamp is used only to judge the first sample of a run: an old
 * entry delivered on mount reads stale at once. Every later sample whose
 * timestamp moved forward, arriving while the run is still live, is aged from
 * its arrival, so a live feed reads live whatever the clock skew or poll
 * latency. A feed that resumes after going stale is judged by its timestamp
 * again.
 *
 * A sample stamped earlier than the delivery before it is an older report
 * delivered late (a host falling back to a slower source after a failed
 * poll). It is ignored, so it neither replaces the newer reading nor reads
 * live. The stamp is compared with the previous delivery only, not with the
 * newest ever seen, so a drone clock that steps back costs one sample rather
 * than freezing the feed until the clock catches up.
 *
 * @license GPL-3.0-or-later
 */

/** The shared `stale_after_ms`: a sample this old or older is stale. */
export const STALE_AFTER_MS = 2000;

/** The payload's agent `ts_ms` (epoch ms, drone clock), or `null`. */
export function agentTimestampOf(payload: unknown): number | null {
  const ts = (payload as { ts_ms?: unknown } | null)?.ts_ms;
  return typeof ts === "number" && Number.isFinite(ts) ? ts : null;
}

/** Tracks when the latest sample of one feed was taken, for ageing. */
export class SampleClock {
  private lastArrivalMs: number | null = null;
  private lastKey: unknown = undefined;
  private lastTsMs: number | null = null;
  private sampledAt: number | null = null;

  constructor(readonly staleAfterMs: number = STALE_AFTER_MS) {}

  /**
   * Record a delivery that arrived at `now`, carrying the agent timestamp
   * `tsMs` (or `null`). `key` identifies the report (defaults to `tsMs`).
   * Returns `null`, and changes no reading, for a delivery the caller must
   * ignore: one whose non-null key equals the last one (the same report
   * delivered again), or one stamped earlier than the delivery before it (an
   * older report delivered late). Otherwise returns the sample time the age
   * is counted from.
   */
  ingest(tsMs: number | null, now: number, key: unknown = tsMs): number | null {
    if (key !== null && key !== undefined && key === this.lastKey) return null;
    this.lastKey = key;
    const previousTsMs = this.lastTsMs;
    if (tsMs !== null) this.lastTsMs = tsMs;
    if (tsMs !== null && previousTsMs !== null && tsMs < previousTsMs) return null;
    const forward = tsMs === null || previousTsMs === null || tsMs > previousTsMs;
    const continuing =
      forward && this.lastArrivalMs !== null && now - this.lastArrivalMs < this.staleAfterMs;
    this.lastArrivalMs = now;
    this.sampledAt = continuing || tsMs === null ? now : Math.min(tsMs, now);
    return this.sampledAt;
  }

  /** When the latest sample was taken, or `null` before the first. */
  get sampleAt(): number | null {
    return this.sampledAt;
  }

  /** Age of the latest sample at `now`, or `null` before the first. */
  ageMs(now: number): number | null {
    return this.sampledAt === null ? null : Math.max(0, now - this.sampledAt);
  }

  /** True once the latest sample is `staleAfterMs` old (and before any). */
  isStale(now: number): boolean {
    const age = this.ageMs(now);
    return age === null || age >= this.staleAfterMs;
  }

  /** Forget every sample, as if the feed had never reported. */
  reset(): void {
    this.lastArrivalMs = null;
    this.lastKey = undefined;
    this.lastTsMs = null;
    this.sampledAt = null;
  }
}
