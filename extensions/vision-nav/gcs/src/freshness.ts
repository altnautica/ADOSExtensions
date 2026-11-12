import { useEffect, useState } from "react";

import { tr, type TFn } from "./i18n";

/**
 * A feed value counts as live for this long after it arrived: the shared
 * `stale_after_ms` the agent cockpit, Mission Control and the TUI use for
 * `nav.state` and the navigation block. The agent publishes both at 1 Hz.
 */
export const STALE_AFTER_MS = 2000;

/** How current a feed is: live, stale with its age, or nothing received yet. */
export type Freshness =
  | { kind: "live" }
  | { kind: "stale"; ageS: number }
  | { kind: "none" };

export const LIVE: Freshness = { kind: "live" };

export function freshnessOf(receivedAt: number | null, now: number): Freshness {
  if (receivedAt === null) return { kind: "none" };
  const age = now - receivedAt;
  return age >= STALE_AFTER_MS ? { kind: "stale", ageS: Math.floor(age / 1000) } : LIVE;
}

/** The payload's agent `ts_ms` (epoch ms, drone clock), or `null`
 * without one. A repeat of the last one is the same report delivered
 * twice; the stores drop it so it neither resets the age nor adds a
 * trend sample. */
export function agentTsOf(payload: unknown): number | null {
  const ts = (payload as { ts_ms?: unknown } | null)?.ts_ms;
  return typeof ts === "number" && Number.isFinite(ts) ? ts : null;
}

/**
 * When a delivered payload was sampled: the agent's `ts_ms` capped at
 * this clock, so a drone clock running ahead never makes a value look
 * older than its arrival, or the arrival time when it carries none. The
 * host replays the sidecar's last entry on mount whatever its age, so
 * the arrival time alone would show an old value as live.
 */
export function sampleTimeOf(payload: unknown, now: number): number {
  const ts = agentTsOf(payload);
  return ts === null ? now : Math.min(ts, now);
}

/** "No data", with the age once a value has gone stale. */
export function noDataLabel(t: TFn, f: Freshness): string {
  return f.kind === "stale"
    ? tr(t, "navigation.noDataAge", { s: f.ageS })
    : tr(t, "navigation.noData");
}

/** The wall clock, re-read once a second so ages and staleness advance
 * while no new value arrives. */
export function useNow(): number {
  const [now, setNow] = useState(() => Date.now());
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);
  return now;
}
