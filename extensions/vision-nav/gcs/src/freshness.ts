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
