/**
 * Wall-clock ticking for "Xs ago" labels and expiring snapshots, and the
 * freshness vocabulary a reading ages through.
 */

import { useEffect, useState } from "react";

/** A reading is stale after this long without an update. */
export const STALE_THRESHOLD_MS = 45_000;

/** A reading is offline (its producer presumed gone) after this long. */
export const OFFLINE_THRESHOLD_MS = 60_000;

export type FreshnessState = "live" | "stale" | "offline" | "unknown";

export interface Freshness {
  state: FreshnessState;
  elapsedMs: number;
  /** "Xs ago" / "Xm Ys ago", or "—" when unknown. */
  label: string;
}

function formatElapsed(ms: number): string {
  const s = Math.max(0, Math.round(ms / 1000));
  if (s < 60) return `${s}s ago`;
  const m = Math.floor(s / 60);
  const rem = s % 60;
  if (m < 60) return rem === 0 ? `${m}m ago` : `${m}m ${rem}s ago`;
  const h = Math.floor(m / 60);
  return `${h}h ${m % 60}m ago`;
}

/** How fresh a reading taken at `updatedAt` is at `now`. */
export function getFreshness(updatedAt: number | null, now: number = Date.now()): Freshness {
  if (updatedAt == null) return { state: "unknown", elapsedMs: 0, label: "—" };
  const elapsedMs = now - updatedAt;
  const state: FreshnessState =
    elapsedMs < STALE_THRESHOLD_MS ? "live" : elapsedMs < OFFLINE_THRESHOLD_MS ? "stale" : "offline";
  return { state, elapsedMs, label: formatElapsed(elapsedMs) };
}

/** The current time, re-rendering the caller every `periodMs`. */
export function useNow(periodMs = 1000): number {
  const [now, setNow] = useState(() => Date.now());
  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), periodMs);
    return () => clearInterval(id);
  }, [periodMs]);
  return now;
}
