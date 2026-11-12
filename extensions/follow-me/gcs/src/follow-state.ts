/**
 * Follow-state subscription hook.
 *
 * The agent half publishes its follow read-back on FOLLOW_STATE_TOPIC (a
 * 1 Hz heartbeat at minimum); the host forwards it to the iframe as a
 * one-way event. Each iframe instance owns its own subscription lifecycle
 * (there is no shared store in the sandbox). The raw payload is snake_case
 * off the wire and normalized to the camelCase FollowState here.
 *
 * The hook also reports the report's age so the tab never shows a silent
 * drone's last values as live: the report is stamped with its own `ts_ms`
 * when the host forwards it, else with the time it arrived, and a 1 Hz tick
 * re-renders so the age keeps moving between reports.
 *
 * @license GPL-3.0-or-later
 */

import { useEffect, useState } from "react";

import type { PluginContext } from "@altnautica/plugin-sdk";

import {
  FOLLOW_STATE_TOPIC,
  normalizeFollowState,
  type FollowState,
  type RawFollowState,
} from "./types";

/** A report this old (or older) is stale: values are blanked, age shown. */
export const FOLLOW_STALE_MS = 2000;

export interface FollowReading {
  /** The latest report, or null before the first one. */
  state: FollowState | null;
  /** Milliseconds since the latest report, or null before the first one. */
  ageMs: number | null;
}

/** Subscribe to the agent follow read-back and surface the latest report
 * with its age. */
export function useFollowState(ctx: PluginContext): FollowReading {
  const [latest, setLatest] = useState<{ state: FollowState; atMs: number } | null>(
    null,
  );
  const [nowMs, setNowMs] = useState(() => Date.now());

  useEffect(() => {
    const off = ctx.events.subscribe<RawFollowState>(
      FOLLOW_STATE_TOPIC,
      (raw) => {
        const now = Date.now();
        const ts = raw?.ts_ms;
        const atMs =
          typeof ts === "number" && Number.isFinite(ts) ? Math.min(ts, now) : now;
        setLatest({ state: normalizeFollowState(raw), atMs });
        setNowMs(now);
      },
    );
    return off;
  }, [ctx]);

  useEffect(() => {
    const id = setInterval(() => setNowMs(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);

  if (!latest) return { state: null, ageMs: null };
  return { state: latest.state, ageMs: Math.max(0, nowMs - latest.atMs) };
}
