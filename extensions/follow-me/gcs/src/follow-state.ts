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
 * drone's last values as live. The SDK's SampleClock does the ageing: the
 * report's own `ts_ms` judges only the first report of a run (an old report
 * replayed on mount reads stale at once), and later reports age from their
 * arrival, so a drone clock running behind or a slow poll never makes a
 * live feed read stale. The same report delivered again is ignored, and so
 * is a report stamped earlier than the one before it. A 1 Hz
 * tick keeps the age moving between reports, and a timer fires at the
 * moment the latest report turns stale.
 *
 * @license GPL-3.0-or-later
 */

import { useEffect, useRef, useState } from "react";

import {
  agentTimestampOf,
  SampleClock,
  STALE_AFTER_MS,
  type PluginContext,
} from "@altnautica/plugin-sdk";

import {
  FOLLOW_STATE_TOPIC,
  normalizeFollowState,
  type FollowState,
  type RawFollowState,
} from "./types";

/** A report this old (or older) is stale: values are blanked, age shown. */
export const FOLLOW_STALE_MS = STALE_AFTER_MS;

export interface FollowReading {
  /** The latest report, or null before the first one. */
  state: FollowState | null;
  /** Milliseconds since the latest report, or null before the first one. */
  ageMs: number | null;
}

/** Subscribe to the agent follow read-back and surface the latest report
 * with its age. */
export function useFollowState(ctx: PluginContext): FollowReading {
  const clockRef = useRef<SampleClock | null>(null);
  if (clockRef.current === null) clockRef.current = new SampleClock();
  const clock = clockRef.current;
  const [state, setState] = useState<FollowState | null>(null);
  const [nowMs, setNowMs] = useState(() => Date.now());

  useEffect(() => {
    clock.reset();
    const off = ctx.events.subscribe<RawFollowState>(
      FOLLOW_STATE_TOPIC,
      (raw) => {
        const now = Date.now();
        if (clock.ingest(agentTimestampOf(raw), now) === null) return;
        setState(normalizeFollowState(raw));
        setNowMs(now);
      },
    );
    return off;
  }, [ctx, clock]);

  useEffect(() => {
    const id = setInterval(() => setNowMs(Date.now()), 1000);
    return () => clearInterval(id);
  }, []);

  const sampleAt = clock.sampleAt;
  useEffect(() => {
    if (sampleAt === null) return;
    const untilStale = sampleAt + FOLLOW_STALE_MS - Date.now();
    if (untilStale <= 0) return;
    const id = setTimeout(() => setNowMs(Date.now()), untilStale);
    return () => clearTimeout(id);
  }, [sampleAt, nowMs]);

  if (state === null) return { state: null, ageMs: null };
  return { state, ageMs: clock.ageMs(nowMs) };
}
