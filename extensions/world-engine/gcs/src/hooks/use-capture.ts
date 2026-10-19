/**
 * @module use-capture
 * @description Polls the mounted device's capture state (1 s) and coverage
 * (2 s) and hands back a client for the capture actions. `absent` means the
 * device answered but runs no capture service (or refused), so the page shows
 * its calm empty state rather than stale numbers.
 */

import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import { useHost } from "../host/context";
import { AtlasControlClient, type AtlasReadiness } from "../lib/agent/atlas-control-client";
import { CaptureClient } from "../lib/agent/capture-client";
import type { CaptureCoverage, CaptureState } from "../lib/capture/capture-state";

export const CAPTURE_POLL_MS = 1000;
export const COVERAGE_POLL_MS = 2000;
/** Consecutive missed polls before the last good state is marked stale. */
export const STALE_AFTER_MISSES = 3;

export interface CaptureFeed {
  client: CaptureClient;
  /** The last state the device answered (kept through missed polls). */
  state: CaptureState | null;
  coverage: CaptureCoverage | null;
  /** The device's Atlas readiness (capture service running, cameras
   * configured), or null while unknown. */
  readiness: AtlasReadiness | null;
  /** True until the first state poll resolves. */
  loading: boolean;
  /** The device has never answered a capture state. */
  absent: boolean;
  /** The shown state is older than {@link STALE_AFTER_MISSES} missed polls. */
  stale: boolean;
  /** Re-read the state now (after an action). */
  refresh: () => void;
}

export interface PollMemory<T> {
  value: T | null;
  misses: number;
}

/** Fold one poll result in: a reply replaces the value and clears the miss
 * count; a miss keeps the last good value and counts. A single missed poll
 * must not blank the page (and restart every camera preview). */
export function applyPoll<T>(prev: PollMemory<T>, reply: T | null): PollMemory<T> {
  return reply === null ? { value: prev.value, misses: prev.misses + 1 } : { value: reply, misses: 0 };
}

/** Run `tick` now and every `ms`, one request at a time. */
function usePoll(tick: () => Promise<void>, ms: number): void {
  useEffect(() => {
    let inFlight = false;
    const run = () => {
      if (inFlight) return;
      inFlight = true;
      void tick().finally(() => {
        inFlight = false;
      });
    };
    run();
    const id = setInterval(run, ms);
    return () => clearInterval(id);
  }, [tick, ms]);
}

export function useCapture(): CaptureFeed {
  const host = useHost();
  const client = useMemo(() => new CaptureClient(host.agent), [host.agent]);
  const [memory, setMemory] = useState<PollMemory<CaptureState>>({ value: null, misses: 0 });
  const [coverage, setCoverage] = useState<CaptureCoverage | null>(null);
  const control = useMemo(() => new AtlasControlClient(host.agent), [host.agent]);
  const [readiness, setReadiness] = useState<AtlasReadiness | null>(null);
  const [loading, setLoading] = useState(true);
  const alive = useRef(true);
  useEffect(() => {
    alive.current = true;
    return () => {
      alive.current = false;
    };
  }, []);

  const readState = useCallback(async () => {
    const s = await client.getState();
    if (!alive.current) return;
    setMemory((m) => applyPoll(m, s));
    setLoading(false);
  }, [client]);
  const readCoverage = useCallback(async () => {
    const c = await client.getCoverage();
    if (alive.current) setCoverage(c);
  }, [client]);
  const readReadiness = useCallback(async () => {
    const r = await control.getReadiness();
    if (alive.current) setReadiness(r);
  }, [control]);

  usePoll(readState, CAPTURE_POLL_MS);
  usePoll(readCoverage, COVERAGE_POLL_MS);
  usePoll(readReadiness, COVERAGE_POLL_MS);

  return {
    client,
    state: memory.value,
    coverage,
    readiness,
    loading,
    absent: !loading && memory.value === null,
    stale: memory.value !== null && memory.misses >= STALE_AFTER_MISSES,
    refresh: () => void readState(),
  };
}
