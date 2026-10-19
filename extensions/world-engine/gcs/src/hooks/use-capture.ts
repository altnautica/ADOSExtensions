/**
 * @module use-capture
 * @description Polls the mounted device's capture state (1 s) and coverage
 * (2 s) and hands back a client for the capture actions. `absent` means the
 * device answered but runs no capture service (or refused), so the page shows
 * its calm empty state rather than stale numbers.
 */

import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import { useHost } from "../host/context";
import { CaptureClient } from "../lib/agent/capture-client";
import type { CaptureCoverage, CaptureState } from "../lib/capture/capture-state";

export const CAPTURE_POLL_MS = 1000;
export const COVERAGE_POLL_MS = 2000;

export interface CaptureFeed {
  client: CaptureClient;
  state: CaptureState | null;
  coverage: CaptureCoverage | null;
  /** True until the first state poll resolves. */
  loading: boolean;
  /** The device answered no capture state. */
  absent: boolean;
  /** Re-read the state now (after an action). */
  refresh: () => void;
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
  const [state, setState] = useState<CaptureState | null>(null);
  const [coverage, setCoverage] = useState<CaptureCoverage | null>(null);
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
    setState(s);
    setLoading(false);
  }, [client]);
  const readCoverage = useCallback(async () => {
    const c = await client.getCoverage();
    if (alive.current) setCoverage(c);
  }, [client]);

  usePoll(readState, CAPTURE_POLL_MS);
  usePoll(readCoverage, COVERAGE_POLL_MS);

  return {
    client,
    state,
    coverage,
    loading,
    absent: !loading && state === null,
    refresh: () => void readState(),
  };
}
