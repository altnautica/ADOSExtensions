/**
 * @module use-sessions
 * @description Polls the mounted device's recorded capture sessions and keeps
 * a per-session upload throughput from successive polls, so the table can
 * show a rate and an ETA next to the progress bar.
 */

import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import { useHost } from "../host/context";
import { CaptureClient } from "../lib/agent/capture-client";
import { uploadRate, type CaptureSession } from "../lib/capture/sessions";

export const SESSIONS_POLL_MS = 3000;

export interface SessionsFeed {
  client: CaptureClient;
  sessions: CaptureSession[];
  /** Upload bytes/s per session id, where two polls allow an estimate. */
  rates: Record<string, number>;
  loading: boolean;
  /** The device answered no session list. */
  absent: boolean;
  refresh: () => void;
}

export function useSessions(): SessionsFeed {
  const host = useHost();
  const client = useMemo(() => new CaptureClient(host.agent), [host.agent]);
  const [sessions, setSessions] = useState<CaptureSession[] | null>(null);
  const [rates, setRates] = useState<Record<string, number>>({});
  const [loading, setLoading] = useState(true);
  const last = useRef<Record<string, { bytesSent: number; atMs: number }>>({});
  const alive = useRef(true);

  const read = useCallback(async () => {
    const list = await client.listSessions();
    if (!alive.current) return;
    const now = Date.now();
    const nextRates: Record<string, number> = {};
    for (const s of list ?? []) {
      if (s.upload.state !== "uploading") continue;
      const sample = { bytesSent: s.upload.bytesSent, atMs: now };
      const rate = uploadRate(last.current[s.sessionId] ?? null, sample);
      if (rate !== null) nextRates[s.sessionId] = rate;
      last.current[s.sessionId] = sample;
    }
    setSessions(list);
    setRates(nextRates);
    setLoading(false);
  }, [client]);

  useEffect(() => {
    alive.current = true;
    let inFlight = false;
    const tick = () => {
      if (inFlight) return;
      inFlight = true;
      void read().finally(() => {
        inFlight = false;
      });
    };
    tick();
    const id = setInterval(tick, SESSIONS_POLL_MS);
    return () => {
      alive.current = false;
      clearInterval(id);
    };
  }, [read]);

  return {
    client,
    sessions: sessions ?? [],
    rates,
    loading,
    absent: !loading && sessions === null,
    refresh: () => void read(),
  };
}
