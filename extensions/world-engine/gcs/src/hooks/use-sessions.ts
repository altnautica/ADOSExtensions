/**
 * @module use-sessions
 * @description Polls the mounted device's recorded capture sessions and keeps
 * a per-session upload throughput from successive polls, so the table can
 * show a rate and an ETA next to the progress bar. A missed poll keeps the
 * last good list (and any open viewer) and only marks it stale.
 */

import { useCallback, useEffect, useMemo, useRef, useState } from "react";

import { useHost } from "../host/context";
import { CaptureClient } from "../lib/agent/capture-client";
import { uploadRate, type CaptureSession } from "../lib/capture/sessions";
import { applyPoll, STALE_AFTER_MISSES, type PollMemory } from "./use-capture";

export const SESSIONS_POLL_MS = 3000;

type UploadSample = { bytesSent: number; atMs: number };

export interface SessionsPoll {
  memory: PollMemory<CaptureSession[]>;
  rates: Record<string, number>;
  samples: Record<string, UploadSample>;
}

export const EMPTY_SESSIONS_POLL: SessionsPoll = { memory: { value: null, misses: 0 }, rates: {}, samples: {} };

/** Fold one poll in: a reply replaces the list and its upload rates; a miss
 * keeps the last good list and rates and counts the miss. */
export function applySessionsPoll(prev: SessionsPoll, list: CaptureSession[] | null, nowMs: number): SessionsPoll {
  const memory = applyPoll(prev.memory, list);
  if (list === null) return { ...prev, memory };
  const rates: Record<string, number> = {};
  const samples = { ...prev.samples };
  for (const s of list) {
    if (s.upload.state !== "uploading") continue;
    const sample = { bytesSent: s.upload.bytesSent, atMs: nowMs };
    const rate = uploadRate(samples[s.sessionId] ?? null, sample);
    if (rate !== null) rates[s.sessionId] = rate;
    samples[s.sessionId] = sample;
  }
  return { memory, rates, samples };
}

export interface SessionsFeed {
  client: CaptureClient;
  sessions: CaptureSession[];
  /** Upload bytes/s per session id, where two polls allow an estimate. */
  rates: Record<string, number>;
  loading: boolean;
  /** The device has never answered a session list. */
  absent: boolean;
  /** The shown list is older than {@link STALE_AFTER_MISSES} missed polls. */
  stale: boolean;
  refresh: () => void;
}

export function useSessions(): SessionsFeed {
  const host = useHost();
  const client = useMemo(() => new CaptureClient(host.agent), [host.agent]);
  const [poll, setPoll] = useState<SessionsPoll>(EMPTY_SESSIONS_POLL);
  const [loading, setLoading] = useState(true);
  const alive = useRef(true);

  const read = useCallback(async () => {
    const list = await client.listSessions();
    if (!alive.current) return;
    const now = Date.now();
    setPoll((p) => applySessionsPoll(p, list, now));
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
    sessions: poll.memory.value ?? [],
    rates: poll.rates,
    loading,
    absent: !loading && poll.memory.value === null,
    stale: poll.memory.value !== null && poll.memory.misses >= STALE_AFTER_MISSES,
    refresh: () => void read(),
  };
}
