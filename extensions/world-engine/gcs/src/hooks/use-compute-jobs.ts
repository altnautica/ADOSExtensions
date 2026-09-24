/**
 * @module use-compute-jobs
 * @description Polls a compute node's job API (through the extension's server
 * on that node) and returns its jobs plus a client for on-demand reads
 * (outputs) and writes (submit, cancel).
 */

import { useEffect, useMemo, useState } from "react";

import { useNodeAgent } from "../host/context";
import { ComputeAgentClient, type ComputeJob } from "../lib/agent/compute-client";

/** How often to poll the node's job list, in ms. Jobs are slow-moving, so a
 * 2 s cadence is plenty (the engine state machine ticks far slower). */
export const COMPUTE_JOBS_POLL_INTERVAL_MS = 2000;

export interface ComputeJobsState {
  jobs: ComputeJob[];
  /** True until the first poll resolves (the workbench shows a spinner). */
  loading: boolean;
  /** True when the job API is unreachable (404 / transport / host refusal) —
   * the workbench shows a calm "awaiting compute node" state, not an error. */
  unreachable: boolean;
  /** A client for on-demand reads and writes, or null when no node is given. */
  client: ComputeAgentClient | null;
}

/**
 * Poll `nodeDeviceId`'s job API. Inert (client null, empty jobs) when no node
 * is given.
 */
export function useComputeJobs(nodeDeviceId: string | null | undefined): ComputeJobsState {
  const agent = useNodeAgent(nodeDeviceId);
  const client = useMemo(
    () => (agent && nodeDeviceId ? new ComputeAgentClient(nodeDeviceId, agent) : null),
    [agent, nodeDeviceId],
  );
  // The poll result is tagged with the client it came from, so a node switch
  // never surfaces the previous node's jobs while the new poll is in flight.
  const [poll, setPoll] = useState<{
    client: ComputeAgentClient | null;
    jobs: ComputeJob[];
    unreachable: boolean;
  }>({ client: null, jobs: [], unreachable: false });

  useEffect(() => {
    if (!client) return;
    let cancelled = false;
    // One request at a time: a slow node answers after the request deadline,
    // and interval ticks would otherwise stack requests.
    let inFlight = false;
    const pollOnce = async () => {
      if (inFlight) return;
      inFlight = true;
      try {
        const result = await client.listJobs();
        if (cancelled) return;
        setPoll(
          result === null
            ? { client, jobs: [], unreachable: true }
            : { client, jobs: result, unreachable: false },
        );
      } finally {
        inFlight = false;
      }
    };
    void pollOnce();
    const handle = setInterval(() => void pollOnce(), COMPUTE_JOBS_POLL_INTERVAL_MS);
    return () => {
      cancelled = true;
      clearInterval(handle);
    };
  }, [client]);

  const fresh = client !== null && poll.client === client;
  return {
    jobs: fresh ? poll.jobs : [],
    loading: client !== null && !fresh,
    unreachable: fresh ? poll.unreachable : false,
    client,
  };
}
