/**
 * @module use-node-model
 * @description What the paired reconstruction node has built from this
 * device's sessions. Resolves the paired compute / workstation node, polls
 * its job API (`use-compute-jobs`, through the Reconstruct extension), and
 * resolves the newest completed reconstruction for a session, correlated by
 * `session_id`, the key the compute job and the capture state share.
 *
 * The node is not device-scoped (a session id is not derived from the
 * capturing device), so the session is the correlation key: the Model view
 * passes the operator's selected session, or null to resolve the newest
 * completed reconstruction on the node (its session selector disambiguates
 * when more than one device reconstructed there).
 */

import { useEffect, useMemo, useState } from "react";

import { pickSplatOutput, viewerForKind, type ModelViewer } from "@altnautica/mapping-ui/components/model/viewer-types";
import type { ComputeAgentClient, ComputeOutput } from "@altnautica/mapping-ui/lib/agent/compute-client";
import type { ArtifactSource } from "@altnautica/mapping-ui/lib/net/artifact-source";
import { isComputeProfile } from "@altnautica/mapping-ui/lib/nodes/workstation-provisioning";
import { useComputeJobs } from "@altnautica/mapping-ui/hooks/use-compute-jobs";
import { usePairedNodes } from "@altnautica/mapping-ui/hooks/use-paired-nodes";

/** A completed reconstruction session on the compute node (a selector entry). */
export interface ModelSession {
  /** The capturing session id (the correlation key). */
  sessionId: string;
  /** The newest completed reconstruct job for this session. */
  jobId: string;
  /** When that job was created (epoch ms). */
  createdMs: number;
}

/** What the node has for the requested session. */
export type ModelStatus =
  /** No compute node is paired. */
  | "inactive"
  /** A compute node is paired but its job API is unreachable (or this
   * module holds no grant to reach the Reconstruct extension). */
  | "unreachable"
  /** The compute node is reachable but has no completed reconstruction for the
   * requested session yet. The caller shows a "building…" state. */
  | "building"
  /** A completed reconstruction is resolved (`artifact` + `viewerHint` set). */
  | "ready";

export interface NodeModel {
  /** Whether a workstation / compute node is paired at all (independent of
   * reachability). Drives the "pair a compute node" guidance vs the generic
   * empty state. */
  hasComputeNode: boolean;
  status: ModelStatus;
  /** The resolved artifact when `status === "ready"`, else null. */
  artifact: ArtifactSource | null;
  /** Viewer derived from the artifact kind when ready, else null. */
  viewerHint: ModelViewer | null;
  /** The reconstruction backend of the resolved artifact, or null when none
   * is resolved or the node names none. */
  backend: string | null;
  /** The resolved compute output and its job's other outputs (for its
   * placement sidecar), or null / [] when none is resolved. */
  output: ComputeOutput | null;
  outputs: ComputeOutput[];
  /** Completed reconstruction sessions on the node, newest-first (selector). */
  sessions: ModelSession[];
  /** The device id of the resolved reconstructor node, or null when none is
   * paired. */
  computeNodeDeviceId: string | null;
  /** A client for the resolved reconstructor node (for on-demand submits like
   * "Reconstruct now"), or null when none is paired. Reuses the job-poll
   * client so there is no second poll loop. */
  computeClient: ComputeAgentClient | null;
}

export interface NodeModelParams {
  /** The session to resolve, or null for the newest completed
   * reconstruction on the node. */
  sessionId: string | null;
  /** The compute node the drone reports it reconstructs on (the capture
   * state's `computeNodeId`), used to pinpoint the node among several paired
   * compute nodes. Optional; falls back to the first reachable one. */
  computeNodeId?: string | null;
}

/** Resolve what the paired compute node built for a session. */
export function useNodeModel({ sessionId, computeNodeId }: NodeModelParams): NodeModel {
  const nodes = usePairedNodes();
  const computeNodes = useMemo(() => nodes.filter((n) => isComputeProfile(n.profile)), [nodes]);

  // Prefer the node the drone reports as its reconstructor; else the first
  // reachable compute node; else the first paired one (reads as unreachable).
  const targetNodeId = useMemo<string | null>(() => {
    const reported = computeNodeId
      ? computeNodes.find((n) => n.deviceId === computeNodeId)
      : undefined;
    return (reported ?? computeNodes.find((n) => n.reachable) ?? computeNodes[0])?.deviceId ?? null;
  }, [computeNodes, computeNodeId]);

  const { jobs, unreachable, client } = useComputeJobs(targetNodeId);

  // Completed reconstruct jobs that carry a session, newest-first. Offload jobs
  // (no session, not a world model) and in-flight jobs are excluded.
  const completed = useMemo(
    () =>
      jobs
        .filter((j) => j.state === "completed" && j.kind === "reconstruct" && Boolean(j.sessionId))
        .sort((a, b) => b.updatedMs - a.updatedMs),
    [jobs],
  );

  const sessions = useMemo<ModelSession[]>(() => {
    const seen = new Set<string>();
    const out: ModelSession[] = [];
    for (const j of completed) {
      const sid = j.sessionId;
      if (!sid || seen.has(sid)) continue;
      seen.add(sid);
      out.push({ sessionId: sid, jobId: j.id, createdMs: j.createdMs });
    }
    return out;
  }, [completed]);

  // The newest completed reconstruction for the requested session, or the
  // newest overall when the session is unknown (the post-flight default).
  const targetJobId = useMemo<string | null>(() => {
    const match = sessionId ? completed.find((j) => j.sessionId === sessionId) : completed[0];
    return match?.id ?? null;
  }, [completed, sessionId]);

  // The target job's splat (else its first output), tagged by job id so a
  // session switch never surfaces the previous job's artifact.
  const [art, setArt] = useState<{
    jobId: string;
    output: ComputeOutput;
    outputs: ComputeOutput[];
    viewer: ModelViewer;
  } | null>(null);

  useEffect(() => {
    if (!client || !targetJobId) return;
    let cancelled = false;
    void client.getOutputs(targetJobId).then((outs) => {
      if (cancelled) return;
      const all = outs ?? [];
      const pick = pickSplatOutput(all) ?? all[0];
      setArt(pick ? { jobId: targetJobId, output: pick, outputs: all, viewer: viewerForKind(pick.kind) } : null);
    });
    return () => {
      cancelled = true;
    };
  }, [client, targetJobId]);

  const resolved = art && art.jobId === targetJobId ? art : null;

  let status: ModelStatus;
  if (!client) status = "inactive";
  else if (resolved) status = "ready";
  else if (unreachable) status = "unreachable";
  else status = "building";

  return {
    hasComputeNode: computeNodes.length > 0,
    status,
    artifact: resolved?.output.source ?? null,
    viewerHint: resolved?.viewer ?? null,
    backend: resolved?.output.backend ?? null,
    output: resolved?.output ?? null,
    outputs: resolved?.outputs ?? [],
    sessions,
    computeNodeDeviceId: targetNodeId,
    computeClient: client,
  };
}
