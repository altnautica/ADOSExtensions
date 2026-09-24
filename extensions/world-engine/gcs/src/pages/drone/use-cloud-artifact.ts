/**
 * The viewable artifact of a finished cloud job record. A job whose output is
 * an artifact path on a paired compute node is read through that node's agent;
 * a plain http(s) output is read directly; anything else (an unfinished job, a
 * placeholder output) has no artifact.
 */

import { useMemo } from "react";

import { useHost } from "../../host/context";
import type { CloudJob } from "../../hooks/use-cloud-jobs";
import { usePairedNodes } from "../../hooks/use-paired-nodes";
import { artifactSource, type ArtifactSource } from "../../lib/net/artifact-source";

export function useCloudArtifact(job: CloudJob | null): ArtifactSource | null {
  const host = useHost();
  const nodes = usePairedNodes();
  const outputUrl = job && job.status === "done" ? job.outputUrl : null;
  const computeNodeId = job?.computeNodeId ?? null;
  const paired = computeNodeId !== null && nodes.some((n) => n.deviceId === computeNodeId);

  return useMemo(() => {
    if (!outputUrl) return null;
    const node = paired && computeNodeId ? { deviceId: computeNodeId, agent: host.nodes.agent(computeNodeId) } : null;
    return artifactSource(outputUrl, node);
  }, [host, outputUrl, computeNodeId, paired]);
}
