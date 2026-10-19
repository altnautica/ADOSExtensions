/**
 * @module use-output-frame
 * @description The frame a compute output's splat is in: its `.place.json`
 * placement when the node wrote one (staged pipeline), else the legacy COLMAP
 * frame (an artifact from before the contract carries no placement). Null
 * until a named sidecar has been read, so the viewer waits for it.
 */

import { useEffect, useState } from "react";

import type { ComputeAgentClient, ComputeOutput } from "../lib/agent/compute-client";
import { coercePlacement, type ArtifactFrame } from "../lib/world/placement";

/** `undefined` while the sidecar is being read. */
export function useOutputFrame(
  client: ComputeAgentClient | null,
  output: ComputeOutput | undefined,
): ArtifactFrame | undefined {
  const key = output?.placeJson ? `${output.jobId}/${output.placeJson}` : null;
  const [read, setRead] = useState<{ key: string; frame: ArtifactFrame } | null>(null);

  useEffect(() => {
    if (!client || !output?.placeJson || !key) return;
    let cancelled = false;
    void client.readJsonFile(output.jobId, output.placeJson).then((raw) => {
      // A sidecar the node named but that cannot be read: local ENU, the
      // contract's default for a staged output.
      if (!cancelled) setRead({ key, frame: coercePlacement(raw) });
    });
    return () => {
      cancelled = true;
    };
  }, [client, output?.jobId, output?.placeJson, key]);

  if (!output) return undefined;
  if (!key) return "colmap";
  return read?.key === key ? read.frame : undefined;
}
