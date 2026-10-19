/**
 * @module use-output-frame
 * @description The frame a compute output's splat is in:
 *  - its `.place.json` placement, or a sibling splat's for a `.rad` LOD file
 *    (which shares the SPZ's placement and has no sidecar of its own);
 *  - local ENU for an output of the staged pipeline that names no sidecar
 *    (a `splat_lod` exists only there);
 *  - the legacy COLMAP frame otherwise (an artifact from before the contract
 *    carries no placement).
 * `undefined` until a named sidecar has been read, so the viewer waits for it.
 */

import { useEffect, useState } from "react";

import { splatPlaceJson } from "../components/atlas/viewer-types";
import type { ComputeAgentClient, ComputeOutput } from "../lib/agent/compute-client";
import { coercePlacement, type ArtifactFrame } from "../lib/world/placement";

/** The frame an output is in without reading anything: null (local ENU) or
 * "colmap" when no sidecar is named, else undefined (read the sidecar). */
export function frameWithoutSidecar(output: ComputeOutput, placeJson: string | null): ArtifactFrame | undefined {
  if (placeJson) return undefined;
  return output.kind === "splat_lod" ? null : "colmap";
}

export function useOutputFrame(
  client: ComputeAgentClient | null,
  output: ComputeOutput | null | undefined,
  outputs: readonly ComputeOutput[],
): ArtifactFrame | undefined {
  const placeJson = output ? splatPlaceJson(outputs, output) : null;
  const key = output && placeJson ? `${output.jobId}/${placeJson}` : null;
  const [read, setRead] = useState<{ key: string; frame: ArtifactFrame } | null>(null);
  const jobId = output?.jobId ?? null;

  useEffect(() => {
    if (!client || !jobId || !placeJson || !key) return;
    let cancelled = false;
    void client.readJsonFile(jobId, placeJson).then((raw) => {
      // A sidecar the node named but that cannot be read: local ENU, the
      // contract's default for a staged output.
      if (!cancelled) setRead({ key, frame: coercePlacement(raw) });
    });
    return () => {
      cancelled = true;
    };
  }, [client, jobId, placeJson, key]);

  if (!output) return undefined;
  const direct = frameWithoutSidecar(output, placeJson);
  if (direct !== undefined) return direct;
  return read?.key === key ? read.frame : undefined;
}
