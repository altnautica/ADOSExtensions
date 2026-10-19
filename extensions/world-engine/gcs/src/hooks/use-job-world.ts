/**
 * @module use-job-world
 * @description Assembles one reconstruction job's outputs into what the world
 * viewer shows: the splat (the `.rad` LOD file when the node built one, else
 * the `.spz`), its placement sidecar, the collision mesh, the floorplan with
 * its room hotspots, the LAZ point cloud and the tour bundle.
 */

import { useEffect, useState } from "react";

import type { WorldScene } from "../components/world/use-spark-scene";
import type { ComputeAgentClient, ComputeOutput } from "../lib/agent/compute-client";
import type { ArtifactSource } from "../lib/net/artifact-source";
import { coerceFloorplan } from "../lib/world/floorplan";
import { coercePlacement } from "../lib/world/placement";

export interface JobWorld {
  scene: WorldScene;
  pointcloud: ArtifactSource | null;
  tour: ArtifactSource | null;
  outputs: ComputeOutput[];
}

export type JobWorldState =
  | { status: "idle" }
  | { status: "loading" }
  | { status: "failed" }
  | { status: "ready"; world: JobWorld };

const byKind = (outputs: readonly ComputeOutput[], kind: string) => outputs.find((o) => o.kind === kind);

/** Build the world from a job's outputs, reading the JSON sidecars it names. */
export async function loadJobWorld(client: ComputeAgentClient, jobId: string): Promise<JobWorld | null> {
  const outputs = await client.getOutputs(jobId);
  if (!outputs) return null;
  const splatOut = byKind(outputs, "splat_lod") ?? byKind(outputs, "splat");
  const plan = byKind(outputs, "floorplan");
  const [placementRaw, planRaw] = await Promise.all([
    splatOut?.placeJson ? client.readJsonFile(jobId, splatOut.placeJson) : Promise.resolve(null),
    plan?.file ? client.readJsonFile(jobId, plan.file) : Promise.resolve(null),
  ]);
  const floorplan = coerceFloorplan(planRaw);
  return {
    outputs,
    scene: {
      splat: splatOut?.source ?? null,
      placement: coercePlacement(placementRaw),
      collision: byKind(outputs, "mesh")?.source ?? null,
      floorplan,
      rooms: (floorplan?.floors ?? []).flatMap((f) =>
        f.rooms.map((r) => ({ name: r.name, floor: f.floor, hotspot: r.hotspot })),
      ),
    },
    pointcloud: (byKind(outputs, "pointcloud") ?? byKind(outputs, "change"))?.source ?? null,
    tour: byKind(outputs, "tour")?.source ?? null,
  };
}

export function useJobWorld(client: ComputeAgentClient | null, jobId: string | null): JobWorldState {
  const [state, setState] = useState<{ key: string; value: JobWorldState } | null>(null);
  const key = client && jobId ? `${client.deviceId}:${jobId}` : null;

  useEffect(() => {
    if (!client || !jobId || !key) return;
    let cancelled = false;
    setState({ key, value: { status: "loading" } });
    void loadJobWorld(client, jobId).then((world) => {
      if (cancelled) return;
      setState({ key, value: world ? { status: "ready", world } : { status: "failed" } });
    });
    return () => {
      cancelled = true;
    };
  }, [client, jobId, key]);

  if (!key) return { status: "idle" };
  return state && state.key === key ? state.value : { status: "loading" };
}
