/**
 * @module use-job-world
 * @description Assembles one reconstruction job's outputs into what the world
 * viewer shows: the splat (the streamed `.rad` LOD first, then the SPZ, then
 * the raw PLY), its placement sidecar (a `.rad` shares the SPZ's), the collision mesh, the floorplan with its room
 * hotspots, warehouse aisles, the LAZ point cloud, the tour bundle and the
 * dataset export.
 */

import { useEffect, useState } from "react";

import { pickSplatOutput, splatPlaceJson } from "../components/atlas/viewer-types";
import type { WorldScene } from "../components/world/use-spark-scene";
import type { ComputeAgentClient, ComputeOutput } from "../lib/agent/compute-client";
import type { ArtifactSource } from "../lib/net/artifact-source";
import { coerceAisles } from "../lib/world/aisles";
import { coerceFloorplan } from "../lib/world/floorplan";
import { coercePlacement } from "../lib/world/placement";

export interface JobWorld {
  scene: WorldScene;
  pointcloud: ArtifactSource | null;
  tour: ArtifactSource | null;
  /** Dataset export (COLMAP sparse model + transforms.json, zipped). */
  export: ArtifactSource | null;
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
  const splatOut = pickSplatOutput(outputs);
  const placeJson = splatPlaceJson(outputs, splatOut);
  const plan = byKind(outputs, "floorplan");
  const aisles = byKind(outputs, "aisles");
  const [placementRaw, planRaw, aislesRaw] = await Promise.all([
    placeJson ? client.readJsonFile(jobId, placeJson) : Promise.resolve(null),
    plan?.file ? client.readJsonFile(jobId, plan.file) : Promise.resolve(null),
    aisles?.file ? client.readJsonFile(jobId, aisles.file) : Promise.resolve(null),
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
      aisles: coerceAisles(aislesRaw),
    },
    pointcloud: (byKind(outputs, "pointcloud") ?? byKind(outputs, "change"))?.source ?? null,
    tour: byKind(outputs, "tour")?.source ?? null,
    export: byKind(outputs, "export")?.source ?? null,
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
