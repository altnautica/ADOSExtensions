/**
 * @module editor/editor-api
 * @description The editor's node contract: the `edit` job's operations, the
 * provenance a version carries (`out/<stem>.v<N>.edits.json`), and the size
 * estimate and volume routes. Shapes are the node's wire shapes.
 */

import type { ComputeAgentClient } from "@altnautica/mapping-ui/lib/agent/compute-client";

import type { Xy } from "./geometry";

export interface DecimateTarget {
  target_points?: number;
  target_percent?: number;
}

export type EditOp =
  | { op: "crop"; keep: "inside" | "outside"; polygon: Xy[]; z?: [number, number] }
  | { op: "delete"; polygon: Xy[]; z?: [number, number] }
  | {
      op: "patch";
      layer: string;
      source: { box: [Xy, Xy] };
      target: { translate: Xy; rotate_deg: number };
    }
  | { op: "delete_layer"; layer: string }
  | ({ op: "decimate" } & DecimateTarget)
  | { op: "export"; format: "laz" | "ply"; crs?: string; decimate?: DecimateTarget };

export type EditRecord = EditOp & {
  removed?: number;
  added?: number;
  self_overlap?: boolean;
  source_layers?: string[];
  deviation_m?: { mean: number; p95: number };
  result: string;
  ts: string;
};

export interface LayerInfo {
  name: string;
  id: number;
  points: number;
}

export interface EditsFile {
  version: number;
  product: string;
  source: string;
  edited: boolean;
  crs?: string;
  layers: LayerInfo[];
  ops: EditRecord[];
}

export const BASE_LAYER = "base";

/** `out/cloud.v2.laz` → `cloud.v2.edits.json` (relative to the job's `out/`). */
export function provenanceFile(rel: string): string {
  const name = rel.slice(rel.lastIndexOf("/") + 1);
  const dot = name.lastIndexOf(".");
  return `${dot > 0 ? name.slice(0, dot) : name}.edits.json`;
}

/** `out/cloud.v2.laz` → `cloud.v2.laz`. */
export function outRel(rel: string): string {
  return rel.startsWith("out/") ? rel.slice(4) : rel;
}

/** The provenance of an artifact nothing has edited. */
export function unedited(source: string, points: number): EditsFile {
  return {
    version: 1,
    product: source,
    source,
    edited: false,
    layers: [{ name: BASE_LAYER, id: 0, points }],
    ops: [],
  };
}

function isEditsFile(v: unknown): v is EditsFile {
  if (!v || typeof v !== "object") return false;
  const e = v as Record<string, unknown>;
  return typeof e.product === "string" && Array.isArray(e.ops) && Array.isArray(e.layers);
}

/** A version's provenance, or null when it has none (an unedited artifact). */
export async function readProvenance(client: ComputeAgentClient, job: string, rel: string): Promise<EditsFile | null> {
  const body = await client.readJsonFile(job, provenanceFile(rel));
  return isEditsFile(body) ? body : null;
}

/** The newest version the product's edits wrote, or null before any edit. */
export async function readLatest(client: ComputeAgentClient, job: string): Promise<EditsFile | null> {
  const body = await client.readJsonFile(job, "edits.json");
  return isEditsFile(body) ? body : null;
}

/** Every version along a provenance chain, oldest first: the source, then
 * each rewriting operation's result. */
export function versionsOf(edits: EditsFile): string[] {
  return [edits.source, ...edits.ops.filter((r) => r.op !== "export").map((r) => r.result)];
}

export type SubmitOutcome = { ok: true; jobId: string } | { ok: false; message: string };

export async function submitEdit(
  client: ComputeAgentClient,
  session: string,
  job: string,
  source: string,
  ops: EditOp[],
): Promise<SubmitOutcome> {
  const r = await client.submit("edit", { dataset_id: session, params: { job, source, ops } });
  return r.ok ? r : { ok: false, message: r.message };
}

async function post(client: ComputeAgentClient, path: string, body: unknown): Promise<Record<string, unknown> | string> {
  const r = await client.jobCall(path, "POST", body);
  if (!r) return "The compute node did not answer.";
  const e = r.json && typeof r.json === "object" ? (r.json as Record<string, unknown>) : {};
  if (r.status >= 200 && r.status < 300) return e;
  return typeof e.detail === "string" ? e.detail : `The node refused (${r.status}).`;
}

/** The predicted points and bytes of writing `source` as `format`. */
export async function estimateSize(
  client: ComputeAgentClient,
  req: { job: string; source: string; format: "laz" | "ply"; decimate?: DecimateTarget },
): Promise<{ points: number; bytes: number } | string> {
  const r = await post(client, "edit/estimate", req);
  if (typeof r === "string") return r;
  return typeof r.points === "number" && typeof r.bytes === "number"
    ? { points: r.points, bytes: r.bytes }
    : "The node's estimate was unreadable.";
}

/** The volume above `baseZ` inside `polygon`, read from the full-resolution
 * version. */
export async function exactVolume(
  client: ComputeAgentClient,
  req: { job: string; source: string; polygon: Xy[]; base_z: number; cell_m: number },
): Promise<number | string> {
  const r = await post(client, "edit/volume", req);
  if (typeof r === "string") return r;
  return typeof r.volume_m3 === "number" ? r.volume_m3 : "The node's measurement was unreadable.";
}
