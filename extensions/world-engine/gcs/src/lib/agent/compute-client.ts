/**
 * @module compute-client
 * @description Client for a compute node's job API, served by the extension
 * on a workstation or compute node under `api/compute/`: submit / list / read
 * reconstruction and offload jobs and their outputs, the node credentials it
 * issues, and the world-stream ticket. Every reply is coerced defensively and
 * a `404` / transport failure returns `null` / `[]`, so a poll never throws
 * and the workbench shows an "awaiting compute node" state.
 */

import type { InlineAgentApi } from "@altnautica/plugin-sdk/inline";

import type { ComputeGpuInfo } from "../../stores/compute-store";
import { artifactSource, type ArtifactSource } from "../net/artifact-source";
import { requestJson, type JsonReply } from "./plugin-http";
import { coerceJobError, coerceJobPipeline, type JobError, type JobPipeline } from "./job-pipeline";

/** The compute node's own listener port, where drones reach its job API and
 * streams directly (the GCS goes through the host instead). */
export const COMPUTE_JOB_PORT = "8092";

/**
 * Coerce the snake_case `gpu` block of the node's compute status into the
 * camelCase {@link ComputeGpuInfo}. Returns null when the block is absent or not
 * an object; each field independently degrades to null when missing or the
 * wrong type, so a partial reading (e.g. name known, utilization not) still
 * surfaces what the node does report. Wire keys: `name` / `cores` /
 * `unified_memory_mb` / `metal` / `utilization_pct`.
 */
export function parseComputeGpu(raw: unknown): ComputeGpuInfo | null {
  if (!raw || typeof raw !== "object" || Array.isArray(raw)) return null;
  const g = raw as Record<string, unknown>;
  const n = (v: unknown): number | null =>
    typeof v === "number" && Number.isFinite(v) ? v : null;
  const s = (v: unknown): string | null =>
    typeof v === "string" && v.length > 0 ? v : null;
  return {
    name: s(g.name),
    cores: n(g.cores),
    unifiedMemoryMb: n(g.unified_memory_mb),
    metal: s(g.metal),
    utilizationPct: n(g.utilization_pct),
  };
}

/** Lifecycle state of a compute job (lowercase on the wire). Left open so a
 * future engine can advertise another state without breaking the type. */
export type ComputeJobState =
  | "queued"
  | "running"
  | "completed"
  | "failed"
  | "cancelled"
  | (string & {});

/** One reconstruction / offload job the engine tracks. */
export interface ComputeJob {
  id: string;
  /** "reconstruct" | "perception_offload" (free-form). */
  kind: string;
  datasetId: string | null;
  state: ComputeJobState;
  /** Progress in `0..1` while running. */
  progress: number;
  /** Where the finished artifact can be fetched, or null until done. */
  resultRef: string | null;
  /** Failure detail when `state` is "failed" (the node's message). */
  error: string | null;
  /** The node's structured error (`E_TOOL_MISSING` with its tool, ...). */
  errorInfo: JobError | null;
  /** The staged reconstruction pipeline, or null for a job without stages. */
  pipeline: JobPipeline | null;
  /** The capturing session a reconstruct job belongs to (lifted from the job's
   * params by the engine), or null for a job that carries none (an offload job,
   * or a reconstruct job from an agent before the session was tagged). Lets the
   * GCS correlate a world-model artifact to a drone's active session. */
  sessionId: string | null;
  /** The Brush training-step count a reconstruct job ran (lifted from the job's
   * `params.steps`), or null for a job that carries none. Lets the GCS label a
   * reconstruction with its detail level. */
  steps: number | null;
  /** The job's params as the engine stored them. A re-run submits them
   * verbatim: an ingest-created reconstruct job carries backend, session_id,
   * steps, generation and device_id, and the worker publishes nothing without
   * device_id. */
  params: Record<string, unknown>;
  createdMs: number;
  updatedMs: number;
}

/** One artifact a finished job produced. */
export interface ComputeOutput {
  id: string;
  jobId: string;
  /** Artifact kind ("splat" | "cloud" | "mesh" | ...). */
  kind: string;
  /** The artifact URI the engine stamped (its own host, often unresolvable). */
  uri: string;
  /** Where a viewer reads the bytes (through the node's extension server), or
   * null for a placeholder with nothing to read. */
  source: ArtifactSource | null;
  /** The concrete reconstruction backend that produced this artifact, lifted
   * from `meta.backend` (`"brush"` / `"gsplat"` / `"msplat"` / `"colmap"`),
   * or null when the node advertises none. */
  backend: string | null;
  /** The raw backend result metadata (`gaussian_count`, `backend`, …) served on
   * the output, or null when absent — kept so a surface can read further detail
   * without a second fetch. */
  meta: Record<string, unknown> | null;
  /** The file name in the job's output directory, or null. */
  file: string | null;
  /** The artifact format (`spz`, `rad`, `glb`, `laz`, `json`, `zip`, ...). */
  format: string | null;
  sizeBytes: number | null;
  /** The output-relative name of the artifact's `.place.json` sidecar, or null. */
  placeJson: string | null;
  createdMs: number;
}

/** One input dataset a job ran (or will run) on. */
export interface ComputeDataset {
  id: string;
  kind: string;
  createdMs: number;
}

/** A job submission. */
export interface ComputeSubmitRequest {
  jobId?: string;
  kind: string;
  datasetId?: string;
  params?: Record<string, unknown>;
}

/** The engine's reply to a job submission. */
export interface ComputeSubmitResult {
  jobId: string;
  state: ComputeJobState;
}

/** A dataset-creation request. */
export interface ComputeDatasetRequest {
  id?: string;
  kind: string;
  meta?: Record<string, unknown>;
}

function num(v: unknown): number {
  return typeof v === "number" && Number.isFinite(v) ? v : 0;
}

function str(v: unknown): string {
  return typeof v === "string" ? v : "";
}

function strOrNull(v: unknown): string | null {
  return typeof v === "string" && v.length > 0 ? v : null;
}

function coerceJob(raw: unknown): ComputeJob | null {
  if (!raw || typeof raw !== "object") return null;
  const e = raw as Record<string, unknown>;
  if (typeof e.id !== "string") return null;
  const params =
    e.params && typeof e.params === "object" && !Array.isArray(e.params)
      ? (e.params as Record<string, unknown>)
      : null;
  return {
    id: e.id,
    kind: str(e.kind),
    datasetId: strOrNull(e.dataset_id),
    state: str(e.state),
    progress: num(e.progress),
    resultRef: strOrNull(e.result_ref),
    error: coerceJobError(e.error)?.message ?? null,
    errorInfo: coerceJobError(e.error),
    pipeline: coerceJobPipeline(e),
    sessionId: strOrNull(e.session_id),
    steps:
      params && typeof params.steps === "number" && params.steps > 0
        ? params.steps
        : null,
    params: params ?? {},
    createdMs: num(e.created_ms),
    updatedMs: num(e.updated_ms),
  };
}

function coerceOutput(raw: unknown): ComputeOutput | null {
  if (!raw || typeof raw !== "object") return null;
  const e = raw as Record<string, unknown>;
  if (typeof e.id !== "string" || typeof e.uri !== "string") return null;
  const meta =
    e.meta && typeof e.meta === "object" && !Array.isArray(e.meta)
      ? (e.meta as Record<string, unknown>)
      : null;
  return {
    id: e.id,
    jobId: str(e.job_id),
    kind: str(e.kind),
    uri: e.uri,
    source: null,
    backend: typeof meta?.backend === "string" && meta.backend.length > 0 ? meta.backend : null,
    meta,
    file: strOrNull(e.file),
    format: strOrNull(e.format),
    sizeBytes: typeof e.size_bytes === "number" ? e.size_bytes : null,
    placeJson: strOrNull(e.place_json),
    createdMs: num(e.created_ms),
  };
}

function coerceDataset(raw: unknown): ComputeDataset | null {
  if (!raw || typeof raw !== "object") return null;
  const e = raw as Record<string, unknown>;
  if (typeof e.id !== "string") return null;
  return { id: e.id, kind: str(e.kind), createdMs: num(e.created_ms) };
}

export class ComputeAgentClient {
  /**
   * @param deviceId The compute node's device id (artifact identity).
   * @param agent The extension's server on that node.
   */
  constructor(
    readonly deviceId: string,
    private readonly agent: InlineAgentApi,
  ) {}

  /**
   * Issue one job-API request. `path` is the segment after `api/compute/`
   * (e.g. `jobs`, `jobs/<id>/cancel`). Returns the status and parsed JSON body
   * (`null` when it is not JSON), or `null` on a transport failure, so a
   * caller that must tell a refusal from an unreachable node can.
   */
  jobCall(path: string, method: "GET" | "POST", body?: unknown): Promise<JsonReply | null> {
    return requestJson(this.agent, `api/compute/${path}`, method, body);
  }

  /** {@link jobCall}, reduced to the parsed body of a 2xx reply, or `null` on
   * non-2xx / transport / parse failure so every caller degrades to an empty
   * state instead of throwing. */
  private async jobRequest(
    path: string,
    method: "GET" | "POST",
    body?: unknown,
  ): Promise<unknown | null> {
    const r = await this.jobCall(path, method, body);
    return r && r.status >= 200 && r.status < 300 ? r.json : null;
  }

  /** List every job on the node. `null` = unreachable; `[]` = reachable + empty. */
  async listJobs(): Promise<ComputeJob[] | null> {
    const body = await this.jobRequest("jobs", "GET");
    if (body === null) return null;
    if (!Array.isArray(body)) return [];
    return body.flatMap((j) => {
      const job = coerceJob(j);
      return job ? [job] : [];
    });
  }

  /** Fetch one job by id, or `null` when missing / unreachable. */
  async getJob(id: string): Promise<ComputeJob | null> {
    const body = await this.jobRequest(`jobs/${encodeURIComponent(id)}`, "GET");
    return body ? coerceJob(body) : null;
  }

  /** A job's output artifacts. `null` = unreachable; `[]` = reachable + none.
   * Each artifact is read back through this node's extension server, since the
   * engine stamps a drifting mDNS `.local` host the browser cannot resolve. */
  async getOutputs(id: string): Promise<ComputeOutput[] | null> {
    const body = await this.jobRequest(
      `jobs/${encodeURIComponent(id)}/outputs`,
      "GET",
    );
    if (body === null) return null;
    if (!Array.isArray(body)) return [];
    return body.flatMap((o) => {
      const out = coerceOutput(o);
      if (!out) return [];
      out.source = out.file
        ? this.fileSource(out.jobId || id, out.file)
        : artifactSource(out.uri, { deviceId: this.deviceId, agent: this.agent });
      return [out];
    });
  }

  /** A file in a job's output directory (`jobs/{id}/files/{rel}`), read
   * through this node's extension server. */
  fileSource(jobId: string, rel: string): ArtifactSource {
    const path = `api/compute/jobs/${encodeURIComponent(jobId)}/files/${rel
      .split("/")
      .map(encodeURIComponent)
      .join("/")}`;
    return {
      key: `${this.deviceId}:${jobId}:${rel}`,
      name: rel.slice(rel.lastIndexOf("/") + 1),
      fetch: (signal, init) => this.agent.fetch(path, { ...init, signal }),
      sibling: (name) => this.fileSource(jobId, `${rel.slice(0, rel.lastIndexOf("/") + 1)}${name}`),
    };
  }

  /** A JSON file from a job's output directory, or null when it is absent. */
  async readJsonFile(jobId: string, rel: string): Promise<unknown | null> {
    try {
      const res = await this.fileSource(jobId, rel).fetch(AbortSignal.timeout(15_000));
      return res.ok ? await res.json() : null;
    } catch {
      return null;
    }
  }

  /**
   * Distance on a job's reconstructed mesh (local ENU metres). `point` snaps
   * each end to the nearest surface; `ray` casts from `a` along direction `b`
   * and measures to the first hit. Null when the node cannot answer.
   */
  async measure(
    jobId: string,
    a: [number, number, number],
    b: [number, number, number],
    mode: "point" | "ray" = "point",
  ): Promise<{ distanceM: number; aHit: number[] | null; bHit: number[] | null } | null> {
    const body = await this.jobRequest("measure", "POST", { job_id: jobId, a, b, mode });
    if (!body || typeof body !== "object") return null;
    const e = body as Record<string, unknown>;
    if (typeof e.distance_m !== "number") return null;
    const hit = (v: unknown) => (Array.isArray(v) && v.length === 3 ? (v as number[]) : null);
    return { distanceM: e.distance_m, aHit: hit(e.a_hit), bHit: hit(e.b_hit) };
  }

  /** Submit a job. Returns the assigned id + initial state, or `null` on failure. */
  async submitJob(
    req: ComputeSubmitRequest,
  ): Promise<ComputeSubmitResult | null> {
    const body = await this.jobRequest("jobs", "POST", {
      job_id: req.jobId,
      kind: req.kind,
      dataset_id: req.datasetId,
      params: req.params ?? null,
    });
    if (!body || typeof body !== "object") return null;
    const e = body as Record<string, unknown>;
    if (typeof e.job_id !== "string") return null;
    return { jobId: e.job_id, state: str(e.state) };
  }

  /**
   * Submit a staged job (`reconstruct {session_id, preset}`, `change {a, b}`,
   * `calibrate {session_id, target}`) with its fields at the top level of the
   * body. A refusal carries the node's error code (`E_TOOL_MISSING`, ...).
   */
  async submit(
    kind: string,
    fields: Record<string, unknown>,
  ): Promise<{ ok: true; jobId: string } | { ok: false; code: string; message: string }> {
    const r = await this.jobCall("jobs", "POST", { ...fields, kind });
    if (!r) return { ok: false, code: "transport_error", message: "The compute node did not answer." };
    const e = r.json && typeof r.json === "object" ? (r.json as Record<string, unknown>) : {};
    if (r.status >= 200 && r.status < 300 && typeof e.job_id === "string") return { ok: true, jobId: e.job_id };
    // `{error: {code, message, tool}}` or the flat `{error: "E_...", tool, detail}`.
    const err =
      typeof e.error === "string"
        ? { code: e.error, message: strOrNull(e.detail) ?? e.error, tool: strOrNull(e.tool) }
        : coerceJobError(e.error);
    const code = err?.code ?? `http_${r.status}`;
    return { ok: false, code, message: err?.tool ? `${err.message} (${err.tool})` : (err?.message ?? code) };
  }

  /** Request cancellation of a queued / running job. Returns whether it took. */
  async cancelJob(id: string): Promise<boolean> {
    const body = await this.jobRequest(
      `jobs/${encodeURIComponent(id)}/cancel`,
      "POST",
    );
    if (!body || typeof body !== "object") return false;
    return (body as Record<string, unknown>).cancelled === true;
  }

  /** Create an input dataset, or `null` on failure. */
  async createDataset(
    req: ComputeDatasetRequest,
  ): Promise<ComputeDataset | null> {
    const body = await this.jobRequest("datasets", "POST", {
      id: req.id,
      kind: req.kind,
      meta: req.meta ?? null,
    });
    return body ? coerceDataset(body) : null;
  }
}
