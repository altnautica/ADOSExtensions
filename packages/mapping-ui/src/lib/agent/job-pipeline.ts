/**
 * @module job-pipeline
 * @description The staged reconstruction pipeline a compute node reports on a
 * job (`stage`, `stages[]`, `gpu`, `eta_s`, `tools`, `error`). Pure coercion
 * plus the readouts the job views share.
 */

import { arr, num, obj, objOrNull, str } from "../capture/coerce";

export type StageState = "pending" | "running" | "done" | "failed" | "skipped";

export interface StageRow {
  id: string;
  state: StageState;
  progress: number | null;
  startedNs: number | null;
  endedNs: number | null;
}

export interface JobError {
  code: string;
  message: string;
  tool: string | null;
}

export interface JobPipeline {
  /** The current stage (`queued`, a stage id, `done`, `failed`). */
  stage: string;
  stageProgress: number | null;
  stages: StageRow[];
  gpu: { name: string | null; utilPct: number | null; vramUsedMb: number | null } | null;
  etaS: number | null;
  tools: { name: string; version: string }[];
  preset: string | null;
}

const STAGE_STATES: readonly StageState[] = ["pending", "running", "done", "failed", "skipped"];

/** A job's error: the node's `{code, message, tool}` object, or a legacy
 * plain string, or null. */
export function coerceJobError(raw: unknown): JobError | null {
  if (typeof raw === "string" && raw.length > 0) return { code: "error", message: raw, tool: null };
  const e = objOrNull(raw);
  const code = str(e?.code);
  if (!e || !code) return null;
  return { code, message: str(e.message) ?? code, tool: str(e.tool) };
}

/** The pipeline block of a job, or null for a job that reports no stages
 * (an offload job, or a node before staged reconstruction); the job view then
 * falls back to the job's flat `progress`. */
export function coerceJobPipeline(raw: Record<string, unknown>): JobPipeline | null {
  const stage = str(raw.stage);
  if (!stage) return null;
  const stages = arr(raw.stages).flatMap((s) => {
    const e = obj(s);
    const id = str(e.id);
    if (!id) return [];
    const state = STAGE_STATES.find((x) => x === e.state) ?? "pending";
    return [{ id, state, progress: num(e.progress), startedNs: num(e.started_ns), endedNs: num(e.ended_ns) }];
  });
  if (stages.length === 0) return null;
  const gpu = objOrNull(raw.gpu);
  return {
    stage,
    stageProgress: num(raw.stage_progress),
    stages,
    gpu: gpu ? { name: str(gpu.name), utilPct: num(gpu.util_pct), vramUsedMb: num(gpu.vram_used_mb) } : null,
    etaS: num(raw.eta_s),
    tools: Object.entries(obj(raw.tools)).flatMap(([name, v]) =>
      typeof v === "string" && v.length > 0 ? [{ name, version: v }] : [],
    ),
    preset: str(raw.preset),
  };
}

/** The node's stage rows, in the order the node reports them. Only reported
 * stages are shown; none is ever assumed done. */
export function stageRows(p: JobPipeline): StageRow[] {
  return p.stages;
}

/** Overall progress `0..1`: completed stages plus the running stage's share. */
export function pipelineFraction(p: JobPipeline): number | null {
  if (p.stage === "done") return 1;
  const rows = stageRows(p).filter((r) => r.state !== "skipped");
  if (rows.length === 0) return null;
  const done = rows.filter((r) => r.state === "done").length;
  const running = rows.find((r) => r.state === "running");
  return Math.min(1, (done + (running?.progress ?? 0)) / rows.length);
}

/** "1 h 04 m" / "12 m" / "40 s". */
export function formatEta(s: number | null): string | null {
  if (s === null) return null;
  const sec = Math.max(0, Math.round(s));
  if (sec < 60) return `${sec} s`;
  const m = Math.floor(sec / 60);
  if (m < 60) return `${m} m`;
  return `${Math.floor(m / 60)} h ${String(m % 60).padStart(2, "0")} m`;
}
