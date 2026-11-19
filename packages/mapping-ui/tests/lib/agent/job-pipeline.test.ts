/**
 * The staged pipeline a compute node reports on a job: coercion, stage rows
 * exactly as the node reports them, overall progress,
 * the ETA readout, and the structured error (`E_TOOL_MISSING` + tool).
 */

import { describe, expect, it } from "vitest";

import {
  coerceJobError,
  coerceJobPipeline,
  formatEta,
  pipelineFraction,
  stageRows,
} from "../../../src/lib/agent/job-pipeline";

const RAW = {
  stage: "train",
  stage_progress: 0.5,
  preset: "hq",
  stages: [
    { id: "prepare", state: "done", started_ns: 1, ended_ns: 2 },
    { id: "sfm", state: "done" },
    { id: "train", state: "running", progress: 0.5 },
    { id: "features", state: "pending" },
  ],
  gpu: { name: "RTX 5070", util_pct: 97, vram_used_mb: 7270 },
  eta_s: 1500,
  tools: { colmap: "4.2.1", gsplat: "1.5.3", bad: 4 },
};

describe("coerceJobPipeline", () => {
  it("reads the stage, rows, GPU, ETA, tools and preset", () => {
    const p = coerceJobPipeline(RAW)!;
    expect(p.stage).toBe("train");
    expect(p.preset).toBe("hq");
    expect(p.gpu).toEqual({ name: "RTX 5070", utilPct: 97, vramUsedMb: 7270 });
    expect(p.tools).toEqual([
      { name: "colmap", version: "4.2.1" },
      { name: "gsplat", version: "1.5.3" },
    ]);
    expect(coerceJobPipeline({ state: "running" })).toBeNull();
  });

  it("renders the node's stages in node order and never invents done stages", () => {
    const rows = stageRows(coerceJobPipeline(RAW)!);
    expect(rows.map((r) => [r.id, r.state])).toEqual([
      ["prepare", "done"],
      ["sfm", "done"],
      ["train", "running"],
      ["features", "pending"],
    ]);
    expect(rows[2]?.progress).toBe(0.5);
  });

  it("is null when the node reports no stages, so the view falls back to progress", () => {
    expect(coerceJobPipeline({ stage: "train", stages: [] })).toBeNull();
    expect(coerceJobPipeline({ stage: "train" })).toBeNull();
  });

  it("weights progress by stage: two done + half of the third of four", () => {
    expect(pipelineFraction(coerceJobPipeline(RAW)!)).toBeCloseTo(2.5 / 4, 12);
    expect(pipelineFraction(coerceJobPipeline({ ...RAW, stage: "done" })!)).toBe(1);
  });

  it("marks the current stage failed when the job failed there", () => {
    const rows = stageRows(coerceJobPipeline({ stage: "failed", stages: [{ id: "sfm", state: "failed" }] })!);
    expect(rows.find((r) => r.id === "sfm")?.state).toBe("failed");
  });
});

describe("coerceJobError", () => {
  it("reads the node's error object and a legacy string", () => {
    expect(coerceJobError({ code: "E_TOOL_MISSING", message: "colmap not found", tool: "colmap" })).toEqual({
      code: "E_TOOL_MISSING",
      message: "colmap not found",
      tool: "colmap",
    });
    expect(coerceJobError("boom")).toEqual({ code: "error", message: "boom", tool: null });
    expect(coerceJobError(null)).toBeNull();
  });
});

describe("formatEta", () => {
  it("reads in s, m, or h + m", () => {
    expect(formatEta(40)).toBe("40 s");
    expect(formatEta(720)).toBe("12 m");
    expect(formatEta(3840)).toBe("1 h 04 m");
    expect(formatEta(null)).toBeNull();
  });
});
