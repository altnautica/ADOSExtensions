/**
 * The capture state and coverage replies, and the return-to-start prompt
 * (only after 3 minutes of recording; a loop-close offer within 2 m).
 */

import { describe, expect, it } from "vitest";

import {
  coerceCaptureState,
  coerceCoverage,
  returnToStartPrompt,
  type CaptureState,
} from "../../../src/lib/capture/capture-state";

const RAW = {
  state: "recording",
  session_id: "visor-1-20261019T101500Z-3fa2",
  started_at_ns: 120000000000,
  duration_s: 312.4,
  site: "Warehouse 3",
  floor: 0,
  room: "Dock",
  segment: 3,
  frames: { total: 12480, dropped_pct: 0.2, per_camera: [{ id: "cam_fl", fps: 20.0, dropped: 3 }, { fps: 1 }] },
  pose: { source: "vio", health: "ok", tracked_features: 143, drift_hint_m: 0.04 },
  coverage: { voxels: 18420, room_pct: 72.0, loop_closed: false, distance_from_start_m: 38.2 },
  health: [{ id: "blur", level: "ok", value: 182.0 }],
  exposure: { locked: true, exposure_us: 4000, gain: 2.0, mode: "walking" },
  storage: { free_bytes: 812345678901, est_remaining_s: 14400 },
};

describe("coerceCaptureState", () => {
  it("reads the documented state body", () => {
    const s = coerceCaptureState(RAW)!;
    expect(s.state).toBe("recording");
    expect(s.frames.perCamera).toEqual([{ id: "cam_fl", fps: 20, dropped: 3 }]);
    expect(s.pose.trackedFeatures).toBe(143);
    expect(s.coverage.distanceFromStartM).toBe(38.2);
    expect(s.exposure).toEqual({ locked: true, exposureUs: 4000, gain: 2, mode: "walking" });
    expect(s.storage.freeBytes).toBe(812345678901);
  });

  it("reads absent fields as unknown, never zero", () => {
    const s = coerceCaptureState({ state: "idle" })!;
    expect(s.durationS).toBeNull();
    expect(s.frames.total).toBeNull();
    expect(s.coverage.roomPct).toBeNull();
    expect(s.exposure.exposureUs).toBeNull();
    expect(coerceCaptureState({})).toBeNull();
  });
});

describe("coerceCoverage", () => {
  it("reads cells, trajectory and markers; needs a positive cell size", () => {
    const c = coerceCoverage({
      cell_m: 0.25,
      cells: [[0, 0, 4], [1, 0, 2], [1, "x", 2]],
      trajectory: [[0, 0, 1.6], [1, 0, 1.6], [2]],
      start: [0, 0, 1.6],
      current: [1, 0, 1.6],
      heading_deg: 90,
    })!;
    expect(c.cells).toEqual([{ ix: 0, iy: 0, hits: 4 }, { ix: 1, iy: 0, hits: 2 }]);
    expect(c.trajectory).toHaveLength(2);
    expect(c.headingDeg).toBe(90);
    expect(coerceCoverage({ cell_m: 0 })).toBeNull();
  });
});

describe("returnToStartPrompt", () => {
  const at = (patch: Partial<CaptureState["coverage"]>, durationS: number): CaptureState => {
    const s = coerceCaptureState(RAW)!;
    return { ...s, durationS, coverage: { ...s.coverage, ...patch } };
  };

  it("stays quiet in the first 3 minutes", () => {
    expect(returnToStartPrompt(at({}, 179))).toEqual({ kind: "none" });
  });

  it("gives the distance back, then offers the loop close within 2 m", () => {
    expect(returnToStartPrompt(at({ distanceFromStartM: 38.2 }, 400))).toEqual({ kind: "return", distanceM: 38.2 });
    expect(returnToStartPrompt(at({ distanceFromStartM: 1.5 }, 400))).toEqual({ kind: "close_loop" });
    expect(returnToStartPrompt(at({ distanceFromStartM: 1.5, loopClosed: true }, 400))).toEqual({ kind: "none" });
  });
});
