import { act, cleanup, render, renderHook, screen, fireEvent, waitFor } from "@testing-library/react";
import { afterEach, describe, expect, it, vi } from "vitest";

// The LAZ decoder's wasm is bundled at build time; these tests decode nothing.
vi.mock("@altnautica/mapping-ui/components/model/viewers/cloud-laz", () => ({ decodeLazEnu: vi.fn() }));

import { versionsOf, type EditsFile } from "../../src/pages/editor/editor-api";
import { buildEditorCloud, countRegion, layerIds } from "../../src/pages/editor/editor-cloud";
import { editorReducer, initialState, nextPatchName, SELF_INTERSECTING } from "../../src/pages/editor/editor-state";
import { pointInPolygon, place, polygonArea, rectPolygon, selfIntersects, volumeAbove, type Xy } from "../../src/pages/editor/geometry";
import { LayersPanel, dependentOf } from "../../src/pages/editor/LayersPanel";
import { describeOp } from "../../src/pages/editor/ProvenancePanel";
import { useSingleWriter } from "../../src/pages/editor/use-editor-product";

const SQUARE: Xy[] = [
  [0, 0],
  [10, 0],
  [10, 10],
  [0, 10],
];

/** A 20 × 20 grid 1 m apart at heights 0 and 2: 800 points. */
function grid(): Float32Array {
  const p: number[] = [];
  for (let i = 0; i < 20; i++) for (let j = 0; j < 20; j++) for (const z of [0, 2]) p.push(i + 0.5, j + 0.5, z);
  return Float32Array.from(p);
}

const EDITS: EditsFile = {
  version: 1,
  product: "out/cloud.v2.laz",
  source: "out/cloud.laz",
  edited: true,
  layers: [
    { name: "base", id: 0, points: 792 },
    { name: "patch-1", id: 1, points: 8 },
  ],
  ops: [
    { op: "crop", keep: "inside", polygon: SQUARE, removed: 0, result: "out/cloud.v1.laz", ts: "2026-11-19T10:00:00Z" },
    {
      op: "patch",
      layer: "patch-1",
      source: { box: [[0, 0], [2, 2]] },
      target: { translate: [30, 0], rotate_deg: 0 },
      added: 8,
      source_layers: ["base"],
      result: "out/cloud.v2.laz",
      ts: "2026-11-19T10:01:00Z",
    },
  ],
};

afterEach(cleanup);

describe("editor geometry", () => {
  it("refuses a polygon that crosses itself and accepts a concave one", () => {
    expect(selfIntersects(SQUARE)).toBe(false);
    expect(selfIntersects([[0, 0], [10, 10], [10, 0], [0, 10]])).toBe(true);
    expect(selfIntersects([[0, 0], [10, 0], [5, 3], [10, 10], [0, 10]])).toBe(false);
    expect(pointInPolygon(SQUARE, 5, 5)).toBe(true);
    expect(pointInPolygon(SQUARE, 15, 5)).toBe(false);
    expect(polygonArea(rectPolygon([2, 3], [6, 1]))).toBe(8);
  });

  it("places a patch by rotating about the box centre, then translating, as the node does", () => {
    const [x, y] = place([[0, 0], [2, 2]], 90, [10, 0], [0.5, 0.5]);
    expect(x).toBeCloseTo(11.5);
    expect(y).toBeCloseTo(0.5);
  });

  it("measures the same volume twice", () => {
    const p = grid();
    const a = volumeAbove(p, () => true, SQUARE, 0, 1);
    expect(a.volumeM3).toBeCloseTo(200);
    expect(volumeAbove(p, () => true, SQUARE, 0, 1)).toEqual(a);
  });
});

describe("editor selection", () => {
  it("counts the shown points inside the selection and re-counts on a depth change", () => {
    const cloud = buildEditorCloud(grid(), null, [{ name: "base", id: 0, points: 800 }]);
    expect(countRegion(cloud, { polygon: SQUARE, z: null }, new Set()).count).toBe(200);
    expect(countRegion(cloud, { polygon: SQUARE, z: [1, 3] }, new Set()).count).toBe(100);
  });

  it("leaves a hidden layer's points out of the count", () => {
    const cloud = buildEditorCloud(grid(), null, [
      { name: "base", id: 0, points: 792 },
      { name: "patch-1", id: 1, points: 8 },
    ]);
    expect(Array.from(layerIds(800, EDITS.layers).slice(790, 794))).toEqual([0, 0, 1, 1]);
    const all = countRegion(cloud, { polygon: [[0, 0], [30, 0], [30, 30], [0, 30]], z: null }, new Set());
    const shown = countRegion(cloud, { polygon: [[0, 0], [30, 0], [30, 30], [0, 30]], z: null }, new Set([1]));
    expect(all.count - shown.count).toBe(8);
  });

  it("returns to Select on Escape from any tool and keeps the selection", () => {
    let s = editorReducer(initialState, { type: "mode", mode: "polygon" });
    s = editorReducer(s, { type: "vertex", at: [0, 0] });
    s = editorReducer(s, { type: "escape" });
    expect(s.mode).toBe("select");
    expect(s.draft).toEqual([]);
    s = editorReducer(s, { type: "select", polygon: SQUARE });
    s = editorReducer(s, { type: "mode", mode: "measure" });
    s = editorReducer(s, { type: "escape" });
    expect(s.selection).toEqual(SQUARE);
  });

  it("refuses a crossing polygon with one sentence", () => {
    let s = editorReducer(initialState, { type: "mode", mode: "polygon" });
    for (const at of [[0, 0], [10, 10], [10, 0], [0, 10]] as Xy[]) s = editorReducer(s, { type: "vertex", at });
    s = editorReducer(s, { type: "close" });
    expect(s.selection).toBeNull();
    expect(s.refusal).toBe(SELF_INTERSECTING);
  });

  it("names the next free patch layer", () => {
    expect(nextPatchName(["base", "patch-1", "patch-3"])).toBe("patch-2");
  });
});

describe("editor provenance and layers", () => {
  it("lists every version along the chain", () => {
    expect(versionsOf(EDITS)).toEqual(["out/cloud.laz", "out/cloud.v1.laz", "out/cloud.v2.laz"]);
  });

  it("describes each operation with what it changed", () => {
    expect(describeOp(EDITS.ops[1]!)).toBe("Patch patch-1, 8 points added");
    expect(describeOp({ op: "decimate", target_percent: 50, removed: 1200, deviation_m: { mean: 0.011, p95: 0.034 }, result: "", ts: "" })).toBe(
      "Decimate, 1,200 points removed, deviation 0.011 m mean, 0.034 m p95",
    );
  });

  it("refuses to delete a layer another patch was made from, naming the dependent", () => {
    const edits: EditsFile = {
      ...EDITS,
      layers: [...EDITS.layers, { name: "patch-2", id: 2, points: 8 }],
      ops: [
        ...EDITS.ops,
        {
          op: "patch",
          layer: "patch-2",
          source: { box: [[30, 0], [32, 2]] },
          target: { translate: [10, 0], rotate_deg: 0 },
          added: 8,
          source_layers: ["patch-1"],
          result: "out/cloud.v3.laz",
          ts: "",
        },
      ],
    };
    expect(dependentOf(edits, "patch-1")).toBe("patch-2");
    const onDelete = vi.fn();
    render(<LayersPanel edits={edits} hidden={[]} readOnly={false} onToggle={() => {}} onDelete={onDelete} />);
    fireEvent.click(screen.getByLabelText("Delete patch-1"));
    expect(onDelete).toHaveBeenCalledWith("patch-1", "patch-2 was made from this layer; delete patch-2 first.");
    fireEvent.click(screen.getByLabelText("Delete patch-2"));
    expect(onDelete).toHaveBeenLastCalledWith("patch-2", null);
  });
});

describe("one writer per product", () => {
  it("opens a second tab on the same product read-only", async () => {
    const first = renderHook(() => useSingleWriter("node-1:rec-1"));
    await act(async () => {});
    const second = renderHook(() => useSingleWriter("node-1:rec-1"));
    const other = renderHook(() => useSingleWriter("node-1:rec-2"));
    await waitFor(() => expect(second.result.current).toBe(true));
    expect(first.result.current).toBe(false);
    expect(other.result.current).toBe(false);
  });
});
