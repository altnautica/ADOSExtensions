import { describe, expect, it } from "vitest";

import { REFRAME_MS, SETTLE_MS, StreamedFraming, type FramingStep } from "../../../src/components/world/streamed-framing";
import { splatFramingBounds, type ForEachSplat } from "../../../src/lib/world/splat-bounds";

/** A 20 m row of splats paged in from the near end, as a view-ordered pager does. */
const ALL = Array.from({ length: 4000 }, (_, i) => ({ x: (i / 4000) * 20, y: 0, z: 0 }));
const pagedIn = (n: number): ForEachSplat => (cb) => {
  const s = { x: 0.05, y: 0.05, z: 0.05 };
  for (let i = 0; i < n; i++) cb(i, ALL[i % ALL.length]!, s, null, 0.9);
};

/** Run the stage's per-frame loop at 60 Hz over `counts(t)`; the extent framed last. */
function run(counts: (t: number) => number, until: number, framing = new StreamedFraming(), userMovesAt = Infinity) {
  let framedMax: number | null = null;
  const steps: FramingStep[] = [];
  for (let t = 0; t <= until; t += 1000 / 60) {
    if (t >= userMovesAt) framing.stop();
    const n = counts(t);
    const step = framing.step(n, t);
    steps.push(step);
    if (step === "frame") framedMax = splatFramingBounds(pagedIn(n), n)!.max[0];
    if (step === "done") break;
  }
  return { framedMax, steps };
}

describe("StreamedFraming", () => {
  it("waits for the first pages, then re-frames as the paged box grows to the full extent", () => {
    // Nothing for 300 ms, then pages arrive over a second, then hold.
    const counts = (t: number) => (t < 300 ? 0 : Math.min(ALL.length, Math.floor(((t - 300) / 1000) * ALL.length)));
    const { framedMax, steps } = run(counts, 5000);
    expect(steps.slice(0, 10).every((s) => s === "wait")).toBe(true);
    expect(steps.filter((s) => s === "frame").length).toBeGreaterThan(2);
    expect(steps.at(-1)).toBe("done");
    expect(framedMax).toBeGreaterThan(19.5);
  });

  it("re-frames at most every REFRAME_MS", () => {
    const framing = new StreamedFraming();
    const at: number[] = [];
    for (let t = 0; t < 1000; t += 1000 / 60) if (framing.step(1 + Math.floor(t), t) === "frame") at.push(t);
    for (let i = 1; i < at.length; i++) expect(at[i]! - at[i - 1]!).toBeGreaterThanOrEqual(REFRAME_MS - 1e-9);
  });

  it("stops SETTLE_MS after the first pages even while pages keep arriving", () => {
    const { steps } = run((t) => 1 + Math.floor(t * 10), SETTLE_MS * 3);
    expect(steps.at(-1)).toBe("done");
    expect(steps.length * (1000 / 60)).toBeLessThan(SETTLE_MS + 400);
  });

  it("never re-frames after the user moves the camera", () => {
    const counts = (t: number) => Math.min(ALL.length, 1 + Math.floor((t / 1000) * ALL.length));
    const { framedMax, steps } = run(counts, 5000, new StreamedFraming(), 200);
    expect(steps.at(-1)).toBe("done");
    // The last framing was on the pages before the move, not the whole row.
    expect(framedMax).toBeLessThan(6);
  });
});
