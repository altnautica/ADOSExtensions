import { describe, expect, it } from "vitest";

import { lockColor, scaleBoxToRendered } from "../src/overlay-draw";

const RECT = { x: 100, y: 50, width: 800, height: 450 };

describe("scaleBoxToRendered", () => {
  it("scales a pod-frame box into the letterboxed rendered rect", () => {
    // A box at the top-left quarter of a 1920x1080 stream.
    const r = scaleBoxToRendered(
      { x: 0, y: 0, width: 960, height: 540 },
      1920,
      1080,
      RECT,
    );
    expect(r.x).toBe(100);
    expect(r.y).toBe(50);
    expect(r.width).toBe(400); // 960 * (800/1920)
    expect(r.height).toBe(225); // 540 * (450/1080)
  });

  it("maps the stream centre to the rect centre", () => {
    const r = scaleBoxToRendered(
      { x: 960, y: 540, width: 0, height: 0 },
      1920,
      1080,
      RECT,
    );
    expect(r.x).toBe(RECT.x + RECT.width / 2);
    expect(r.y).toBe(RECT.y + RECT.height / 2);
  });

  it("does not divide by zero on a zero-size stream", () => {
    const r = scaleBoxToRendered({ x: 10, y: 10, width: 5, height: 5 }, 0, 0, RECT);
    expect(Number.isFinite(r.x)).toBe(true);
    expect(Number.isFinite(r.width)).toBe(true);
  });
});

describe("lockColor", () => {
  it("maps lock states to distinct colours", () => {
    expect(lockColor("locked")).not.toBe(lockColor("uncertain"));
    expect(lockColor("lost")).not.toBe(lockColor("locked"));
    expect(lockColor(null)).toBe(lockColor(undefined));
  });
});
