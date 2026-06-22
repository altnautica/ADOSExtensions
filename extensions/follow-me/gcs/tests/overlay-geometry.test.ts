import { describe, expect, it } from "vitest";

import {
  clientToFramePixel,
  hitTestDetection,
  placeBox,
} from "../src/overlay-geometry";
import type { OverlayDetectionItem, RenderedRect } from "../src/types";

const RECT: RenderedRect = { left: 100, top: 50, width: 640, height: 480 };

describe("placeBox", () => {
  it("maps a frame box onto the rendered rect with the letterbox offset", () => {
    const placed = placeBox(
      { x: 320, y: 240, width: 64, height: 48 },
      640,
      480,
      RECT,
    );
    // 320/640 = 0.5 across, 240/480 = 0.5 down, scaled by rect + offset.
    expect(placed.left).toBeCloseTo(100 + 0.5 * 640, 3);
    expect(placed.top).toBeCloseTo(50 + 0.5 * 480, 3);
    expect(placed.width).toBeCloseTo((64 / 640) * 640, 3);
    expect(placed.height).toBeCloseTo((48 / 480) * 480, 3);
  });

  it("clamps a box that overruns the frame edge", () => {
    const placed = placeBox(
      { x: 600, y: 460, width: 200, height: 200 },
      640,
      480,
      RECT,
    );
    // The box may not paint past the right/bottom edge of the rendered rect.
    expect(placed.left + placed.width).toBeLessThanOrEqual(
      RECT.left + RECT.width + 0.001,
    );
    expect(placed.top + placed.height).toBeLessThanOrEqual(
      RECT.top + RECT.height + 0.001,
    );
  });
});

describe("clientToFramePixel", () => {
  it("inverts the rendered rect back to a source-frame pixel", () => {
    // Wrapper-relative click at the rect center.
    const pt = clientToFramePixel(
      RECT.left + RECT.width / 2,
      RECT.top + RECT.height / 2,
      640,
      480,
      RECT,
    );
    expect(pt).not.toBeNull();
    expect(pt!.x).toBeCloseTo(320, 3);
    expect(pt!.y).toBeCloseTo(240, 3);
  });

  it("returns null for a click in the letterbox bars", () => {
    expect(clientToFramePixel(10, 10, 640, 480, RECT)).toBeNull();
  });
});

describe("hitTestDetection", () => {
  const small: OverlayDetectionItem = {
    bbox: { x: 300, y: 220, width: 40, height: 40 },
    classLabel: "person",
    confidence: 0.9,
    trackId: 7,
    lockState: null,
  };
  const large: OverlayDetectionItem = {
    bbox: { x: 100, y: 100, width: 400, height: 300 },
    classLabel: "person",
    confidence: 0.8,
    trackId: 3,
    lockState: null,
  };

  it("returns the smallest containing box when boxes overlap", () => {
    const hit = hitTestDetection({ x: 320, y: 240 }, [large, small]);
    expect(hit).toBe(small);
  });

  it("returns null when no box contains the point", () => {
    expect(hitTestDetection({ x: 5, y: 5 }, [large, small])).toBeNull();
  });
});
