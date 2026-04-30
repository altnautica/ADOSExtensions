import { describe, it, expect } from "vitest";

import {
  celsiusAt,
  clientToFrame,
  frameExtrema,
  frameToCanvas,
} from "../src/spotMeter";
import {
  DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
  KELVIN_C_OFFSET,
  type ThermalFrame,
} from "../src/types";

function celsiusToY16(celsius: number): number {
  return Math.round(
    (celsius + KELVIN_C_OFFSET) / DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
  );
}

function frameOf(width: number, height: number, fillC: number): ThermalFrame {
  const raw = celsiusToY16(fillC);
  const y16 = new Uint16Array(width * height).fill(raw);
  return { timestampNs: 0, sequence: 0, width, height, y16 };
}

describe("clientToFrame", () => {
  it("maps the canvas centre to the frame centre", () => {
    const point = clientToFrame(
      { clientX: 50, clientY: 50 },
      { width: 100, height: 100 },
      { width: 160, height: 120 },
    );
    expect(point.x).toBe(80);
    expect(point.y).toBe(60);
  });

  it("clamps clicks beyond the canvas to the frame edge", () => {
    const point = clientToFrame(
      { clientX: 1000, clientY: 1000 },
      { width: 100, height: 100 },
      { width: 160, height: 120 },
    );
    expect(point.x).toBe(159);
    expect(point.y).toBe(119);
  });

  it("returns the origin when the canvas has no size", () => {
    const point = clientToFrame(
      { clientX: 5, clientY: 5 },
      { width: 0, height: 0 },
      { width: 160, height: 120 },
    );
    expect(point).toEqual({ x: 0, y: 0 });
  });
});

describe("celsiusAt", () => {
  it("returns null when the frame is missing", () => {
    expect(celsiusAt(null, 0, 0)).toBeNull();
  });

  it("reads the temperature at a frame coordinate", () => {
    const frame = frameOf(4, 2, 25);
    const reading = celsiusAt(frame, 2, 1);
    expect(reading).not.toBeNull();
    expect(reading).toBeCloseTo(25, 6);
  });

  it("returns null for out-of-range coordinates", () => {
    const frame = frameOf(4, 2, 25);
    expect(celsiusAt(frame, -1, 0)).toBeNull();
    expect(celsiusAt(frame, 5, 0)).toBeNull();
    expect(celsiusAt(frame, 0, 5)).toBeNull();
  });

  it("respects an override TLinear resolution", () => {
    const frame: ThermalFrame = {
      timestampNs: 0,
      sequence: 0,
      width: 1,
      height: 1,
      y16: new Uint16Array([2750]),
      resolutionKPerCount: 0.1,
    };
    // 2750 counts at 0.1 K/count = 275.0 K = 1.85 deg C
    expect(celsiusAt(frame, 0, 0)).toBeCloseTo(1.85, 6);
  });
});

describe("frameExtrema", () => {
  it("reports min and max in deg C", () => {
    const y16 = new Uint16Array(4);
    y16[0] = celsiusToY16(0);
    y16[1] = celsiusToY16(10);
    y16[2] = celsiusToY16(20);
    y16[3] = celsiusToY16(60);
    const frame: ThermalFrame = {
      timestampNs: 0,
      sequence: 0,
      width: 2,
      height: 2,
      y16,
    };
    const extrema = frameExtrema(frame);
    expect(extrema.minC).toBeCloseTo(0, 6);
    expect(extrema.maxC).toBeCloseTo(60, 6);
  });
});

describe("frameToCanvas", () => {
  it("places the marker at the centre of the target pixel", () => {
    const point = frameToCanvas(
      { x: 80, y: 60 },
      { width: 320, height: 240 },
      { width: 160, height: 120 },
    );
    // Pixel centre: (80 + 0.5)/160 * 320 = 161
    expect(point.clientX).toBeCloseTo(161, 6);
    expect(point.clientY).toBeCloseTo(121, 6);
  });
});
