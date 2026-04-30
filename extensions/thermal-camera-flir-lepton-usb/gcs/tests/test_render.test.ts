import { describe, it, expect } from "vitest";

import { listPalettes, paletteLut, PALETTE_SIZE } from "../src/palettes";
import { makeImageData, paintFrame } from "../src/render";
import {
  DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
  KELVIN_C_OFFSET,
  type PaletteName,
  type ThermalFrame,
} from "../src/types";

function celsiusToY16(celsius: number): number {
  return Math.round(
    (celsius + KELVIN_C_OFFSET) / DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
  );
}

function syntheticFrame(width: number, height: number): ThermalFrame {
  const y16 = new Uint16Array(width * height);
  // Linear ramp from 0 deg C at index 0 to 100 deg C at the last pixel.
  const lo = celsiusToY16(0);
  const hi = celsiusToY16(100);
  for (let i = 0; i < y16.length; i += 1) {
    const t = i / Math.max(1, y16.length - 1);
    y16[i] = Math.round(lo + (hi - lo) * t);
  }
  return {
    timestampNs: 1,
    sequence: 1,
    width,
    height,
    y16,
  };
}

describe("palettes", () => {
  it("ships three palettes that all fill 256 RGB triples", () => {
    const names = listPalettes();
    expect(names).toEqual(["ironbow", "rainbow", "grayscale"]);
    for (const name of names) {
      const lut = paletteLut(name as PaletteName);
      expect(lut.length).toBe(PALETTE_SIZE * 3);
    }
  });

  it("has a strictly monotonic grayscale", () => {
    const lut = paletteLut("grayscale");
    for (let i = 0; i < PALETTE_SIZE; i += 1) {
      expect(lut[i * 3]).toBe(i);
      expect(lut[i * 3 + 1]).toBe(i);
      expect(lut[i * 3 + 2]).toBe(i);
    }
  });

  it("rejects unknown palette names", () => {
    expect(() => paletteLut("not-a-palette" as PaletteName)).toThrow(
      /unknown palette/,
    );
  });
});

describe("paintFrame", () => {
  it("fills the entire RGBA buffer with full alpha", () => {
    const frame = syntheticFrame(8, 4);
    const img = makeImageData(8, 4);
    paintFrame(frame, { palette: "grayscale" }, img);
    for (let i = 3; i < img.data.length; i += 4) {
      expect(img.data[i]).toBe(255);
    }
  });

  it("paints the linear ramp from black to white in grayscale", () => {
    const frame = syntheticFrame(8, 1);
    const img = makeImageData(8, 1);
    paintFrame(frame, { palette: "grayscale" }, img);
    // First pixel is the min, last is the max -> 0 and 255.
    expect(img.data[0]).toBe(0);
    expect(img.data[1]).toBe(0);
    expect(img.data[2]).toBe(0);
    const last = (8 - 1) * 4;
    expect(img.data[last]).toBe(255);
    expect(img.data[last + 1]).toBe(255);
    expect(img.data[last + 2]).toBe(255);
  });

  it("respects a fixed-range AGC by clamping below and above", () => {
    const frame = syntheticFrame(4, 1);
    const img = makeImageData(4, 1);
    paintFrame(
      frame,
      {
        palette: "grayscale",
        fixedRange: { minC: 25, maxC: 75 },
      },
      img,
    );
    // 0 deg C is below the floor: clamps to LUT[0] = (0,0,0).
    expect(img.data[0]).toBe(0);
    // The ramp's last sample is 100 deg C, above the ceiling: clamps to
    // LUT[255] = (255,255,255).
    const last = (4 - 1) * 4;
    expect(img.data[last]).toBe(255);
  });

  it("blends an isotherm highlight when configured", () => {
    const frame = syntheticFrame(4, 1);
    const img = makeImageData(4, 1);
    paintFrame(
      frame,
      {
        palette: "grayscale",
        isotherm: { enabled: true, lowerC: 30, upperC: 70 },
      },
      img,
    );
    // Inside the isotherm band the red channel must be clearly above the
    // green and blue channels (the helper blends 50% red over the
    // grayscale color).
    let highlit = false;
    for (let i = 0; i < 4; i += 1) {
      const r = img.data[i * 4] ?? 0;
      const g = img.data[i * 4 + 1] ?? 0;
      const b = img.data[i * 4 + 2] ?? 0;
      if (r > g + 30 && r > b + 30) {
        highlit = true;
        break;
      }
    }
    expect(highlit).toBe(true);
  });

  it("rejects ImageData with a mismatched size", () => {
    const frame = syntheticFrame(8, 4);
    const img = makeImageData(4, 4);
    expect(() => paintFrame(frame, { palette: "grayscale" }, img)).toThrow(
      /size/,
    );
  });
});
