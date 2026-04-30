/**
 * Palette LUTs for the thermal overlay. The Python and TypeScript
 * sides build their tables from the same anchor-stop formulas so a
 * frame painted on the agent matches a frame painted on the GCS for a
 * given palette name.
 *
 * Each palette is a ``Uint8ClampedArray`` of length 768 (256 RGB
 * triples). A normalised intensity ``t in [0, 1]`` selects the index
 * ``floor(t * 255)``.
 */

import type { PaletteName } from "./types";

export const PALETTE_SIZE = 256;

type Stop = readonly [number, number, number];

const IRONBOW_STOPS: ReadonlyArray<Stop> = [
  [0, 0, 0],
  [50, 0, 80],
  [170, 30, 0],
  [255, 160, 0],
  [255, 255, 255],
];

// Rainbow palette: cool to hot ramp with strictly increasing perceived
// lightness. Endpoints stay blue-dominant (cool) and red-dominant (hot)
// so a spot at higher temperature reads visually brighter than a
// cooler spot. Mirrors the Python LUT in palettes.py.
const RAINBOW_STOPS: ReadonlyArray<Stop> = [
  [0, 0, 80],
  [0, 60, 180],
  [60, 140, 200],
  [160, 180, 100],
  [240, 180, 80],
  [255, 220, 200],
];

function clip(v: number): number {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return v;
}

function gradient(stops: ReadonlyArray<Stop>): Uint8ClampedArray {
  if (stops.length < 2) {
    throw new Error("at least two color stops required");
  }
  const segments = stops.length - 1;
  const out = new Uint8ClampedArray(PALETTE_SIZE * 3);
  for (let i = 0; i < PALETTE_SIZE; i += 1) {
    const position = (i / (PALETTE_SIZE - 1)) * segments;
    const segIndex = Math.min(Math.floor(position), segments - 1);
    const local = position - segIndex;
    const lo = stops[segIndex] as Stop;
    const hi = stops[segIndex + 1] as Stop;
    out[i * 3] = clip(Math.round(lo[0] + (hi[0] - lo[0]) * local));
    out[i * 3 + 1] = clip(Math.round(lo[1] + (hi[1] - lo[1]) * local));
    out[i * 3 + 2] = clip(Math.round(lo[2] + (hi[2] - lo[2]) * local));
  }
  return out;
}

function buildGrayscale(): Uint8ClampedArray {
  const out = new Uint8ClampedArray(PALETTE_SIZE * 3);
  for (let i = 0; i < PALETTE_SIZE; i += 1) {
    out[i * 3] = i;
    out[i * 3 + 1] = i;
    out[i * 3 + 2] = i;
  }
  return out;
}

export const IRONBOW_LUT = gradient(IRONBOW_STOPS);
export const RAINBOW_LUT = gradient(RAINBOW_STOPS);
export const GRAYSCALE_LUT = buildGrayscale();

const PALETTES: Record<PaletteName, Uint8ClampedArray> = {
  ironbow: IRONBOW_LUT,
  rainbow: RAINBOW_LUT,
  grayscale: GRAYSCALE_LUT,
};

export function paletteLut(name: PaletteName): Uint8ClampedArray {
  const lut = PALETTES[name];
  if (!lut) throw new Error(`unknown palette: ${name}`);
  return lut;
}

export function listPalettes(): ReadonlyArray<PaletteName> {
  return ["ironbow", "rainbow", "grayscale"];
}
