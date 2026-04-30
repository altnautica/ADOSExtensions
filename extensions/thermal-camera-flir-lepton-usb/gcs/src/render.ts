/**
 * Canvas painter for the thermal overlay.
 *
 * The agent already encodes a colorized H.264 stream that the host's
 * video pane displays. This canvas paints the same Y16 grid through a
 * GCS-selected palette so the operator can swap palettes interactively
 * without restarting the agent's encode pipeline.
 */

import { paletteLut, PALETTE_SIZE } from "./palettes";
import {
  DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
  KELVIN_C_OFFSET,
  type IsothermConfig,
  type PaletteName,
  type ThermalFrame,
} from "./types";

export interface RenderOptions {
  palette: PaletteName;
  isotherm?: IsothermConfig;
  /**
   * Fixed-range AGC overrides for the lo/hi mapping. When unset the
   * painter scans the frame for its own extrema (linear AGC).
   */
  fixedRange?: { minC: number; maxC: number };
}

/**
 * Paint a Y16 frame into an existing ``ImageData`` buffer.
 *
 * The buffer must match ``frame.width`` and ``frame.height``. Returns
 * the same buffer for convenience.
 */
export function paintFrame(
  frame: ThermalFrame,
  options: RenderOptions,
  imageData: ImageData,
): ImageData {
  if (
    imageData.width !== frame.width ||
    imageData.height !== frame.height
  ) {
    throw new Error(
      `imageData size (${imageData.width}x${imageData.height}) does not ` +
        `match frame (${frame.width}x${frame.height})`,
    );
  }

  const lut = paletteLut(options.palette);
  const pixels = frame.width * frame.height;
  const resolution =
    frame.resolutionKPerCount ?? DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT;

  const { loY16, hiY16 } = computeRange(frame, options, resolution);
  const span = Math.max(1, hiY16 - loY16);

  const isotherm = options.isotherm;
  const isothermLoY16 = isotherm
    ? celsiusToY16(isotherm.lowerC, resolution)
    : null;
  const isothermHiY16 = isotherm
    ? celsiusToY16(isotherm.upperC, resolution)
    : null;

  const out = imageData.data;
  for (let i = 0; i < pixels; i += 1) {
    const raw = frame.y16[i];
    if (raw === undefined) continue;
    const clamped = raw < loY16 ? loY16 : raw > hiY16 ? hiY16 : raw;
    const t = (clamped - loY16) / span;
    const idx = Math.min(
      PALETTE_SIZE - 1,
      Math.max(0, Math.floor(t * (PALETTE_SIZE - 1))),
    );
    const base = idx * 3;
    let r = lut[base] ?? 0;
    let g = lut[base + 1] ?? 0;
    let b = lut[base + 2] ?? 0;
    if (
      isotherm &&
      isotherm.enabled &&
      isothermLoY16 !== null &&
      isothermHiY16 !== null &&
      raw >= isothermLoY16 &&
      raw <= isothermHiY16
    ) {
      // Blend a red highlight at 50% over the palette colour.
      r = Math.round(r * 0.5 + 255 * 0.5);
      g = Math.round(g * 0.5);
      b = Math.round(b * 0.5);
    }
    const dst = i * 4;
    out[dst] = r;
    out[dst + 1] = g;
    out[dst + 2] = b;
    out[dst + 3] = 255;
  }
  return imageData;
}

/**
 * Helper: build a fresh ``ImageData`` for a frame size. Tests use this
 * to avoid pulling in DOM types.
 */
export function makeImageData(width: number, height: number): ImageData {
  return {
    data: new Uint8ClampedArray(width * height * 4),
    width,
    height,
    colorSpace: "srgb",
  } as ImageData;
}

function computeRange(
  frame: ThermalFrame,
  options: RenderOptions,
  resolution: number,
): { loY16: number; hiY16: number } {
  if (options.fixedRange) {
    const lo = celsiusToY16(options.fixedRange.minC, resolution);
    const hi = celsiusToY16(options.fixedRange.maxC, resolution);
    return { loY16: Math.min(lo, hi), hiY16: Math.max(lo, hi) };
  }
  let lo = Number.POSITIVE_INFINITY;
  let hi = Number.NEGATIVE_INFINITY;
  for (let i = 0; i < frame.width * frame.height; i += 1) {
    const v = frame.y16[i];
    if (v === undefined) continue;
    if (v < lo) lo = v;
    if (v > hi) hi = v;
  }
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) {
    return { loY16: 0, hiY16: 1 };
  }
  return { loY16: lo, hiY16: hi };
}

function celsiusToY16(celsius: number, resolution: number): number {
  return Math.round((celsius + KELVIN_C_OFFSET) / resolution);
}
