/**
 * Spot-meter helper.
 *
 * The overlay binds a click handler to the canvas; that handler maps
 * the mouse event coordinates to frame-space, queries
 * :func:`celsiusAt` against the latest received Y16 grid, and updates
 * the meter cursor.
 *
 * Pure functions only. The plugin entry point owns the DOM wiring.
 */

import {
  DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT,
  KELVIN_C_OFFSET,
  type ThermalFrame,
} from "./types";

export interface CanvasPoint {
  /** Mouse client X within the canvas, in pixels. */
  clientX: number;
  /** Mouse client Y within the canvas, in pixels. */
  clientY: number;
}

export interface CanvasRect {
  width: number;
  height: number;
}

export interface FramePoint {
  x: number;
  y: number;
}

/**
 * Map a click within the canvas DOM rect to a frame-space pixel
 * coordinate, clamped to the frame bounds.
 */
export function clientToFrame(
  click: CanvasPoint,
  canvasRect: CanvasRect,
  frame: { width: number; height: number },
): FramePoint {
  if (canvasRect.width <= 0 || canvasRect.height <= 0) {
    return { x: 0, y: 0 };
  }
  const fx = (click.clientX / canvasRect.width) * frame.width;
  const fy = (click.clientY / canvasRect.height) * frame.height;
  const x = Math.min(frame.width - 1, Math.max(0, Math.floor(fx)));
  const y = Math.min(frame.height - 1, Math.max(0, Math.floor(fy)));
  return { x, y };
}

/**
 * Read the temperature at a frame coordinate from the latest Y16 grid.
 * Returns ``null`` when the frame has not yet been received or when
 * the requested coordinate is out of range.
 */
export function celsiusAt(
  frame: ThermalFrame | null,
  x: number,
  y: number,
): number | null {
  if (!frame) return null;
  if (x < 0 || y < 0) return null;
  if (x >= frame.width || y >= frame.height) return null;
  const index = y * frame.width + x;
  const raw = frame.y16[index];
  if (raw === undefined) return null;
  const resolution =
    frame.resolutionKPerCount ?? DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT;
  return raw * resolution - KELVIN_C_OFFSET;
}

/**
 * Compute the (min, max) celsius pair for the entire frame. Useful for
 * the AGC linear mode where the GCS chooses to render the full frame
 * range without round-tripping the agent's reported extrema.
 */
export function frameExtrema(
  frame: ThermalFrame,
): { minC: number; maxC: number } {
  const resolution =
    frame.resolutionKPerCount ?? DEFAULT_TLINEAR_RESOLUTION_K_PER_COUNT;
  let minRaw = Number.POSITIVE_INFINITY;
  let maxRaw = Number.NEGATIVE_INFINITY;
  for (let i = 0; i < frame.width * frame.height; i += 1) {
    const v = frame.y16[i];
    if (v === undefined) continue;
    if (v < minRaw) minRaw = v;
    if (v > maxRaw) maxRaw = v;
  }
  if (!Number.isFinite(minRaw) || !Number.isFinite(maxRaw)) {
    return { minC: 0, maxC: 0 };
  }
  return {
    minC: minRaw * resolution - KELVIN_C_OFFSET,
    maxC: maxRaw * resolution - KELVIN_C_OFFSET,
  };
}

/**
 * Map a frame-space spot meter coordinate back to canvas-space pixel
 * offsets so the cursor visual lines up with the underlying pixel.
 */
export function frameToCanvas(
  point: FramePoint,
  canvasRect: CanvasRect,
  frame: { width: number; height: number },
): CanvasPoint {
  if (frame.width <= 0 || frame.height <= 0) {
    return { clientX: 0, clientY: 0 };
  }
  const clientX = ((point.x + 0.5) / frame.width) * canvasRect.width;
  const clientY = ((point.y + 0.5) / frame.height) * canvasRect.height;
  return { clientX, clientY };
}
