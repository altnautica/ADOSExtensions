/**
 * Overlay geometry: source-frame pixels <-> rendered video rect.
 *
 * The iframe cannot import the host's overlay component, so the
 * letterbox-corrected positioning and the inverse click hit-test are
 * reimplemented here as pure functions. A box is expressed as a fraction
 * of the source frame so it scales onto whatever size the host's video is
 * currently rendered at; a click in CSS pixels is mapped back to a source
 * frame pixel and the smallest box containing it is the designation pick.
 *
 * @license GPL-3.0-or-later
 */

import type { BBox, OverlayDetectionItem, RenderedRect } from "./types";

/** A box placed in the rendered rect, in CSS px relative to the wrapper. */
export interface PlacedBox {
  left: number;
  top: number;
  width: number;
  height: number;
}

/**
 * Place a source-frame box onto the rendered (letterbox-corrected) rect.
 * The box fractions are computed against the frame resolution then scaled
 * by the rendered rect and offset by its top-left, so the box lands
 * exactly over the visible video pixels (not the letterbox bars).
 */
export function placeBox(
  bbox: BBox,
  frameWidth: number,
  frameHeight: number,
  rect: RenderedRect,
): PlacedBox {
  const fw = frameWidth > 0 ? frameWidth : 1;
  const fh = frameHeight > 0 ? frameHeight : 1;
  const fx = clamp01(bbox.x / fw);
  const fy = clamp01(bbox.y / fh);
  const fwFrac = clamp01(bbox.width / fw);
  const fhFrac = clamp01(bbox.height / fh);
  const left = rect.left + fx * rect.width;
  const top = rect.top + fy * rect.height;
  const width = Math.min(rect.width - fx * rect.width, fwFrac * rect.width);
  const height = Math.min(rect.height - fy * rect.height, fhFrac * rect.height);
  return {
    left,
    top,
    width: Math.max(0, width),
    height: Math.max(0, height),
  };
}

/**
 * Map a pointer position (CSS px relative to the wrapper top-left) back to
 * a source-frame pixel. Returns null when the click falls outside the
 * rendered video rect (in the letterbox bars).
 */
export function clientToFramePixel(
  clientX: number,
  clientY: number,
  frameWidth: number,
  frameHeight: number,
  rect: RenderedRect,
): { x: number; y: number } | null {
  if (rect.width <= 0 || rect.height <= 0) return null;
  const dx = clientX - rect.left;
  const dy = clientY - rect.top;
  if (dx < 0 || dy < 0 || dx > rect.width || dy > rect.height) return null;
  const x = (dx / rect.width) * frameWidth;
  const y = (dy / rect.height) * frameHeight;
  return { x, y };
}

/** Whether a frame-space point lies inside a frame-space box. */
function pointInBox(px: number, py: number, b: BBox): boolean {
  return (
    px >= b.x && px <= b.x + b.width && py >= b.y && py <= b.y + b.height
  );
}

/** Area of a box (frame px^2). */
function boxArea(b: BBox): number {
  return Math.max(0, b.width) * Math.max(0, b.height);
}

/**
 * The smallest detection box that contains a frame-space point. The
 * smallest-containing rule lets the operator click a small subject inside
 * a larger overlapping box and still pick the intended one. Returns null
 * when no box contains the point.
 */
export function hitTestDetection(
  framePoint: { x: number; y: number },
  detections: OverlayDetectionItem[],
): OverlayDetectionItem | null {
  let best: OverlayDetectionItem | null = null;
  let bestArea = Number.POSITIVE_INFINITY;
  for (const det of detections) {
    if (!pointInBox(framePoint.x, framePoint.y, det.bbox)) continue;
    const area = boxArea(det.bbox);
    if (area < bestArea) {
      best = det;
      bestArea = area;
    }
  }
  return best;
}

function clamp01(v: number): number {
  if (v < 0) return 0;
  if (v > 1) return 1;
  return v;
}
