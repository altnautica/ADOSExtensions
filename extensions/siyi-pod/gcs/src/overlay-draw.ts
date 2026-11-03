/**
 * Pure geometry + drawing for the pod video HUD.
 *
 * The host pushes the `video.overlay` props (`ctx.videoOverlay.onProps`): the
 * letterbox-correct rendered rect and the fresh detection batch in source-frame
 * pixels. The pure helpers scale a box into the rendered rect and pick a lock
 * colour; `drawHud` paints the reticle, track boxes, and range/zoom readout
 * onto a 2D context.
 */

import type {
  VideoOverlayHostProps,
  VideoOverlayRenderedRect,
} from "@altnautica/plugin-sdk";

import type { PodState } from "./types";

export interface PixelRect {
  x: number;
  y: number;
  width: number;
  height: number;
}

/** Scale a source-frame box into the rendered video rect. */
export function scaleBoxToRendered(
  box: { x: number; y: number; width: number; height: number },
  frameWidth: number,
  frameHeight: number,
  rect: VideoOverlayRenderedRect,
): PixelRect {
  const sx = frameWidth > 0 ? rect.width / frameWidth : 1;
  const sy = frameHeight > 0 ? rect.height / frameHeight : 1;
  return {
    x: rect.left + box.x * sx,
    y: rect.top + box.y * sy,
    width: box.width * sx,
    height: box.height * sy,
  };
}

export function lockColor(lockState: string | null | undefined): string {
  switch (lockState) {
    case "locked":
      return "#3ad07e";
    case "uncertain":
      return "#e0a53a";
    case "lost":
      return "#d05656";
    default:
      return "#3aa0d0";
  }
}

/** Paint the pod HUD. No-op when the 2D context or props are missing. */
export function drawHud(
  c: CanvasRenderingContext2D | null,
  props: VideoOverlayHostProps | null,
  state: PodState | null,
): void {
  if (!c || !props) return;
  const { renderedRect: rect, detections } = props;
  c.clearRect(0, 0, c.canvas.width, c.canvas.height);

  // Center reticle.
  const cx = rect.left + rect.width / 2;
  const cy = rect.top + rect.height / 2;
  c.strokeStyle = "#3aa0d0";
  c.lineWidth = 1;
  c.beginPath();
  c.moveTo(cx - 12, cy);
  c.lineTo(cx + 12, cy);
  c.moveTo(cx, cy - 12);
  c.lineTo(cx, cy + 12);
  c.stroke();

  // Track boxes from the fresh batch; none once the host marks it stale.
  if (detections) {
    const { frameWidth, frameHeight } = detections;
    for (const det of detections.items) {
      const r = scaleBoxToRendered(det.bbox, frameWidth, frameHeight, rect);
      c.strokeStyle = lockColor(det.lockState);
      c.lineWidth = 2;
      c.strokeRect(r.x, r.y, r.width, r.height);
    }
  }

  // Range + zoom readout.
  c.fillStyle = "#e6f2ff";
  c.font = "12px system-ui, sans-serif";
  const lines: string[] = [];
  if (state?.laser_range_m != null) {
    lines.push(`RNG ${state.laser_range_m.toFixed(0)} m`);
  }
  if (state?.zoom != null && state.zoom > 1) {
    lines.push(`${state.zoom.toFixed(1)}x`);
  }
  lines.forEach((line, i) => c.fillText(line, rect.left + 8, rect.top + 18 + i * 16));
}
