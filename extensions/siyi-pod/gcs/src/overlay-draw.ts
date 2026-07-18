/**
 * Pure geometry + drawing for the pod video HUD.
 *
 * The host streams `video.overlay.props` with the letterbox-correct rendered
 * rect and the pod's republished tracker boxes in pod-frame pixels. The pure
 * helpers scale a box into the rendered rect and pick a lock colour; `drawHud`
 * paints the reticle, track boxes, and range/zoom readout onto a 2D context.
 */

import type {
  OverlayDetection,
  PodState,
  RenderedRect,
  VideoOverlayProps,
} from "./types";

export interface PixelRect {
  x: number;
  y: number;
  width: number;
  height: number;
}

/** Scale a pod-frame box into the rendered video rect. */
export function scaleBoxToRendered(
  box: { x: number; y: number; width: number; height: number },
  streamWidth: number,
  streamHeight: number,
  rect: RenderedRect,
): PixelRect {
  const sx = streamWidth > 0 ? rect.width / streamWidth : 1;
  const sy = streamHeight > 0 ? rect.height / streamHeight : 1;
  return {
    x: rect.x + box.x * sx,
    y: rect.y + box.y * sy,
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
  props: VideoOverlayProps | null,
  state: PodState | null,
): void {
  if (!c || !props) return;
  const { renderedRect: rect, streamWidth, streamHeight } = props;
  c.clearRect(0, 0, c.canvas.width, c.canvas.height);

  // Center reticle.
  const cx = rect.x + rect.width / 2;
  const cy = rect.y + rect.height / 2;
  c.strokeStyle = "#3aa0d0";
  c.lineWidth = 1;
  c.beginPath();
  c.moveTo(cx - 12, cy);
  c.lineTo(cx + 12, cy);
  c.moveTo(cx, cy - 12);
  c.lineTo(cx, cy + 12);
  c.stroke();

  // Track boxes (the pod's republished tracker).
  for (const det of props.detections ?? []) {
    const r = scaleBoxToRendered(det.bbox, streamWidth, streamHeight, rect);
    c.strokeStyle = lockColor(det.lockState);
    c.lineWidth = 2;
    c.strokeRect(r.x, r.y, r.width, r.height);
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
  lines.forEach((line, i) => c.fillText(line, rect.x + 8, rect.y + 18 + i * 16));
}
