import { useEffect, useRef, type CSSProperties } from "react";

import { TOKENS } from "../theme/tokens";

/** One corner of a detected tag drawn on the overlay. */
export interface DetectionCorner {
  x: number;
  y: number;
}

/** A detected tag's four corners + its tag id. */
export interface DetectionShape {
  tagId: number;
  corners: [DetectionCorner, DetectionCorner, DetectionCorner, DetectionCorner];
}

interface Props {
  /** ``MediaStream`` from the camera. Pass null to render an empty
   * preview state. The consumer owns the stream lifecycle. */
  stream: MediaStream | null;
  /** Detected tag overlays. The component draws coloured polygons
   * around each tag's corners. */
  detections?: DetectionShape[];
  /** Pixel width of the underlying frame the detections reference. */
  frameWidth?: number;
  /** Pixel height of the underlying frame the detections reference. */
  frameHeight?: number;
  /** Optional caption rendered above the video. */
  caption?: string;
  /** Optional callback fired on every animation frame with the
   * latest canvas snapshot (PNG data URL). Used by the calibration
   * wizard's per-frame quality scoring. */
  onFrame?: (dataUrl: string) => void;
  /** Throttle ``onFrame`` invocations to this interval. Default 200ms. */
  frameThrottleMs?: number;
}

/**
 * Camera preview with a real-time overlay canvas. Wraps a ``<video>``
 * element that consumes the provided ``MediaStream`` and a
 * ``<canvas>`` element layered on top that draws detection polygons.
 *
 * Detection drawing runs in a ``requestAnimationFrame`` loop so the
 * overlay stays in sync with the video frame rate. The optional
 * ``onFrame`` callback periodically samples the video into a PNG
 * data URL the consumer can keep for capture / quality scoring.
 */
export function CameraPreview({
  stream,
  detections = [],
  frameWidth = 640,
  frameHeight = 480,
  caption,
  onFrame,
  frameThrottleMs = 200,
}: Props): JSX.Element {
  const videoRef = useRef<HTMLVideoElement | null>(null);
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const lastFrameRef = useRef<number>(0);

  // Attach the stream to the video element.
  useEffect(() => {
    const video = videoRef.current;
    if (video === null) return;
    if (stream === null) {
      video.srcObject = null;
      return;
    }
    video.srcObject = stream;
    video.play().catch(() => {
      // Autoplay policy on some browsers blocks unmuted play; we
      // attach as muted in JSX below so this should not fire, but
      // keep the catch quiet.
    });
  }, [stream]);

  // Draw detections + sample frames on every animation tick.
  useEffect(() => {
    let raf = 0;
    function tick(now: number) {
      const canvas = canvasRef.current;
      if (canvas !== null) {
        const ctx = canvas.getContext("2d");
        if (ctx !== null) {
          ctx.clearRect(0, 0, canvas.width, canvas.height);
          ctx.strokeStyle = TOKENS.ok;
          ctx.lineWidth = 2;
          ctx.font = "12px system-ui, sans-serif";
          ctx.fillStyle = TOKENS.ok;
          for (const det of detections) {
            const c = det.corners;
            ctx.beginPath();
            ctx.moveTo(c[0].x, c[0].y);
            ctx.lineTo(c[1].x, c[1].y);
            ctx.lineTo(c[2].x, c[2].y);
            ctx.lineTo(c[3].x, c[3].y);
            ctx.closePath();
            ctx.stroke();
            ctx.fillText(`${det.tagId}`, c[0].x + 4, c[0].y + 14);
          }
        }
      }
      if (onFrame !== undefined && videoRef.current !== null) {
        if (now - lastFrameRef.current >= frameThrottleMs) {
          lastFrameRef.current = now;
          try {
            const v = videoRef.current;
            const cap = document.createElement("canvas");
            cap.width = frameWidth;
            cap.height = frameHeight;
            const cx = cap.getContext("2d");
            if (cx !== null) {
              cx.drawImage(v, 0, 0, frameWidth, frameHeight);
              onFrame(cap.toDataURL("image/png"));
            }
          } catch {
            // ignore; video may not have a frame yet
          }
        }
      }
      raf = window.requestAnimationFrame(tick);
    }
    raf = window.requestAnimationFrame(tick);
    return () => window.cancelAnimationFrame(raf);
  }, [detections, onFrame, frameThrottleMs, frameWidth, frameHeight]);

  return (
    <div style={wrapper} data-testid="ext-ui-camera-preview">
      {caption !== undefined ? <span style={captionStyle}>{caption}</span> : null}
      <div style={mediaBox}>
        <video
          ref={videoRef}
          style={video}
          autoPlay
          muted
          playsInline
          data-testid="ext-ui-camera-preview-video"
        />
        <canvas
          ref={canvasRef}
          width={frameWidth}
          height={frameHeight}
          style={canvas}
          data-testid="ext-ui-camera-preview-canvas"
        />
      </div>
    </div>
  );
}

const wrapper: CSSProperties = {
  display: "flex",
  flexDirection: "column",
  gap: "0.375rem",
};
const captionStyle: CSSProperties = {
  fontSize: "0.7rem",
  color: TOKENS.textMuted,
};
const mediaBox: CSSProperties = {
  position: "relative",
  width: "100%",
  aspectRatio: "4 / 3",
  background: TOKENS.surface2,
  borderRadius: "0.375rem",
  overflow: "hidden",
};
const video: CSSProperties = {
  position: "absolute",
  inset: 0,
  width: "100%",
  height: "100%",
  objectFit: "cover",
  display: "block",
};
const canvas: CSSProperties = {
  position: "absolute",
  inset: 0,
  width: "100%",
  height: "100%",
  pointerEvents: "none",
};
